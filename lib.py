import smbus
import time
import numpy as np
from MCP4922 import MCP4922
import sys
import RPi.GPIO as GPIO
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
from datetime import date
import os
import ADS1256
import DAC8532 

DEVICE_BUS_INFLATION = 1 #I2C bus used for inflation
DEVICE_BUS_DEFLATION = 3 #I2C bus used for deflation
DEVICE_ADDR = 0x01 #Default Sensirion SFM4100-Air device address
SCALE_FACTOR = 1000 #Used to convert the value recieved to slm(standard litre per minute)  
PRESSURE_ERROR = -(6.246343882527859 + 0.5100433517496086 + 10.65)
PRESSURE_VOLT_DIV_FACT = 1.46 # Voltage divider factor for pressure sensor
MV_AVG_DEPTH = 10 # Depth of moving average
AIR_MASS_PER_LITRE = 1.292e-3 # in kg at STP
TARGET_MASS = 20e-6 # in kg
MAX_SET_VALUE = 5 #Maximum voltage to be set
MIN_SET_VALUE = 0 #Minimum voltage to be set
OFFSET = 0
CONTROL_OFFSET = 0
TIME_DURATION = 4 + CONTROL_OFFSET
CURRENT_DRIVER_SENSE_RES_VALUE = 2.08


"""The calibrated signal read from the flow sensor is a signed
INTEGER number (two's complement number). The
INTEGER value can be converted to the physical value by
dividing it by the scale factor (mass flow = sensor output/ 
scale factor)."""



def log_data(log,time,actual_flow, target_flow,pressure, 
             current_mass, current_gt_mass, desired_Mass, 
             set_voltage_inflation, sense_current, set_voltage_deflation):
    """
    Reads and stores data in the file open in .csv format
    """
    
    # voltage = actual_channel.voltage 
    # set_pt_voltage = set_point_channel.voltage 
    # voltage_pressure = pressure_ch.voltage 
    today = date.today()
    log.write("{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}\n".format(str(today.strftime("%d/%m/%Y")),
        str(time),str(actual_flow),str (target_flow),
        str(pressure),str(current_mass),str(current_gt_mass), 
        str(desired_Mass), str(set_voltage_inflation), str(sense_current), str(set_voltage_deflation)))

def convertVolt2DacVal(volt_val):
    """
    Returns a converted value of the voltage input between a number between 0 to 4096 to be given to
    to the DAC for set point value
    """
    return int(((volt_val-OFFSET)/MAX_SET_VALUE) * 4096)

def flow_start_voltage_pressure (pressure):
    a,b,c = [0.70487578, -0.28534515 , 2.76591885]
    return a * pressure + b * pressure**2 + c

class CubicInterpolation:

    def __init__(self, mass_initial, mass_to_reach):
        self.completion_time = TIME_DURATION
        self.a_0 = mass_initial
        self.a_1 = 0
        self.a_2 = (3/(self.completion_time**2))*(mass_to_reach - mass_initial)
        self.a_3 = - (2/(self.completion_time**3))*(mass_to_reach - mass_initial)
        
    def getMass(self,t):
        return self.a_0 + self.a_1 * t + self.a_2 * t**2 + self.a_3 * t**3

    def getFlow(self,t):
        return self.a_1 + 2 * self.a_2 * t + 3 * self.a_3 * t**2

class ControlFlow:

    def __init__(self, proportional_gain, derivative_gain, integral_gain, previous_t):
        
        self.KP = proportional_gain
        self.KD = derivative_gain
        self.KI = integral_gain
        self.previous_error = 0
        self.previous_time = previous_t
        self.integrative_error = 0


    def contolLaw(self, controlVoltage, target_value, actual_value):
        self.target = target_value
        self.actual = actual_value
        # self.target_flow = target_flow
        # self.measured_flow = measured_flow


        self.error = self.target - self.actual
        self.del_t = time.time() - self.previous_time
        self.previous_time = time.time()

        # self.flow_error = self.target_flow - self.measured_flow
        
        self.derivative_error = (self.error - self.previous_error)/self.del_t
        self.integrative_error += (self.error - self.previous_error) * self.del_t
        
        controlVoltage = controlVoltage + self.error * self.KP + self.derivative_error* self.KD  + self.integrative_error * self.KI
        controlVoltage = max(min(MAX_SET_VALUE, controlVoltage), MIN_SET_VALUE)
        
        self.previous_error = self.error
        
        return controlVoltage

def interpol_fn(mass_initial, mass_to_reach, time_duration,num_time_steps, func_type = "Cubic"):
    """
    The function creates a cubic interpolation between two mass states
    """
    current_time = 0
    t = np.linspace(current_time,current_time + time_duration,num_time_steps)
    h = 0.1
    if func_type == "Cubic":    
        a_0 = mass_initial
        a_1 = 0
        a_2 = (3/(time_duration**2))*(mass_to_reach - mass_initial)
        a_3 = - (2/(time_duration**3))*(mass_to_reach - mass_initial)
        Mass_t = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3
        Mass_dot_t = np.diff(Mass_t)/h ## Function?
    return Mass_t, Mass_dot_t, t

class sinosoidalTrajectory:
    def __init__(self,mass_initial, mass_to_reach):
        self.initial_mass = mass_initial
        self.final_mass = mass_to_reach
        self.completion_time = 2 * TIME_DURATION
        self.amplitude = (mass_to_reach - mass_initial)/2
        self.frequency = 1 / self.completion_time

    def getMass(self,t):
        return self.amplitude*np.sin(2 * np.pi * self.frequency * t - np.pi/2 ) + (self.initial_mass + self.final_mass)/2 #/ self.completion_time
    
    def getFlow(self,t):
        return self.amplitude*np.cos(2 * np.pi * self.frequency * t - np.pi/2) 

class trajectoryGenerator:
    def choose(trajectoryName,mass_initial, mass_to_reach):
        if trajectoryName == "cubic":
            print("cubic trajectory")
            return CubicInterpolation(mass_initial, mass_to_reach)
        if trajectoryName == "sin":
            print("Using sinosoidal trajectory")
            return sinosoidalTrajectory(mass_initial, mass_to_reach)

def twosComp(val, bits):
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val

def convert_kgs2lmin(flow_kgs):
    """
    Converts the flow rate in kg/s to l/min
    """

    flow_lmin = flow_kgs * 60/AIR_MASS_PER_LITRE 
    return flow_lmin


def convert_lmin2kgs(value_lmin):
    """
    Converts flowrate from l/min to kg/s
    """ 

    kgs_val = value_lmin * (AIR_MASS_PER_LITRE)/60
    return kgs_val

def convert_volt2pressure(voltage_pressure_sensor):
    v_cc = 5.0
    pressure = (((voltage_pressure_sensor*PRESSURE_VOLT_DIV_FACT/(v_cc))-0.04)/0.00369) + PRESSURE_ERROR
    # voltage = self.df["pressure sensor"].to_numpy().tolist()

    return pressure

def getVolume():
    # Basic Volume
    # small tube: (13.5cm + 16.5 + 11.5 + 1.5 + 65 cm) * 0.2cm
    # big: 197.5cm, small: 101.0cm
    # elastic tube: 45cm * 0.2
    # T-connector: (3.6cm + 1.8cm) * 0.4cm
    # elastic-to-small-connector: 4.5cm * 0.4cm
    # split-connector: (2 * 2cm + 1.5cm) * 0.4cm
    # valve: 2 * 0.5 * 10cm * 0.5cm


    setup_length = [13.5, 16.5, 11.5, 1.5, 197.5, 45, 65, 3.6, 1.8, 4.5, 5.5, 5, 5]
    setup_diameter = [0.2, 0.2, 0.2, 0.2, 0.5, 0.2, 0.2, 0.4, 0.4, 0.4, 0.4, 0.5, 0.5]

    system_volume = 0

    for i in range(len(setup_length)):
        system_volume += np.pi * (setup_diameter[i] / 2) ** 2 * setup_length[i]

    return system_volume

def computeMassDelta(p_after, p_before=0.0, r=287.058, t=293.15):
        """
        Computes the air mass change given a known channel volume

            Arguments:
                p_after --  pressure in kPa (relative to ambient)
                p_before -- pressure in kPa (relative to ambient)
                V_reference -- volume in ccm

                r = 287.058 -- specific gas constant of air at room temperature
                t = 293.15 --  air temperature in Kelvin (20 degrees Celsius, room temperature)

                returns -- mass change in mg
        """

        v_reference = getVolume()

        # scale volume into cubic meter (0.01**3 = 1e-6)
        v_reference = v_reference * 1e-6
        # print("Volume = ", v_reference)
        # compute pressure change in Pascal:
        delta_p = 1e3 * (p_after - p_before)

        # from the ideal gas law: p*V = mRT
        delta_m = delta_p * v_reference / (r * t)

        # return the mass in kg:

        return delta_m 

def ReadSensirion(device_bus):
    t2 = time.time()
    read_value = device_bus.read_i2c_block_data(DEVICE_ADDR,0xF1,2) # 5 ms TODO: Make it faster # measurement triggering command 0xF1 or hF1
    t3 = time.time()
    read_value = read_value[0]<<8 | read_value[1]

    flow_value = twosComp(read_value,16) #getting the twos complement
    flow_value = flow_value/SCALE_FACTOR

    return flow_value

def convert_volt2current(volt):
    return volt / CURRENT_DRIVER_SENSE_RES_VALUE


def runKelly(params, log, mode):

    if mode == 'inflation':
        i = params['i']
        traj = trajectoryGenerator.choose(params['trajType'],params['initialMass'],params['finalMass'])            
        print("Final Mass for inflation: ",params['finalMass'])
        params['t_start'] = time.time()

        while(True  & (params['t'] <= TIME_DURATION) & (params['p_after1'] < 250.0)): # & (params['CurrentMass'] <= params['finalMass'] )): #&  (avg_flow_value>=0.5)):
            time_loop_start = time.time()
            flow_val = ReadSensirion(params['bus_inf']) # 5 ms                         
            params['p_after1'] = convert_volt2pressure(params['adc'].ADS1256_GetChannalValue(params['pressure_channel']) * 5.0/0x7fffff) # 8 ms Full scale output is 0x7fffff for the ADC
            
            if params['p_after1'] > 200:
                print("Pressure exceeded, stopping....")
                break

            #Moving Average of the measure flow and pressure values
            params['flow_values'][i] = flow_val                
            params['pressure_values'][i] = params['p_after1']

            if i < MV_AVG_DEPTH:
                params['time_spent'] = time.time() - params['t_start']
                cf = ControlFlow(params['controlKP'], params['controlKD'], params['controlKI'], time.time())
            
            if i >= MV_AVG_DEPTH:                    
                avgFlowValue = np.mean(params['flow_values'][i-MV_AVG_DEPTH:i])               
                avgPressureValue = np.mean(params['pressure_values'][i-MV_AVG_DEPTH:i]) 
              
                sense_current_inf = 0

                targetFlow = convert_kgs2lmin(traj.getFlow(params['t']))
                desiredMass = traj.getMass(params['t'])
                # print("Desired Mass during inflation: {} at time {}".format(desiredMass,params['t']))

                pressure_diff = params['supply_pressure'] - params['p_after1']/100

                params['set_voltage_inf'] = cf.contolLaw(params['set_voltage_inf'],desiredMass,params['CurrentMass'])
               
                params['dac'].DAC8532_Out_Voltage(params['select_channel_inf'], params['set_voltage_inf']) # 3 ms
                
                del_t_corrected = time.time() - time_loop_start
                params['CurrentMass'] = params['CurrentMass'] +  del_t_corrected * convert_lmin2kgs(avgFlowValue)
               
                params['Current_GT_mass'] = computeMassDelta(avgPressureValue,p_before=0.0, r=287.058, t=293.15)                    
                params['t']= time.time() - params['t_start'] - params['time_spent'] 
                params['time_val'] = params['t']
                
                params['time_log']  = time.time() - params['t_absolute_start'] - params['time_spent']
                
                log_data(log, params['time_log'], avgFlowValue, targetFlow,
                    avgPressureValue, params['CurrentMass'], params['Current_GT_mass'], 
                    desiredMass, params['set_voltage_inf'],sense_current_inf, params['set_voltage_def'])

            timeOffset = time.time()-time_loop_start
                
            if params['del_t'] - timeOffset <= 0:
                time.sleep(0)
            else:
                time.sleep(params['del_t'] - timeOffset)
            i+=1

    if mode == 'deflation':
        traj = trajectoryGenerator.choose(params['trajType'],params['finalMass'],params['initialMass'])

        print('Cycle: ')
        print("Final Mass for deflation: ",params['initialMass'])
        i = params['i']
        params['t_start'] = time.time()

        while(True  & (params['t'] <=  TIME_DURATION)): #&  (avg_flow_value>=0.5)):
            time_loop_start = time.time()
            flow_val = ReadSensirion(params['bus_def']) # 5 ms                         
            params['p_after1'] = convert_volt2pressure(params['adc'].ADS1256_GetChannalValue(params['pressure_channel']) * 5.0/0x7fffff) # 8 ms Full scale output is 0x7fffff for the ADC
            
            if params['p_after1'] > 200:
                print("Pressure exceeded, stopping....")
                break

            #Moving Average of the measure flow and pressure values
            params['flow_values'][i] = flow_val                
            params['pressure_values'][i] = params['p_after1']

            if i < MV_AVG_DEPTH:
                params['time_spent'] = time.time() - params['t_start']            
                cf = ControlFlow(params['controlKP'], params['controlKD'], params['controlKI'], time.time())

            if i >= MV_AVG_DEPTH:                    
                avgFlowValue = np.mean(params['flow_values'][i-MV_AVG_DEPTH:i])               
                avgPressureValue = np.mean(params['pressure_values'][i-MV_AVG_DEPTH:i]) 
                
                sense_current_def = 0

                targetFlow = convert_kgs2lmin(traj.getFlow(params['t']))
                desiredMass = traj.getMass(params['t'])
                # print("Desired Mass during deflation: {} at time {}".format(desiredMass,params['t']))
                print("Desired Mass during deflation: ", desiredMass)
                
                # print("Current Mass during deflation: ", params['CurrentMass'])
                pressure_diff = params['supply_pressure'] - params['p_after1']/100
                
                params['set_voltage_def'] = cf.contolLaw(params['set_voltage_def'],params['CurrentMass'],desiredMass)
               
                params['dac'].DAC8532_Out_Voltage(params['select_channel_def'], params['set_voltage_def']) # 3 ms
                
                del_t_corrected = time.time() - time_loop_start
                params['CurrentMass'] = params['CurrentMass'] -  del_t_corrected * convert_lmin2kgs(avgFlowValue)
               
                params['Current_GT_mass'] = computeMassDelta(avgPressureValue,p_before=0.0, r=287.058, t=293.15)                    
                params['t']= time.time() - params['t_start'] - params['time_spent'] 
                params['time_val'] = params['t']
                params['time_log'] = time.time() - params['t_absolute_start'] - params['time_spent']
                log_data(log,params['time_log'],-avgFlowValue,targetFlow,
                    avgPressureValue,params['CurrentMass'], params['Current_GT_mass'], 
                    desiredMass, params['set_voltage_inf'], sense_current_def, params['set_voltage_def'])

            # print("Current Mass after deflation: ",params['CurrentMass'])
        
            timeOffset = time.time()-time_loop_start
                
            if params['del_t'] - timeOffset <= 0:
                time.sleep(0)
            else:
                time.sleep(params['del_t'] - timeOffset)
            i+=1

    return params

def runKelly_open_loop(params, log, mode):

    if mode == 'inflation':
        i = params['i']
        traj = trajectoryGenerator.choose(params['trajType'],params['initialMass'],params['finalMass'])            
        print("Final Mass for inflation: ",params['finalMass'])
        params['t_start'] = time.time()

        while(True & (params['p_after1'] < 250.0)): # & (params['CurrentMass'] <= params['finalMass'] )): #&  (avg_flow_value>=0.5)):
            time_loop_start = time.time()
            flow_val = ReadSensirion(params['bus_inf']) # 5 ms                         
            params['p_after1'] = convert_volt2pressure(params['adc'].ADS1256_GetChannalValue(params['pressure_channel']) * 5.0/0x7fffff) # 8 ms Full scale output is 0x7fffff for the ADC
            
            if params['p_after1'] > 200:
                print("Pressure exceeded, stopping....")
                break

            #Moving Average of the measure flow and pressure values
            params['flow_values'][i] = flow_val                
            params['pressure_values'][i] = params['p_after1']

            if i < MV_AVG_DEPTH:
                params['time_spent'] = time.time() - params['t_start']
                
            
            if i >= MV_AVG_DEPTH:                    
                avgFlowValue = np.mean(params['flow_values'][i-MV_AVG_DEPTH:i])               
                avgPressureValue = np.mean(params['pressure_values'][i-MV_AVG_DEPTH:i]) 
              
                sense_current_inf = 0

                targetFlow = convert_kgs2lmin(traj.getFlow(params['t']))
                desiredMass = traj.getMass(params['t'])
                # print("Desired Mass during inflation: {} at time {}".format(desiredMass,params['t']))

                pressure_diff = params['supply_pressure'] - params['p_after1']/100

                
                params['dac'].DAC8532_Out_Voltage(params['select_channel_inf'], params['set_voltage_inf']) # 3 ms
                
                del_t_corrected = time.time() - time_loop_start
                params['CurrentMass'] = params['CurrentMass'] +  del_t_corrected * convert_lmin2kgs(avgFlowValue)
               
                params['Current_GT_mass'] = computeMassDelta(avgPressureValue,p_before=0.0, r=287.058, t=293.15)                    
                params['t']= time.time() - params['t_start'] - params['time_spent'] 
                params['time_val'] = params['t']
                
                params['time_log']  = time.time() - params['t_absolute_start'] - params['time_spent']
                
                log_data(log, params['time_log'], avgFlowValue, targetFlow,
                    avgPressureValue, params['CurrentMass'], params['Current_GT_mass'], 
                    desiredMass, params['set_voltage_inf'],sense_current_inf, params['set_voltage_def'])

            timeOffset = time.time()-time_loop_start
                
            if params['del_t'] - timeOffset <= 0:
                time.sleep(0)
            else:
                time.sleep(params['del_t'] - timeOffset)
            i+=1

    if mode == 'deflation':
        traj = trajectoryGenerator.choose(params['trajType'],params['finalMass'],params['initialMass'])

        print('Cycle: ')
        print("Final Mass for deflation: ",params['initialMass'])
        i = params['i']
        params['t_start'] = time.time()

        while(True): #&  (avg_flow_value>=0.5)):
            time_loop_start = time.time()
            flow_val = ReadSensirion(params['bus_def']) # 5 ms                         
            params['p_after1'] = convert_volt2pressure(params['adc'].ADS1256_GetChannalValue(params['pressure_channel']) * 5.0/0x7fffff) # 8 ms Full scale output is 0x7fffff for the ADC
            
            if params['p_after1'] > 200:
                print("Pressure exceeded, stopping....")
                break

            #Moving Average of the measure flow and pressure values
            params['flow_values'][i] = flow_val                
            params['pressure_values'][i] = params['p_after1']

            if i < MV_AVG_DEPTH:
                params['time_spent'] = time.time() - params['t_start']            
               

            if i >= MV_AVG_DEPTH:                    
                avgFlowValue = np.mean(params['flow_values'][i-MV_AVG_DEPTH:i])               
                avgPressureValue = np.mean(params['pressure_values'][i-MV_AVG_DEPTH:i])                
                sense_current_def = 0

                targetFlow = convert_kgs2lmin(traj.getFlow(params['t']))
                desiredMass = traj.getMass(params['t'])
                print("Desired Mass during deflation: ", desiredMass)
                
                # print("Current Mass during deflation: ", params['CurrentMass'])
                pressure_diff = params['supply_pressure'] - params['p_after1']/100           
                
               
                params['dac'].DAC8532_Out_Voltage(params['select_channel_def'], params['set_voltage_def']) # 3 ms
                
                del_t_corrected = time.time() - time_loop_start
                params['CurrentMass'] = params['CurrentMass'] -  del_t_corrected * convert_lmin2kgs(avgFlowValue)
               
                params['Current_GT_mass'] = computeMassDelta(avgPressureValue,p_before=0.0, r=287.058, t=293.15)                    
                params['t']= time.time() - params['t_start'] - params['time_spent'] 
                params['time_val'] = params['t']
                params['time_log'] = time.time() - params['t_absolute_start'] - params['time_spent']
                log_data(log,params['time_log'],-avgFlowValue,targetFlow,
                    avgPressureValue,params['CurrentMass'], params['Current_GT_mass'], 
                    desiredMass, params['set_voltage_inf'], sense_current_def, params['set_voltage_def'])
        
            timeOffset = time.time()-time_loop_start
                
            if params['del_t'] - timeOffset <= 0:
                time.sleep(0)
            else:
                time.sleep(params['del_t'] - timeOffset)
            i+=1

    return params