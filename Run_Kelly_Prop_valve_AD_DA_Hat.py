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
PRESSURE_ERROR = -(6.246343882527859 + 0.5100433517496086)
PRESSURE_VOLT_DIV_FACT = 1.46 # Voltage divider factor for pressure sensor
MV_AVG_DEPTH = 10 # Depth of moving average
AIR_MASS_PER_LITRE = 1.292e-3 # in kg at STP
TARGET_MASS = 80e-6 # in kg
MAX_SET_VALUE = 5 #Maximum voltage to be set
MIN_SET_VALUE = 0 #Minimum voltage to be set
OFFSET = 0
CONTROL_OFFSET = 0
TIME_DURATION = 4 + CONTROL_OFFSET

"""The calibrated signal read from the flow sensor is a signed
INTEGER number (two's complement number). The
INTEGER value can be converted to the physical value by
dividing it by the scale factor (mass flow = sensor output/ 
scale factor)."""

file_path = '/home/pi/Inflating_and_deflating_finger_data.csv'

if os.path.exists(file_path):
    os.remove(file_path)

def convertVolt2DacVal(volt_val):
    """
    Returns a converted value of the voltage input between a number between 0 to 4096 to be given to
    to the DAC for set point value
    """
    return int(((volt_val-OFFSET)/MAX_SET_VALUE) * 4096)

class CubicInterpolation:

    def __init__(self, Mass_initial, Mass_to_reach):
        self.completion_time = TIME_DURATION
        self.a_0 = Mass_initial
        self.a_1 = 0
        self.a_2 = (3/(self.completion_time**2))*(Mass_to_reach - Mass_initial)
        self.a_3 = - (2/(self.completion_time**3))*(Mass_to_reach - Mass_initial)
        

    def getMass(self,t):
        return self.a_0 + self.a_1 * t + self.a_2 * t**2 + self.a_3 * t**3

    def getFlow(self,t):
        return self.a_1 + 2 * self.a_2 * t + 3 * self.a_3 * t**2

class ControlFlow:

    def __init__(self, target_value, measured_value, proportional_gain, derivative_gain, integral_gain):
        self.target = target_value
        self.measured = measured_value
        self.KP = proportional_gain
        self.KD = derivative_gain
        self.KI = integral_gain
        self.prev_error = 0
        self.sum_error = 0

    def contolLaw(self, controlVoltage):
        # print(self.prev_error)
        error = self.target - self.measured
        # print(error)
        controlVoltage = controlVoltage + error * self.KP + self.prev_error * self.KD + self.sum_error * self.KI
        controlVoltage = max(min(MAX_SET_VALUE, controlVoltage), MIN_SET_VALUE)
        self.prev_error = error
        self.sum_error = self.sum_error + error
        # print(self.sum_error)
        return controlVoltage

def interpol_fn(Mass_initial, Mass_to_reach, time_duration,num_time_steps, func_type = "Cubic"):
    """
    The function creates a cubic interpolation between two mass states
    """
    current_time = 0
    t = np.linspace(current_time,current_time + time_duration,num_time_steps)
    h = 0.1
    if func_type == "Cubic":    
        a_0 = Mass_initial
        a_1 = 0
        a_2 = (3/(time_duration**2))*(Mass_to_reach - Mass_initial)
        a_3 = - (2/(time_duration**3))*(Mass_to_reach - Mass_initial)
        Mass_t = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3
        Mass_dot_t = np.diff(Mass_t)/h ## Function?
    return Mass_t, Mass_dot_t, t

def log_data(time,actual_flow,target_flow,pressure, current_mass, current_gt_mass, desired_Mass, set_voltage):
    """
    Reads and stores data in the file open in .csv format
    """
    
    # voltage = actual_channel.voltage 
    # set_pt_voltage = set_point_channel.voltage 
    # voltage_pressure = pressure_ch.voltage 
    today = date.today()
    log.write("{0},{1},{2},{3},{4},{5},{6},{7}, {8}\n".format(str(today.strftime("%d/%m/%Y")),str(time),str (actual_flow),str (target_flow),str(pressure),str(current_mass),str(current_gt_mass), str(desired_Mass), str(set_voltage)))

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
    read_value = device_bus.read_i2c_block_data(DEVICE_ADDR,0xF1,3) # measurement triggering command 0xF1 or hF1
    # checkSum = read_value[2]
    # checkSumError = checkCrc(read_value,checkSum)
    read_value = read_value[0]<<8 | read_value[1]

    flow_value = twosComp(read_value,16) #getting the twos complement
    flow_value = flow_value/SCALE_FACTOR

    return flow_value




if __name__ == '__main__':

    i=0
    p_after1 = 0
    time_val = 0
    NumberOfRuns = 100000
    avg_flow_value = 4
    
    ###### del_t ######
    del_t = 0.005

    #Storing flow and pressure values for calculating movingaverage
    flow_values = np.empty(NumberOfRuns+2)
    pressure_values = np.empty(NumberOfRuns+2)
    
    # For Sensirion
    bus_inf = smbus.SMBus(DEVICE_BUS_INFLATION)
    bus_def = smbus.SMBus(DEVICE_BUS_DEFLATION) 

    #For reading ADC values
    # spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI) # uses SPI_0 bus
    # cs = digitalio.DigitalInOut(board.CE1) # chooses CE1 of SPI_0 bus for chip select
    adc = ADS1256.ADS1256() 
    adc.ADS1256_init()


    #Channel number for reading pressure on ADC
    pressure_channel = 0                                                                                                                

    # For DAC
    # dac = MCP4922(spibus=1,spidevice=2) # by default chooses the CE0 for chip select
    select_channel_inf = DAC8532.channel_A
    select_channel_def = DAC8532.channel_B
    dac = DAC8532.DAC8532()
    select_value = 0
    dac.DAC8532_Out_Voltage(DAC8532.channel_A, select_value)
    dac.DAC8532_Out_Voltage(DAC8532.channel_B, select_value)
    
    # start time of recording data
    t_start = time.time() + MV_AVG_DEPTH * del_t
    time_spent =0

    set_voltage_inf = 2.3
    set_voltage_def = 2.3


    #### ---- Control gains ---- ####
    controlKP = 0.060
    # controlKP = float(input("Proportional Gain: "))
    controlKD = 0.000
    # controlKD = float(input("Derivative Gain: "))
    controlKI = 0.00
    # controlKI = float(input("Integral Gain: "))
    t = 0 
    try:
        with open(file_path, "a") as log: 
            Current_mass = 0
            Current_GT_mass = 0
            cubicSpline = CubicInterpolation(0, TARGET_MASS)
            # Inflation Loop
            while(True  & (time_val <= TIME_DURATION) & (p_after1 < 250.0) & (Current_mass <= TARGET_MASS)): #&  (avg_flow_value>=0.5)):
                time_loop_start = time.time()
            	
                flow_val = ReadSensirion(bus_inf)               
                p_after1 = convert_volt2pressure(adc.ADS1256_GetChannalValue(pressure_channel) * 5.0/0x7fffff) # Full scale output is 0x7fffff for the ADC

                #Moving Average of the measure flow and pressure values
                flow_values[i] = flow_val
                pressure_values[i] = p_after1

                if i < MV_AVG_DEPTH:
                    time_spent = time.time() - t_start
                    # print('Time spent: ', time_spent)

                if i >= MV_AVG_DEPTH:
                    avg_flow_value = np.mean(flow_values[i-MV_AVG_DEPTH:i])            	
                    avg_pressure_value = np.mean(pressure_values[i-MV_AVG_DEPTH:i]) 


                    p_after1 = convert_volt2pressure(adc.ADS1256_GetChannalValue(pressure_channel) * 5.0/0x7fffff) # Full scale output is 0x7fffff for the ADC
                    # print("Pressure in kpa: ", p_after1)                    

                    # Control                
                    # print("time: ", t)
                    
                    
                    targetFlow = convert_kgs2lmin(cubicSpline.getFlow(t))
                    desiredMass = cubicSpline.getMass(t)
                    cf = ControlFlow(targetFlow, avg_flow_value, controlKP, controlKD, controlKI)

                    set_voltage_inf = cf.contolLaw(set_voltage_inf)
                    dac.DAC8532_Out_Voltage(select_channel_inf, set_voltage_inf)
                    # print("avg flow value: (l/min) ", avg_flow_value)
                    del_t_corrected = time.time() - time_loop_start
                    Current_mass = Current_mass +  del_t_corrected * convert_lmin2kgs(avg_flow_value)


                    # Current_GT_mass = Current_GT_mass + computeMassDelta(avg_pressure_value,p_before=0.0, r=287.058, t=293.15)
                    Current_GT_mass = computeMassDelta(avg_pressure_value,p_before=0.0, r=287.058, t=293.15)
                    # print("Change in mass from ideal gas:", computeMassDelta(avg_pressure_value,p_before=0.0, r=287.058, t=293.15))
                    
                    # time_val = time.time()-t_start
                    t= time.time() - t_start - time_spent 
                    time_val = t
                    log_data(time_val,avg_flow_value,targetFlow, avg_pressure_value, Current_mass, Current_GT_mass, desiredMass, set_voltage_inf)
                time_offset = time.time()-time_loop_start
                # print("Time offset:", time_offset)
                # Give time for sampling
                if del_t - time_offset <= 0:
                    time.sleep(del_t)
                else:
                    time.sleep(del_t - time_offset)
                i=i+1
            print("Inflation complete")
            dac.DAC8532_Out_Voltage(0x30, 0)
            cubicSpline = CubicInterpolation(Current_mass,0)
            print("Current mass after inflation:" ,Current_mass)
            controlKP = 0.050
            #Deflation loop
            t_start_def = time.time()
            i = 0
            t = 0
            while(True  & (p_after1 < 250.0) & (time_val <= 2 * TIME_DURATION)): #& (time_val <= 2 * TIME_DURATION) &  (avg_flow_value>=0.5)):
                time_loop_start = time.time()
                
                flow_val = ReadSensirion(bus_def)               
                p_after1 = convert_volt2pressure(adc.ADS1256_GetChannalValue(pressure_channel) * 5.0/0x7fffff)

                #Moving Average of the measure flow and pressure values
                flow_values[i] = flow_val
                pressure_values[i] = p_after1

                if i < MV_AVG_DEPTH:
                    time_spent = time.time() - t_start_def
                    # print(time_spent)
                    # print('Time spent: ', time_spent)

                if i >= MV_AVG_DEPTH:
                    avg_flow_value = np.mean(flow_values[i-MV_AVG_DEPTH:i])             
                    avg_pressure_value = np.mean(pressure_values[i-MV_AVG_DEPTH:i]) 


                    p_after1 = convert_volt2pressure(adc.ADS1256_GetChannalValue(pressure_channel) * 5.0/0x7fffff) # Full scale output is 0x7fffff for the ADC
                    # print("Pressure in kpa: ", p_after1)                    

                    # Control                
                    # print("time: ", t)                    
                    
                    targetFlow = -convert_kgs2lmin(cubicSpline.getFlow(t))
                    print("time: ",t)
                    desiredMass = cubicSpline.getMass(t)
                    cf = ControlFlow(targetFlow, avg_flow_value, controlKP, controlKD, controlKI)

                    set_voltage_def = cf.contolLaw(set_voltage_def)
                    dac.DAC8532_Out_Voltage(select_channel_def, set_voltage_def)
                    # dac.setVoltage(select_channel_def, convertVolt2DacVal(set_voltage_def))
                    # print("avg flow value: (l/min) ", avg_flow_value)
                    del_t_corrected = time.time() - time_loop_start
                    Current_mass = Current_mass -  del_t_corrected * convert_lmin2kgs(avg_flow_value)


                    # Current_GT_mass = Current_GT_mass + computeMassDelta(avg_pressure_value,p_before=0.0, r=287.058, t=293.15)
                    Current_GT_mass = computeMassDelta(avg_pressure_value,p_before=0.0, r=287.058, t=293.15)
                    # print("Change in mass from ideal gas:", computeMassDelta(avg_pressure_value,p_before=0.0, r=287.058, t=293.15))
                    
                    # time_val = time.time()-t_start
                    t= time.time() - t_start_def - time_spent 
                    time_val = time.time() - t_start - time_spent
                    # time_val_log = time.time() - t_start - time_spent
                    log_data(time_val,-avg_flow_value,-targetFlow, avg_pressure_value, Current_mass, Current_GT_mass, desiredMass, set_voltage_def)
                time_offset = time.time()-time_loop_start

                if (Current_mass <= 0) :
                    break
                # print("Time offset:", time_offset)
                # Give time for sampling
                if del_t - time_offset <= 0:
                    time.sleep(del_t)
                else:
                    time.sleep(del_t - time_offset)
                i=i+1
            print("Deflation complete")
            print("Data recorded")
            print("Time elapsed: ", time_val)
            dac.DAC8532_Out_Voltage(0x30, 0)
            dac.DAC8532_Out_Voltage(0x34, 0)
            # dac.setVoltage(1, 0)
            # dac.setVoltage(0, 0)
            # dac.setVoltage(1, 0)
            # dac.shutdown(0)
            # dac.shutdown(1)
            GPIO.cleanup()
            sys.exit(0)

    except KeyboardInterrupt:   # Press CTRL C to exit program
        time.sleep(1)
        dac.DAC8532_Out_Voltage(0x30, 0)
        dac.DAC8532_Out_Voltage(0x34, 0)
        time.sleep(1)
        # dac.setVoltage(1, 0)
        # dac.shutdown(0)
        # dac.shutdown(1)
        GPIO.cleanup()
        sys.exit(0)
        print("Keyboard interupt, exiting program...")
