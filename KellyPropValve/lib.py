
import time
import numpy as np
import sys

from datetime import date
import os



DEVICE_BUS_INFLATION = 1 #I2C bus used for inflation
DEVICE_BUS_DEFLATION = 3 #I2C bus used for deflation
DEVICE_ADDR = 0x01 #Default Sensirion SFM4100-Air device address
SCALE_FACTOR = 1000 #Used to convert the value recieved to slm(standard litre per minute)  
PRESSURE_ERROR = 0- 25.26156426462598#-(6.246343882527859 + 0.5100433517496086 + 10.65 + 3.52)
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
PRESSURE_OFFSET_INTERVAL = 30
PRESSURE_OFFSET_WITHIN_LOOP_INF = True
PRESSURE_OFFSET_WITHIN_LOOP_DEF = True
MASS_TOLERANCE = 0.5e-6



"""The calibrated signal read from the flow sensor is a signed
INTEGER number (two's complement number). The
INTEGER value can be converted to the physical value by
dividing it by the scale factor (mass flow = sensor output/ 
scale factor)."""



def log_data(log,time,actual_flow_inflation, actual_flow_deflation,target_flow,pressure, 
             current_mass, current_gt_mass, desired_Mass, 
             set_voltage_inflation, sense_current, set_voltage_deflation,
             set_baseline_voltage_inf,set_baseline_voltage_def):
    """
    Reads and stores data in the file open in .csv format
    """
    today = date.today()
    log.write("{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10},{11},{12},{13}\n".format(str(today.strftime("%d/%m/%Y")),
        str(time),str(actual_flow_inflation),str(actual_flow_deflation), str (target_flow),
        str(pressure),str(current_mass),str(current_gt_mass), 
        str(desired_Mass), str(set_voltage_inflation), str(sense_current), str(set_voltage_deflation), str(set_baseline_voltage_inf),str(set_baseline_voltage_def)))

def convertVolt2DacVal(volt_val):
    """
    Returns a converted value of the voltage input between a number between 0 to 4096 to be given to
    to the DAC for set point value
    """
    return int(((volt_val-OFFSET)/MAX_SET_VALUE) * 4096)

def flow_start_voltage_pressure (pressure_difference, mode):
    if mode == 'inflation':
        a,b = [-0.00865027 , 5.5377518]
    if mode == 'deflation':
        a,b = [-0.00906008 , 4.65781621]
    return a * pressure_difference + b

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

    def __init__(self, previous_t, params_dictionary):
        
        self.KP_inf = params_dictionary['KP_inf']
        self.KD_inf = params_dictionary['KD_inf']
        self.KI_inf = params_dictionary['KI_inf']


        self.KP_def = params_dictionary['KP_def']
        self.KD_def = params_dictionary['KD_def']
        self.KI_def = params_dictionary['KI_def']

        self.D_inf = params_dictionary['D_inf']
        self.D_def = params_dictionary['D_def']

        self.previous_error = 0
        self.previous_time = previous_t
        self.integrative_error = 0
        self.previous_flow_error = 0


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
        
        controlVoltage =  self.error * self.KP + self.derivative_error* self.KD  + self.integrative_error * self.KI
        controlVoltage = max(min(MAX_SET_VALUE, controlVoltage), MIN_SET_VALUE)
        
        self.previous_error = self.error
        
        return controlVoltage

    def contolLawFlowError(self, controlVoltage, target_value, actual_value,target_flow,measured_flow):
        self.target = target_value
        self.actual = actual_value
        self.target_flow = target_flow
        self.measured_flow = measured_flow


        self.error = self.target - self.actual
        self.del_t = time.time() - self.previous_time
        self.previous_time = time.time()

        self.flow_error = self.target_flow - self.measured_flow

        
        # self.derivative_error = (self.error - self.previous_error)/self.del_t
        self.integrative_error += (self.error - self.previous_error) * self.del_t
        
        controlVoltage =  self.error * self.KP + self.flow_error* self.KD  + self.integrative_error * self.KI
        # print("Flow error: {} Mass error: {} Control Voltage: {}".format(self.flow_error,self.error,controlVoltage))
        controlVoltage = max(min(MAX_SET_VALUE, controlVoltage), MIN_SET_VALUE)
        # print("Flow error: {} Mass error: {} Control Voltage: {}".format(self.flow_error,self.error,controlVoltage))
        self.previous_error = self.error
        
        return controlVoltage

    def simulControl(self, controlVoltage_inflation, controlVoltage_deflation,target_value, actual_value,target_flow,measured_flow, base_voltage_inf, base_voltage_def):
        self.target = target_value
        self.actual = actual_value
        self.target_flow = target_flow
        self.measured_flow = measured_flow


        self.mass_error = self.target - self.actual
        self.del_t = time.time() - self.previous_time
        self.previous_time = time.time()
        self.flow_error = self.target_flow - self.measured_flow
        self.derivative_flow_error = (self.flow_error - self.previous_flow_error)/ self.del_t

        
        # self.integrative_error += (self.error - self.previous_error) * self.del_t

        if self.mass_error > 0:
            # print("Inflation")
            controlVoltage_inflation = self.mass_error * self.KP_inf + self.flow_error * self.KD_inf + self.derivative_flow_error * self.D_inf
            controlVoltage_deflation = base_voltage_def


            
        if self.mass_error < 0:  
            # print("Deflation")
            controlVoltage_inflation = base_voltage_inf          
            controlVoltage_deflation = - self.mass_error * self.KP_def + self.flow_error * self.KD_def + self.derivative_flow_error * self.D_def

        self.previous_flow_error = self.flow_error
            
        return controlVoltage_inflation,controlVoltage_deflation

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

class stepFunction:
    def __init__(self,mass_initial, mass_to_reach):
        self.initial_mass = mass_initial
        self.final_mass = mass_to_reach

    def getMass(self,t):
        return self.final_mass
    
    def getFlow(self,t):
        return 0 ##  TODO

class trajectoryGenerator:
    def choose(trajectoryName,mass_initial, mass_to_reach):
        if trajectoryName == "cubic":
            print("cubic trajectory")
            return CubicInterpolation(mass_initial, mass_to_reach)
        if trajectoryName == "sin":
            print("Using sinosoidal trajectory")
            return sinosoidalTrajectory(mass_initial, mass_to_reach)
        if trajectoryName == 'step':
            print("Stepping to target mass")
            return 

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
    v_cc = 5.1
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


def runKelly(params, log):
    
    traj = trajectoryGenerator.choose(params['trajType'],params['CurrentMass'],params['finalMass'])
    print("Final Mass for deflation: ",params['finalMass'])
    i = params['i']
    
    # Setting baseline voltage for control
    params['pressure_inside'] = convert_volt2pressure(params['adc'].ADS1256_GetChannalValue(params['pressure_channel']) * 5.0/0x7fffff) # 8 ms Full scale output is 0x7fffff for the ADC
   
    params['set_voltage_inf_base'] = flow_start_voltage_pressure(params['supply_pressure']-params['pressure_inside'],'inflation')
    params['set_voltage_def_base'] = flow_start_voltage_pressure(params['pressure_inside'],'deflation')

    params['set_voltage_inf'] = params['set_voltage_inf_base'] 
    params['set_voltage_def'] = params['set_voltage_def_base']

    flowValue_inf = ReadSensirion(params['bus_inf'])
    flowValue_def = ReadSensirion(params['bus_def'])
            
    
    cf = ControlFlow(time.time(), params)

    params['t_start'] = time.time()

    while(True  & (abs(params['CurrentMass'] - params['finalMass']) >= MASS_TOLERANCE)): #(params['t'] < TIME_DURATION)) : #&  (avg_flow_value>=0.5)):
        time_loop_start = time.time()                     
        params['pressure_inside'] = convert_volt2pressure(params['adc'].ADS1256_GetChannalValue(params['pressure_channel']) * 5.0/0x7fffff) # 8 ms Full scale output is 0x7fffff for the ADC
        
        if params['pressure_inside'] > 200:
            print("Pressure exceeded, stopping....")
            break

        params['time_spent'] = 0                        
          
        pressureValue = params['pressure_inside']
        
        sense_current_def = 0

        desiredFlow = traj.getFlow(params['t'])
        desiredMass = traj.getMass(params['t'])
        print("Desired mass: ", desiredMass)       
        

        if params['CurrentMass'] < desiredMass:      
            params['set_voltage_inf'],params['set_voltage_def'] = cf.simulControl(params['set_voltage_inf'], params['set_voltage_def'],desiredMass,params['CurrentMass'],desiredFlow,convert_lmin2kgs(flowValue_inf))
            
            # Including pressure offset
            pressure_diff = params['supply_pressure'] - params['pressure_inside']
            if PRESSURE_OFFSET_WITHIN_LOOP_INF == True:
                if i%PRESSURE_OFFSET_INTERVAL == 0:  
                    print("Setting baseline voltage inflation")                      
                    params['set_voltage_inf_base'] = flow_start_voltage_pressure(pressure_diff,'inflation')
            
            params['set_voltage_inf'] += params['set_voltage_inf_base']
            params['set_voltage_inf'] = max(min(MAX_SET_VALUE, params['set_voltage_inf']), MIN_SET_VALUE)            
            
            params['dac'].DAC8532_Out_Voltage(params['select_channel_inf'], params['set_voltage_inf']) # 3 ms
            time.sleep(params['del_t'])
            params['dac'].DAC8532_Out_Voltage(params['select_channel_def'], 0)
            flowValue_inf = ReadSensirion(params['bus_inf'])
            flowValue_def = 0
            print("set_voltage_inf", params['set_voltage_inf'])
            sign = 1
            
        if params['CurrentMass'] >= desiredMass:
            params['set_voltage_inf'],params['set_voltage_def'] = cf.simulControl(params['set_voltage_inf'], params['set_voltage_def'],desiredMass,params['CurrentMass'],desiredFlow,convert_lmin2kgs(flowValue_def))

            # Including pressure offset
            pressure_diff = params['pressure_inside']
            if PRESSURE_OFFSET_WITHIN_LOOP_DEF == True:
                if i%PRESSURE_OFFSET_INTERVAL == 0:  
                    print("Setting baseline voltage deflation")                      
                    params['set_voltage_def_base'] = flow_start_voltage_pressure(pressure_diff,'deflation')
            
            params['set_voltage_def'] += params['set_voltage_def_base']
            params['set_voltage_def'] = max(min(MAX_SET_VALUE, params['set_voltage_def']), MIN_SET_VALUE)
            
            params['dac'].DAC8532_Out_Voltage(params['select_channel_inf'], 0) # 3 ms
            time.sleep(params['del_t'])
            params['dac'].DAC8532_Out_Voltage(params['select_channel_def'], params['set_voltage_def'])
            flowValue_inf = 0
            flowValue_def = ReadSensirion(params['bus_def'])
            
            sign = -1      
       
        
        del_t_corrected = time.time() - time_loop_start
        params['CurrentMass'] = params['CurrentMass'] +  del_t_corrected * convert_lmin2kgs(flowValue_inf) - del_t_corrected * convert_lmin2kgs(flowValue_def)
        
        
        params['Current_GT_mass'] = computeMassDelta(pressureValue,p_before=0.0, r=287.058, t=293.15)                    
        params['t']= time.time() - params['t_start'] - params['time_spent'] 
        
        params['time_log'] = time.time() - params['t_absolute_start'] - params['time_spent']

        if params['do_log'] == True:
            log_data(log,params['time_log'],flowValue_inf, -flowValue_def, convert_kgs2lmin(desiredFlow),
                pressureValue,params['CurrentMass'], params['Current_GT_mass'], 
                desiredMass, params['set_voltage_inf'], sense_current_def, params['set_voltage_def'],
                params['set_voltage_inf_base'], params['set_voltage_def_base'])

    
        timeOffset = time.time()-time_loop_start
        print("Time offset: ", timeOffset)
    
        i+=1

    return params

# def runKelly_v2(params,log):

#     print("Final Mass for deflation: ",params['desiredMass'])
#         i = params['i']
        
#         # Setting baseline voltage for control
#         params['pressure_inside'] = convert_volt2pressure(params['adc'].ADS1256_GetChannalValue(params['pressure_channel']) * 5.0/0x7fffff) # 8 ms Full scale output is 0x7fffff for the ADC
       
#         params['set_voltage_inf_base'] = flow_start_voltage_pressure(params['supply_pressure']-params['pressure_inside'],'inflation')
#         params['set_voltage_def_base'] = flow_start_voltage_pressure(params['pressure_inside'],'deflation')
                  
#         params['pressure_inside'] = convert_volt2pressure(params['adc'].ADS1256_GetChannalValue(params['pressure_channel']) * 5.0/0x7fffff) # 8 ms Full scale output is 0x7fffff for the ADC
        
#         # if params['pressure_inside'] > 200:
#         #     print("Pressure exceeded, stopping....")
#         #     return params

#         params['time_spent'] = 0                        
          
#         pressureValue = params['pressure_inside']
        
#         sense_current_def = 0    
        
#         cf = ControlFlow(time.time(), self.CONTROL_PARAMS)
        
#         if params['CurrentMass'] < params['desiredMass']: 

#             params['set_voltage_inf'], params['set_voltage_def'] = cf.simulControl(params['set_voltage_inf'], params['set_voltage_def'],params['desiredMass'],params['CurrentMass'],convert_lmin2kgs(self.targetFlow),convert_lmin2kgs(self.flowValue_inf))
             
#             # Including pressure offset
#             pressure_diff = params['supply_pressure'] - params['pressure_inside']
#             if PRESSURE_OFFSET_WITHIN_LOOP_INF == True:
#                 if i%PRESSURE_OFFSET_INTERVAL == 0:  
#                     print("Setting baseline voltage inflation")                      
#                     params['set_voltage_inf_base'] = flow_start_voltage_pressure(pressure_diff,'inflation')
            
#             params['set_voltage_inf'] += params['set_voltage_inf_base']
#             params['set_voltage_inf'] = max(min(MAX_SET_VALUE, params['set_voltage_inf']), MIN_SET_VALUE)            
            
#             params['dac'].DAC8532_Out_Voltage(params['select_channel_inf'], params['set_voltage_inf']) # 3 ms
#             time.sleep(params['del_t'])
#             params['dac'].DAC8532_Out_Voltage(params['select_channel_def'], 0)
#             self.flowValue_inf = ReadSensirion(params['bus_inf'])
#             self.flowValue_def = 0
#             print("set_voltage_inf", params['set_voltage_inf'])
#             sign = 1
            
#         if params['CurrentMass'] >= desiredMass:
#             params['set_voltage_inf'], params['set_voltage_def'] = cf.simulControl(params['set_voltage_inf'], params['set_voltage_def'],params['desiredMass'],params['CurrentMass'],convert_lmin2kgs(self.targetFlow),convert_lmin2kgs(self.flowValue_def))

#             # Including pressure offset
#             pressure_diff = params['pressure_inside']
#             if PRESSURE_OFFSET_WITHIN_LOOP_DEF == True:
#                 if i%PRESSURE_OFFSET_INTERVAL == 0:  
#                     print("Setting baseline voltage deflation")                      
#                     params['set_voltage_def_base'] = flow_start_voltage_pressure(pressure_diff,'deflation')
            
#             params['set_voltage_def'] += params['set_voltage_def_base']
#             params['set_voltage_def'] = max(min(MAX_SET_VALUE, params['set_voltage_def']), MIN_SET_VALUE)
            
#             params['dac'].DAC8532_Out_Voltage(params['select_channel_inf'], 0) # 3 ms
#             time.sleep(params['del_t'])
#             params['dac'].DAC8532_Out_Voltage(params['select_channel_def'], params['set_voltage_def'])
#             self.flowValue_inf = 0
#             self.flowValue_def = ReadSensirion(params['bus_def'])
            
#             sign = -1      
       
        
#         del_t_corrected = time.time() - time_loop_start
#         rospy.Subscriber("FlowSensorValues", FlowSensor, self.flow_callback)

#         params['CurrentMass'] = params['CurrentMass'] +  del_t_corrected * convert_lmin2kgs(self.flowValue_inf) - del_t_corrected * convert_lmin2kgs(self.flowValue_def)
        
        
#         params['Current_GT_mass'] = computeMassDelta(pressureValue,p_before=0.0, r=287.058, t=293.15)                    
#         params['t']= time.time() - params['t_start'] - params['time_spent'] 
#         print("Time: ", params['t'])
#         params['time_log'] = time.time() - params['t_absolute_start'] - params['time_spent']

#         if params['do_log'] == True:
#             log_data(log,params['time_log'],flowValue_inf, -flowValue_def, convert_kgs2lmin(desiredFlow),
#                 pressureValue,params['CurrentMass'], params['Current_GT_mass'], 
#                 desiredMass, params['set_voltage_inf'], sense_current_def, params['set_voltage_def'],
#                 params['set_voltage_inf_base'], params['set_voltage_def_base'])


#         timeOffset = time.time()-time_loop_start
#         print("Time offset: ", timeOffset)

#         i+=1

#         return params
# def runKelly_without_control(params, log):    
    
#     print("Final Mass for deflation: ",params['finalMass'])
#     i = params['i']
    
#     # Setting baseline voltage for control
#     params['pressure_inside'] = convert_volt2pressure(params['adc'].ADS1256_GetChannalValue(params['pressure_channel']) * 5.0/0x7fffff) # 8 ms Full scale output is 0x7fffff for the ADC
   
#     params['set_voltage_inf_base'] = flow_start_voltage_pressure(params['supply_pressure']-params['pressure_inside'],'inflation')
#     params['set_voltage_def_base'] = flow_start_voltage_pressure(params['pressure_inside'],'deflation')

#     params['set_voltage_inf'] = params['set_voltage_inf_base'] 
#     params['set_voltage_def'] = params['set_voltage_def_base']

#     flowValue_inf = ReadSensirion(params['bus_inf'])
#     flowValue_def = ReadSensirion(params['bus_def'])      
    
#     time_loop_start = params['time_loop_start']                     
#     params['pressure_inside'] = convert_volt2pressure(params['adc'].ADS1256_GetChannalValue(params['pressure_channel']) * 5.0/0x7fffff) # 8 ms Full scale output is 0x7fffff for the ADC
    
#     if params['pressure_inside'] > 200:
#         print("Pressure exceeded, stopping....")
       

#     params['time_spent'] = 0                        
      
#     pressureValue = params['pressure_inside']
    
#     sense_current_def = 0        
    

#     if params['CurrentMass'] < params['desiredMass']:              
#         # Including pressure offset
#         pressure_diff = params['supply_pressure'] - params['pressure_inside']
#         if PRESSURE_OFFSET_WITHIN_LOOP_INF == True:
#             if i%PRESSURE_OFFSET_INTERVAL == 0:  
#                 print("Setting baseline voltage inflation")                      
#                 params['set_voltage_inf_base'] = flow_start_voltage_pressure(pressure_diff,'inflation')
        
#         params['set_voltage_inf'] += params['set_voltage_inf_base']
#         params['set_voltage_inf'] = max(min(MAX_SET_VALUE, params['set_voltage_inf']), MIN_SET_VALUE)            
        
#         params['dac'].DAC8532_Out_Voltage(params['select_channel_inf'], params['set_voltage_inf']) # 3 ms
#         time.sleep(params['del_t'])
#         params['dac'].DAC8532_Out_Voltage(params['select_channel_def'], 0)
#         flowValue_inf = ReadSensirion(params['bus_inf'])
#         flowValue_def = 0
#         print("set_voltage_inf", params['set_voltage_inf'])
#         sign = 1
        
#     if params['CurrentMass'] >= desiredMass:
#         # Including pressure offset
#         pressure_diff = params['pressure_inside']
#         if PRESSURE_OFFSET_WITHIN_LOOP_DEF == True:
#             if i%PRESSURE_OFFSET_INTERVAL == 0:  
#                 print("Setting baseline voltage deflation")                      
#                 params['set_voltage_def_base'] = flow_start_voltage_pressure(pressure_diff,'deflation')
        
#         params['set_voltage_def'] += params['set_voltage_def_base']
#         params['set_voltage_def'] = max(min(MAX_SET_VALUE, params['set_voltage_def']), MIN_SET_VALUE)
        
#         params['dac'].DAC8532_Out_Voltage(params['select_channel_inf'], 0) # 3 ms
#         time.sleep(params['del_t'])
#         params['dac'].DAC8532_Out_Voltage(params['select_channel_def'], params['set_voltage_def'])
#         flowValue_inf = 0
#         flowValue_def = ReadSensirion(params['bus_def'])
        
#         sign = -1      
   
    
#     del_t_corrected = time.time() - time_loop_start
#     params['CurrentMass'] = params['CurrentMass'] +  del_t_corrected * convert_lmin2kgs(flowValue_inf) - del_t_corrected * convert_lmin2kgs(flowValue_def)
    
    
#     params['Current_GT_mass'] = computeMassDelta(pressureValue,p_before=0.0, r=287.058, t=293.15)                    
#     params['t']= time.time() - params['t_start'] - params['time_spent'] 
    
#     params['time_log'] = time.time() - params['t_absolute_start'] - params['time_spent']

#     if params['do_log'] == True:
#         log_data(log,params['time_log'],flowValue_inf, -flowValue_def, convert_kgs2lmin(desiredFlow),
#             pressureValue,params['CurrentMass'], params['Current_GT_mass'], 
#             desiredMass, params['set_voltage_inf'], sense_current_def, params['set_voltage_def'],
#             params['set_voltage_inf_base'], params['set_voltage_def_base'])


#     timeOffset = time.time()-time_loop_start
#     print("Time offset: ", timeOffset)

#     i+=1

#     return params

