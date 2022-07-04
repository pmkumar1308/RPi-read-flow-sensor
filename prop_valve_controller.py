# from __future__ import print_function

import smbus
import time
import numpy as np
import sys
import RPi.GPIO as GPIO
import busio
import digitalio
import board
from datetime import date
import os
import rospy
from prop_valves.srv import ControlVariables
from prop_valves.msg import FlowSensor, Control, DataLog
from prop_valves import ADS1256, DAC8532, lib



DEVICE_BUS_INFLATION = 1 #I2C bus used for inflation
DEVICE_BUS_DEFLATION = 3 #I2C bus used for deflation
MV_AVG_DEPTH = 10 # Depth of moving average
MAX_SET_VALUE = 5 #Maximum voltage to be set
MIN_SET_VALUE = 0 #Minimum voltage to be set
OFFSET = 0
CONTROL_OFFSET = 0
PRESSURE_OFFSET_INTERVAL = 30
PRESSURE_OFFSET_WITHIN_LOOP_INF = True
PRESSURE_OFFSET_WITHIN_LOOP_DEF = True

       

       
class RunPropValve:

    def __init__(self):
        self.flowValue_inf = 0
        self.flowValue_def = 0
        self.control_params = {}

        self.current_mass = 0
        self.control_params['KP_inf'] = rospy.get_param('KP_inf')
        self.control_params['KD_inf'] = rospy.get_param('KD_inf')
        self.control_params['KI_inf'] = 0

        self.control_params['KP_def'] = rospy.get_param('KP_def')
        self.control_params['KD_def'] = rospy.get_param('KD_def')
        self.control_params['KI_def'] = 0 

        self.targetMass = 0
        self.targetFlow = 0
        self

    def flowCallback(self, data):
        self.flowValue_inf = data.flowValueInflation
        self.flowValue_def = data.flowValueDeflation

    def controlCallback(self,data):
        self.targetMass = data.targetMass * 1e-6
        self.targetFlow = data.targetFlow
        # rospy.set_param("actuating", True)

    # def publish_data(self,pub_object):
    #     msg = DataLog()
    #     msg.time = param_dict['time_log']
    #     msg.flowValueInflation = rk.flowValue_inf
    #     msg.flowValueDeflation = rk.flowValue_def
    #     msg.targetFlow = 

        

    def runKelly(self, params):    
        time_loop_start = params['time_loop_start']
        print("Target Mass: {} mg".format(params['desiredMass']*1e6))
        print(f"Target flow: {self.targetFlow}")
        i = params['i']
        
        # Setting baseline voltage for control
        params['pressure_inside'] = lib.convert_volt2pressure(params['adc'].ADS1256_GetChannalValue(params['pressure_channel']) * 5.0/0x7fffff) # 8 ms Full scale output is 0x7fffff for the ADC
       

        params['set_voltage_inf_base'] = lib.flow_start_voltage_pressure(params['supply_pressure']-params['pressure_inside'],'inflation')
        params['set_voltage_def_base'] = lib.flow_start_voltage_pressure(params['pressure_inside'],'deflation')
                  
        # params['pressure_inside'] = lib.convert_volt2pressure(params['adc'].ADS1256_GetChannalValue(params['pressure_channel']) * 5.0/0x7fffff) # 8 ms Full scale output is 0x7fffff for the ADC
        

        params['time_spent'] = 0                        
          
        pressureValue = params['pressure_inside']
        
        sense_current_def = 0    
        
        cf = lib.ControlFlow(time.time(), self.control_params)
        
        if params['current_mass'] < params['desiredMass']: 

            params['set_voltage_inf'], params['set_voltage_def'] = cf.simulControl(params['set_voltage_inf'], params['set_voltage_def'],params['desiredMass'],
                params['current_mass'],lib.convert_lmin2kgs(self.targetFlow),
                lib.convert_lmin2kgs(self.flowValue_inf), params['set_voltage_inf_base'],params['set_voltage_def_base'])
             
            # Including pressure offset
            pressure_diff = params['supply_pressure'] - params['pressure_inside']
            if PRESSURE_OFFSET_WITHIN_LOOP_INF == True:
                if i%PRESSURE_OFFSET_INTERVAL == 0:  
                                         
                    params['set_voltage_inf_base'] = lib.flow_start_voltage_pressure(pressure_diff,'inflation')
                    params['set_voltage_def_base'] = lib.flow_start_voltage_pressure(pressure_diff,'deflation')
            
            params['set_voltage_inf'] += params['set_voltage_inf_base']
            params['set_voltage_inf'] = max(min(MAX_SET_VALUE, params['set_voltage_inf']), MIN_SET_VALUE)            
            
            params['dac'].DAC8532_Out_Voltage(params['select_channel_inf'],params['set_voltage_inf'])# params['set_voltage_inf']) # 3 ms
            
            params['dac'].DAC8532_Out_Voltage(params['select_channel_def'], 0) #params['set_voltage_def_base'])
            # print("flowValue deflation {} l/min  and {} mg".format(self.flowValue_inf, lib.convert_lmin2kgs(self.flowValue_inf) * 1e6))
            self.flowValue_inf = lib.ReadSensirion(params['bus_inf'])
            self.flowValue_def = lib.ReadSensirion(params['bus_def'])         
            
            sign = 1
            
        if params['current_mass'] >= params['desiredMass']:
            params['set_voltage_inf'], params['set_voltage_def'] = cf.simulControl(params['set_voltage_inf'], params['set_voltage_def'],params['desiredMass'],
                params['current_mass'],lib.convert_lmin2kgs(-self.targetFlow),
                lib.convert_lmin2kgs(self.flowValue_def),params['set_voltage_inf_base'],params['set_voltage_def_base'])

            # Including pressure offset
            pressure_diff = params['pressure_inside']
            if pressure_diff < 10:
                params['current_mass'] = 0
                params['stop'] = True
                print("Finger empty... cannot deflate further")
                return params
            if PRESSURE_OFFSET_WITHIN_LOOP_DEF == True:
                if i%PRESSURE_OFFSET_INTERVAL == 0:  
                    # print("Setting baseline voltage deflation") 
                    params['set_voltage_inf_base'] = lib.flow_start_voltage_pressure(pressure_diff,'inflation')                     
                    params['set_voltage_def_base'] = lib.flow_start_voltage_pressure(pressure_diff,'deflation')
            
            params['set_voltage_def'] += params['set_voltage_def_base']
            params['set_voltage_def'] = max(min(MAX_SET_VALUE, params['set_voltage_def']), MIN_SET_VALUE)
            
            params['dac'].DAC8532_Out_Voltage(params['select_channel_inf'],0) # params['set_voltage_inf_base']) # 3 ms
           
            params['dac'].DAC8532_Out_Voltage(params['select_channel_def'], params['set_voltage_def'])
            # print("flowValue deflation {} l/min  and {} mg".format(self.flowValue_def, lib.convert_lmin2kgs(self.flowValue_def) * 1e6))
            self.flowValue_inf = lib.ReadSensirion(params['bus_inf'])
            self.flowValue_def = lib.ReadSensirion(params['bus_def'])
            
            sign = -1      
       
        
        del_t_corrected = time.time() - time_loop_start
        # rospy.Subscriber("FlowSensorValues", FlowSensor, self.flowCallback)
       
        # print("del_t_corrected: ", del_t_corrected)

        params['current_mass'] = params['current_mass'] +  del_t_corrected * lib.convert_lmin2kgs(self.flowValue_inf) - del_t_corrected * lib.convert_lmin2kgs(self.flowValue_def)
        
        # print("Current mass inside the loop: ", params['current_mass'])
        params['Current_GT_mass'] = lib.computeMassDelta(pressureValue,p_before=0.0, r=287.058, t=293.15)                    
        params['t']= time.time() - params['t_start'] - params['time_spent'] 
        # print("Time: ", params['t']) 

        timeOffset = time.time()-time_loop_start
        # print("Time offset: ", timeOffset)

        i+=1

        return params

 

if __name__ == '__main__': 

    rk = RunPropValve()
    param_dict = {

            'i':0,
            'p_after1' : 0,
            'time_val' : 0,
            'NumberOfRuns' : 100000,
            'avg_flow_value' : 4  ,      

            ###### del_t for sampling ######
            'del_t' : 0.02,# s
           
            
            # For Sensirion I2C bus
            'bus_inf' : smbus.SMBus(DEVICE_BUS_INFLATION),
            'bus_def' : smbus.SMBus(DEVICE_BUS_DEFLATION),

            # For reading ADC values through SPI bus
            'adc':ADS1256.ADS1256(),

            # Channel number for reading pressure and voltage at sense resistor on ADC
            'pressure_channel' : 0,
            'sense_resistor_channel_inf' : 1,
            'sense_resistor_channel_def' : 2,                                                                                                               

            # For DAC reading through SPI
            'select_channel_inf' : DAC8532.channel_A,
            'select_channel_def' : DAC8532.channel_B,
            'dac' : DAC8532.DAC8532(),
            
            
            # start time of recording data
            't_absolute_start' : time.time(),
            
            'time_spent' : 0,

            # Setting baseline control voltage
            'supply_pressure' : 310, # kPa
            'set_voltage_inf' : 2.5,# V
            'set_voltage_def' : 0, # V

            #####-----Define masses-----#####
            'initialMass' : 0,
            'finalMass' : 20e-6, # mg

            #### ---- Control gains ---- ####
            'KP_inf' : 700000,
            # controlKP = float(input("Proportional Gain: "))
            'KD_inf' : 3500,
            # controlKD = float(input("Derivative Gain: "))
            'KI_inf' : 0,

            'KP_def' : 700000,
            # controlKP = float(input("Proportional Gain: "))
            'KD_def' : 3500,
            # controlKD = float(input("Derivative Gain: "))
            'KI_def' : 0,
            # controlKI = float(input("Integral Gain: "))
            
            # Initialising time for the computation of the trajectory
            't' : 0,

            # Initialising masses which are computed
            'Current_GT_mass' : 0,
            'current_mass': rk.current_mass,

            ### Select Trajectory type ###
            "trajType" : "cubic",

            "do_log" : True,

            "stop": False
        }

    #Initialising arrays for storing flow and pressure values for calculating movingaverage
    # param_dict['flow_values']=np.empty(param_dict['NumberOfRuns']+2)
    # param_dict['pressure_values'] = np.empty(param_dict['NumberOfRuns']+2)
    param_dict['t_start'] = time.time() 


    param_dict['adc'].ADS1256_init()
    param_dict['dac'].DAC8532_Out_Voltage(DAC8532.channel_A, 0)
    param_dict['dac'].DAC8532_Out_Voltage(DAC8532.channel_B, 0)

    rospy.init_node("prop_valve_controller")      
   
    pressure_inside = lib.convert_volt2pressure(param_dict['adc'].ADS1256_GetChannalValue(param_dict['pressure_channel']) * 5.0/0x7fffff)

    rospy.Subscriber("ControlTargets",Control, rk.controlCallback)
    if pressure_inside < 10:
        print("pressure_inside less than 10...")
        rospy.set_param("current_mass",0)
    pub = rospy.Publisher('DataLogValues', DataLog, queue_size=100)
    rate = rospy.Rate(100)
    try:
        while not rospy.is_shutdown():
            MASS_TOLERANCE = rospy.get_param("mass_tol")              
                

            param_dict['current_mass'] = rospy.get_param("current_mass")
            print("Current mass from get_param: ", rospy.get_param("current_mass"))
            
            
            param_dict['t_start'] = time.time()
            
            print(f"targetMass:  {rk.targetMass} targetFlow: {rk.targetFlow}")
            
            param_dict['stop'] = False
            # rate_logging = rospy.Rate(100)

            rk.control_params['KP_inf'] = rospy.get_param('KP_inf')
            rk.control_params['KD_inf'] = rospy.get_param('KD_inf')

            rk.control_params['KP_def'] = rospy.get_param('KP_def')
            rk.control_params['KD_def'] = rospy.get_param('KD_def')

            while (abs(param_dict['current_mass'] - rk.targetMass) >= MASS_TOLERANCE) and (param_dict['stop'] == False):
                # print("Target changed...actuating")
                
                param_dict['time_loop_start'] = time.time()  
                param_dict['desiredMass'] = rk.targetMass
                param_dict = rk.runKelly(param_dict)                                    

                param_dict['time_log'] = time.time() - param_dict['t_absolute_start'] 

                if param_dict['do_log'] == True:
                    pub.publish(param_dict['time_log'], rk.flowValue_inf,-rk.flowValue_def,
                        rk.targetFlow, param_dict['pressure_inside'],param_dict['current_mass'] ,param_dict['Current_GT_mass'], rk.targetMass,
                        param_dict['set_voltage_inf'], param_dict['set_voltage_def'],param_dict['set_voltage_inf_base'],
                        param_dict['set_voltage_def_base'])
                # rate_logging.sleep()

                print("Current mass after request complete: " + str(param_dict['current_mass'] * 1e6) + " mg")
                rospy.set_param("current_mass", param_dict['current_mass'])
            param_dict['dac'].DAC8532_Out_Voltage(0x30, 0)
            param_dict['dac'].DAC8532_Out_Voltage(0x34, 0)
           
            rate.sleep()




    except rospy.ROSInterruptException:   # Press CTRL C to exit program
        time.sleep(1)
        param_dict['dac'].DAC8532_Out_Voltage(0x30, 0)
        param_dict['dac'].DAC8532_Out_Voltage(0x34, 0)
        time.sleep(1)
        GPIO.cleanup()
        sys.exit(0)
        pass
        print("Keyboard interupt, exiting program...")    

    

    