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
import lib


file_path = '/home/pi/Offset_voltage_pressure_data_inflation.csv'

if os.path.exists(file_path):
    os.remove(file_path)

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
# TIME_DURATION = 2 + CONTROL_OFFSET
CURRENT_DRIVER_SENSE_RES_VALUE = 2.08
NUM_CYCLES = 5
MAX_PRESSURE = 25 # kPa
PRESSURE_TOLERANCE = 2 #kPa
SUPPLY_PRESSURE = 310 #kPa
VOLTAGE_STEP = 0.01
FLOW_THRESHOLD = 0.1

def log_data_exp(log,pressure_val, actual_pressure_step, 
             voltage_value, supply_pressure):
    """
    Reads and stores data in the file open in .csv format
    """
    
    # voltage = actual_channel.voltage 
    # set_pt_voltage = set_point_channel.voltage 
    # voltage_pressure = pressure_ch.voltage 
    today = date.today()
    log.write("{0}, {1}, {2}, {3}, {4}\n".format(str(today.strftime("%d/%m/%Y")),
        str(pressure_val),str(actual_pressure_step),str (voltage_value), str(supply_pressure)))

if __name__ == '__main__':

    pressure_offset = {}

    param_dict = {

        'i':0,
        'p_after1' : 0,
        'time_val' : 0,
        'NumberOfRuns' : 100000,
        'avg_flow_value' : 4  ,      

        ###### del_t for sampling ######
        'del_t' : 0.035,# s
       
        
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
        'supply_pressure' : 3, # bar
        'set_voltage_inf' : 2.5,# V
        'set_voltage_def' : 0, # V

        #####-----Define masses-----#####
        'initialMass' : 0,
        'finalMass' : 10e-6, # mg

        #### ---- Control gains ---- ####
        'controlKP' : 1100,
        # controlKP = float(input("Proportional Gain: "))
        'controlKD' : 500,
        # controlKD = float(input("Derivative Gain: "))
        'controlKI' : 300,
        # controlKI = float(input("Integral Gain: "))
        
        # Initialising time for the computation of the trajectory
        't' : 0,

        # Initialising masses which are computed
        'CurrentMass' : 0,
        'Current_GT_mass' : 0,

        ### Select Trajectory type ###
        "trajType" : "sin"
        }

    #Initialising arrays for storing flow and pressure values for calculating movingaverage
    param_dict['flow_values']=np.empty(param_dict['NumberOfRuns']+2)
    param_dict['pressure_values'] = np.empty(param_dict['NumberOfRuns']+2)
    param_dict['t_start'] = time.time() + MV_AVG_DEPTH * param_dict['del_t']
    
    
    param_dict['adc'].ADS1256_init()
    param_dict['dac'].DAC8532_Out_Voltage(DAC8532.channel_A, 0)
    param_dict['dac'].DAC8532_Out_Voltage(DAC8532.channel_B, 0)


    try:
        with open(file_path, "a") as log:     
            cycle = 0
            initial_pressure = lib.convert_volt2pressure(param_dict['adc'].ADS1256_GetChannalValue(param_dict['pressure_channel']) * 5.0/0x7fffff)
            while True:
                if initial_pressure > 4:
                    #Checking the initial pressure to make sure the tube is empty:
                    initial_pressure = lib.convert_volt2pressure(param_dict['adc'].ADS1256_GetChannalValue(param_dict['pressure_channel']) * 5.0/0x7fffff)
                    print(".........Deflating the tube, initial pressure inside the tube: ", initial_pressure)                    
                    param_dict['dac'].DAC8532_Out_Voltage(DAC8532.channel_B, 5)
                else:
                    param_dict['dac'].DAC8532_Out_Voltage(DAC8532.channel_B, 0)
                    break  
           
            ## CYCLES START ##
            pressure = lib.convert_volt2pressure(param_dict['adc'].ADS1256_GetChannalValue(param_dict['pressure_channel']) * 5.0/0x7fffff)
            pressure_steps = np.arange(10,200,10)
            for pressure_step in pressure_steps:              
                print("Pressure step: ", pressure_step)
                param_dict['set_voltage_inf'] = 0
                print("Inflation started")
                while True: 
                ############################ INFLATION ##########################

                    if pressure < pressure_step:
                        param_dict['i']=0
                        param_dict['t']=0
                        param_dict['time_val'] = 0
                        param_dict['initialMass'] = 10e-6
                        param_dict['finalMass'] = 35e-6
                        param_dict['do_log'] = False
                        param_dict['del_t'] = 0.02
                       
                        param_dict['set_voltage_def'] = 0      
                        
                        # Control valve for inflation
                        if param_dict['set_voltage_inf'] < MAX_SET_VALUE:
                            param_dict['set_voltage_inf'] += VOLTAGE_STEP
                        print("Setting voltage: ", param_dict['set_voltage_inf']) 
                        param_dict['dac'].DAC8532_Out_Voltage(param_dict['select_channel_inf'], param_dict['set_voltage_inf'])

                        pressure = lib.convert_volt2pressure(param_dict['adc'].ADS1256_GetChannalValue(param_dict['pressure_channel']) * 5.0/0x7fffff)
                        if pressure > pressure_step:
                            param_dict['set_voltage_inf'] = 0
                            param_dict['dac'].DAC8532_Out_Voltage(DAC8532.channel_A, 0)
                            print("Pressure reached = ", pressure)
                            break
                            
                param_dict['i'] = 0 
                i =  param_dict['i']
                time.sleep(1) 
                while True:
                    
                    # print("i: ", i)
                    print("Setting voltage deflation: ", param_dict['set_voltage_def']) 
                    param_dict['do_log'] = False
                    
                    # Control valve for inflation                    
                    pressure = lib.convert_volt2pressure(param_dict['adc'].ADS1256_GetChannalValue(param_dict['pressure_channel']) * 5.0/0x7fffff)
                    flow_val = lib.ReadSensirion(param_dict['bus_def'])
                    param_dict['flow_values'][i] = flow_val

                    if i > MV_AVG_DEPTH:
                        avgFlowValue = np.mean(param_dict['flow_values'][i-MV_AVG_DEPTH:i])
                        param_dict['set_voltage_def'] += VOLTAGE_STEP
                        param_dict['dac'].DAC8532_Out_Voltage(param_dict['select_channel_def'], param_dict['set_voltage_def'])
                        print("Flow value is: ", avgFlowValue)
                        if  avgFlowValue > FLOW_THRESHOLD or param_dict['set_voltage_def'] >= MAX_SET_VALUE:
                            pressure_offset[str(pressure_step)] = param_dict['set_voltage_def']                        
                            log_data_exp(log,pressure, pressure_step,param_dict['set_voltage_def'], 0)
                            param_dict['dac'].DAC8532_Out_Voltage(DAC8532.channel_B, 0)
                            break
                    i = i + 1
                
            # Writing Data to csv
            
                
                
                
                # ############################# DEFLATION ###########################
                # print("Deflation started")
                # param_dict['i']=0
                # param_dict['t']=0
                # param_dict['t_start'] = time.time()               
                # param_dict['initialMass'] = 10e-6
                # param_dict['finalMass'] = 35e-6
                # param_dict['cycle'] = cycle

                # param_dict['set_voltage_inf'] = 0
                # param_dict['set_voltage_def'] = 3.75

                # pressure_inside_valve = lib.convert_volt2pressure(param_dict['adc'].ADS1256_GetChannalValue(param_dict['pressure_channel']) * 5.0/0x7fffff)/100 # in bar
                # set_voltage_def = lib.flow_start_voltage_pressure(pressure_inside_valve)
                
                # # print("The flow start voltage for deflation: ",set_voltage_def)
                # # print("The pressure after inflation is: ", pressure_inside_valve)

                # # print("Time val: ", param_dict['time_val'])
                # # print("Remaining time for deflation: ",2 * TIME_DURATION - param_dict['time_val'])
                
                # # Control valve for deflation
                # param_dict = lib.runKelly_open_loop(param_dict, log, mode="deflation")

                # cycle += 1
                # print("Deflation complete")
                # param_dict['dac'].DAC8532_Out_Voltage(DAC8532.channel_B, 0)
                # print("Current mass after deflation: " + str(param_dict['CurrentMass'] * 1e6) + " mg")
                # print("Cycle(s) completed: ",cycle)

            ############################### END OF CYCLNG ##########################
            print("Experiment complete")
            print("Data recorded")
            # print("Time elapsed: ", param_dict['time_log'])
            param_dict['dac'].DAC8532_Out_Voltage(0x30, 0)
            param_dict['dac'].DAC8532_Out_Voltage(0x34, 0)
            GPIO.cleanup()
            sys.exit(0)

    except KeyboardInterrupt:   # Press CTRL C to exit program
        time.sleep(1)
        param_dict['dac'].DAC8532_Out_Voltage(0x30, 0)
        param_dict['dac'].DAC8532_Out_Voltage(0x34, 0)
        time.sleep(1)
        GPIO.cleanup()
        sys.exit(0)
        print("Keyboard interupt, exiting program...")
