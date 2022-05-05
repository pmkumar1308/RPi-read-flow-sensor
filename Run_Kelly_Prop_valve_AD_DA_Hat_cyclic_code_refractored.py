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


file_path = '/home/pi/Inflating_and_deflating_finger_data.csv'

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
NUM_CYCLES = 20

if __name__ == '__main__':

    param_dict = {

        'i':0,
        'p_after1' : 0,
        'time_val' : 0,
        'NumberOfRuns' : 100000,
        'avg_flow_value' : 4  ,      

        ###### del_t for sampling ######
        'del_t' : 0.028,# s
       
        
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
        'finalMass' : 20e-6, # mg

        #### ---- Control gains ---- ####
        'controlKP' : 1100,
        # controlKP = float(input("Proportional Gain: "))
        'controlKD' : 0.000,
        # controlKD = float(input("Derivative Gain: "))
        'controlKI' : 0.00,
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
                if initial_pressure > 2:
                    #Checking the initial pressure to make sure the tube is empty:
                    initial_pressure = lib.convert_volt2pressure(param_dict['adc'].ADS1256_GetChannalValue(param_dict['pressure_channel']) * 5.0/0x7fffff)
                    print(".........Deflating the tube, initial pressure inside the tube: ", initial_pressure)                    
                    param_dict['dac'].DAC8532_Out_Voltage(DAC8532.channel_B, 5)
                else:
                    param_dict['dac'].DAC8532_Out_Voltage(DAC8532.channel_B, 0)
                    break  

            # Increasing mass to initial mass i.e from 0 to initialMass
            param_dict['i']=0
            param_dict['t']=0
            param_dict = lib.runKelly(param_dict, log, mode="inflation")

            print("Current mass after inflation to initial mass: " + str(param_dict['CurrentMass'] * 1e6) + " mg")
           
            ## CYCLES START ##
            while cycle < NUM_CYCLES:
            # Cycle between two masses
                
                
                ############################ INFLATION ##########################
                param_dict['i']=0
                param_dict['t']=0
                param_dict['time_val'] = 0
                param_dict['initialMass'] = 20e-6
                param_dict['finalMass'] = 80e-6
                param_dict['set_voltage_inf'] = 2.85
                param_dict['set_voltage_def'] = 0
                param_dict['cycle'] = cycle

                # start time of recording data
                param_dict['t_start'] = time.time() + MV_AVG_DEPTH * param_dict['del_t']                
                param_dict['time_spent'] =0

                print("Inflation started")
                # print("Current mass: ", param_dict['CurrentMass'])
                # print("Time: ",param_dict['time_val'])
                # print("pressure", param_dict['p_after1'])
                # print("Target Mass: ", param_dict['finalMass'])
                
                # Control valve for inflation
                param_dict = lib.runKelly(param_dict, log, mode="inflation")

                print("Inflation complete")
                param_dict['dac'].DAC8532_Out_Voltage(DAC8532.channel_A, 0)
                print("Current mass after inflation: " + str(param_dict['CurrentMass'] * 1e6) + " mg")

                
                print(param_dict['i'])
                
                ############################# DEFLATION ###########################
                print("Deflation started")
                param_dict['i']=0
                param_dict['t']=0
                param_dict['t_start'] = time.time()               
                param_dict['initialMass'] = 20e-6
                param_dict['finalMass'] = 80e-6
                param_dict['cycle'] = cycle

                param_dict['set_voltage_inf'] = 0
                param_dict['set_voltage_def'] = 3.75

                pressure_inside_valve = lib.convert_volt2pressure(param_dict['adc'].ADS1256_GetChannalValue(param_dict['pressure_channel']) * 5.0/0x7fffff)/100 # in bar
                set_voltage_def = lib.flow_start_voltage_pressure(pressure_inside_valve)
                
                # print("The flow start voltage for deflation: ",set_voltage_def)
                # print("The pressure after inflation is: ", pressure_inside_valve)

                # print("Time val: ", param_dict['time_val'])
                # print("Remaining time for deflation: ",2 * TIME_DURATION - param_dict['time_val'])
                
                # Control valve for deflation
                param_dict = lib.runKelly(param_dict, log, mode="deflation")

                cycle += 1
                print("Deflation complete")
                param_dict['dac'].DAC8532_Out_Voltage(DAC8532.channel_B, 0)
                print("Current mass after deflation: " + str(param_dict['CurrentMass'] * 1e6) + " mg")
                print("Cycle(s) completed: ",cycle)

            ############################### END OF CYCLNG ##########################
            print("All cycles complete")
            print("Data recorded")
            print("Time elapsed: ", param_dict['time_log'])
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
