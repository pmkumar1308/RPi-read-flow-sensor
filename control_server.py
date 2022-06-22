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
from prop_valves import ADS1256, DAC8532, lib

file_path = '/home/pi/Inflating_and_deflating_finger_data.csv'

if os.path.exists(file_path):
    os.remove(file_path)

DEVICE_BUS_INFLATION = 1 #I2C bus used for inflation
DEVICE_BUS_DEFLATION = 3 #I2C bus used for deflation
DEVICE_ADDR = 0x01 #Default Sensirion SFM4100-Air device address
SCALE_FACTOR = 1000 #Used to convert the value recieved to slm(standard litre per minute)  
PRESSURE_ERROR = - 2.54 #-(6.246343882527859 + 0.5100433517496086 + 10.65)
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
NUM_CYCLES = 10




class prop_valve_controller:
    def __init__():
        self.CONTROL_PARAMS = {}

        self.CONTROL_PARAMS['KP_inf'] = 70000
        self.CONTROL_PARAMS['KD_inf'] = 3000
        self.CONTROL_PARAMS['KI_inf'] = 0

        self.CONTROL_PARAMS['KP_def'] = 70000
        self.CONTROL_PARAMS['KD_def'] = 3000
        self.CONTROL_PARAMS['KI_def'] = 0  

    def control(req):       
        global CONTROL_PARAMS
        # Increasing mass to initial mass i.e from 0 to initialMass
        cf = lib.ControlFlow(time.time(), CONTROL_PARAMS)
        setVoltageInf, setVoltageDef = cf.simulControl(req.currentVoltageInflation, req.currentVoltageDeflation, req.targetMass, req.currentMass, req.targetFlow, req.currentFlow)

        return setVoltageInf, setVoltageDef             

        
                    
    def set_control_variables_server():
        
        rospy.init_node('prop_valve_actuator')

        rospy.Service('set_control_variables', ControlVariables, control) 

        rospy.loginfo("Service server started. Ready to get requests.")

        rospy.spin()

 

if __name__ == '__main__':     

    set_control_variables_server()

    