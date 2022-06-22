import rospy
from std_msgs.msg import String
from prop_valves.msg import FlowSensor
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
from prop_valves import lib, ADS1256, DAC8532

DEVICE_BUS_INFLATION = 1 #I2C bus used for inflation
DEVICE_BUS_DEFLATION = 3 #I2C bus used for deflation
DEVICE_ADDR = 0x01 #Default Sensirion SFM4100-Air device address
SCALE_FACTOR = 1000 #Used to convert the value recieved to slm(standard litre per minute)  
PRESSURE_ERROR = 0 - 25.26156426462598#-(6.246343882527859 + 0.5100433517496086 + 10.65 + 3.52)
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
MASS_TOLERANCE = 1e-6

def flow_value_publisher(params):
    pub = rospy.Publisher('FlowSensorValues', FlowSensor, queue_size=10)
    rospy.init_node('flow_value_publisher', anonymous=True)
    rate = rospy.Rate(200) # 10hz
    while not rospy.is_shutdown():
        flow_value_inf = lib.ReadSensirion(params['bus_inf'])
        flow_value_def = lib.ReadSensirion(params['bus_def'])
        msg = FlowSensor()
        msg.flowValueInflation = flow_value_inf
        msg.flowValueDeflation = flow_value_def
        print("Flow Value inflation: ", flow_value_inf)
        print("Flow Value Deflation: ", flow_value_def)
        rospy.loginfo(flow_value_inf)
        pub.publish(flow_value_inf, flow_value_def)
       
        rate.sleep()

if __name__ == '__main__':
    
    param_dict = {
        'bus_inf' : smbus.SMBus(DEVICE_BUS_INFLATION),
        'bus_def' : smbus.SMBus(DEVICE_BUS_DEFLATION),
    }


    try:
        flow_value_publisher(param_dict)
    except rospy.ROSInterruptException:
        pass