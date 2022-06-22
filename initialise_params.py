import rospy
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
from prop_valves.msg import FlowSensor
from prop_valves import lib, ADS1256, DAC8532, lib

rospy.set_param('current_mass', 0)

rospy.set_param('actuating', False)

rospy.set_param('i2c_bus_inflation',1)
rospy.set_param('i2c_bus_deflation',3)

rospy.set_param('spi_adc_pressure_channel',0)

rospy.set_param('dac_channel_inf',DAC8532.channel_A)
rospy.set_param('dac_channel_def',DAC8532.channel_B)


