import lib
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

param_dict = {       

        # For reading ADC values through SPI bus
        'adc':ADS1256.ADS1256(),

        # Channel number for reading pressure and voltage at sense resistor on ADC
        'pressure_channel' : 0,

		# For DAC reading through SPI
		'select_channel_inf' : DAC8532.channel_A,
		'select_channel_def' : DAC8532.channel_B,
		'dac' : DAC8532.DAC8532(),
        
        }
param_dict['adc'].ADS1256_init()

pressure = lib.convert_volt2pressure(param_dict['adc'].ADS1256_GetChannalValue(param_dict['pressure_channel']) * 5.0/0x7fffff)

print("The pressure is: ",pressure)