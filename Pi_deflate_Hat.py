import os
import time
import sys
import RPi.GPIO as GPIO
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import ADS1256
import DAC8532 


GPIO.setmode(GPIO.BCM)   # use the Broadcom pin numbering
GPIO.setwarnings(False)             # disable warnings


if __name__ == '__main__':
    adc = ADS1256.ADS1256() 
    adc.ADS1256_init()
    dac = DAC8532.DAC8532() # by default chooses the CE0 for chip select
    dac.DAC8532_Out_Voltage(DAC8532.channel_A, 0)
    dac.DAC8532_Out_Voltage(DAC8532.channel_B, 0)
    try:   
        while True: 
            select_value = 0
            dac.DAC8532_Out_Voltage(DAC8532.channel_A, select_value)
            dac.DAC8532_Out_Voltage(DAC8532.channel_B, 5)

    except KeyboardInterrupt:   # Press CTRL C to exit program
        dac.DAC8532_Out_Voltage(DAC8532.channel_A, 0)
        dac.DAC8532_Out_Voltage(DAC8532.channel_B, 0)
        # dac.shutdown(0)
        # dac.shutdown(1)
        GPIO.cleanup()
        sys.exit(0)
