#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import os
import time
from MCP4922 import MCP4922
import sys
import RPi.GPIO as GPIO
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn


# In[ ]:


# Setting the voltage set point value

GPIO.setmode(GPIO.BCM)                 # use the Broadcom pin numbering
GPIO.setwarnings(False)             # disable warnings


if __name__ == '__main__':
    dac = MCP4922()

    try:
        while True:
            print("Regular setVoltage() Function")
            select_value = int(raw_input("Select Value: "))
            select_channel = int(raw_input("Select Channel, 0 or 1: "))
            dac.setVoltage(select_channel, select_value)
    except KeyboardInterrupt:   # Press CTRL C to exit program
        dac.setVoltage(0, 0)
        dac.setVoltage(1, 0)
        dac.shutdown(0)
        dac.shutdown(1)
        GPIO.cleanup()
        sys.exit(0)

# Read the Actual value from the sensor
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D5)
mcp = MCP.MCP3008(spi, cs)
channel = AnalogIn(mcp, MCP.P0)

GPIO.setmode(GPIO.BCM)
GPIO.setup(17,  GPIO.OUT)

while True:
    print('Raw ADC Value: ', channel.value)
    print('ADC Voltage: ' + str(channel.voltage) + 'V')
    if channel.voltage > 2.0:
        GPIO.output(17, True)
    else:
        GPIO.output(17, False)
    time.sleep(0.5)


# In[ ]:




