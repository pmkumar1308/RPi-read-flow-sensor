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


spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.CE1)
mcp = MCP.MCP3008(spi, cs)
channel = AnalogIn(mcp, MCP.P1)

GPIO.setmode(GPIO.BCM)   # use the Broadcom pin numbering
GPIO.setwarnings(False)             # disable warnings
i=0
time_delay=0.001

if __name__ == '__main__':
    dac = MCP4922(spibus=1,spidevice=2) # by default chooses the CE0 for chip select
    print("Regular setVoltage() Function")
    select_value = int(input("Select Value: "))
    select_channel = int(input("Select Channel, 0 or 1: "))

    try:
        with open("/home/pi/RPi-read-flow-sensor/Voltage_readings_Festo_step.csv", "a") as log:
            while True and i<100:
                # Setting the voltage set point value
                dac.setVoltage(select_channel, select_value)

                print('ADC Voltage: ' + str(channel.voltage * 3.2) + ' V')            
                voltage = channel.voltage * 3.2
                log.write("{0},{1}\n".format(str(i),str(voltage)))
                time.sleep(time_delay)
                i = i+time_delay
            
    except KeyboardInterrupt:   # Press CTRL C to exit program
        dac.setVoltage(0, 0)
        dac.setVoltage(1, 0)
        dac.shutdown(0)
        dac.shutdown(1)
        GPIO.cleanup()
        sys.exit(0)



 




