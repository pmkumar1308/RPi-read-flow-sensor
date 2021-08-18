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

if os.path.exists('/home/pi/RPi-read-flow-sensor/Voltage_readings_Festo_step.csv'):
    os.remove('/home/pi/RPi-read-flow-sensor/Voltage_readings_Festo_step.csv')

spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI) # uses SPI_0 bus
cs = digitalio.DigitalInOut(board.CE1) # chooses CE1 of SPI_0 bus
mcp = MCP.MCP3008(spi, cs)
channel = AnalogIn(mcp, MCP.P1)

GPIO.setmode(GPIO.BCM)   # use the Broadcom pin numbering
GPIO.setwarnings(False)             # disable warnings
i=0
time_delay=0.001
CON_VAL = 3.3
OFFSET = 0
select_channel_dac = 0

def convert_bin2volt(volt_val):
    return int(((volt_val-OFFSET)/CON_VAL) * 4096)

if __name__ == '__main__':
    dac = MCP4922(spibus=1,spidevice=2) # by default chooses the CE2 of SPI_1 bus for chip select
    # print("Regular setVoltage() Function")
    # select_value_voltage = int(input("Select Value in volts: "))
    # select_value = convert_bin2volt(select_value_voltage)
    # print("Select value= %d" %select_value)
    # time.sleep(5)
    # select_channel = int(input("Select Channel, 0 or 1: "))
    

    try:
        with open("/home/pi/RPi-read-flow-sensor/Voltage_readings_Festo_step.csv", "a") as log:
            while True and i<5:
                dac.setVoltage(select_channel_dac, convert_bin2volt(0) ) # set to 0 V
                voltage = channel.voltage * 3.2
                log.write("{0},{1}\n".format(str(i),str(voltage)))
                i = i+time_delay

            while True and i<10:
                # Setting the voltage set point value
                dac.setVoltage(select_channel_dac, convert_bin2volt(1) ) # set to 1 V

                # Reading the voltage 
                print('ADC Voltage: ' + str(channel.voltage * 3.2) + ' V')            
                voltage = channel.voltage * 3.2
                log.write("{0},{1}\n".format(str(i),str(voltage)))
                time.sleep(time_delay)
                i = i+time_delay

               
            while True and i<15:
                # Setting the voltage set point value
                dac.setVoltage(select_channel_dac, convert_bin2volt(3) ) # set to 3 V  

                # Reading the voltage 
                print('ADC Voltage: ' + str(channel.voltage * 3.2) + ' V')            
                voltage = channel.voltage * 3.2
                log.write("{0},{1}\n".format(str(i),str(voltage)))
                time.sleep(time_delay)
                i = i+time_delay

            dac.setVoltage(0, 0)
            dac.setVoltage(1, 0)
            dac.shutdown(0)
            dac.shutdown(1)
            GPIO.cleanup()
            sys.exit(0)
            
    except KeyboardInterrupt:   # Press CTRL C to exit program
        dac.setVoltage(0, 0)
        dac.setVoltage(1, 0)
        dac.shutdown(0)
        dac.shutdown(1)
        GPIO.cleanup()
        sys.exit(0)



 




