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
channel = AnalogIn(mcp, MCP.P0)

GPIO.setmode(GPIO.BCM)  # use the Broadcom pin numbering
GPIO.setwarnings(False)  # disable warnings

if __name__ == '__main__':
    dac = MCP4922()  # by default chooses the CE0 for chip select 1
    print("Regular setVoltage() Function")
    select_value = int(input("Select Value: ")) # sets the voltage value
    select_channel = int(input("Select Channel, 0 or 1: "))

    try:
        while True:
            # Setting the voltage set point value
            dac.setVoltage(select_channel, select_value)

            # Read the Actual value from the sensor
            print('Raw ADC Value: ', channel.value)
            print('ADC Voltage: ' + str(channel.voltage) + 'V')
            time.sleep(0.5)

    except KeyboardInterrupt:  # Press CTRL C to exit program
        dac.setVoltage(0, 0)
        dac.setVoltage(1, 0)
        dac.shutdown(0)
        dac.shutdown(1)
        GPIO.cleanup()
        sys.exit(0)



 




