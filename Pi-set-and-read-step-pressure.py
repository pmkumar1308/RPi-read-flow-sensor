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

if os.path.exists('/home/pi/RPi-read-flow-sensor/Voltage_readings_Festo_step_pressure.csv'):
    os.remove('/home/pi/RPi-read-flow-sensor/Voltage_readings_Festo_step_pressure.csv')

GPIO.setmode(GPIO.BCM)   # use the Broadcom pin numbering
GPIO.setwarnings(False)  # disable warnings


CON_VAL = 3.3 # conversion factor for setting voltage based on amplification of OP-Amp
OFFSET = 0 # if there is a offset to be included in the OP-Amp output voltage

time_delay=0.001 # time delay for sampling in seconds
select_channel_dac = 0 

def convert_volt2dacVal(volt_val):
    """
    Returns a converted value of the voltage input between a number between 0 to 4096
    """
    return int(((volt_val-OFFSET)/CON_VAL) * 4096)

def read_store_volt(time,actual_channel,set_point_channel):
    """
    Reads and stores voltage readings from ADC in the file open in .csv format
    """
    voltage = actual_channel.voltage * 3.2
    set_pt_voltage = set_point_channel.voltage * 3.2
    log.write("{0},{1},{2}\n".format(str(time),str(voltage),str(set_pt_voltage)))


if __name__ == '__main__':
    
    # For DAC MCP4922
    dac = MCP4922(spibus=1,spidevice=2) # chooses the CE2 of SPI_1 bus for chip select
    
    # For ADC MCP3008
    spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI) # uses SPI_0 bus
    cs = digitalio.DigitalInOut(board.CE1) # chooses CE1 of SPI_0 bus for chip select
    adc = MCP.MCP3008(spi, cs)
    channel_actual = AnalogIn(adc, MCP.P1) # channel for reading actual value
    channel_set_pt = AnalogIn(adc, MCP.P2) # channel for reading set point value

    i=0
    set_volt = 0
    end_time = 30
    end_step = 5
    step=1
    tolerance = 0.01
    step_voltage = 0.3
    try:
        with open("/home/pi/RPi-read-flow-sensor/Voltage_readings_Festo_step_pressure.csv", "a") as log:
            start = time.time()
            while True and  time.time()-start < end_time and set_volt<3:                
                dac.setVoltage(select_channel_dac, convert_volt2dacVal(set_volt) ) # set to 0 V
                read_store_volt(time.time()-start,channel_actual, channel_set_pt)

                if  (time.time()-start) % (step * end_step) < tolerance and step < 4:
                    end = time.time()
                    print(f"The time elapsed for this step: {end - start} s")
                    set_volt = set_volt + step_voltage                    
                    step=step + 1
                    print(f"Set volt = {set_volt} V at step {step}")

                if  (time.time()-start) % (step * end_step) < tolerance and step >= 4:
                    end = time.time()
                    print(f"The time elapsed for this step: {end - start} s")
                    set_volt = set_volt - step_voltage
                    step= step + 1
                    print(f"Set volt = {set_volt} V at step {step}")

                # if step >= 4:
                #     set_volt = 0

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



 




