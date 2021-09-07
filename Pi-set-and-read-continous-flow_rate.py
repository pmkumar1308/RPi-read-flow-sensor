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

file_path = '/home/pi/RPi-read-flow-sensor/Voltage_readings_Festo_continous_flow_rate.csv'

if os.path.exists(file_path):
    os.remove(file_path)

GPIO.setmode(GPIO.BCM)   # use the Broadcom pin numbering
GPIO.setwarnings(False)  # disable warnings


MAX_SET_VAL = 3.3 # conversion factor for setting voltage based on amplification of OP-Amp
OFFSET = 0 # if there is a offset to be included in the OP-Amp output voltage
TIME_DELAY_RESPONSE = 0.002 # for giving the chips time to respond
VOLUME_INSIDE  = 0.1


def convert_volt2dacVal(volt_val):
    """
    Returns a converted value of the voltage input between a number between 0 to 4096
    """
    return int(((volt_val-OFFSET)/MAX_SET_VAL) * 4096)

def read_store_volt(time,actual_channel,set_point_channel):
    """
    Reads and stores voltage readings from ADC in the file open in .csv format
    """
    voltage = actual_channel.voltage * 3.2
    set_pt_voltage = set_point_channel.voltage * 3.2
    log.write("{0},{1},{2}\n".format(str(time),str(voltage),str(set_pt_voltage)))

def run_valve_flow(start_time, ch_adc_act,ch_adc_set,ch_dac,flow_rate, max_volt=1):
    valve_start_time = time.time()
    Valve_opening_time = VOLUME_INSIDE/(flow_rate/60)
    print("run")
    while True and (time.time()-valve_start_time)<Valve_opening_time: 
        set_voltage = (9.8 * (flow_rate + 4/9.8))/20 
        # print(set_voltage)                       
        dac.setVoltage(ch_dac, convert_volt2dacVal(set_voltage)) # set to 0 V
        read_store_volt(time.time()-start_time,ch_adc_act, ch_adc_set)
        time.sleep(TIME_DELAY_RESPONSE)
        # print(ch_adc_set.voltage *3.2)
         
    time.sleep(TIME_DELAY_RESPONSE)
    dac.setVoltage(ch_dac, 0)
    
        

if __name__ == '__main__':

    # For DAC MCP4922
    dac = MCP4922(spibus=1,spidevice=2) # chooses the CE2 of SPI_1 bus for chip select
    
    # For ADC MCP3008
    spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI) # uses SPI_0 bus
    cs = digitalio.DigitalInOut(board.CE1) # chooses CE1 of SPI_0 bus for chip select
    adc = MCP.MCP3008(spi, cs) 
    
    # DAC channels to set voltage
    channel_dac_inf = 0 # inflation
    channel_dac_def = 1 # deflation
    

    # ADC Channels to read values
    channel_actual_inf = AnalogIn(adc, MCP.P1) # channel for reading actual value of inflation v/v
    channel_set_pt_inf = AnalogIn(adc, MCP.P2) # channel for reading set point value of inflation v/v

    channel_actual_def = AnalogIn(adc, MCP.P3) # channel for reading actual value of defflation v/v
    channel_set_pt_def = AnalogIn(adc, MCP.P4) # channel for reading set point value of defflation v/v

    set_volt = 0 # initial set voltage
    flow_rate_inf = float(input("Please input the inflation flow rate (in l/min): "))
    flow_rate_def = float(input("Please input the deflation flow rate (in l/min): "))
    # max_voltage = float(input("Please input the max voltage to be set: "))

    # flow_rate_inf = 2
    # flow_rate_def = 2
    max_voltage = 1
    try:
        with open(file_path, "a") as log:
            start = time.time() 

            # inflation
            run_valve_flow(start,channel_actual_inf,channel_set_pt_inf,channel_dac_inf,flow_rate_inf,max_volt = max_voltage)
            
            # deflation
            run_valve_flow(start,channel_actual_def,channel_set_pt_def,channel_dac_def,flow_rate_def, max_volt = max_voltage)

            dac.setVoltage(0, 0)
            print("End", time.time()-start)
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