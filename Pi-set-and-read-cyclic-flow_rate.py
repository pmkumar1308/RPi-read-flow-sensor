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

file_path = '/home/pi/RPi-read-flow-sensor/Voltage_readings_Festo_cyclic_flow_rate1.csv'

if os.path.exists(file_path):
    os.remove(file_path)

GPIO.setmode(GPIO.BCM)   # use the Broadcom pin numbering
GPIO.setwarnings(False)  # disable warnings


MAX_SET_VAL = 3.3 # conversion factor for setting voltage based on amplification of OP-Amp
OFFSET = 0 # if there is a offset to be included in the OP-Amp output voltage
TIME_DELAY_RESPONSE = 0.002 # for giving the chips time to respond
MAX_MASS_INSIDE  = 8e-5 # in kg
AIR_MASS_PER_LITRE = 1.292e-3 # in kg at STP
SLOW_DEF_DELAY = 0.2 # slow deflation delay for reducing jerk
SLOW_FLO = 0.2 # the slow flow rate to avoid jerk in l/min
TIME_TO_FILL = (MAX_MASS_INSIDE/AIR_MASS_PER_LITRE) * 60  # the time taken to fill the finger with 80 mg of air at STP in s
NUM_CYCLES = 10

def convert_volt2dacVal(volt_val):
    """
    Returns a converted value of the voltage input between a number between 0 to 4096
    """
    return int(((volt_val-OFFSET)/MAX_SET_VAL) * 4096)

def read_store_volt(time,actual_channel,set_point_channel, pressure_ch):
    """
    Reads and stores voltage readings from ADC in the file open in .csv format
    """
    voltage = actual_channel.voltage * 3.2
    set_pt_voltage = set_point_channel.voltage * 3.2
    log.write("{0},{1},{2}\n".format(str(time),str (convert_volt2flow(voltage)),str(convert_volt2flow(set_pt_voltage), str(convert_volt2flow(pressure_ch.voltage*1.46)))))

def convert_volt2flow(volt):
    flow_val = ((volt * 20)/9.8)-(4/9.8)
    return flow_val

def convert_flow2volt(flow):
    volt_value = (9.8 * (flow + 4/9.8))/20
    return volt_value

def run_valve_flow(start_time, ch_adc_act,ch_adc_set,ch_dac,ch_pressure,flow_rate, max_volt=1, mode="Inflation"):
    set_voltage = 0
    valve_start_time = time.time()
    total_time = 0
    Valve_opening_time = TIME_TO_FILL/flow_rate # in seconds
    Current_mass = 0
    print("run")
    while set_voltage < max_volt and total_time<5: 
        loop_start_time = time.time()
        set_voltage = convert_flow2volt(flow_rate)                     
        dac.setVoltage(ch_dac, convert_volt2dacVal(set_voltage)) # set to 0 V
        
        # if mode == "Deflation":
        #     read_store_volt(time.time()-start_time,ch_adc_act, ch_adc_set,ch_pressure)
        time.sleep(TIME_DELAY_RESPONSE)
        if mode == "Inflation":
            if Current_mass > MAX_MASS_INSIDE:
                break
            Current_mass = Current_mass +  ((time.time() - loop_start_time) /60 )* convert_volt2flow(ch_adc_act.voltage*3.2) *(AIR_MASS_PER_LITRE)
        if mode == "Deflation":
            if Current_mass <= 0:
                break
            Current_mass = Current_mass -  ((time.time() - loop_start_time) /60 )* convert_volt2flow(ch_adc_act.voltage*3.2) *(AIR_MASS_PER_LITRE)
    time.sleep(TIME_DELAY_RESPONSE)
    dac.setVoltage(ch_dac, 0)
    total_time = total_time + (time.time()-loop_start_time)
    
        

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

    pressure_channel = AnalogIn(adc,MCP.P5) # channel for reading pressure voltage values

    op_mode = input("Please specify operation mode (m for manual and d for default): ")

    if op_mode ==  "m":
        flow_rate_inf = float(input("Please input the inflation flow rate (in l/min): "))
        flow_rate_def = float(input("Please input the deflation flow rate (in l/min): "))
    
    else:
        flow_rate_inf = 1
        flow_rate_def = 1
    max_voltage = 3.3
    start = time.time()
    try:
        with open(file_path, "a") as log:
             
            cycle =1
            while cycle <= NUM_CYCLES:

                # inflation
                print('Inflating')
                run_valve_flow(start,channel_actual_inf,channel_set_pt_inf,channel_dac_inf,pressure_channel,flow_rate_inf,max_volt = max_voltage, mode = "Inflation")
                time.sleep(0.1)

                # deflation
                print('Deflating')
                run_valve_flow(start,channel_actual_def,channel_set_pt_def,channel_dac_def,pressure_channel,flow_rate_def, max_volt = max_voltage, mode = "Deflation")
                time.sleep(0.1)
                print("Cycle number: ", cycle)
                cycle = cycle + 1

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