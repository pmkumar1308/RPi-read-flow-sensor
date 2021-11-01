import os
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

file_path = '/home/pi/RPi-read-flow-sensor/Voltage_readings_Festo_cyclic_flow_rate1.csv'

if os.path.exists(file_path):
    os.remove(file_path)

GPIO.setmode(GPIO.BCM)   # use the Broadcom pin numbering
GPIO.setwarnings(False)  # disable warnings

RAMP_DELAY = 0.2 # delay to ramp the response in case of step flows
MAX_SET_VAL = 3.3 # conversion factor for setting voltage based on amplification of OP-Amp
VOLT_DIV_FACT = 3.2 # The step down factor in the volatge divider for Prop valves
OFFSET = 0 # if there is a offset to be included in the OP-Amp output voltage
TIME_DELAY_RESPONSE = 0.002 # for giving the chips time to respond
MAX_MASS_INSIDE  = 8e-5 # in kg
AIR_MASS_PER_LITRE = 1.292e-3 # in kg at STP
SLOW_DEF_DELAY = 0.2 # slow deflation delay for reducing jerk
SLOW_FLO = 0.2 # the slow flow rate to avoid jerk in l/min
TIME_TO_FILL = (MAX_MASS_INSIDE/AIR_MASS_PER_LITRE) * 60  # the time taken to fill the finger with 80 mg of air at STP in s
NUM_CYCLES = 5
CURRENT_MASS =0 

def convert_volt2dacVal(volt_val):
    """
    Returns a converted value of the voltage input between a number between 0 to 4096 to be given to
    to the DAC for set point value
    """
    return int(((volt_val-OFFSET)/MAX_SET_VAL) * 4096)

def read_store_volt(time,actual_channel,set_point_channel, pressure_ch):
    """
    Reads and stores voltage readings from ADC in the file open in .csv format
    """
    voltage = actual_channel.voltage * VOLT_DIV_FACT
    set_pt_voltage = set_point_channel.voltage * VOLT_DIV_FACT
    log.write("{0},{1},{2}\n".format(str(time),str (convert_volt2flow(voltage)),str(convert_volt2flow(set_pt_voltage), str(convert_volt2flow(pressure_ch.voltage*1.46)))))

def convert_volt2flow(volt):
    """
    Converts the voltage value from the sensor to the flow rate in l/min
    """
    flow_val = ((volt * 20)/9.8)-(4/9.8)
    return flow_val

def convert_flow2volt(flow):
    """
    Converts the flow rate in l/min to the corresponding voltage value
    """
    volt_value = (9.8 * (flow + 4/9.8))/20
    return volt_value

def convert_lmin2kgs(ch_adc):
    """
    Converts flowrate from l/min to kg/s
    """ 

    kgs_val = convert_volt2flow(ch_adc.voltage*VOLT_DIV_FACT) * (AIR_MASS_PER_LITRE)/60
    return kgs_val

def convert_kgs2lmin(flow_kgs):
    """
    Converts the flow rate in kg/s to l/min
    """

    flow_lmin = flow_kgs * 60/AIR_MASS_PER_LITRE 
    return flow_lmin


def interpol_fn(Mass_initial, Mass_to_reach, time_duration,num_time_steps, func_type = "Cubic"):
    """
    The function creates a cubic interpolation between two mass states
    """
    current_time = 0
    t = np.linspace(current_time,current_time + time_duration,num_time_steps)
    h = 0.1
    if func_type == "Cubic":    
        a_0 = Mass_initial
        a_1 = 0
        a_2 = (3/(time_duration**2))*(Mass_to_reach - Mass_initial)
        a_3 = - (2/(time_duration**3))*(Mass_to_reach - Mass_initial)
        Mass_t = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3
        Mass_dot_t = np.diff(Mass_t)/h
    return Mass_t, Mass_dot_t, t

def ramp_fn (Mass_flow_rate_final):
    """
    The linear ramp function creates a ranp while inflation and delfation when a step funtion voltage 
    is used.
    """
    current_t = 0
    t_range = np.linspace(current_t, current_t + RAMP_DELAY, 20)
    a_1 = Mass_flow_rate_final/RAMP_DELAY
    m_dot = a_1*t_range
    return m_dot



def run_valve_flow(start_time, time_dur, ch_adc_act,ch_adc_set,ch_dac,ch_pressure,flow_rate,Current_mass, max_volt=1, mode="Inflation"):
    
    set_voltage = 0
    valve_start_time = time.time()
    total_time = 0

    print(Current_mass)
    print("run")
    
    
    time_steps_num = 50
    loop_start_time = time.time()
    if mode == "Inflation":
        mass_t,flow_rate, time_range = interpol_fn(0, MAX_MASS_INSIDE, 4, time_steps_num, func_type = "Cubic")
        del_t = time_range[1]-time_range[0]
        for f in flow_rate:
            set_voltage = convert_flow2volt(convert_kgs2lmin(f))                                        
            dac.setVoltage(ch_dac, convert_volt2dacVal(set_voltage)) 
            time.sleep(del_t)
            if Current_mass > MAX_MASS_INSIDE:
                break            
            Current_mass = Current_mass +  del_t * convert_lmin2kgs(ch_adc_act)
            print(time.time() - loop_start_time)

    if mode == "Deflation":
        mass_t,flow_rate, time_range = interpol_fn(Current_mass, 0, 4, time_steps_num, func_type = "Cubic")
        flow_rate = - flow_rate
        del_t = time_range[1]-time_range[0]
        for f in flow_rate:
            set_voltage = convert_flow2volt(convert_kgs2lmin(f))                                        
            dac.setVoltage(ch_dac, convert_volt2dacVal(set_voltage)) 
            time.sleep(del_t)
            if Current_mass < 0.0000002:
                CURRENT_MASS = 0 
                break 
            Current_mass = Current_mass -  del_t * convert_lmin2kgs(ch_adc_act)
            print(Current_mass)


    time.sleep(TIME_DELAY_RESPONSE)
    dac.setVoltage(ch_dac, 0)
    return Current_mass
    
        

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

    channel_actual_def = AnalogIn(adc, MCP.P4) # channel for reading actual value of defflation v/v
    channel_set_pt_def = AnalogIn(adc, MCP.P3) # channel for reading set point value of defflation v/v

    pressure_channel = AnalogIn(adc, MCP.P5) # channel for reading pressure voltage values

    op_mode = input("Please specify operation mode (m for manual and d for default): ")

    if op_mode ==  "m":
        flow_rate_inf = float(input("Please input the inflation flow rate (in l/min): "))
        flow_rate_def = float(input("Please input the deflation flow rate (in l/min): "))
    
    else:
        flow_rate_inf = 1 # 1 l/min
        flow_rate_def = 1
    max_voltage = 3.3
    start = time.time()
    duration_of_step = 4 
    try:
        with open(file_path, "a") as log:
            curr_mass = 0
            cycle =1
            while cycle <= NUM_CYCLES:
                # inflation
                print('Inflating')
                curr_mass = run_valve_flow(start,duration_of_step, channel_actual_inf,channel_set_pt_inf,channel_dac_inf,pressure_channel,flow_rate_inf,curr_mass, max_volt = max_voltage, mode = "Inflation")
                time.sleep(0.1)

                # deflation
                print('Deflating')
                curr_mass = run_valve_flow(start,duration_of_step, channel_actual_def,channel_set_pt_def,channel_dac_def,pressure_channel,flow_rate_def,curr_mass, max_volt = max_voltage, mode = "Deflation")
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