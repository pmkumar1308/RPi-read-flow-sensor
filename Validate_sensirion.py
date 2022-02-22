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

DEVICE_BUS = 1 #I2C bus used
DEVICE_ADDR = 0x01 #Default Sensirion SFM4100-Air device address
SCALE_FACTOR = 1000 #Used to convert the value recieved to slm(standard litre per minute)  

"""The calibrated signal read from the sensor is a signed
INTEGER number (two's complement number). The
INTEGER value can be converted to the physical value by
dividing it by the scale factor (mass flow = sensor output/ 
scale factor)."""

bus = smbus.SMBus(DEVICE_BUS)
i=0


def convert_lmin2kgs(value_lmin):
    """
    Converts flowrate from l/min to kg/s
    """ 

    kgs_val = value_lmin * (AIR_MASS_PER_LITRE)/60
    return kgs_val

def convert_volt2pressure(voltage_pressure_sensor):
    v_cc = 5.0
    pressure = (((voltage_pressure_sensor*PRESSURE_VOLT_DIV_FACT/(v_cc))-0.04)/0.00369) + PRESSURE_ERROR
    # voltage = self.df["pressure sensor"].to_numpy().tolist()

    return pressure

def getVolume():
    # Basic Volume
    # small tube: (13.5cm + 16.5 + 11.5 + 1.5 + 65 cm) * 0.2cm
    # big: 197.5cm, small: 101.0cm
    # elastic tube: 45cm * 0.2
    # T-connector: (3.6cm + 1.8cm) * 0.4cm
    # elastic-to-small-connector: 4.5cm * 0.4cm
    # split-connector: (2 * 2cm + 1.5cm) * 0.4cm
    # valve: 2 * 0.5 * 10cm * 0.5cm


    setup_length = [13.5, 16.5, 11.5, 1.5, 197.5, 45, 65, 3.6, 1.8, 4.5, 5.5, 5, 5]
    setup_diameter = [0.2, 0.2, 0.2, 0.2, 0.5, 0.2, 0.2, 0.4, 0.4, 0.4, 0.4, 0.5, 0.5]

    system_volume = 0

    for i in range(len(setup_length)):
        system_volume += np.pi * (setup_diameter[i] / 2) ** 2 * setup_length[i]

    return system_volume

def computeMassDelta(self, p_after, p_before=0.0, r=287.058, t=293.15):
        """
        Computes the air mass change given a known channel volume

            Arguments:
                p_after --  pressure in kPa (relative to ambient)
                p_before -- pressure in kPa (relative to ambient)
                V_reference -- volume in ccm

                r = 287.058 -- specific gas constant of air at room temperature
                t = 293.15 --  air temperature in Kelvin (20 degrees Celsius, room temperature)

                returns -- mass change in mg
        """

        v_reference = getVolume()

        # scale volume into cubic meter (0.01**3 = 1e-6)
        v_reference = v_reference * 1e-6

        # compute pressure change in Pascal:
        delta_p = 1e3 * (p_after - p_before)

        # from the ideal gas law: p*V = mRT
        delta_m = delta_p * v_reference / (r * t)

        # return the mass in kg:
        return delta_m *1e3

if __name__ == '__main__':
    
    try:
        with open(file_path, "w") as log: 
            while(True & i<100):
            	# bus.write_byte(DEVICE_ADDR, 0xF1)
            	del_t = 0.05

            	pressure_channel = AnalogIn(adc, MCP.P7)
            	time.sleep(del_t)

            	read_value = bus.read_i2c_block_data(DEVICE_ADDR,0xF1,2) # measurement triggering command 0xF1 or hF1
            	read_value = read_value[0]<<8 | read_value[1]
            	flow_value = twos_comp(read_value,16) #getting the twos complement
            	flow_value = flow_value/SCALE_FACTOR
                
                p_after1 = convert_volt2pressure(ch_pressure.voltage)

            	Current_mass = Current_mass +  del_t * convert_lmin2kgs(flow_value)
            	Current_GT_mass = Current_GT_mass + computeMassDelta(p_after1, r=287.058, t=293.15)
            	# print("Read_value",read_value)
            	# print("The flow_value in l/min: ", flow_value)
            	
            	i=i+1
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
