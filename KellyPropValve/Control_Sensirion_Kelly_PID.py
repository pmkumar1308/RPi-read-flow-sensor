import time
import numpy as np
import os
from MCP4922 import MCP4922
import sys
import RPi.GPIO as GPIO
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
from datetime import date
import smbus


POLYNOMIAL = 0x131
CHECKSUM_ERROR = 0x04
DEVICE_BUS = 1 # I2C bus used
DEVICE_ADDR = 0x01 #Default Sensirion SFM4100-Air device address
SCALE_FACTOR = 1000 #Used to convert the value recieved to slm(standard litre per minute) 
MAX_DEPTH = 10 # for moving average depth 
MAX_SET_VALUE = 5 #Maximum voltage to be set
MIN_SET_VALUE = 0 #Minimum voltage to be set
OFFSET = 0


file_path = '/home/pi/Data_Kelly_Sensirion.csv'

def log_data(time,actual_flow,target_flow):
    """
    Reads and stores data in the file open in .csv format
    """
    
    # voltage = actual_channel.voltage 
    # set_pt_voltage = set_point_channel.voltage 
    # voltage_pressure = pressure_ch.voltage 
    today = date.today()
    log.write("{0},{1},{2},{3}\n".format(str(today.strftime("%d/%m/%Y")),str(time),str (actual_flow),str(target_flow)))



def convertVolt2DacVal(volt_val):
    """
    Returns a converted value of the voltage input between a number between 0 to 4096 to be given to
    to the DAC for set point value
    """
    return int(((volt_val-OFFSET)/MAX_SET_VALUE) * 4096)


def twosComp(val, bits):
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val

def checkCrc(read_values, checksum):
	crc = 0
	for i in range(len(read_values)):
		crc = crc ^ read_values[i]
		for bit in range(8,0):
			if crc & 0x80:
				crc = (crc<<1) ^ POLYNOMIAL
			else:
				crc = crc << 1
		if crc != checksum:
			return CHECKSUM_ERROR
		else:
			return 0

class ControlFlow:

	def __init__(self, target_value, measured_value, proportional_gain, derivative_gain, integral_gain):
		self.target = target_value
		self.measured = measured_value
		self.KP = proportional_gain
		self.KD = derivative_gain
		self.KI = integral_gain
		self.prev_error = 0
		self.sum_error = 0

	def contolLaw(self, controlVoltage):
		error = self.target - self.measured
		controlVoltage = controlVoltage + error * self.KP + self.prev_error * self.KD + self.sum_error * self.KI
		controlVoltage = max(min(MAX_SET_VALUE, controlVoltage), MIN_SET_VALUE)
		self.prev_error = error
		self.sum_error = self.sum_error + error
		return controlVoltage



def ReadSensirion(device_bus):
	read_value = device_bus.read_i2c_block_data(DEVICE_ADDR,0xF1,3) # measurement triggering command 0xF1 or hF1
	checkSum = read_value[2]
	checkSumError = checkCrc(read_value,checkSum)
	read_value = read_value[0]<<8 | read_value[1]



	flow_value = twosComp(read_value,16) #getting the twos complement
	flow_value = flow_value/SCALE_FACTOR

	return flow_value


if __name__ == '__main__':

	i=0
	NumberOfRuns = 1000
	flow_values = np.empty(NumberOfRuns+2)

	# For Sensirion
	bus = smbus.SMBus(DEVICE_BUS) 

	# For DAC
	dac = MCP4922(spibus=1,spidevice=2) # by default chooses the CE0 for chip select
	select_channel = 0
	set_voltage = 0
	dac.setVoltage(select_channel, convertVolt2DacVal(set_voltage))

	targetFlow = float(input("Target Value in l/min: "))
	controlKP = float(input("Proportional Gain: "))
	# controlKD = 0.5 * controlKP
	controlKD = float(input("Derivative Gain: "))
	# controlKI = 0.5 * controlKD
	controlKI = float(input("Integral Gain: "))


	
	
	del_t = 0.0005
	t_start = time.time() + MAX_DEPTH * del_t #Should we add this?
	try:
		with open(file_path, "w") as log:  
			while(True & (i<=NumberOfRuns)):				
				flow_val = ReadSensirion(bus)				
				
				#Moving Average of the measure flow values
				flow_values[i] = flow_val
				if i >= MAX_DEPTH:
					avg_flow_value = np.mean(flow_values[i-MAX_DEPTH:i])

					# Control				
					cf = ControlFlow(targetFlow, avg_flow_value, controlKP, controlKD, controlKI)
					set_voltage = cf.contolLaw(set_voltage)
					dac.setVoltage(select_channel, convertVolt2DacVal(set_voltage))
					if  (i%100) == 0:
						print("The flow_value in l/min:(Sensirion) ", avg_flow_value)
					# print("Time: ", time.time()-t_start)
					time_val = time.time()-t_start
					log_data(time_val,avg_flow_value,targetFlow)
					

				# Give time for sampling
				time.sleep(del_t)
				i=i+1
			print("Print Data recorded")
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
