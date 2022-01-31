import smbus
import time

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
def twos_comp(val, bits):
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val


while(True & i<100):
	# bus.write_byte(DEVICE_ADDR, 0xF1)
	read_value = bus.read_i2c_block_data(DEVICE_ADDR,0xF1,2) # measurement triggering command 0xF1 or hF1
	read_value = read_value[0]<<8 | read_value[1]
	flow_value = twos_comp(read_value,16) #getting the twos complement
	flow_value = flow_value/SCALE_FACTOR
	print("The flow_value in l/min: ", flow_value)
	time.sleep(1)
	i=i+1