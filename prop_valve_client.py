from __future__ import print_function

import time
import numpy as np
import sys
from datetime import date
import os
import rospy
from prop_valves.srv import ControlVariables
from prop_valves.msg import FlowSensor, Control


file_path = '/home/pi/Inflating_and_deflating_finger_data.csv'

if os.path.exists(file_path):
    os.remove(file_path)

DEVICE_BUS_INFLATION = 1 #I2C bus used for inflation
DEVICE_BUS_DEFLATION = 3 #I2C bus used for deflation
DEVICE_ADDR = 0x01 #Default Sensirion SFM4100-Air device address
SCALE_FACTOR = 1000 #Used to convert the value recieved to slm(standard litre per minute)  
PRESSURE_ERROR = - 2.54 #-(6.246343882527859 + 0.5100433517496086 + 10.65)
PRESSURE_VOLT_DIV_FACT = 1.46 # Voltage divider factor for pressure sensor
MV_AVG_DEPTH = 10 # Depth of moving average
AIR_MASS_PER_LITRE = 1.292e-3 # in kg at STP
TARGET_MASS = 20e-6 # in kg
MAX_SET_VALUE = 5 #Maximum voltage to be set
MIN_SET_VALUE = 0 #Minimum voltage to be set
OFFSET = 0
CONTROL_OFFSET = 0
# TIME_DURATION = 2 + CONTROL_OFFSET
CURRENT_DRIVER_SENSE_RES_VALUE = 2.08
NUM_CYCLES = 10
MASS_TOLERANCE = 1e-6
                    
def usage():
    return "Please input like this: %s [target_mass(in mg) target_flow(in l/min)]"%sys.argv[0]

def convert_lmin2kgs(value_lmin):
    """
    Converts flowrate from l/min to kg/s
    """ 

    kgs_val = value_lmin * (AIR_MASS_PER_LITRE)/60
    return kgs_val

if __name__ == '__main__':

    try:
        if len(sys.argv) == 3:
            target_mass = float(sys.argv[1])
            target_flow = float(sys.argv[2])      
        else:
            print(usage())
        
        pub = rospy.Publisher("ControlTargets", Control, queue_size = 5)   
        rospy.init_node('prop_valve_client')
        rate = rospy.Rate(200)
        
        time.sleep(1)
        # while ~(rospy.get_param("actuating")):
        msg = Control()
        msg.targetMass = target_mass
        msg.targetFlow = target_flow
        pub.publish(target_mass, target_flow)
            # rate.sleep()
        
        print("Publishing target mass: %s mg and target flow %s l/min"%(target_mass, target_flow))


    except KeyboardInterrupt:   # Press CTRL C to exit program
        print("Keyboard interupt, exiting program...")