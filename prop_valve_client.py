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
TIME_DURATION = 5 + CONTROL_OFFSET
CURRENT_DRIVER_SENSE_RES_VALUE = 2.08
NUM_CYCLES = 1
MASS_TOLERANCE = 1e-6

class CubicInterpolation:

    def __init__(self, mass_initial, mass_to_reach):
        self.completion_time = TIME_DURATION
        self.a_0 = mass_initial
        self.a_1 = 0
        self.a_2 = (3/(self.completion_time**2))*(mass_to_reach - mass_initial)
        self.a_3 = - (2/(self.completion_time**3))*(mass_to_reach - mass_initial)
        
    def getMass(self,t):
        return self.a_0 + self.a_1 * t + self.a_2 * t**2 + self.a_3 * t**3

    def getFlow(self,t):
        return self.a_1 + 2 * self.a_2 * t + 3 * self.a_3 * t**2

class sinosoidalTrajectory:
    def __init__(self,mass_initial, mass_to_reach):
        self.initial_mass = mass_initial
        self.final_mass = mass_to_reach
        self.completion_time = 2 * TIME_DURATION
        self.amplitude = (mass_to_reach - mass_initial)/2
        self.frequency = 1 / self.completion_time

    def getMass(self,t):
        return self.amplitude*np.sin(2 * np.pi * self.frequency * t - np.pi/2 ) + (self.initial_mass + self.final_mass)/2 #/ self.completion_time
    
    def getFlow(self,t):
        return self.amplitude*np.cos(2 * np.pi * self.frequency * t - np.pi/2) 

class CubicTrjectoryCyclic:
    def __init__(self,mass_initial, mass_to_reach, num_cycles):
        self.completion_time = TIME_DURATION
        self.a_0 = mass_initial
        self.a_1 = 0
        self.a_2 = (3/(self.completion_time**2))*(mass_to_reach - mass_initial)
        self.a_3 = - (2/(self.completion_time**3))*(mass_to_reach - mass_initial)
        self.num_cycles = num_cycles

    def getMass(self,t):
        return self.a_0 + self.a_1 * t + self.a_2 * t**2 + self.a_3 * t**3
    
    def getFlow(self,t):
        return self.a_1 + 2 * self.a_2 * t + 3 * self.a_3 * t**2


class stepFunction:
    def __init__(self,mass_initial, mass_to_reach):
        self.initial_mass = mass_initial
        self.final_mass = mass_to_reach

    def getMass(self,t):
        return self.final_mass
    
    def getFlow(self,t):
        return 0 ##  TODO

class trajectoryGenerator:
    def choose(trajectoryName,mass_initial, mass_to_reach):
        if trajectoryName == "cubic":
            print("cubic trajectory")
            return CubicInterpolation(mass_initial, mass_to_reach)
        if trajectoryName == "sin":
            print("Using sinosoidal trajectory")
            return sinosoidalTrajectory(mass_initial, mass_to_reach)
        if trajectoryName == 'step':
            print("Stepping to target mass")
            return 
                    
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

        # target_mass = float(input("Input Target Mass: "))
        target_mass = 70
        traj_type = input("Trajectory type: ")
        # traj_type = "cubic"
        current_mass = rospy.get_param("current_mass")

        traj = trajectoryGenerator.choose(traj_type,current_mass,target_mass)
        
        pub = rospy.Publisher("ControlTargets", Control, queue_size = 1)   
        rospy.init_node('prop_valve_client')
        rate = rospy.Rate(100)
        traj_start_time = time.time()
        t =0
        num_cycles = NUM_CYCLES
        cycles = 0
        initial_mass = 40
        while cycles < num_cycles:
            traj = trajectoryGenerator.choose(traj_type,initial_mass,target_mass)
            time_start=time.time()
            t=0
            while t < (TIME_DURATION): 
                
                desired_mass = traj.getMass(t)
                desired_flow = traj.getFlow(t)
                msg = Control()
                msg.targetMass = desired_mass
                msg.targetFlow = desired_flow
                pub.publish(desired_mass, desired_flow) 
                print(f"Published mass: {desired_mass} Published Flow: {desired_flow}") 


                t = time.time() - time_start
                rate.sleep()

            
            traj = trajectoryGenerator.choose(traj_type,target_mass,initial_mass) 
            time_start=time.time()
            t=0
            while t < (TIME_DURATION): 
                
                desired_mass = traj.getMass(t)               
                desired_flow = traj.getFlow(t)
                msg = Control()
                msg.targetMass = desired_mass
                msg.targetFlow = desired_flow
                pub.publish(desired_mass, desired_flow) 


                t = time.time() - time_start
                rate.sleep()
            cycles = cycles +1


            desired_mass_traj = traj.getMass(t)
            
            t = time.time() - traj_start_time
                # 
        
        # print("Publishing target mass: %s mg and target flow %s l/min"%(target_mass, target_flow))


    except KeyboardInterrupt:   # Press CTRL C to exit program
        print("Keyboard interupt, exiting program...")