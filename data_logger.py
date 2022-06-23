import rospy
import os
from prop_valves.msg import DataLog
from prop_valves import lib
import time


file_path = '/home/muthukumar/Inflating_and_deflating_finger_data.csv'



class DataLogger:

	def __init__(self,log):
		self.log = log
		self.data = {
		'time_log' : 0,
		'flow_value_inf': 0,
		'flow_value_def': 0,
		'target_flow' : 0,
		'pressure_inside' : 0,
		'current_mass' : 0,
		'current_gt_mass' : 0,
		'target_mass' : 0,
		'set_voltage_inf' : 0,
		'set_voltage_def' : 0,
		'set_voltage_inf_base': 0,
		'set_voltage_def_base': 0
		}


	def dataLogCallback(self,data):
		# self.data['time_log'] = data.time
		# self.data['flow_value_inf'] = data.flowValueInflation
		# self.data['flow_value_def'] = data.flowValueDeflation
		# self.data['target_flow'] = data.targetFlow
		# self.data['pressure_inside'] = data.pressureInside
		# self.data['current_mass'] = data.currentMass
		# self.data['current_gt_mass'] = data.currentGTMass
		# self.data['target_mass'] = data.targetMass
		# self.data['set_voltage_inf'] = data.setVoltageInflation
		# self.data['set_voltage_def'] = data.setVoltageDeflation
		# self.data['set_voltage_inf_base'] = data.setVoltageInflationBase
		# self.data['set_voltage_def_base'] = data.setVoltageDeflationBase
		

		lib.log_data(self.log,data.time,data.flowValueInflation, data.flowValueDeflation, data.targetFlow, data.pressureInside,data.currentMass, data.currentGTMass,data.targetMass, data.setVoltageInflation, 0, data.setVoltageDeflation,data.setVoltageInflationBase, data.setVoltageDeflationBase)



	def logData(self):
		lib.log_data(self.log,self.data['time_log'],self.data['flow_value_inf'], self.data['flow_value_def'], self.data['target_flow'],self.data['pressure_inside'],self.data['current_mass'], self.data['current_gt_mass'],self.data['target_mass'], self.data['set_voltage_inf'], 0, self.data['set_voltage_def'],self.data['set_voltage_inf_base'], self.data['set_voltage_def_base'])


if __name__ == '__main__':
	rospy.init_node('data_logger_node', anonymous=True)


	
	with open(file_path, "a") as log:
		

		dl = DataLogger(log)

		t =0
		time_start = time.time()
		while not rospy.is_shutdown():
			if t > 260:
				if os.path.exists(file_path):
					os.remove(file_path)
					t=0
			rospy.Subscriber("DataLogValues",DataLog, dl.dataLogCallback)

			print("Data Logged")
			t = time.time() - time_start
