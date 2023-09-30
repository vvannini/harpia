#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import *
from nav_msgs.msg import *
from harpia_msgs.msg import DronePose
from harpia_msgs.msg import UAV as UAVHarpia
from geometry_msgs.msg import *
from sensor_msgs.msg import NavSatFix, Imu, BatteryState


# --- 
from libs.anomalyDetection import checkAnomaly, pca_model, scaler_model
from libs import anomalyDetection,clustering,loadData, NoiseGenerator

from libs import uavAction as action
import time
import math
import numpy as np
import json


import os
import sys
import select  

from colorama import Fore, Back, Style


def get_harpia_root_dir():
	return rospy.get_param("/harpia_home", default=os.path.expanduser("~/harpia"))

def sequential_noise(errorType, data):
	sequential = {'roll':[],
						  'pitch':[],
						  'yaw':[],
						  'heading':[], 
						  'rollRate':[],
						  'pitchRate':[],
						  'yawRate':[],
						  'groundSpeed':[],
						  'climbRate':0, # ?
						  'altitudeRelative':[],
						  'throttlePct':[]}
	for key in sequential:
		sequential[key] = NoiseGenerator.noisyData(errorType, data,key, 1., 5.)

	return sequential

def kill_mission():
	rospy.loginfo("Killing System")
	pub = rospy.Publisher('/harpia/control/kill_mission', String, queue_size=50)
	rate = rospy.Rate(1) # 10hz
	
	while not rospy.is_shutdown():
		pub.publish("kill")
		rate.sleep()

# Classes

class UAV(object):
	def __init__(self):
		self.sequential = {'roll':-math.inf,
						  'pitch':-math.inf,
						  'yaw' : -math.inf,
						  'heading':-math.inf, 
						  'rollRate':-math.inf,
						  'pitchRate':-math.inf,
						  'yawRate':-math.inf,
						  'groundSpeed':-math.inf,
						  'climbRate':-math.inf, 
						  'altitudeRelative':-math.inf,
						  'throttlePct':-math.inf}

		self.user_response_time = 30
		self.classifier_win_time = 10
		self.action_win_time = 60
		self.sub_pose   = rospy.Subscriber('/drone_info/pose'  , DronePose, self.pose_callback)
		self.sub_hardware = rospy.Subscriber('/hapia/uav', UAVHarpia, self.hardware_callback)

	
	def pose_callback(self, data): 
		self.sequential['roll'] = data.roll
		self.sequential['pitch'] = data.pitch
		self.sequential['yaw'] = data.yaw
		self.sequential['heading'] = data.heading
		self.sequential['rollRate'] = data.rollRate
		self.sequential['pitchRate'] = data.pitchRate
		self.sequential['yawRate'] = data.yawRate
		self.sequential['groundSpeed'] = data.groundSpeed
		self.sequential['throttlePct'] = data.throttle
		self.sequential['altitudeRelative'] = data.altRelative
		self.sequential['climbRate'] = data.climbRate

	def hardware_callback(self, data): 
		self.user_response_time = data.fault_settings.user_response
		self.classifier_win_time = data.fault_settings.classifier_time
		self.action_win_time = data.fault_settings.action_time
   


def land():
	mavros_cmd(
		'/mavros/cmd/land',
		CommandTOL,
		error_msg="Landing failed",
		altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0
	)


def log(ErrorTime, ArrayPct, uav_action, Action=None, FalsePositive='n'):
	# log path
	root_dir = get_harpia_root_dir()
	log_path = os.path.join(root_dir, "results/mission_log.json")

	# open log
	with open(log_path, "r") as log_file:
		log_file = json.load(log_file)

	fault = {
		'time': ErrorTime,
		'ErrorPct': ArrayPct,
		'Action': Action,
		'FalsePositive': FalsePositive,
		'uav_stat': uav_action
	}
	
	log_file[-1]['detected_fault'].append(fault) 
	

	# dump log
	with open(log_path, 'w') as outfile:
		json.dump(log_file, outfile, indent=4)

def listener(input_error, error_type, error_start_time, error_end_time):

	rospy.init_node('anomaly_detector', anonymous=True)
	kenny = UAV()

	flag_aux = 0


	startMain = time.time()

	uav_stat = {
		'normal':0,
		'noise': 0,
		'mild': 0,
		'abnormal': 0, 
		'data': [],
		'last_state': 'none'
	}

	uav_action = []

	pct_normal = 0
	pct_noise = 0
	pct_mild = 0
	pct_abnormal = 0

	flag_error = (input_error == 1)
	print(flag_error)

	time_win = kenny.classifier_win_time

	start = time.time()
	module1 = anomalyDetection.pca_model()
	end = time.time()
	print("\n>> Loaded PCA Module in {} seconds.".format(round((end-start),3)))


	start = time.time()
	module2 = anomalyDetection.scaler_model()
	end = time.time()
	print("\n>> Loaded Scaler Module in {} seconds.".format(round((end-start),3)))

	start = time.time()
	module3 = anomalyDetection.tree_model()
	end = time.time()
	print("\n>> Loaded Tree Module in {} seconds.".format(round((end-start),3)))

	rospy.loginfo("Anomaly Detector Ready")

	# while not rospy.is_shutdown():
	startMain = time.time()
	win = 0 
	last_win = 0
	uav_stat, flag = anomalyDetection.checkAnomaly(kenny.sequential,uav_stat, module1, module2, module3, win)
	response_time = time.time()

	flag_asked = False

	while flag:
		win = int(round((time.time()-startMain),3)//time_win)
		if last_win != win:
			# print(win)
			total = uav_stat['normal'] + uav_stat['noise'] + uav_stat['mild'] + uav_stat['abnormal']
			pct_normal = uav_stat['normal'] / total
			pct_noise = uav_stat['noise'] / total
			pct_mild = uav_stat['mild'] / total
			pct_abnormal = uav_stat['abnormal'] / total

			ArrayPct = [pct_normal, pct_noise, pct_mild, pct_abnormal]

			if(pct_noise <= 0.7 and pct_abnormal == 0.0 and pct_mild <= 0.05):
				uav_action.append(0) # ok

			elif(pct_noise > 0.7 and pct_abnormal == 0.0 and pct_mild <= 0.1):
				uav_action.append(1) # soft warning 

			elif(0.1 < pct_mild <= 0.4 and pct_abnormal == 0.0): 
				uav_action.append(2) # warning

			elif(pct_abnormal< 0.05 and 0.4<pct_mild<=0.6):
				uav_action.append(4) # base

			elif(pct_abnormal< 0.1 or pct_mild>0.6):
				uav_action.append(8) # base

			elif(pct_abnormal<0.2):
				uav_action.append(16) # base

			else:
				uav_action.append(32) # land 

			if  len(uav_action) == 1 or uav_action[-1] != uav_action[-2]:
				if uav_action[-1] == 0:
					print(Fore.BLACK + Back.GREEN + "Normal Pattern")
					print(Style.RESET_ALL)
				elif uav_action[-1] == 1:
					print(Fore.BLACK + Back.CYAN + "Light Noise Pattern")
					print(Style.RESET_ALL)
				elif uav_action[-1] == 2:
					print(Fore.WHITE + Back.BLUE + "Mild Noise Pattern")
					print(Style.RESET_ALL)
				elif uav_action[-1] == 4:
					print(Fore.WHITE + Back.YELLOW + "Strong Noise Pattern")
					print(Style.RESET_ALL)
				elif uav_action[-1] == 8:
					print(Fore.WHITE + Back.YELLOW + "Soft Anomalous Pattern")
					print(Style.RESET_ALL)
				elif uav_action[-1] == 16:
					print(Fore.WHITE + Back.MAGENTA + "Mild Anomalous Pattern")
					print(Style.RESET_ALL)
				else:
					print(Fore.WHITE + Back.RED + "Strong Anomalous Pattern!")
					print(Style.RESET_ALL)


			#reset params
			last_win = win
			# print(uav_action)
			uav_stat['normal'] = 0
			uav_stat['noise'] = 0
			uav_stat['mild'] = 0
			uav_stat['abnormal'] = 0
			uav_stat['data'] = []
		

	
		if(flag_error and time.time()-startMain > error_start_time) and (time.time()-startMain < error_end_time):
			if flag_aux == 0:
				print('noisyData')
				flag_aux = 1
			error_data = sequential_noise(error_type, kenny.sequential)
			uav_stat, flag = anomalyDetection.checkAnomaly(error_data,uav_stat, module1, module2, module3, win)

		else:
			if flag_aux == 1:
				print('normal Data')
				log(time.time()-startMain, ArrayPct, uav_action,'normal', 'n')
				flag_aux = 0
			uav_stat, flag = anomalyDetection.checkAnomaly(kenny.sequential,uav_stat, module1, module2, module3, win)

		action_win = int(kenny.action_win_time/kenny.classifier_win_time)
		flag_action = np.sum(uav_action[-action_win:])
		

		if(flag_action>18 and time.time() > response_time):
			if(flag_action<24):
				rospy.logwarn("Found anomalous behavior") 
			else:
				if(not flag_asked):
					print("Do you want to continue? (y/[n])")
					start_input = time.time()
					flag_asked = True


				elapsed_time = time.time() - start_input
				# print(elapsed_time)
				# print(kenny.user_response_time)
				# print(elapsed_time >= kenny.user_response_time)
				
				wait_time = kenny.user_response_time if flag_action<48 else kenny.user_response_time/2
				if elapsed_time >= wait_time:
					print("\nTimeout reached. Exiting.")
					if(flag_action>48):
						print('land')
						log(start_input-startMain, ArrayPct, uav_action,'land', 'n')
						action.land()
					else:
						print('base')
						log(start_input-startMain, ArrayPct, uav_action,'base', 'n')
						try:
							action.go_to_base()
						except:
							action.land()
					# kill_mission()

				if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
					user_input = input().lower()
					response_time = time.time() + kenny.action_win_time 
					print("User response: ",user_input)

					if user_input == 'y':
						print("Continuing...")
						log(start_input, ArrayPct,uav_action, 'Continuing', user_input)
						flag_asked = False

					elif user_input == 'n':
						if(flag_action>48):
							print('land')
							log(start_input-startMain, ArrayPct, uav_action,'land', user_input)
							action.land()
							flag_asked = False

						else:
							print('base')
							try:
								log(start_input-startMain, ArrayPct, uav_action, 'base', user_input)
								action.go_to_base()
							except:
								action.land()
							flag_asked = False

						# kill_mission()
			time.sleep(1)
		
		rospy.Rate(1)
		

	rospy.spin()

if __name__ == '__main__':
	harpia_root = get_harpia_root_dir()

	PATH  = os.path.join(harpia_root, "json/")

	sim_config = os.path.join(PATH, "sconfig.json")

	with open(sim_config, "r") as sim_file:
		sim_file = json.load(sim_file)

	input_error = sim_file["input_error"]
	error_type = sim_file["error_type"]
	error_start_time = sim_file["error_start_time"]
	error_end_time = sim_file["error_end_time"]
	print(sim_file)

	listener(input_error, error_type, error_start_time, error_end_time)
