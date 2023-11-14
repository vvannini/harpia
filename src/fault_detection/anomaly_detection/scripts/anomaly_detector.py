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
    """
    Get the root directory of the Harpia project.

    Returns:
    - str: The absolute path to the root directory of the Harpia project.
    """
    return rospy.get_param("/harpia_home", default=os.path.expanduser("~/harpia"))

def sequential_noise(errorType, data):
    """
    Generate sequential noisy data for different flight parameters.

    Args:
    - errorType (int): Type of noise to be applied.
    - data (pd.Series): Flight data containing various parameters.

    Returns:
    - dict: Sequential noisy data for different flight parameters.
    """
    sequential = {
        'roll': [],
        'pitch': [],
        'yaw': [],
        'heading': [],
        'rollRate': [],
        'pitchRate': [],
        'yawRate': [],
        'groundSpeed': [],
        'climbRate': 0,  # ?
        'altitudeRelative': [],
        'throttlePct': []
    }

    for key in sequential:
        sequential[key] = NoiseGenerator.noisyData(errorType, data, key, 1., 5.)

    return sequential

def kill_mission():
    """
    Sends a kill command to the Harpia mission system.

    This function publishes a "kill" message to the '/harpia/control/kill_mission' topic to terminate the mission system.

    Returns:
    - None
    """
    rospy.loginfo("Killing System")
    pub = rospy.Publisher('/harpia/control/kill_mission', String, queue_size=50)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        pub.publish("kill")
        rate.sleep()

# Classes

class UAV(object):
    """
    Represents a Unmanned Aerial Vehicle (UAV) in the Harpia system.

    Attributes:
    - sequential (dict): Dictionary containing sequential data of the UAV, including roll, pitch, yaw, heading, roll rate,
                        pitch rate, yaw rate, ground speed, climb rate, altitude relative, and throttle percentage.
                        Initialized with negative infinity values.
    - user_response_time (int): Time limit for user response to faults in seconds. Default is 30 seconds.
    - classifier_win_time (int): Time window for the fault classifier in seconds. Default is 10 seconds.
    - action_win_time (int): Time window for fault mitigation actions in seconds. Default is 60 seconds.

    Subscribers:
    - sub_pose: Subscribes to the '/drone_info/pose' topic to receive pose information updates.
    - sub_hardware: Subscribes to the '/hapia/uav' topic to receive hardware information updates.

    Methods:
    - pose_callback(data): Callback function for processing pose information from the UAV.
    - hardware_callback(data): Callback function for processing hardware information from the UAV.

    """

    def __init__(self):
        self.sequential = {
            'roll': -math.inf,
            'pitch': -math.inf,
            'yaw': -math.inf,
            'heading': -math.inf,
            'rollRate': -math.inf,
            'pitchRate': -math.inf,
            'yawRate': -math.inf,
            'groundSpeed': -math.inf,
            'climbRate': -math.inf,
            'altitudeRelative': -math.inf,
            'throttlePct': -math.inf
        }

        self.user_response_time = 30
        self.classifier_win_time = 10
        self.action_win_time = 60
        self.sub_pose = rospy.Subscriber('/drone_info/pose', DronePose, self.pose_callback)
        self.sub_hardware = rospy.Subscriber('/hapia/uav', UAVHarpia, self.hardware_callback)

    def pose_callback(self, data):
        """
        Callback function for processing pose information from the UAV.

        Args:
        - data (DronePose): Pose information received from the UAV.

        Returns:
        - None
        """
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
        """
        Callback function for processing hardware information from the UAV.

        Args:
        - data (UAVHarpia): Hardware information received from the UAV.

        Returns:
        - None
        """
        self.user_response_time = data.fault_settings.user_response
        self.classifier_win_time = data.fault_settings.classifier_time
        self.action_win_time = data.fault_settings.action_time
 


def land():
    """
    Initiates a landing command for the UAV using MAVROS.

    Uses the '/mavros/cmd/land' service to command the UAV to land.

    Parameters:
    - None

    Returns:
    - None
    """
    mavros_cmd(
        '/mavros/cmd/land',
        CommandTOL,
        error_msg="Landing failed",
        altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0
    )


def log(ErrorTime, ArrayPct, uav_action, Action=None, FalsePositive='n'):
    """
    Log information about a detected fault or anomaly.

    Parameters:
    - ErrorTime (float): The time when the error or fault was detected.
    - ArrayPct (float): The percentage of fault or anomaly in the UAV data.
    - uav_action (dict): Information about the UAV's status through time.
    - Action (str, optional): The specific action taken in response to the fault.
    - FalsePositive (str, optional): Indicates whether the detection is a false positive.

    Returns:
    - None
    """
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
	"""
    ROS listener for anomaly detection and response.

    Parameters:
    - input_error (int): Flag indicating the presence of input error.
    - error_type (int): Type of error for generating noisy data.
    - error_start_time (float): Time when the error starts.
    - error_end_time (float): Time when the error ends.

    Returns:
    - None
    """
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

	# Loading the modules
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

	# Main loop to verify drone fault
	while flag:
		win = int(round((time.time()-startMain),3)//time_win)

		#Start of new windown of time to classify
		if last_win != win:

			# Make pct for each flag
			total = uav_stat['normal'] + uav_stat['noise'] + uav_stat['mild'] + uav_stat['abnormal']
			pct_normal = uav_stat['normal'] / total
			pct_noise = uav_stat['noise'] / total
			pct_mild = uav_stat['mild'] / total
			pct_abnormal = uav_stat['abnormal'] / total

			ArrayPct = [pct_normal, pct_noise, pct_mild, pct_abnormal]

			# Set action number to uav_action depending on the pct
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

			# Print if is a new type of action flag
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
		

		# Set input error when set
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
		
		# Verify if drone has a need to peform a action
		if(flag_action>18 and time.time() > response_time):
			if(flag_action<24):
				rospy.logwarn("Found anomalous behavior") 
			else:
				# Ask for user confirmation.
				if(not flag_asked):
					print("Do you want to continue? (y/[n])")
					start_input = time.time()
					flag_asked = True


				elapsed_time = time.time() - start_input
				
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
	"""
	UAV Anomaly Detection and Response.

	This script initializes the UAV anomaly detection and response system. It reads simulation configurations from
	the 'sconfig.json' file, including information about input errors, error types, and error timings. The script
	then invokes the 'listener' function to continuously monitor UAV data for anomalies, take appropriate actions,
	and prompt the user for responses.

	Usage:
	- Ensure that the 'sconfig.json' file with simulation configurations is present in the 'json/' directory
	within the Harpia root directory.
	
	Note: This script assumes that the necessary functions and modules for anomaly detection, UAV control, and
		ROS communication are correctly implemented and available.

"""
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
