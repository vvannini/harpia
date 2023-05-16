#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import NavSatFix, Imu, BatteryState


# --- 
from libs.anomalyDetection import findAnomaly, clusters, checkValues
from libs import anomalyDetection,clustering,loadData, NoiseGenerator
#S,NoiseGenerator #user libraries
# import numpy as np
# import pandas as pd
# import colorama;colorama.init(autoreset=True)
# import termcolor
# from colorama import Fore, Back, Style
# import progressbar
import time
import math

def sequential_noise(data):
    sequential = {'roll':[],
                          'pitch':[],
                          'heading':[], #yaw
                          'rollRate':[],
                          'pitchRate':[],
                          'yawRate':[],
                          'groundSpeed':[],
                          'climbRate':0, # ?
                          'altitudeRelative':[],
                          'throttlePct':[]}
    for key in sequential:
        sequential[key] = NoiseGenerator.noisyData(data,key, 1., 50000.)

    return sequential


# Classes

class UAV(object):
    def __init__(self):
        self.sub_pose   = rospy.Subscriber('/drone_info/pose'  , DronePose, self.pose_callback)
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
        # self.weightedCluster = []

    
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



def listener():

    rospy.init_node('listener', anonymous=True)
    kenny = UAV()


    startMain = time.time()

    _,statistics = loadData.loadData(printColumnNames=True) #loading previous flights statistics

    # anomaly classifier (static):
    start_1 = time.time()
    module1 = anomalyDetection.findAnomaly()
    end_1 = time.time()
    print("\n>> Loaded Anomaly Detection Module in {} seconds.\n".format(round((end_1-start_1),3)))


    # clustering method (static):
    start_2 = time.time()
    module2 = clustering.clusters()
    end_2 = time.time()
    print("\n>> Loaded Clustering Module in {} seconds.\n".format(round((end_2-start_2),3)))

    # while not rospy.is_shutdown():
    _, flag = anomalyDetection.checkValues(kenny.sequential, statistics, module1, module2)

    while flag:
        if(time.time()-startMain > 1) and (time.time()-startMain < 120):
            print('noisyData')
            
            error_data = sequential_noise(kenny.sequential)


            error_data['rollRate'] = error_data['rollRate'] * 1000
            # error_data['yawRate'] = error_data['yawRate'] * 1000
            # error_data['pitchRate'] = error_data['pitchRate'] * 1000

            print(error_data['rollRate'], kenny.sequential['rollRate'])
            print(error_data['yawRate'], kenny.sequential['yawRate'])
            print(error_data['pitchRate'], kenny.sequential['pitchRate'])

            # print(error_data['yawRate'], kenny.sequential['yawRate'])
            # print(error_data['pitchRate'], kenny.sequential['pitchRate'])

            # print(error_data)
            _, flag = anomalyDetection.checkValues(error_data, statistics, module1, module2)
        elif(time.time()-startMain > 140):
            break
        else:
            _, flag = anomalyDetection.checkValues(kenny.sequential, statistics, module1, module2)
        # print(kenny.sequential)
        rospy.Rate(1)
        


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
