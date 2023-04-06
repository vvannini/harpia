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
        self.sub_pose     = rospy.Subscriber('/mavros/local_position/pose'  , PoseStamped              , self.pose_callback)
        self.sub_imu      = rospy.Subscriber('/mavros/imu/data'             , Imu                      , self.imu_callback)
        self.sub_vfr_hud  = rospy.Subscriber('/mavros/vfr_hud'              , VFR_HUD  , self.vfr_hud_callback)
        self.sub_g_pos    = rospy.Subscriber('/mavros/global_position/local', Odometry , self.globalp_callback)
        self.sequential = {'roll':[],
                          'pitch':[],
                          'heading':[], #yaw
                          'rollRate':[],
                          'pitchRate':[],
                          'yawRate':[],
                          'groundSpeed':[],
                          'climbRate':0, # ?
                          'altitudeRelative':[],
                          'throttlePct':[]}
        # self.weightedCluster = []

    
    def pose_callback(self, data): 
        self.sequential['roll'] = data.pose.position.x
        self.sequential['pitch'] = data.pose.position.y
        self.sequential['heading'] = data.pose.position.z

    def imu_callback(self, data): 
        self.sequential['rollRate'] = data.angular_velocity.x
        self.sequential['pitchRate'] = data.angular_velocity.y
        self.sequential['yawRate'] = data.angular_velocity.z

    def vfr_hud_callback(self, data): 
        self.sequential['groundSpeed'] = data.groundspeed
        self.sequential['throttlePct'] = data.throttle

    def globalp_callback(self, data): 
        self.sequential['altitudeRelative'] = data.pose.pose.position.z

    # def unsubscribe(self):
    #     self.sub.unregister()

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
