#!/usr/bin/env python3

import sys, traceback

import math
import os
import pandas as pd
import pickle
import json
import rospy
import time
import requests

from libs import predictor

from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import *
from rosplan_dispatch_msgs.srv import *

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from sensor_msgs.msg import BatteryState


from harpia_msgs.msg import *
from mavros_msgs.msg import *
from harpia_msgs.srv import *

# ---
# CLASSES

class Plan(object):
    def __init__(self):
        self.sub  = rospy.Subscriber("rosplan_parsing_interface/complete_plan", CompletePlan, self.plan_callback)   
        self.plan = CompletePlan()
    
    def plan_callback(self, data): 
        self.plan = data
        self.unsubscribe()

    def unsubscribe(self):
        self.sub.unregister()

class Drone(object):
    def __init__(self):
        self.sub = rospy.Subscriber('mavros/global_position/global',  NavSatFix, self.global_position_callback)  
        self.sub1 = rospy.Subscriber('mavros/battery',  BatteryState, self.battery_state_callback)  
        self.latitude = float("inf")
        self.longitude = float("inf")
        self.battery =  float("inf")
    
    def global_position_callback(self, data): 
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.unsubscribe()

    def battery_state_callback(self, data): 
        self.battery = data.percentage * 100
        # self.unsubscribe()

    def unsubscribe(self):
        self.sub.unregister()

# ---
# UTILS
def log(index):
    harpia_root_dir = get_harpia_root_dir()

    # log path
    log_path = os.path.join(harpia_root_dir, "results")
    log_json = os.path.join(log_path, "mission_log.json")

    # open log
    with open(log_json, "r") as log_file:
        log_file = json.load(log_file)

    # add replan
    log_file[-1]['knn'][index] += 1

    # dump log
    with open(log_json, 'w') as outfile:
        json.dump(log_file, outfile, indent=4)

def euclidean_distance(A, B):
    return math.sqrt((B.x - A.x) ** 2 + (B.y - A.y) ** 2)

def calc_dist_path(path):
    dist = 0
    i = 1
    for p in path:
        if p != path[-1]:
            A = CartesianPoint(p[0], p[1])
            B = CartesianPoint(path[i][0], path[i][1])
            dist += euclidean_distance(A, B)
            i += 1

    return dist

def wait_until(check, msg=None, rate=1):
    rate = rospy.Rate(rate)

    while not check():
        if rospy.is_shutdown(): return False
        if msg is not None: rospy.loginfo(msg)
        rate.sleep()

    return True


# ---
# SERVER


def weather_check(data):
    p = Plan()
    uav = Drone()

    # while p.plan.plan ==  []:
    #     print("Waiting for Complete Plan...")
    #     rospy.sleep(1)
    # ArrayActions = p.plan.plan[req.action_id:]

    while uav.latitude ==  float("inf"):
            print("Waiting for UAV info...")
            rospy.sleep(1)

    apikey = '' # api key for the openwheater api 

    address="https://api.openweathermap.org/data/2.5/weather?lat={}&lon={}&appid={}&units=metric".format(uav.latitude,uav.longitude,apikey)

    conect =  requests.get(address)

    current_weather = conect.json()

    if (current_weather['cod'] == 200):
        print('we have weather info')
        print(current_weather['weather'])
    else:
        print('error getting weather info')

    return WeatherCheckResponse(0)

def weather_check_server():
    rospy.init_node("weather_check_server")

    srv = rospy.Service('harpia/weather_check', WeatherCheck, weather_check)

    startMain = time.time()
    #____________// MAIN ARCHITECTURE //____________#
    rospy.loginfo('<Weather Service> METAR Data Analysis - Prediction Model')

    start_1 = time.time()
    reg_model = predictor.GradientBooster()
    end_1 = time.time()
    rospy.loginfo(f'<Weather Service> Loaded Gradient Booster Algorythm in {round((end_1-start_1),3)} seconds.')


    rospy.loginfo("Weather Check Service Ready")
    srv.spin()

if __name__ == "__main__":
    weather_check_server()
