#!/usr/bin/env python3

import rospy

# Brings in the SimpleActionClient
import actionlib
from actionlib import GoalStatus
import sys, select

import math
import json
import time
import os
import argparse
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *

from std_msgs.msg import String
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from harpia_msgs.msg import *
from harpia_msgs.srv import *


feedback = 0


def feedback_callback(feedback_msg):
        # print(feedback_msg)
        global feedback
        feedback = feedback_msg.status
        # print('Received feedback: {}'.format(feedback))
        # return feedback

def test_client():
    rospy.wait_for_service('/harpia/weather_check')
    
    try:
        check = rospy.ServiceProxy('/harpia/weather_check', WeatherCheck)
        resp1 = check()
        return resp1.replan
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
    # client = rospy.ServiceProxy('harpia/weather_check_server', WeatherCheck)


    # client.wait_for_server()


    # Prints out the result of executing the action
    return -10

if __name__ == '__main__':
    try:
        result = test_client()
        rospy.loginfo(f"Result: {result}")
    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
