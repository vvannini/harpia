#!/usr/bin/env python3

import rospy
import actionlib
import sys
import math
import json
import time
import os
import rosnode, psutil
from shutil import copyfile
from itertools import count
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import *
from rosplan_dispatch_msgs.srv import *

from std_msgs.msg import String

from harpia_msgs.msg import *

def get_harpia_root_dir():
    return rospy.get_param("/harpia_home", default=os.path.expanduser("~/harpia"))

def parse_file_plan():
    root_dir = get_harpia_root_dir()
    plan_path = os.path.join(root_dir, 'pddl/plan.pddl')

    success = False
    cpu_time = float('Inf')
    total_time = 0

    for line in open(plan_path):
        if 'Solution Found' in line: success = True
        if '; Time' in line:
            aux = line.split(' ')
            cpu_time = float(aux[-1].rstrip())
        if ': (' in line:
            aux = line.split(' ')
            # TODO: using `eval` is never very safe, change this to something more robust
            total_time += eval(aux[-1].rstrip())[0]

    # log path
    log_dir = os.path.join(root_dir, "results/plans/")

    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    ith_log_name = lambda i: os.path.join(log_dir, f"{i}.pddl")

    # Create an iterator over the unused log file names and use the next one available
    log_names = ((ith_log_name(i), i) for i in count(0) if not os.path.exists(ith_log_name(i)))
    log_path, id = next(log_names)

    copyfile(plan_path, log_path)
   
    return success, cpu_time, total_time, id

def log():
    # log path
    root_dir = get_harpia_root_dir()
    log_path = os.path.join(root_dir, "results/mission_log.json")

    # open log
    with open(log_path, "r") as log_file:
        log_file = json.load(log_file)


    sucsses, cpu_time, total_time, plan_id = parse_file_plan()
    # add replan
    log_file[-1]['cpu_time'].append(cpu_time) 
    log_file[-1]['plans'].append(plan_id) 

    # dump log
    with open(log_path, 'w') as outfile:
        json.dump(log_file, outfile, indent=4)

class Plan(object):
    def __init__(self):
        self.sub  = rospy.Subscriber("rosplan_parsing_interface/complete_plan", CompletePlan, self.plan_callback)   
        self.sub2 = rospy.Subscriber("/rosplan_plan_dispatcher/action_dispatch", ActionDispatch, self.action_callback)   
        self.plan = CompletePlan()
        self.current_action = ActionDispatch()
    
    def plan_callback(self, data): 
        self.plan = data
        self.unsubscribe()

    def action_callback(self, data): 
        self.current_action = data

    def end_mission(self):
        print(self.plan.plan[-1].action_id)
        print(self.current_action.action_id)
        return self.plan.plan[-1].action_id == self.current_action.action_id

    def unsubscribe(self):
        self.sub.unregister()

def mission_planning(req):
    p = Plan()
    has_plan = False
    if(has_plan and p.end_mission): return True


    rospy.loginfo("Creating problem")
    if not call_problem_generator(): return None

    rate = rospy.Rate(10) # 1 Hz
    rate.sleep() # Sleeps for 1/rate sec

    rospy.loginfo("Calling Plan genarator")
    if not call_plan_generator(): return None

    has_plan = True

    rospy.loginfo("Calling Parser")
    log()
    if not call_parser(): return None

    rospy.loginfo("Call Dispach")
    rospy.loginfo(req)

    if not call_dispach(): 
        return None

    return True
'''
    Callers for ROSPlan Services
'''

def try_call_srv(topic, msg_ty):
    rospy.wait_for_service(topic)
    try:
        query_proxy = rospy.ServiceProxy(topic, msg_ty)
        query_proxy()
        return True
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

def call_problem_generator(): return try_call_srv('/rosplan_problem_interface/problem_generation_server', Empty)
def call_plan_generator():    return try_call_srv('/rosplan_planner_interface/planning_server', Empty)
def call_parser():            return try_call_srv('/rosplan_parsing_interface/parse_plan', Empty)
def call_dispach():            return try_call_srv('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)

# def call_dispach():
#     rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
#     try:
#         query_proxy = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
#         result = query_proxy()
#         return result.success
#     except rospy.ServiceException as e:
#         rospy.logerr(f"Service call failed: {e}")
#         # Should we return false here? I guess...
#         return None

def mission_planning_server():
    rospy.init_node('mission_planning_server')
    srv = rospy.Service('harpia/mission_planning', Empty, mission_planning)
    rospy.loginfo("Mission Planning Ready")
    srv.spin()

if __name__ == '__main__':
    mission_planning_server()
