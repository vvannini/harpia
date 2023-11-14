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
    """
    Retrieves the root directory path for the Harpia project.

    The root directory is obtained from the ROS parameter '/harpia_home'. If the
    parameter is not set, the default path is set to the user's home directory
    followed by '/harpia'.

    Returns:
        str: The path to the Harpia root directory.
    """
    return rospy.get_param("/harpia_home", default=os.path.expanduser("~/harpia"))

def parse_file_plan():
    """
    Parses the PDDL plan file generated during mission execution.

    Reads the PDDL plan file located at '/harpia/pddl/plan.pddl', extracts
    relevant information such as success, CPU time, and total time. It also
    saves a copy of the plan file in the '/harpia/results/plans/' directory.

    Returns:
        tuple: A tuple containing the following information:
            - bool: True if a solution is found, False otherwise.
            - float: The CPU time for plan execution.
            - float: The total time calculated from the plan.
            - int: Unique identifier for the plan log.
    """
    root_dir = get_harpia_root_dir()
    plan_path = os.path.join(root_dir, 'pddl/plan.pddl')

    success = False
    cpu_time = float('Inf')
    total_time = 0

    for line in open(plan_path):
        if 'Solution Found' in line:
            success = True
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

def log(cpu_time):
    """
    Updates the mission log with information about the current mission execution.

    Reads the existing mission log file at '/harpia/results/mission_log.json',
    extracts relevant information from the PDDL plan, and updates the log with
    the CPU time, success status, total time, and plan identifier.

    Parameters:
        cpu_time (float): The CPU time for the current mission execution.

    Returns:
        None
    """
    # log path
    root_dir = get_harpia_root_dir()
    log_path = os.path.join(root_dir, "results/mission_log.json")

    # open log
    with open(log_path, "r") as log_file:
        log_file = json.load(log_file)

    success, _, total_time, plan_id = parse_file_plan()
    
    # add replan
    log_file[-1]['cpu_time'].append(cpu_time)
    log_file[-1]['plans'].append(plan_id)

    # dump log
    with open(log_path, 'w') as outfile:
        json.dump(log_file, outfile, indent=4)

class Plan(object):
    """
    Represents a ROS plan for a mission and provides callbacks for updating the plan.

    Attributes:
        sub (rospy.Subscriber): ROS subscriber for the complete plan topic.
        sub2 (rospy.Subscriber): ROS subscriber for the action dispatch topic.
        plan (CompletePlan): The complete plan for the mission.
        current_action (ActionDispatch): The current action being executed in the mission.
    """

    def __init__(self):
        """
        Initializes the Plan object with ROS subscribers and plan-related attributes.
        """
        self.sub = rospy.Subscriber("rosplan_parsing_interface/complete_plan", CompletePlan, self.plan_callback)
        self.sub2 = rospy.Subscriber("/rosplan_plan_dispatcher/action_dispatch", ActionDispatch, self.action_callback)
        self.plan = CompletePlan()
        self.current_action = ActionDispatch()

    def plan_callback(self, data):
        """
        Callback function for updating the plan when a new complete plan is received.

        Parameters:
            data (CompletePlan): The new complete plan received.
        """
        self.plan = data
        self.unsubscribe()

    def action_callback(self, data):
        """
        Callback function for updating the current action being executed.

        Parameters:
            data (ActionDispatch): The action dispatch information for the current action.
        """
        self.current_action = data

    def end_mission(self):
        """
        Checks if the mission has reached its end based on the comparison of action IDs.

        Returns:
            bool: True if the plan's last action ID matches the current action's ID, indicating
                  the end of the mission; False otherwise.
        """
        return self.plan.plan[-1].action_id == self.current_action.action_id

    def unsubscribe(self):
        """
        Unsubscribes from the complete plan topic to stop receiving plan updates.
        """
        self.sub.unregister()

def mission_planning(req):
    """
    Performs mission planning by generating a problem, calling the plan generator,
    parsing the generated plan, and dispatching the plan for execution.

    Parameters:
        req:.

    Returns:
        bool or None: True if the mission planning is successful, None otherwise.
    """
    p = Plan()
    has_plan = False

    # Check if a plan exists and the mission has ended
    if has_plan and p.end_mission:
        return True

    start = time.time()
    rospy.loginfo("Creating problem")

    # Call the problem generator
    if not call_problem_generator():
        return None

    rate = rospy.Rate(10)  # 1 Hz
    rate.sleep()  # Sleeps for 1/rate sec

    rospy.loginfo("Calling Plan generator")

    # Call the plan generator
    if not call_plan_generator():
        return None

    has_plan = True

    rospy.loginfo("Calling Parser")

    # Call the parser
    if not call_parser():
        return None

    end = time.time()
    log(round((end - start), 3))
    rospy.loginfo("Call Dispatch")
    rospy.loginfo(req)

    # Call the dispatch
    if not call_dispatch():
        return None

    return True

'''
    Callers for ROSPlan Services
'''

def try_call_srv(topic, msg_ty):
    """
    Attempts to call a ROS service.

    Parameters:
        topic (str): The topic of the ROS service.
        msg_ty (Message): The type of the ROS service.

    Returns:
        bool or None: True if the service call is successful, None otherwise.
    """
    rospy.wait_for_service(topic)
    try:
        query_proxy = rospy.ServiceProxy(topic, msg_ty)
        query_proxy()
        return True
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

def call_problem_generator():
    """
    Calls the ROS service for problem generation.

    Returns:
        bool or None: True if the service call is successful, None otherwise.
    """
    return try_call_srv('/rosplan_problem_interface/problem_generation_server', Empty)

def call_plan_generator():
    """
    Calls the ROS service for plan generation.

    Returns:
        bool or None: True if the service call is successful, None otherwise.
    """
    return try_call_srv('/rosplan_planner_interface/planning_server', Empty)

def call_parser():
    """
    Calls the ROS service for plan parsing.

    Returns:
        bool or None: True if the service call is successful, None otherwise.
    """
    return try_call_srv('/rosplan_parsing_interface/parse_plan', Empty)

def call_dispatch():
    """
    Calls the ROS service for plan dispatch.

    Returns:
        bool or None: True if the service call is successful, None otherwise.
    """
    return try_call_srv('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)


def mission_planning_server():
    """
    Initializes a ROS node and sets up a service for mission planning.

    The service '/harpia/mission_planning' is created using the `mission_planning` function.

    Returns:
        None
    """
    rospy.init_node('mission_planning_server')
    srv = rospy.Service('harpia/mission_planning', Empty, mission_planning)
    rospy.loginfo("Mission Planning Ready")
    srv.spin()

if __name__ == '__main__':
    mission_planning_server()
