#!/usr/bin/env python3

import rospy

# Brings in the SimpleActionClient
import actionlib
from actionlib import GoalStatus

import sys
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

def get_harpia_root_dir():
    return rospy.get_param("/harpia_home", default=os.path.expanduser("~/harpia"))

def geo_to_cart(geo_point, geo_home):
    def calc_y(lat, lat_):
        return (lat - lat_) * (10000000.0 / 90)

    def calc_x(longi, longi_, lat_):
        return (longi - longi_) * (
            6400000.0 * (math.cos(lat_ * math.pi / 180) * 2 * math.pi / 360)
        )

    x = calc_x(geo_point.longitude, geo_home.longitude, geo_home.latitude)
    y = calc_y(geo_point.latitude, geo_home.latitude)

    return Point(x, y, geo_point.altitude)

def file_check_id(fname):
    if os.path.exists(fname):
        with open(fname, "r") as log_file:
            log_file = json.load(log_file)

        log_id = log_file[-1]["id"] + 1

        return log_id, log_file
    else:
      print("Creating File.")
      open(fname, "w")
      
      return 0, []

def write_log(log, log_file):
    with open(log_file, 'w') as outfile:
        json.dump(log, outfile, indent=4)
    outfile.close()


def total_goals(mission):
    total_goals = 0
    for step in mission["mission_execution"]:
        total_goals += 1
    return total_goals

def create_log(map, mission):
    log = {}
    log_path = os.path.join(LOG_DIR, "mission_log.json")
    id_log, log_file = file_check_id(log_path)

    log['id']          = id_log
    log['map_id']      = map['id']
    log['mission_id']  = mission['id']
    log['total_goals'] = total_goals(mission)
    log['replan']      = 0
    log['knn']         = [0, 0, 0]
    log['cpu_time']    = []
    log['bn_net']      = []
    log['plans']       = []

    log_file.append(log)
    print(log['id'])

    write_log(log_file, log_path)


def test_client(hardware, map, mission):
    """
    Creates a MissionPlannerGoal message from mission
    data with op = 2. Sends the message to the action server
    "harpia/mission_goal_manager" and then publishes to "/harpia/mission" while
    the action server state is < 2.
    """

    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('harpia/mission_goal_manager', MissionPlannerAction)
    pub = rospy.Publisher("/harpia/mission", Mission, queue_size=100)

    create_log(map, mission)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()


    # Creates a goal to send to the action server.
    goal = MissionPlannerGoal()
    goal.op = 1 # first add in the kwoledge base
    goal.mission = get_objects(hardware, map, mission)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    # client.wait_for_result()

    rate = rospy.Rate(10)
    print(client.get_state())
    print([GoalStatus.PENDING, GoalStatus.ACTIVE])
    while client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
        pub.publish(goal.mission)

        if rospy.is_shutdown():
            rospy.logwarn("Shutting down test_client before mission completion")
            break

        rate.sleep()

    # Prints out the result of executing the action
    return client.get_result()

'''
    Hello again my brave friend,
    I know this entire code it's not ideal, but it'sa try,

   get_regions -> is an auxiliar function to create a list of a certain type of regions, as bases, nfz and roi read from the json file
   get_uav -> same as regions but for the uav
   get_map -> same as regions but for the map
   get_goals -> same as regions but for the goals

   get_objects -> call others functions to create a mission object

'''

def get_regions(tag, home):
    region_list = []
    for r in map[tag]:
        region = Region()

        region.id = r["id"]
        region.name = r["name"]
        region.center.geo.latitude = r["center"][1]
        region.center.geo.longitude = r["center"][0]
        region.center.geo.altitude = r["center"][2]

        region.center.cartesian = geo_to_cart(region.center.geo, home)



        for p in r["geo_points"]:
            point = RegionPoint()

            point.geo.latitude = p[1]
            point.geo.longitude =p[0]
            point.geo.altitude = p[2]

            point.cartesian = geo_to_cart(point.geo, home)

            region.points.append(point)
        region_list.append(region)
    return region_list

def get_uav(hardware):
    uav = UAV()
    uav.id = hardware['id']
    uav.name = hardware['name']

    uav.camera.name = hardware["camera"]['name']
    uav.camera.open_angle.x = hardware["camera"]['open_angle']['x']
    uav.camera.open_angle.y = hardware["camera"]['open_angle']['y']
    uav.camera.sensor.x = hardware["camera"]['sensor']['x']
    uav.camera.sensor.y = hardware["camera"]['sensor']['y']
    uav.camera.resolution.x = hardware["camera"]['resolution']['x']
    uav.camera.resolution.y = hardware["camera"]['resolution']['y']

    uav.camera.max_zoom = hardware["camera"]['max_zoom']
    uav.camera.mega_pixel = hardware["camera"]['mega_pixel']
    uav.camera.focus_distance = hardware["camera"]['focus_distance']
    uav.camera.shutter_time = hardware["camera"]['shutter_time']
    uav.camera.trigger = hardware["camera"]['trigger']
    uav.camera.weight = hardware["camera"]['weight']

    uav.battery.amp = hardware['battery']['amp']
    uav.battery.voltage = hardware['battery']['voltage']
    uav.battery.cells = hardware['battery']['cells']
    uav.battery.min = hardware['battery']['min']
    uav.battery.max = hardware['battery']['max']
    uav.battery.capacity = hardware['battery']['capacity']
    uav.battery.recharge_rate = hardware['battery']['recharge_rate']
    uav.battery.discharge_rate = hardware['battery']['discharge_rate']

    uav.frame.type = hardware['frame']['type']
    uav.frame.weight = hardware['frame']['weight']
    uav.frame.max_velocity = hardware['frame']['max_velocity']
    uav.frame.efficient_velocity = hardware['frame']['efficient_velocity']

    uav.input_capacity = hardware['input_capacity']

    return uav

def get_map(map):
    m = Map()
    m.id = map["id"]
    # mission.map.name =  map['name']

    home = GeoPoint()

    home.latitude = map["geo_home"][1]
    home.longitude = map["geo_home"][0]
    home.altitude = map["geo_home"][2]

    m.geo_home = home
    m.roi = get_regions("roi", home)
    m.nfz = get_regions("nfz", home)
    m.bases = get_regions("bases", home)

    return m

def get_goals(mission):
    goals = []

    for g in mission["mission_execution"]:
        goal =  Goal()
        goal.action = g["command"]
        goal.region = g["instructions"]["area"]
        goals.append(goal) 

    return goals

def get_objects(hardware, map, goals):
    mission = Mission()
    mission.uav = get_uav(hardware)
    mission.map = get_map(map)
    mission.goals = get_goals(goals)

    return mission

def parse_args():
    parser = argparse.ArgumentParser(description="Runs a mission with a map")

    parser.add_argument("mission_id" , type=int, help="Path to the json mission description")
    parser.add_argument("map_id"     , type=int, help="Path to the json map description")
    parser.add_argument("hardware_id", type=int, help="Path to the json hardware description")

    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()

    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    # os.system("rosnode kill test_client")        
    rospy.init_node('test_client1')

    harpia_root = get_harpia_root_dir()

    PATH    = os.path.join(harpia_root, "json/")
    LOG_DIR = os.path.join(harpia_root, "results/")

    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)

    #get file names
    map_filename     = os.path.join(PATH, "mapa.json")
    mission_filename = os.path.join(PATH, "missao.json")
    hw_filename      = os.path.join(PATH, "hardware.json")

    with open(mission_filename, "r") as mission_file:
        mission_file = json.load(mission_file)
        mission = mission_file[args.mission_id]

    with open(map_filename, "r") as map_file:
        map_file = json.load(map_file)
        map = map_file[args.map_id]

    with open(hw_filename, "r") as hw_file:
        hw_file = json.load(hw_file)
        hardware = hw_file[args.hardware_id]

    try:
        result = test_client(hardware, map, mission)
        rospy.loginfo(f"Result: {result}")
    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
