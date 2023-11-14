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

from mavros_msgs.srv import CommandHome

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from harpia_msgs.msg import *

feedback = 0

def get_harpia_root_dir():
    """
    Get the root directory path for the Harpia system.

    This function uses rospy.get_param to retrieve the value associated with the key "/harpia_home".
    If the parameter is not set, it defaults to the expanded user home directory with the subdirectory "harpia".

    Parameters:
    None

    Returns:
    str: The path to the Harpia root directory.
    """
    return rospy.get_param("/harpia_home", default=os.path.expanduser("~/harpia"))

def geo_to_cart(geo_point, geo_home):
    """
    Convert a geographical (latitude, longitude, altitude) point to Cartesian coordinates (x, y, z) relative to a home point.

    Parameters:
    - geo_point (GeographicPoint): The geographical point to be converted.
    - geo_home (GeographicPoint): The geographical home point used as the reference.

    Returns:
    Point: The Cartesian coordinates (x, y, z) relative to the home point.
    """
    def calc_y(lat, lat_home):
        return (lat - lat_home) * (10000000.0 / 90)

    def calc_x(longi, longi_home, lat_home):
        return (longi - longi_home) * (
            6400000.0 * (math.cos(lat_home * math.pi / 180) * 2 * math.pi / 360)
        )

    x = calc_x(geo_point.longitude, geo_home.longitude, geo_home.latitude)
    y = calc_y(geo_point.latitude, geo_home.latitude)

    return Point(x, y, geo_point.altitude)

def file_check_id(fname):
    """
    Check if a file with the given filename exists. If it exists, read the file,
    extract the last entry's ID, increment it, and return the incremented ID along with the entire file contents.
    If the file doesn't exist, create an empty file and return 0 as the initial ID along with an empty list.

    Parameters:
    - fname (str): The filename to check.

    Returns:
    Tuple[int, List[Dict]]: A tuple containing the incremented ID and the file contents.
    """
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
    """
    Write the given log to a file with the provided filename.

    Parameters:
    - log (dict): The log data to be written.
    - log_file (str): The filename to write the log to.
    """
    with open(log_file, 'w') as outfile:
        json.dump(log, outfile, indent=4)

def total_goals(mission):
    """
    Calculate the total number of goals in the mission execution.

    Parameters:
    - mission (dict): The mission data containing the "mission_execution" list.

    Returns:
    - int: The total number of goals in the mission execution.
    """
    total_goals = 0
    for step in mission["mission_execution"]:
        total_goals += 1
    return total_goals


def create_log(map, mission):
    """
    Create a log entry for a mission.

    Parameters:
    - map (dict): The map data.
    - mission (dict): The mission data.

    Returns:
    - None
    """
    log = {}
    log_path = os.path.join(LOG_DIR, "mission_log.json")
    id_log, log_file = file_check_id(log_path)

    log['id']          = id_log
    log['map_id']      = map['id']
    log['mission_id']  = mission['id']
    log['total_goals'] = total_goals(mission)
    log['replan']      = 0
    log['knn']         = []
    log['cpu_time']    = []
    log['bn_net']      = []
    log['plans']       = []
    log['detected_fault'] = []

    log_file.append(log)
    print(log['id'])

    write_log(log_file, log_path)


def feedback_callback(feedback_msg):
    """
    Callback function to handle feedback messages.

    Parameters:
    - feedback_msg: The feedback message.

    Returns:
    - None
    """
    global feedback
    feedback = feedback_msg.status
    # You can uncomment the following line if you want to print the received feedback
    # print('Received feedback: {}'.format(feedback))
    # Note: Uncommenting the above line may result in printing messages during execution.
    # Uncomment based on your debugging needs.
    # return feedback

def test_client(hardware, map, mission, mission_file):
    """
    Test client for interacting with a mission goal manager action server.

    Parameters:
    - hardware (Hardware): An object representing hardware information.
    - map (Map): An object representing map information.
    - mission (Mission): An object representing mission information.
    - mission_file (list): A list containing mission details.

    Returns:
    - The result of executing the action.

    Notes:
        Creates a MissionPlannerGoal message from the hardware, map and mission
        data with op = 0. Sends the message to the action server
        "harpia/mission_goal_manager" and then publishes to "/harpia/mission" while
        the action server state is < 2.
    """

    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('harpia/mission_goal_manager', MissionPlannerAction)
    pub = rospy.Publisher("/harpia/mission", Mission, queue_size=100)
    pub2 = rospy.Publisher("/harpia/ChangeMission", ChangeMission, queue_size=100)
    pub3 = rospy.Publisher("/harpia/uav", UAV, queue_size=100)
    # pub3 = rospy.Publisher("/harpia/Uav", UAV, queue_size=100)

    create_log(map, mission)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()


    # Creates a goal to send to the action server.
    goal = MissionPlannerGoal()
    goal.op = 0 # first add in the kwoledge base
    goal.mission = get_objects(hardware, map, mission)

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=feedback_callback)

    set_home_position(goal.mission.uav.home.latitude, goal.mission.uav.home.longitude, goal.mission.uav.home.altitude)

    # Waits for the server to finish performing the action.
    # client.wait_for_result()
    print("AD D GOAL   -> 1")
    print("REMOVE GOAL -> 2")
    print("mission_id goal_op")
    rate = rospy.Rate(1)
    while client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
        pub.publish(goal.mission)
        pub3.publish(goal.mission.uav)
        i, o, e = select.select([sys.stdin], [], [], 1)
        if(i):
            mission_id, goal_op = sys.stdin.readline().strip().split()
            msg = ChangeMission()
            msg.op = int(goal_op)
            # print( mission_file[int(mission_id)])
            msg.goals = get_goals(mission_file[int(mission_id)])

            print("feedback", feedback)
            while feedback == 0:
                # print(feedback)
                pub2.publish(msg)
                pub.publish(goal.mission)
                # rate.sleep()
                
            msg.op = 0


            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub2.publish(msg)
            pub.publish(goal.mission)
            pub.publish(goal.mission)
            pub.publish(goal.mission)
            pub.publish(goal.mission)
            pub.publish(goal.mission)

            print("Mission: " + str(mission_id)+ " Operation: "+str(goal_op))
            
            # mission = mission_file[int(mission_id)]
            # print(mission)
            # goal.mission = get_objects(hardware, map, mission)
            # client.send_goal(goal)
            # pub.publish(goal.mission)

        if rospy.is_shutdown():
            rospy.logwarn("Shutting down test_client before mission completion")
            break

        rate.sleep()

    # Prints out the result of executing the action
    return client.get_result()


def get_regions(tag, home):
    """
    Generate a list of Region objects based on the provided map data.

    Parameters:
    - tag (str): The tag identifying the specific map in the map dictionary.
    - home (GeoPoint): The home location for coordinate conversion.

    Returns:
    - list: A list of Region objects containing region details.
    """

    region_list = []

    # Iterate through regions in the specified map
    for r in map[tag]:
        region = Region()

        # Assign region details from the map data
        region.id = r["id"]
        region.name = r["name"]
        region.center.geo.latitude = r["center"][1]
        region.center.geo.longitude = r["center"][0]
        region.center.geo.altitude = r["center"][2]

        # Convert center coordinates to Cartesian coordinates
        region.center.cartesian = geo_to_cart(region.center.geo, home)

        # Iterate through geo_points in the region
        for p in r["geo_points"]:
            point = RegionPoint()

            # Assign point details from the map data
            point.geo.latitude = p[1]
            point.geo.longitude = p[0]
            point.geo.altitude = p[2]

            # Convert point coordinates to Cartesian coordinates
            point.cartesian = geo_to_cart(point.geo, home)

            # Append the point to the region's points list
            region.points.append(point)

        # Append the region to the list of regions
        region_list.append(region)

    return region_list


def get_uav(hardware):
    """
    Generate a UAV (Unmanned Aerial Vehicle) object based on the provided hardware data.

    Parameters:
    - hardware (dict): A dictionary containing hardware specifications for the UAV.

    Returns:
    - UAV: A UAV object containing UAV details.
    """

    uav = UAV()

    # Assign UAV details from the hardware data
    uav.id = hardware['id']
    uav.name = hardware['name']

    # Assign camera details from the hardware data
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

    # Assign battery details from the hardware data
    uav.battery.amp = hardware['battery']['amp']
    uav.battery.voltage = hardware['battery']['voltage']
    uav.battery.cells = hardware['battery']['cells']
    uav.battery.min = hardware['battery']['min']
    uav.battery.max = hardware['battery']['max']
    uav.battery.capacity = hardware['battery']['capacity']
    uav.battery.recharge_rate = hardware['battery']['recharge_rate']
    uav.battery.discharge_rate = hardware['battery']['discharge_rate']

    # Assign frame details from the hardware data
    uav.frame.type = hardware['frame']['type']
    uav.frame.weight = hardware['frame']['weight']
    uav.frame.max_velocity = hardware['frame']['max_velocity']
    uav.frame.efficient_velocity = hardware['frame']['efficient_velocity']

    uav.input_capacity = hardware['input_capacity']

    # Assign fault settings from the hardware data
    uav.fault_settings.user_response = hardware['fault_settings']['user_response']
    uav.fault_settings.classifier_time = hardware['fault_settings']['classifier_time']
    uav.fault_settings.action_time = hardware['fault_settings']['action_time']

    # Assign home location details from the hardware data
    uav.home.latitude = hardware['home']['lat']
    uav.home.longitude = hardware['home']['lon']
    uav.home.altitude = hardware['home']['alt']

    return uav


def get_map(map):
    """
    Generate a Map object based on the provided map data.

    Parameters:
    - map (dict): A dictionary containing map specifications.

    Returns:
    - Map: A Map object containing map details.
    """

    m = Map()

    # Assign map details from the map data
    m.id = map["id"]

    # Create a GeoPoint for the home location
    home = GeoPoint()
    home.latitude = map["geo_home"][1]
    home.longitude = map["geo_home"][0]
    home.altitude = map["geo_home"][2]

    # Assign home location and regions details to the Map object
    m.geo_home = home
    m.roi = get_regions("roi", home)
    m.nfz = get_regions("nfz", home)
    m.bases = get_regions("bases", home)

    return m


def get_goals(mission):
    """
    Generate a list of Goal objects based on the provided mission data.

    Parameters:
    - mission (dict): A dictionary containing mission specifications.

    Returns:
    - list: A list of Goal objects representing the mission goals.
    """

    goals = []

    # Iterate over mission goals and create Goal objects
    for g in mission["mission_execution"]:
        goal = Goal()
        goal.action = g["command"]
        goal.region = g["instructions"]["area"]
        goals.append(goal)

    return goals


def get_objects(hardware, map, goals):
    """
    Generate a Mission object based on the provided hardware, map, and mission goals.

    Parameters:
    - hardware (dict): A dictionary containing hardware specifications.
    - map (dict): A dictionary containing map specifications.
    - goals (dict): A dictionary containing mission goals.

    Returns:
    - Mission: A Mission object representing the complete mission configuration.
    """

    mission = Mission()
    mission.uav = get_uav(hardware)
    mission.map = get_map(map)
    mission.goals = get_goals(goals)

    return mission



def set_home_position(latitude, longitude, altitude, current_gps=True, yaw=0):
    """
    Set the home position for the UAV.

    Parameters:
    - latitude (float): The latitude of the home position.
    - longitude (float): The longitude of the home position.
    - altitude (float): The altitude of the home position.
    - current_gps (bool): If True, use the current GPS position as the home.
    - yaw (float): The yaw orientation at the home position.

    Returns:
    None
    """
    # Wait for the required services to become available
    rospy.wait_for_service('/mavros/cmd/set_home')

    try:
        # Create a proxy for the CommandHome service
        set_home_service = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)

        # Create a CommandHome request with the desired home position
        home_request = CommandHome()

        # Call the service to set the new home position
        response = set_home_service.call(current_gps, yaw, latitude, longitude, altitude)

        if response.success:
            rospy.loginfo("Home position set successfully!")
        else:
            rospy.logerr("Failed to set home position: %s", response.result)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def parse_args():
    """
    Parse command-line arguments for running a mission with a map.

    Returns:
    argparse.Namespace: An object containing parsed arguments.
    """
    parser = argparse.ArgumentParser(description="Runs a mission with a map")

    # Define command-line arguments
    parser.add_argument("mission_id", type=int, help="Path to the json mission description")
    parser.add_argument("map_id", type=int, help="Path to the json map description")
    parser.add_argument("hardware_id", type=int, help="Path to the json hardware description")

    # Parse and return the arguments
    return parser.parse_args()


if __name__ == '__main__':
    # Parse command-line arguments
    args = parse_args()

    # Initialize a rospy node for SimpleActionClient to publish and subscribe over ROS
    rospy.init_node('test_client')

    # Get the root directory for Harpia
    harpia_root = get_harpia_root_dir()

    # Set paths for JSON files and results
    PATH = os.path.join(harpia_root, "json/")
    LOG_DIR = os.path.join(harpia_root, "results/")

    # Create the results directory if it doesn't exist
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)

    # Get file names
    map_filename = os.path.join(PATH, "mapa.json")
    mission_filename = os.path.join(PATH, "missao.json")
    hw_filename = os.path.join(PATH, "hardware.json")

    # Read JSON files
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
        # Run the test client with provided hardware, map, and mission data
        result = test_client(hardware, map, mission, mission_file)
        rospy.loginfo(f"Result: {result}")
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted before completion")
