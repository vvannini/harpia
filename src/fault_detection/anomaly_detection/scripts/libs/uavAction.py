import os
import joblib



import rospy
import actionlib
import sys
import math
import json
import time
import os
import rosnode, psutil
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import *
from rosplan_dispatch_msgs.srv import *
from diagnostic_msgs.msg import KeyValue

from std_msgs.msg import String

from harpia_msgs.msg import ChangeMission, MissionPlannerAction, MissionPlannerGoal, MissionPlannerFeedback, MissionPlannerResult, Mission, MissionPlannerActionGoal
from harpia_msgs.srv import PathPlanning


from mavros_msgs.msg import *
from mavros_msgs.srv import *

from sensor_msgs.msg import *
from geographic_msgs.msg import *
from geometry_msgs.msg import *

from actionlib_msgs.msg import GoalStatusArray, GoalID

import collections

KB_UPDATE_ADD_KNOWLEDGE = 0
KB_UPDATE_RM_KNOWLEDGE = 2
KB_UPDATE_ADD_GOAL = 1
KB_UPDATE_RM_GOAL = 3
KB_UPDATE_ADD_METRIC = 4

KB_ITEM_INSTANCE = 0
KB_ITEM_FACT = 1
KB_ITEM_FUNCTION = 2
KB_ITEM_EXPRESSION = 3
KB_ITEM_INEQUALITY = 4

OP_UPDATE_KNOWLEDGE_BASE = 0
OP_REPLAN                = 1
OP_ADD_RM_GOALS          = 2


CartesianPoint = collections.namedtuple("CartesianPoint", "x y")

absPath = os.path.dirname(__file__)
relPath = "dependencies"
filesPath = os.path.join(absPath,relPath)

class Drone(object):
    """
    Represents a drone and provides callbacks for handling global position,
    battery state, and waypoint reached events.

    Attributes:
    - sub_position (rospy.Subscriber): Subscriber for global position updates.
    - sub_battery (rospy.Subscriber): Subscriber for battery state updates.
    - sub_mission (rospy.Subscriber): Subscriber for waypoint reached events.
    - latitude (float): Latitude of the drone's global position.
    - longitude (float): Longitude of the drone's global position.
    - battery (float): Battery percentage of the drone.
    - current (int): Current waypoint sequence number.

    Methods:
    - global_position_callback(data): Callback for handling global position updates.
    - battery_state_callback(data): Callback for handling battery state updates.
    - reached_callback(data): Callback for handling waypoint reached events.
    """

    def __init__(self):
        """
        Initialize a Drone object with empty or None attributes.
        """
        self.sub_position = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.global_position_callback)
        self.sub_battery = rospy.Subscriber('mavros/battery', BatteryState, self.battery_state_callback)
        self.sub_mission = rospy.Subscriber('mavros/mission/reached', WaypointReached, self.reached_callback)
        self.latitude = None
        self.longitude = None
        self.battery = None
        self.current = None

    def global_position_callback(self, data):
        """
        Callback for handling global position updates.

        Parameters:
        - data: NavSatFix message containing global position information.
        """
        self.latitude = data.latitude
        self.longitude = data.longitude

    def battery_state_callback(self, data):
        """
        Callback for handling battery state updates.

        Parameters:
        - data: BatteryState message containing battery state information.
        """
        self.battery = data.percentage * 100

    def reached_callback(self, data):
        """
        Callback for handling waypoint reached events.

        Parameters:
        - data: WaypointReached message containing information about the reached waypoint.
        """
        self.current = data.wp_seq + 1


class MissionC(object):
    """
    Represents a mission controller for handling mission-related information.

    Attributes:
    - Mission_Sub (rospy.Subscriber): Subscriber for mission updates.
    - mission_info (Mission or None): Information about the current mission.

    Methods:
    - mission_callback(data): Callback for handling mission updates.
    - get_mission(): Get the current mission information.
    """

    def __init__(self):
        """
        Initialize a MissionC object with an empty or None mission_info attribute.
        """
        self.Mission_Sub = rospy.Subscriber('harpia/mission', Mission, self.mission_callback)
        self.mission_info = None

    def mission_callback(self, data):
        """
        Callback for handling mission updates.

        Parameters:
        - data: Mission message containing information about the mission.
        """
        self.mission_info = data

    def get_mission(self):
        """
        Get the current mission information.

        Returns:
        - Mission or None: Information about the current mission, or None if not available.
        """
        return self.mission_info



def wait_until(check, msg=None, rate=1):
    """
    Wait until the specified condition is met.

    Parameters:
    - check (function): A function that returns True when the desired condition is met.
    - msg (str, optional): An optional message to log while waiting.
    - rate (float, optional): Rate at which to check the condition in Hz.

    Returns:
    - bool: True if the condition is met, False if ROS is shutdown or an interrupt occurs.
    """
    rate = rospy.Rate(rate)

    while not check():
        if rospy.is_shutdown(): return False
        if msg is not None: rospy.loginfo(msg)
        rate.sleep()

    return True


def geo_to_cart(geo_point, geo_home):
    """
    Convert geographic coordinates to Cartesian coordinates.

    Parameters:
    - geo_point (GeoPoint): The geographic point to convert.
    - geo_home (GeoPoint): The home/reference point for the Cartesian coordinate system.

    Returns:
    - CartesianPoint: The corresponding Cartesian coordinates.
    """
    def calc_y(lat, lat_):
        return (lat - lat_) * (10000000.0 / 90)

    def calc_x(longi, longi_, lat_):
        return (longi - longi_) * (
            6400000.0 * (math.cos(lat_ * math.pi / 180) * 2 * math.pi / 360)
        )
    x = calc_x(geo_point.longitude, geo_home.longitude, geo_home.latitude)
    y = calc_y(geo_point.latitude, geo_home.latitude)

    return CartesianPoint(x, y)



def euclidean_distance(A, B):
    return math.sqrt((B.x - A.x) ** 2 + (B.y - A.y) ** 2)

def find_at(map, goals, latitude, longitude):
    """
    Finds a base or region within a 50m radius of the provided geographic coordinates.
    
    Parameters:
    - map (Map): The map containing bases, regions, and geo_home.
    - goals (List[Goal]): List of goals to determine region names.
    - latitude (float): The latitude of the current location.
    - longitude (float): The longitude of the current location.

    Returns:
    - Base or Region or None: The base or region within the specified radius, or None if not found.
    """
    # Get current latitude and longitude in Cartesian coordinates
    cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), map.geo_home)

    # Check bases
    for base in map.bases:
        d = euclidean_distance(base.center.cartesian, cart_location)
        if d <= 50:
            return base

    # Determine region names from goals
    region_names = {g.region for g in goals}

    # Check regions
    for region in map.roi:
        if region.name in region_names:
            d = euclidean_distance(cart_location, region.center.cartesian)
            if d <= 50:
                return region

    return None


def find_nearest_base(map, latitude, longitude):
    """
    Finds the nearest base from the provided geographic coordinates.

    Parameters:
    - map (Map): The map containing bases and geo_home.
    - latitude (float): The latitude of the current location.
    - longitude (float): The longitude of the current location.

    Returns:
    - Base: The nearest base from the specified coordinates.
    """
    # Convert current latitude and longitude to Cartesian coordinates
    cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), map.geo_home)

    # Find the nearest base based on Cartesian distance
    return min(map.bases, key=lambda base: euclidean_distance(base.center.cartesian, cart_location))


def call_path_planning(r_from, r_to, map):
    """
    Calls the path planning service to find a path from one region to another.

    Parameters:
    - r_from (Region): The starting region.
    - r_to (Region): The destination region.
    - map (Map): The map containing regions and other relevant information.

    Returns:
    - PathPlanningResponse or None: The response from the path planning service, or None if the service call fails.
    """
    rospy.wait_for_service("/harpia/path_planning")
    
    try:
        # Call the path planning service
        query_proxy = rospy.ServiceProxy("/harpia/path_planning", PathPlanning)
        resp = query_proxy(r_from.center, r_to.center, r_from.name, r_to.name, 3, map)
        return resp
    except rospy.ServiceException as e:
        # Log an error message if the service call fails
        rospy.logerr("Service call failed: %s" % e)
        return None

'''
    Callers for MAVRos Services
'''

def mavros_cmd(topic, msg_ty, error_msg="MAVROS command failed: ", **kwargs):
    """
    Sends a command to MAVROS via a service.

    Parameters:
    - topic (str): The topic name of the MAVROS service.
    - msg_ty (ROS Service Type): The ROS service type for the MAVROS command.
    - error_msg (str): Error message prefix to be logged if the MAVROS command fails.
    - **kwargs: Additional keyword arguments to be passed to the MAVROS service.

    Returns:
    - None
    """
    rospy.wait_for_service(topic)
    
    try:
        # Call the MAVROS service
        service_proxy = rospy.ServiceProxy(topic, msg_ty)
        response = service_proxy(**kwargs)
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        # Log an error message if the MAVROS command fails
        rospy.logerr(f"{error_msg} {e}")

def land():
    """
    Initiates a landing command using MAVROS.

    The function calls the MAVROS service for landing, providing necessary parameters.

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


def set_mode(mode):
    """
    Sets the flight mode of the drone using MAVROS.

    The function calls the MAVROS service to set the flight mode of the drone.

    Parameters:
    - mode (str): The desired flight mode to set.

    Returns:
    - None
    """
    mavros_cmd(
        '/mavros/set_mode',
        SetMode,
        error_msg="Set mode failed",
        custom_mode=mode
    )


def clear_mission():
    """
    Clears the mission waypoints using MAVROS.

    The function calls the MAVROS service to clear the mission waypoints of the drone.

    Parameters:
    - None

    Returns:
    - None
    """
    mavros_cmd(
        '/mavros/mission/clear',
        WaypointClear,
        error_msg="Clear mission failed"
    )


def send_route(route):
    """
    Sends a route to the drone using MAVROS.

    The function calls the MAVROS service to push a set of waypoints as a route to the drone.

    Parameters:
    - route (WaypointList): The route containing the waypoints to be sent.

    Returns:
    - None
    """
    mavros_cmd(
        '/mavros/mission/push',
        WaypointPush,
        error_msg="Send route failed",
        start_index=route.current_seq,
        waypoints=route.waypoints
    )


def go_to_base():
    """
    Go to the nearest base immediately and land.

    This function initiates the drone to go to the nearest base by finding the current location of the drone,
    determining the nearest base, calculating the path planning, and sending the route to the drone through MAVROS.
    After arriving at the base, the drone will land.

    Returns:
    - Base: The nearest base reached by the drone.
    """
    uav = Drone()
    mission_obj = MissionC()

    # Check if mission information is available
    if not wait_until(lambda: mission_obj.mission_info is not None, msg="Waiting for Mission..."):
        rospy.logerr("CRITICAL - MissionGoalManager was terminated while going to the nearest base and will not finish the action")
        return None

    # Check if UAV position is available
    if not wait_until(lambda: uav.latitude is not None, msg="Waiting for position..."):
        rospy.logerr("CRITICAL - MissionGoalManager was terminated while going to the nearest base and will not finish the action")
        return None

    # Find the current region or base
    at = find_at(mission_obj.mission_info.map, mission_obj.mission_info.goals, uav.latitude, uav.longitude)
    # Find the nearest base
    base = find_nearest_base(mission_obj.mission_info.map, uav.latitude, uav.longitude)
    rospy.sleep(5)

    # Calculate path planning to the nearest base
    route = call_path_planning(at, base, mission_obj.mission_info.map)

    # Send the calculated route to the UAV
    clear_mission()
    rospy.loginfo("Send Route")
    send_route(route.waypoints)

    # Set mode to mission
    rospy.loginfo("Set Mode")
    set_mode("AUTO.MISSION")

    # Wait until the drone arrives at the base
    uav.current = 0
    while uav.current < len(route.waypoints.waypoints):
        rospy.sleep(1)

    # Land the drone
    land()
    rospy.sleep(15)

    return base
