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



absPath = os.path.dirname(__file__)
relPath = "dependencies"
filesPath = os.path.join(absPath,relPath)

class Drone(object):
    def __init__(self):
        self.sub_position = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.global_position_callback)
        self.sub_battery  = rospy.Subscriber('mavros/battery', BatteryState, self.battery_state_callback)
        self.sub_mission  = rospy.Subscriber('mavros/mission/reached', WaypointReached, self.reached_callback)
        self.latitude     = None
        self.longitude    = None
        self.battery      = None
        self.current      = None


    def global_position_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude

    def battery_state_callback(self, data):
        self.battery = data.percentage * 100

    def reached_callback(self, data):
        self.current = data.wp_seq + 1

class Mission(object):

    def __init__(self):
        self.Mission_Sub  = rospy.Subscriber('/harpia/mission_goal_manager/goal', MissionPlannerActionGoal, self.mission_callback)
        self.mission = None

    def mission_callback(self, data):
        self.mission = data.goal.mission
        
    def get_mission(self):
        return self.mission


def wait_until(check, msg=None, rate=1):
    rate = rospy.Rate(rate)

    while not check():
        if rospy.is_shutdown(): return False
        if msg is not None: rospy.loginfo(msg)
        rate.sleep()

    return True

def find_at(map, goals, latitude, longitude):
    """
    Finds a base that is withing a 50m radius. If none is found, try to find a region which has a center
    within the same range. If none is found, return `None`. If there is one of these waypoints in range,
    the drone is considered at that waypoint.
    """

    #get current lat long
    cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), map.geo_home)

    for base in map.bases:
        d = euclidean_distance(base.center.cartesian, cart_location)
        # rospy.loginfo(f"base = {base.name} distance={d}")
        if d <= 50:
            return base

    region_names = { g.region for g in goals }

    for region in map.roi:
        if region.name in region_names:
            d = euclidean_distance(cart_location, region.center.cartesian)
            # rospy.loginfo(f"base = {region.name} distance={d}")
            if d <= 50:
                return region

    return None


def find_nearest_base(map, latitude, longitude):
    cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), map.geo_home)

    return min(map.bases, key=lambda base: euclidean_distance(base.center.cartesian, cart_location))


def call_path_planning(r_from, r_to, map):
    rospy.wait_for_service("/harpia/path_planning")
    try:
        query_proxy = rospy.ServiceProxy("/harpia/path_planning", PathPlanning)
        resp = query_proxy(r_from.center, r_to.center, r_from.name, r_to.name, 3, map)
        return resp
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None

'''
    Callers for MAVRos Services
'''

def mavros_cmd(topic, msg_ty, error_msg="MAVROS command failed: ", **kwargs):
    rospy.wait_for_service(topic)
    try:
        service_proxy = rospy.ServiceProxy(topic, msg_ty)
        response = service_proxy(**kwargs)
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        rospy.logerr(f"{error_msg} {e}")

def land():
    mavros_cmd(
        '/mavros/cmd/land',
        CommandTOL,
        error_msg="Landing failed",
        altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0
    )

def set_mode(mode):
    mavros_cmd(
        '/mavros/set_mode',
        SetMode,
        error_msg="Set mode failed",
        custom_mode=mode
    )

def clear_mission():
    mavros_cmd(
        '/mavros/mission/clear',
        WaypointClear,
        error_msg="Clear mission failed"
    )

def send_route(route):
    mavros_cmd(
        '/mavros/mission/push',
        WaypointPush,
        error_msg="Send route failed",
        start_index=route.current_seq,
        waypoints=route.waypoints
    )

def go_to_base():
    """
    Go to nearest base immediately and land.
    """
    uav = Drone()
    mission_obj = Mission()
    mission = mission_obj.get_mission()
    print(mission)

    rate = rospy.Rate(1)

    if not wait_until(lambda: uav.latitude is not None, msg="Waiting for position..."):
        rospy.logerr("CRITICAL - MissionGoalManager was terminated while going to nearest base and will not finish the action")
        return

    at = find_at(mission.map, mission.goals, uav.latitude, uav.longitude)
    base = find_nearest_base(mission.map, uav.latitude, uav.longitude)

    route = call_path_planning(at, base, mission.map)

    # send route to uav
    clear_mission()
    rospy.loginfo("Send Route")
    send_route(route.waypoints)

    # set mode to mission
    rospy.loginfo("Set Mode")
    set_mode("AUTO.MISSION")

    # wait to arrive.
    uav.current = 0
    while uav.current < len(route.waypoints.waypoints):
        rospy.sleep(1)

    # land
    land()
    rospy.sleep(30)

    return base