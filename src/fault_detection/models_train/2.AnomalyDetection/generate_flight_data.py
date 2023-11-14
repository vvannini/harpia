#!/usr/bin/env python3

import rospy
import pyproj
from shapely.geometry import mapping, Point as PointGeometry
from shapely.ops import transform
from functools import partial
from std_msgs.msg import String
from mavros_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from harpia_msgs.msg import *
from sensor_msgs.msg import NavSatFix, Imu, BatteryState

from mavros_msgs.srv import *

import sys
sys.path.append('../../anomaly_detection/scripts')

# --- 
# from libs.anomalyDetection import findAnomaly, clusters, checkValues
from libs import anomalyDetection,clustering,loadData, NoiseGenerator
import time
import math

#---
import numpy
import json

table = [
    {"Type": "Takeoff and landing", "Takeoff Alt": 5},
    {"Type": "Takeoff and landing", "Takeoff Alt": 15},
    {"Type": "Takeoff and landing", "Takeoff Alt": 30},
    {"Type": "Hovering", "Hovering Alt": 5},
    {"Type": "Hovering", "Hovering Alt": 15},
    {"Type": "Hovering", "Hovering Alt": 30},
    {"Type": "Line flight", "Direction": "F", "Distance": 10, "Start Alt": 15, "End Alt": 15},
    {"Type": "Line flight", "Direction": "F", "Distance": 10, "Start Alt": 15, "End Alt": 10},
    {"Type": "Line flight", "Direction": "F", "Distance": 15, "Start Alt": 50, "End Alt": 30},
    {"Type": "Line flight", "Direction": "F", "Distance": 5, "Start Alt": 7, "End Alt": 5},
    {"Type": "Line flight", "Direction": "B","Distance": 10, "Start Alt": 15, "End Alt": 10},
    {"Type": "Line flight", "Direction": "B","Distance": 15, "Start Alt": 50, "End Alt": 30},
    {"Type": "Line flight", "Direction": "B","Distance": 5, "Start Alt": 7, "End Alt": 5},
    {"Type": "Line flight", "Direction": "L", "Distance": 10, "Start Alt": 15, "End Alt": 10},
    {"Type": "Line flight", "Direction": "L", "Distance": 15, "Start Alt": 50, "End Alt": 30},
    {"Type": "Line flight", "Direction": "L", "Distance": 5, "Start Alt": 7, "End Alt": 5},
    {"Type": "Line flight", "Direction": "R", "Distance": 10, "Start Alt": 15, "End Alt": 10},
    {"Type": "Line flight", "Direction": "R", "Distance": 15, "Start Alt": 50, "End Alt": 30},
    {"Type": "Line flight", "Direction": "R", "Distance": 5, "Start Alt": 7, "End Alt": 5},
    {"Type": "Circular flight", "Radius": 5, "Wind speed": None},
    {"Type": "Circular flight", "Radius": 25, "Wind speed": None},
    {"Type": "Circular flight", "Radius": 50, "Wind speed": None},
    {"Type": "8", "Radius": 50, "Start Alt": 7, "End Alt": 5},
    {"Type": "alt", "delta_Alt": 5}

]


def get_harpia_root_dir():
    """
    Get the root directory of the Harpia system.

    This function retrieves the root directory of the Harpia system using the ROS parameter '/harpia_home'. 
    If the parameter is not found, the default root directory is set to the user's home directory with the 'harpia' folder.

    Returns:
        str: The absolute path to the Harpia root directory.

    Example:
        root_dir = get_harpia_root_dir()
        print(f"Harpia root directory: {root_dir}")
    """
    return rospy.get_param("/harpia_home", default=os.path.expanduser("~/harpia"))


def sequential_noise(data):
      """
    Generate sequential noisy data for various UAV parameters.

    Parameters:
        data (dict): A dictionary containing UAV parameters.

    Returns:
        dict: A dictionary with keys representing UAV parameters and values containing lists of sequential noisy data.

    Example:
        # Assuming 'uav_data' is a dictionary containing UAV parameters
        noisy_sequence = sequential_noise(uav_data)
        print(f"Noisy sequence for roll: {noisy_sequence['roll']}")
    """
    sequential = {'roll':[],
                          'pitch':[],
                          'yaw':[],
                          'heading':[], #yaw
                          'rollRate':[],
                          'pitchRate':[],
                          'yawRate':[],
                          'groundSpeed':[],
                          'climbRate':0, # ?
                          'altitudeRelative':[],
                          'throttlePct':[]}
    for key in sequential:
        sequential[key] = NoiseGenerator.noisyData(1,data,key, 1., 50000.)

    return sequential

def go_to_base(mission, uav):
    """
    Go to the nearest base immediately and land.

    Parameters:
        mission (Mission): An object representing the mission with map, goals, and other details.
        uav (UAV): An object representing the UAV with latitude, longitude, and other details.

    Returns:
        Base: An object representing the nearest base reached by the UAV.

    Example:
        # Assuming 'current_mission' is an instance of the Mission class
        # and 'current_uav' is an instance of the UAV class
        base_reached = go_to_base(current_mission, current_uav)
        print(f"UAV reached the nearest base: {base_reached.name}")
    """

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

# Classes
    

class UAV(object):
    """
    Class representing a UAV (Unmanned Aerial Vehicle) with ROS (Robot Operating System) integration.

    Attributes:
        sequential (dict): Dictionary containing sequential data such as roll, pitch, yaw, etc.
        noise (dict): Dictionary containing noise data generated from the sequential data.
        armed (bool): Indicates whether the UAV is armed.
        land_ex (float): Extended state variable indicating the UAV's landing state.
        alt (float): Altitude of the UAV.
        lat (float): Latitude of the UAV.
        lon (float): Longitude of the UAV.
        mode (str): Current mode of the UAV.
        guided (bool): Indicates whether the UAV is in guided mode.
        manual_input (bool): Indicates whether manual input is being used.
        system_status (int): Status of the UAV system.
        vtol_state (int): VTOL (Vertical Take-Off and Landing) state of the UAV.
        landed_state (int): Landed state of the UAV.
        status (int): Status of the UAV.
        last_commanded_altitude (float): Altitude of the last commanded position.
        current (int): Current waypoint sequence.
        self_check (int): Counter for self-checking purposes.
        qtd_sub (int): Quantity of subscribers.
        curr_vel (TwistStamped): Current velocity of the UAV.
        des_pose (PoseStamped): Desired pose of the UAV.
        cur_pose (PoseStamped): Current pose of the UAV.

    Methods:
        vel_cb(msg): Callback function for velocity subscriber.
        pos_cb(msg): Callback function for position subscriber.
        pose_callback(data): Callback function for pose subscriber.
        state_callback(data): Callback function for state subscriber.
        state_ex_callback(data): Callback function for extended state subscriber.
        gps_callback(data): Callback function for GPS data subscriber.
        isReadyToTargetFly(): Checks if the UAV is ready to fly in OFFBOARD mode.
        reached_callback(data): Callback function for waypoint reached subscriber.
        change_altitude(delta): Changes the altitude of the UAV by a given delta.
        periodic_setpoint(event): Periodically sets the desired pose for the UAV.
    """  
    def __init__(self):
        self.sequential = {'roll':-float("inf"),
                          'pitch':-float("inf"),
                          'yaw' : -float("inf"),
                          'heading':-float("inf"), 
                          'rollRate':-float("inf"),
                          'pitchRate':-float("inf"),
                          'yawRate':-float("inf"),
                          'groundSpeed':-float("inf"),
                          'climbRate':-float("inf"), 
                          'altitudeRelative':-float("inf"),
                          'throttlePct':-float("inf")}
        self.noise = {'roll':-float("inf"),
                          'pitch':-float("inf"),
                          'yaw' : -float("inf"),
                          'heading':-float("inf"), 
                          'rollRate':-float("inf"),
                          'pitchRate':-float("inf"),
                          'yawRate':-float("inf"),
                          'groundSpeed':-float("inf"),
                          'climbRate':-float("inf"), 
                          'altitudeRelative':-float("inf"),
                          'throttlePct':-float("inf")}
        
        # ----
        self.armed = None
        self.land_ex = -float("inf")
        self.alt = -float("inf")
        self.lat = -float("inf")
        self.lon = -float("inf")
        self.mode =  None
        self.guided = None
        self.manual_input = None
        self.system_status = None
        self.vtol_state = None
        self.landed_state = None
        self.status = None
        self.last_commanded_altitude = None
        self.current = None
        self.self_check = 0
        self.qtd_sub = 4

        # ----
        self.curr_vel = TwistStamped()
        self.des_pose = PoseStamped()
        self.cur_pose = PoseStamped()

        # ----
        self.sub_pose     = rospy.Subscriber('/drone_info/pose'      , DronePose    , self.pose_callback)
        self.sub_state    = rospy.Subscriber('/mavros/state'         , State        , self.state_callback)
        self.sub_ex_state = rospy.Subscriber('/mavros/extended_state', ExtendedState, self.state_ex_callback)
        self.sub_gps      = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        self.vel_pub      = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.vel_sub      = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self.vel_cb)
        self.pos_sub      = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pos_cb)
        self.sub_mission  = rospy.Subscriber('mavros/mission/reached', WaypointReached, self.reached_callback)

        #---
        self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)


    def vel_cb(self, msg):
        self.curr_vel = msg

    def pos_cb(self, msg):
        self.cur_pose = msg

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
        self.sequential['climbRate'] = data.climbRate

        self.noise = sequential_noise(self.sequential)
        # self.noise = self.sequential

        self.self_check += 1

    def state_callback(self, data):
        self.armed = data.armed
        self.mode = data.mode
        self.guided = data.guided
        self.manual_input = data.manual_input
        self.system_status = data.system_status

        self.self_check += 1

    def state_ex_callback(self, data):
        self.vtol_state = data.vtol_state
        self.landed_state = data.landed_state

        self.self_check += 1

    def gps_callback(self, data):
        self.lat = data.latitude
        self.lon = data.longitude
        self.alt = data.altitude
        self.status = data.status

        self.self_check += 1
        
    def isReadyToTargetFly(self):
        print(self.mode)
        if(self.mode=='OFFBOARD'):
            return True

    def reached_callback(self, data):
        self.current = data.wp_seq + 1

    def change_altitude(self, delta):
        # Create the PoseStamped message for the new altitude
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        # Set the new altitude (z position)
        pose.pose.position.x = self.cur_pose.pose.position.x
        pose.pose.position.y = self.cur_pose.pose.position.y
        pose.pose.position.z = self.cur_pose.pose.position.z + delta

        # Publish the message and update the last commanded altitude
        print("Sending new altitude")
        print(pose)
        self.pub.publish(pose)

        self.last_commanded_altitude = pose.pose.position.z

    def periodic_setpoint(self, event):
        # You might want to adjust this method to send more precise setpoints
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.cur_pose.pose.position.x
        pose.pose.position.y = self.cur_pose.pose.position.y
        pose.pose.position.z = self.cur_pose.pose.position.z
        self.pub.publish(pose)

    
# ------------   Callers for MAVRos Services

def mavros_cmd(topic, msg_ty, error_msg="MAVROS command failed: ", **kwargs):
     """
    Send a command to MAVROS through a ROS service.

    Parameters:
        topic (str): The topic of the MAVROS command service.
        msg_ty (ROS message type): The ROS message type for the service.
        error_msg (str): Custom error message in case the service call fails.
        **kwargs: Additional keyword arguments to be passed to the service.

    Returns:
        None

    Raises:
        rospy.ServiceException: If the service call fails.

    Example:
        mavros_cmd('/mavros/cmd/takeoff', CommandTOL, altitude=5)
    """
    rospy.wait_for_service(topic)
    try:
        service_proxy = rospy.ServiceProxy(topic, msg_ty)
        response = service_proxy(**kwargs)
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        rospy.logerr(f"{error_msg} {e}")

def land():
     """
    Initiate landing through MAVROS.

    This function sends a landing command to MAVROS using the '/mavros/cmd/land' service.

    Parameters:
        None

    Returns:
        None
    """
    mavros_cmd(
        '/mavros/cmd/land',
        CommandTOL,
        error_msg="Landing failed",
        altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0
    )

def takeoff(alt, uav):
     """
    Initiate takeoff through MAVROS.

    This function sends a takeoff command to MAVROS using the '/mavros/cmd/takeoff' service.

    Parameters:
        alt (float): The desired altitude for takeoff.
        uav (UAV): An instance of the UAV class representing the drone.

    Returns:
        None

    """
    mavros_cmd(
        '/mavros/cmd/takeoff',
        CommandTOL,
        error_msg="Takeoff failed",
        altitude=uav.alt+alt, latitude=uav.lat, longitude=uav.lon, min_pitch=0, yaw=0
    )

def arm():
    """
    Arm the drone through MAVROS.

    This function sends an arming command to MAVROS using the '/mavros/cmd/arming' service.

    Parameters:
        None

    Returns:
        None
    """
    mavros_cmd(
        '/mavros/cmd/arming',
        CommandBool,
        value=True,
        error_msg="Arming failed",
    )

def set_mode(mode):
    """
    Set the flight mode of the drone through MAVROS.

    This function sends a command to set the flight mode using the '/mavros/set_mode' service.

    Parameters:
        mode (str): The desired flight mode.

    Returns:
        None

    Example:
        set_mode("GUIDED")
    """
    mavros_cmd(
        '/mavros/set_mode',
        SetMode,
        error_msg="Set mode failed",
        custom_mode=mode,
        base_mode=0
    )

def clear_mission():
     """
    Clear the mission waypoints through MAVROS.

    This function sends a command to clear the mission waypoints using the '/mavros/mission/clear' service.

    Parameters:
        None

    Returns:
        None
    """
    mavros_cmd(
        '/mavros/mission/clear',
        WaypointClear,
        error_msg="Clear mission failed"
    )

def send_route(route):
    """
    Send a route to the drone's mission through MAVROS.

    This function sends a route (list of waypoints) to the drone's mission using the '/mavros/mission/push' service.

    Parameters:
        route (Route): The route to be sent, containing waypoints.

    Returns:
        None
    """
    mavros_cmd(
        '/mavros/mission/push',
        WaypointPush,
        error_msg="Send route failed",
        start_index=route.current_seq,
        waypoints=route.waypoints
    )


def set_home_position(latitude, longitude, altitude):
    """
    Set the home position for the drone through MAVROS.

    This function sets the home position for the drone using the '/mavros/cmd/set_home' service.

    Parameters:
        latitude (float): The latitude of the home position.
        longitude (float): The longitude of the home position.
        altitude (float): The altitude of the home position.

    Returns:
        None
    """
    rospy.wait_for_service('/mavros/cmd/set_home')
    try:
        set_home_service = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
        response = set_home_service(0, 0, latitude, longitude, altitude)        
        if response.success:
            rospy.loginfo("Home position set successfully!")
        else:
            rospy.logerr("Failed to set home position.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

# ------------ BEHAVIORS



def takeoff_land(kenny:UAV, alt:int, flight_time:int,flag_fault:bool) -> list:
    """
    Perform takeoff, flight, and landing for a specified duration.

    This function controls the drone to perform takeoff, hover for a specified duration,
    and then land. The flight data, including either sequential or noisy data, is recorded during the flight.

    Parameters:
        kenny (UAV): An instance of the UAV class representing the drone.
        alt (int): The altitude to reach during takeoff.
        flight_time (int): The duration of the flight in seconds.
        flag_fault (bool): A flag indicating whether to use noisy data during the flight.

    Returns:
        list: A list containing the flight data recorded during the flight.

    Example:
        flight_data = takeoff_land(kenny, 10, 60, False)
    """
    flight_list = []
    start = time.time()

    while(time.time()-start < flight_time):
        set_mode("AUTO.LOITER")

        while(not kenny.armed):
            arm()
            rospy.sleep(1)

        
        takeoff(10, kenny)
        while(kenny.landed_state !=2):
            flight_list.append(kenny.noise) if flag_fault else flight_list.append(kenny.sequential)
            rospy.sleep(1)
        i = 0
        while(i < 10):
            flight_list.append(kenny.noise) if flag_fault else flight_list.append(kenny.sequential)
            #make flight straigh line
            rospy.sleep(1) 
            i += 1

        land()
        while(kenny.landed_state != 1):
            flight_list.append(kenny.noise) if flag_fault else flight_list.append(kenny.sequential)
            rospy.sleep(1)

    return flight_list

def hovering(kenny:UAV, alt:int, flight_time:int,flag_fault:bool) -> list:
     """
    Perform hovering at a specific altitude for a specified duration.

    This function controls the drone to perform takeoff, hover at a specified altitude, and record flight data.
    The flight data, including either sequential or noisy data, is recorded during the hovering.

    Parameters:
        kenny (UAV): An instance of the UAV class representing the drone.
        alt (int): The altitude to reach during takeoff.
        flight_time (int): The duration of the hovering in seconds.
        flag_fault (bool): A flag indicating whether to use noisy data during hovering.

    Returns:
        list: A list containing the flight data recorded during hovering.

    Example:
        hover_data = hovering(kenny, 10, 60, False)
    """
    flight_list = []

    set_mode("AUTO.LOITER")

    while(not kenny.armed):
            arm()
            rospy.sleep(1)

    takeoff(10, kenny)
    while(kenny.landed_state !=2):
        rospy.sleep(1)
    
    start = time.time()
    while(time.time()-start < flight_time):
        flight_list.append(kenny.noise) if flag_fault else flight_list.append(kenny.sequential)
        rospy.sleep(1) 

    # land()
    # while(kenny.landed_state != 1):
    #     rospy.sleep(1)

    return flight_list

def line(lat, lon, distance, alt, direction):
    """
    Generate waypoints for flying in a straight line in a specific direction.

    This function generates waypoints for flying in a straight line from the given latitude and longitude,
    covering a specified distance in a particular direction. The waypoints are added to a WaypointList.

    Parameters:
        lat (float): The initial latitude of the starting point.
        lon (float): The initial longitude of the starting point.
        distance (float): The distance to cover in the specified direction (in meters).
        alt (float): The altitude at which the waypoints should be set.
        direction (str): The direction of the line ('R' for right, 'L' for left, 'B' for back, 'F' for forward).

    Returns:
        WaypointList: A list of waypoints representing a straight line in the specified direction.

    Example:
        line_route = line(37.7749, -122.4194, 250, 10, 'F')
    """
    longitude = lon
    latitude = lat
    alt = 10
    # distance = 250

    if direction == 'R':
        theta = 180 # right
    elif direction == 'L':
        theta = 0 #left
    elif direction == 'B':
        theta = 270 #back
    elif direction == 'F':
        theta = 90 # forward 

    geo_route = WaypointList()

    dx = distance* math.cos(math.radians(theta))  # theta measured clockwise from due east
    dy = distance* math.sin(math.radians(theta))  # dx, dy same units as R

    delta_longitude = dx / (111320 * math.cos(latitude))  # dx, dy in meters
    delta_latitude = dy / 110540  # result in degrees long/lat

    longitude = longitude + delta_longitude
    latitude = latitude + delta_latitude


    geo_wp = Waypoint()
    geo_wp.frame = 3
    geo_wp.command = 16
    if geo_route.waypoints == []:
        geo_wp.is_current = True
    else:
        geo_wp.is_current = False
    geo_wp.autocontinue = True
    geo_wp.param1 = 0
    geo_wp.param2 = 0
    geo_wp.param3 = 0
    geo_wp.param4 = 0
    geo_wp.x_lat = latitude
    geo_wp.y_long = longitude
    geo_wp.z_alt = alt
    geo_route.waypoints.append(geo_wp)

    return geo_route

def line_flight(uav, min_alt, max_alt,distance, direction, flight_time, flag_fault):
    """
    Perform a flight in a straight line with a UAV.

    This function performs a flight in a straight line with a UAV, covering a specified distance in a particular direction.
    The UAV takes off to the specified minimum altitude, then flies along the generated line waypoints. The flight lasts for
    a specified duration. Flight data is collected and added to the `flight_list`.

    Parameters:
        uav (UAV): An instance of the UAV class representing the UAV.
        min_alt (float): The minimum altitude at which the UAV takes off.
        max_alt (float): The maximum altitude at which the UAV flies during the line flight.
        distance (float): The distance to cover in the specified direction (in meters).
        direction (str): The direction of the line ('R' for right, 'L' for left, 'B' for back, 'F' for forward).
        flight_time (int): The duration of the flight in seconds.
        flag_fault (bool): A flag indicating whether to use noisy data (True) or sequential data (False).

    Returns:
        list: A list of flight data collected during the line flight.
    """
    target = Pose()
    target.position.x = 0
    target.position.y = 0
    target.position.z = 5

    # vController = VelocityController()

    flight_list = []

    set_mode("AUTO.LOITER")

    while(not uav.armed):
            arm()
            rospy.sleep(1)

    takeoff(min_alt, uav)
    while(uav.landed_state !=2):
        rospy.sleep(1)

    start = time.time()
    while(time.time()-start < flight_time):
        route = WaypointList()

        route.waypoints = line(uav.lat, uav.lon, distance, max_alt, direction)
        route.current_seq = 0

        # send route to uav
        clear_mission()
        uav.current = 0
        rospy.loginfo("Send Route")
        send_route(route.waypoints)

        # set mode to mission
        rospy.loginfo("Set Mode")
        set_mode("AUTO.MISSION")
        
        rospy.sleep(1)
        while(uav.current < len(route.waypoints.waypoints)):
            flight_list.append(uav.noise) if flag_fault else flight_list.append(uav.sequential)
            rospy.sleep(1)
        rospy.sleep(10)

    # set_mode("RTL")

    # land()
    # while(uav.landed_state != 1):
    #     rospy.sleep(1)

    return flight_list
    # return None
    
def circle(lat, lon, raio, alt):
    """
    Generate waypoints for a circular flight path around a specified location.

    This function generates waypoints for a circular flight path around a specified location using the given latitude,
    longitude, radius, and altitude. The circular flight path is created as a polygon, and waypoints are defined based on
    the perimeter of this polygon.

    Parameters:
        lat (float): The latitude of the center of the circle.
        lon (float): The longitude of the center of the circle.
        raio (float): The radius of the circular flight path (in meters).
        alt (float): The altitude at which the circular flight path is performed.

    Returns:
        WaypointList: A list of waypoints defining the circular flight path.
     """
    point = PointGeometry(lon, lat)
    local_azimuthal_projection = f"+proj=aeqd +R=6371000 +units=m +lat_0={point.y} +lon_0={point.x}"

    wgs84_to_aeqd = partial(
        pyproj.transform,
        pyproj.Proj('+proj=longlat +datum=WGS84 +no_defs'),
        pyproj.Proj(local_azimuthal_projection),
    )

    aeqd_to_wgs84 = partial(
        pyproj.transform,
        pyproj.Proj(local_azimuthal_projection),
        pyproj.Proj('+proj=longlat +datum=WGS84 +no_defs'),
    )

    point_transformed = transform(wgs84_to_aeqd, point)

    buffer = point_transformed.buffer(raio)

    buffer_wgs84 = transform(aeqd_to_wgs84, buffer)

    # print(buffer_wgs84)
    coord = mapping(buffer_wgs84)
    # print(coord)

    # Create polygon from lists of points
    x = []
    y = []

    some_poly = buffer_wgs84
    # Extract the point values that define the perimeter of the polygon
    x, y = some_poly.exterior.coords.xy

    route = WaypointList()
    for i, j in zip(x, y):
        geo_wp = Waypoint()
        geo_wp.frame = 3
        geo_wp.command = 16
        if route.waypoints == []:
            geo_wp.is_current = True
        else:
            geo_wp.is_current = False
        geo_wp.autocontinue = True
        geo_wp.param1 = 0
        geo_wp.param2 = 0
        geo_wp.param3 = 0
        geo_wp.param4 = 0
        geo_wp.x_lat = j
        geo_wp.y_long = i
        geo_wp.z_alt = 10
        route.waypoints.append(geo_wp)

    return route

def circular_flight(uav, raio, alt, flight_time, flag_fault):
    """
    Perform a circular flight around a specified location.

    This function performs a circular flight around a specified location using the given UAV object, radius, altitude, flight
    time, and flag indicating whether to simulate a fault. The UAV is assumed to be armed and take off before executing the
    circular flight path. The flight path is defined by waypoints generated using the `circle` function.

    Parameters:
        uav (UAV): The UAV object representing the drone.
        raio (float): The radius of the circular flight path (in meters).
        alt (float): The altitude at which the circular flight path is performed.
        flight_time (int): The duration of the circular flight in seconds.
        flag_fault (bool): A flag indicating whether to simulate a fault during the flight.

    Returns:
        list: A list of flight data (either sequential or noisy) collected during the circular flight.
    """
    flight_list = []
    set_mode("AUTO.LOITER")

    while(not uav.armed):
            arm()
            rospy.sleep(1)

    takeoff(10, uav)
    while(uav.landed_state !=2):
        rospy.sleep(1)

    ## do flight
    route = WaypointList()
    route.waypoints = circle(uav.lat, uav.lon, raio, alt)
    route.current_seq = 0

    # send route to uav
    clear_mission()
    rospy.loginfo("Send Route")
    send_route(route.waypoints)

    # set mode to mission
    rospy.loginfo("Set Mode")
    set_mode("AUTO.MISSION")

    # wait to arrive.
    uav.current = 0
    start = time.time()
    while( uav.current < len(route.waypoints.waypoints) and time.time()-start < flight_time):
        flight_list.append(uav.noise) if flag_fault else flight_list.append(uav.sequential)
        rospy.sleep(1)

    # land()
    # while(uav.landed_state != 1):
    #     rospy.sleep(1)

    return flight_list
    
def eight_waypoints(center_lat, center_lon, min_altitude, max_altitude, radius):
    """
    Generate waypoints for an eight-shaped flight pattern.

    This function generates waypoints for an eight-shaped flight pattern based on the specified center coordinates, minimum and
    maximum altitudes, and radius. The flight pattern consists of 32 points forming a figure-eight shape. The waypoints are
    returned as a list of tuples containing latitude, longitude, and altitude values.

    Parameters:
        center_lat (float): The latitude of the center point for the flight pattern.
        center_lon (float): The longitude of the center point for the flight pattern.
        min_altitude (float): The minimum altitude for the flight pattern.
        max_altitude (float): The maximum altitude for the flight pattern.
        radius (float): The radius of the figure-eight flight pattern.

    Returns:
        WaypointList: A list of waypoints for the figure-eight flight pattern.

    Example:
        waypoints = eight_waypoints(37.7749, -122.4194, 10, 30, 50)
    """
    waypoints = []
    # Calculate the coordinates for the eight-shaped flight pattern
    num_points = 32  # Number of points in the pattern
    angle_increment = 2 * math.pi / num_points  # Angle between each point

    altitude_range = max_altitude - min_altitude
    altitude_increment = altitude_range / (num_points - 1)  # Increment in altitude between each point

    for i in range(num_points):
        angle = i * angle_increment

        # Calculate the coordinates for each waypoint
        x = radius * math.cos(angle)
        y = radius * math.sin(2 * angle) / 2  # Divide sin(2 * angle) by 2 to create the figure-eight shape

        lat = center_lat + (y / 111111)  # Convert y-coordinate to latitude (assuming 1 degree is approximately 111111 meters)
        lon = center_lon + (x / (111111 * math.cos(center_lat)))  # Convert x-coordinate to longitude (adjusting for latitude)

        # Calculate the altitude for each waypoint based on the progressive increase and decrease
        altitude = min_altitude + (i * altitude_increment)
        if i >= num_points // 2:
            altitude = max_altitude - ((i - num_points // 2) * altitude_increment)

        waypoint = (lat, lon, altitude)
        waypoints.append(waypoint)

    route = WaypointList()
    for wp in waypoints:
        geo_wp = Waypoint()
        geo_wp.frame = 3
        geo_wp.command = 16
        if route.waypoints == []:
            geo_wp.is_current = True
        else:
            geo_wp.is_current = False
        geo_wp.autocontinue = True
        geo_wp.param1 = 0
        geo_wp.param2 = 0
        geo_wp.param3 = 0
        geo_wp.param4 = 0
        geo_wp.x_lat = wp[0]
        geo_wp.y_long = wp[1]
        geo_wp.z_alt = wp[2]
        route.waypoints.append(geo_wp)

    return route

def eight_flight(uav, raio, min_alt, max_alt, flight_time, flag_fault):
    """
    Perform an eight-shaped flight pattern with the UAV.

    This function commands the UAV to perform an eight-shaped flight pattern based on the specified radius, minimum altitude,
    maximum altitude, flight time, and fault flag. The UAV follows the generated waypoints for the eight-shaped pattern.

    Parameters:
        uav (UAV): The UAV object representing the drone.
        raio (float): The radius of the figure-eight flight pattern.
        min_alt (float): The minimum altitude for the flight pattern.
        max_alt (float): The maximum altitude for the flight pattern.
        flight_time (int): The total flight time for the eight-shaped pattern.
        flag_fault (bool): A flag indicating whether to use fault-injected data during the flight.

    Returns:
        list: A list of recorded data during the flight.

    """
    flight_list = []
    set_mode("AUTO.LOITER")

    while(not uav.armed):
            arm()
            rospy.sleep(1)

    takeoff(10, uav)
    while(uav.landed_state !=2):
        rospy.sleep(1)

    ## do flight
    route = WaypointList()
    route.waypoints = eight_waypoints(uav.lat, uav.lon, min_alt, max_alt, raio)
    route.current_seq = 0

    # send route to uav
    clear_mission()
    rospy.loginfo("Send Route")
    send_route(route.waypoints)

    # set mode to mission
    rospy.loginfo("Set Mode")
    set_mode("AUTO.MISSION")

    # wait to arrive.
    uav.current = 0
    start = time.time()
    while( uav.current < len(route.waypoints.waypoints) and time.time()-start < flight_time):
        flight_list.append(uav.noise) if flag_fault else flight_list.append(uav.sequential)
        rospy.sleep(1)

    # land()
    # while(uav.landed_state != 1):
    #     rospy.sleep(1)

    return flight_list
 
def vertical_line(lat, lon, alt_change):
    """
    Generate waypoints for a vertical line flight pattern.

    This function generates waypoints for a vertical line flight pattern based on the specified latitude, longitude,
    and altitude change. The waypoints form a straight vertical line, and the UAV follows this path during the flight.

    Parameters:
        lat (float): The latitude of the starting point for the vertical line.
        lon (float): The longitude of the starting point for the vertical line.
        alt_change (float): The change in altitude for the vertical line.

    Returns:
        WaypointList: A list of waypoints representing the vertical line.
    """
    altitude = alt_change

    geo_route = WaypointList()

    geo_wp = Waypoint()
    geo_wp.frame = 3
    geo_wp.command = 16
    if not geo_route.waypoints:
        geo_wp.is_current = True
    else:
        geo_wp.is_current = False
    geo_wp.autocontinue = True
    geo_wp.param1 = 0
    geo_wp.param2 = 0
    geo_wp.param3 = 0
    geo_wp.param4 = 0
    geo_wp.x_lat = lat
    geo_wp.y_long = lon
    geo_wp.z_alt = altitude
    geo_route.waypoints.append(geo_wp)

    return geo_route

def altitude_flight(kenny:UAV, alt:int, flight_time:int,flag_fault:bool) -> list:
    """
    Perform a flight maneuver with changes in altitude using waypoints.

    This function performs a flight maneuver with changes in altitude using waypoints. It includes takeoff, ascending
    to a specified altitude, flying at that altitude for a specified time, descending to a lower altitude, and then
    landing. The flight data, including noise if specified, is recorded during the maneuver.

    Parameters:
        kenny (UAV): An instance of the UAV class representing the unmanned aerial vehicle.
        alt (int): The target altitude to reach during the flight.
        flight_time (int): The total time duration for the flight maneuver.
        flag_fault (bool): A flag indicating whether to use fault-injected data (True) or sequential data (False).

    Returns:
        list: A list of recorded flight data during the maneuver.

    """
    flight_list = []

    set_mode("AUTO.LOITER")

    while(not kenny.armed):
            arm()
            rospy.sleep(1)

    takeoff(10, kenny)
    while(kenny.landed_state !=2):
        rospy.sleep(1)
    rospy.sleep(1)
    
    # rospy.Timer(rospy.Duration(0.1), kenny.periodic_setpoint) # Send setpoint every 0.1 seconds
    # rospy.sleep(2)  # Let a few setpoints be sent before requesting mode change
    
    set_mode("AUTO.MISSION")
    route = WaypointList()

    route.waypoints = vertical_line(kenny.lat, kenny.lon, kenny.alt+alt)
    route.current_seq = 0

    # send route to uav
    clear_mission()
    kenny.current = 0
    rospy.loginfo("Send Route")
    send_route(route.waypoints)

    # # set mode to mission
    rospy.loginfo("Set Mode")
    set_mode("AUTO.MISSION")
    start = time.time()
    while(time.time()-start < flight_time):
        flight_list.append(kenny.noise) if flag_fault else flight_list.append(kenny.sequential)
        rospy.sleep(1)

    set_mode("AUTO.LOITER")


    rospy.sleep(1)
    

    
    route = WaypointList()
    route.waypoints = vertical_line(kenny.lat, kenny.lon, 5)

    route.current_seq = 0

    clear_mission()
    kenny.current = 0
    rospy.loginfo("Send Route")
    send_route(route.waypoints)

    # # set mode to mission
    rospy.loginfo("Set Mode")
    set_mode("AUTO.MISSION")
    start = time.time()
    while(time.time()-start < flight_time):
        flight_list.append(kenny.noise) if flag_fault else flight_list.append(kenny.sequential)
        rospy.sleep(1)

    set_mode("AUTO.LOITER")

    # land()
    # while(kenny.landed_state != 1):
    #     rospy.sleep(1)

    return flight_list

# ------------ MAIN
def listener():
    """
    Main function to generate flight data for a drone.

    This function initializes a ROS node, sets up a UAV instance, and generates flight data based on predefined maneuvers.
    The flight data includes takeoff and landing, hovering, circular flight, line flight, eight-shaped flight, and altitude change.
    The generated data is saved in a JSON file.

    Parameters:
    None

    Returns:
    None
    """
    rospy.init_node('genarate_flight_data', anonymous=True)
    kenny = UAV()
    flight_list = []

    ## quantity of each flight will be executed 
    qtd_good_exe = 15
    # qtd_good_exe = 0
    qtd_fault_exe = 0

    flight_time = 180 #s

    qtd_exe = qtd_fault_exe + qtd_good_exe

    while(kenny.self_check < kenny.qtd_sub):
        print("Waiting for drone data...")
        print(kenny.self_check)
        rospy.sleep(1)

    home_latitude = -22.001333
    home_longitude = -47.934152
    home_altitude = 847.142652

    set_home_position(home_latitude, home_longitude, home_altitude)
    # eight_flight(kenny, 5, 10, flight_time, False)

    file_path = "flight_data_gauss_fixed_good.json"


    # print(kenny.sequential)
    # print(kenny.armed)
    # print(kenny.vtol_state)
    j = 0
    for flight in table:
        for i in range(0, qtd_exe):
            flag_fault = i < qtd_fault_exe

            if flight["Type"] == "Takeoff and landing":
                print("*-------------------------------------*")
                print((j))
                print(flight["Type"] +"   "+ str(flag_fault))
                data = takeoff_land(kenny, flight["Takeoff Alt"], flight_time,flag_fault)
                flight_data = {"Type": flight["Type"],
                                    "error": flag_fault, 
                                    "id":(j),
                                    "data": data
                                    }
            elif flight["Type"] == "Hovering":
                print("*-------------------------------------*")
                print((j))
                print(flight["Type"])
                data = hovering(kenny, flight["Hovering Alt"], flight_time,flag_fault)
                flight_data ={"Type": flight["Type"],
                                    "error": flag_fault, 
                                    "id":(j),
                                    "data": data
                                    }
            elif flight["Type"] == "Circular flight":
                print("*-------------------------------------*")
                print((j))
                print(flight["Type"])
                data = circular_flight(kenny, flight["Radius"], 10, flight_time, flag_fault)
                flight_data ={"Type": flight["Type"],
                                    "error": flag_fault, 
                                    "id":(j),
                                    "data": data
                                    }
            elif flight["Type"] == "Line flight":
                print("*-------------------------------------*")
                print((j))
                print(flight["Type"]+" "+flight["Direction"])
                data = line_flight(kenny, flight["Start Alt"], flight["End Alt"],flight["Distance"], flight["Direction"], flight_time, flag_fault)
                flight_data ={"Type": flight["Type"]+" "+flight["Direction"],
                                    "error": flag_fault, 
                                    "id":(j),
                                    "data": data
                                    }
            elif flight["Type"] == "8":
                print("*-------------------------------------*")
                print(j)
                print(flight["Type"])
                data = eight_flight(kenny,flight["Radius"], flight["Start Alt"], flight["End Alt"], flight_time, flag_fault)
                flight_data ={"Type": flight["Type"],
                                    "error": flag_fault, 
                                    "id":(j),
                                    "data": data
                                    }
            elif flight["Type"] == "alt":
                print("*-------------------------------------*")
                print(j)
                print(flight["Type"])
                data = altitude_flight(kenny,flight["delta_Alt"], flight_time, flag_fault)
                flight_data ={"Type": flight["Type"],
                                    "error": flag_fault, 
                                    "id":(j),
                                    "data": data
                                    }
            #     {"Type": "alt", "delta_Alt": 5}, altitude_flight(kenny:UAV, alt:int, flight_time:int,flag_fault:bool) -> list:


            with open(file_path, "a") as json_file:
                    json.dump(flight_data, json_file)
                    json_file.write('\n')
            j= j + 1


    print(flight_list)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
