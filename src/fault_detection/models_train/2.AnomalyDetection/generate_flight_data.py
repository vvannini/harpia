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

# table = [
#     {"Type": "Takeoff and landing", "Takeoff Alt": 5},
#     {"Type": "Takeoff and landing", "Takeoff Alt": 15},
#     {"Type": "Takeoff and landing", "Takeoff Alt": 30},
#     {"Type": "Hovering", "Hovering Alt": 5},
#     {"Type": "Hovering", "Hovering Alt": 15},
#     {"Type": "Hovering", "Hovering Alt": 30},
#     {"Type": "Line flight", "Direction": "F", "Distance": 10, "Start Alt": 15, "End Alt": 15},
#     {"Type": "Line flight", "Direction": "F", "Distance": 10, "Start Alt": 15, "End Alt": 10},
#     {"Type": "Line flight", "Direction": "F", "Distance": 15, "Start Alt": 50, "End Alt": 30},
#     {"Type": "Line flight", "Direction": "F", "Distance": 5, "Start Alt": 7, "End Alt": 5},
#     {"Type": "Line flight", "Direction": "B","Distance": 10, "Start Alt": 15, "End Alt": 10},
#     {"Type": "Line flight", "Direction": "B","Distance": 15, "Start Alt": 50, "End Alt": 30},
#     {"Type": "Line flight", "Direction": "B","Distance": 5, "Start Alt": 7, "End Alt": 5},
#     {"Type": "Line flight", "Direction": "L", "Distance": 10, "Start Alt": 15, "End Alt": 10},
#     {"Type": "Line flight", "Direction": "L", "Distance": 15, "Start Alt": 50, "End Alt": 30},
#     {"Type": "Line flight", "Direction": "L", "Distance": 5, "Start Alt": 7, "End Alt": 5},
#     {"Type": "Line flight", "Direction": "R", "Distance": 10, "Start Alt": 15, "End Alt": 10},
#     {"Type": "Line flight", "Direction": "R", "Distance": 15, "Start Alt": 50, "End Alt": 30},
#     {"Type": "Line flight", "Direction": "R", "Distance": 5, "Start Alt": 7, "End Alt": 5},
#     {"Type": "Circular flight", "Radius": 5, "Wind speed": None},
#     {"Type": "Circular flight", "Radius": 25, "Wind speed": None},
#     {"Type": "Circular flight", "Radius": 50, "Wind speed": None},
#     {"Type": "8", "Radius": 50, "Start Alt": 7, "End Alt": 5},
# ]

table = [
    {"Type": "8", "Radius": 50, "Start Alt": 7, "End Alt": 5},
]


def get_harpia_root_dir():
    return rospy.get_param("/harpia_home", default=os.path.expanduser("~/harpia"))

'''
    behaivors 
'''

def sequential_noise(data):
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
        sequential[key] = NoiseGenerator.noisyData(data,key, 1., 50000.)

    return sequential

def go_to_base(mission, uav):
    """
    Go to nearest base immediately and land.
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
# ------------   Callers for MAVRos Services

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

def takeoff(alt, uav):
    mavros_cmd(
        '/mavros/cmd/takeoff',
        CommandTOL,
        error_msg="Takeoff failed",
        altitude=uav.alt+alt, latitude=uav.lat, longitude=uav.lon, min_pitch=0, yaw=0
    )

def arm():
    mavros_cmd(
        '/mavros/cmd/arming',
        CommandBool,
        value=True,
        error_msg="Arming failed",
    )

def set_mode(mode):
    mavros_cmd(
        '/mavros/set_mode',
        SetMode,
        error_msg="Set mode failed",
        custom_mode=mode,
        base_mode=0
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

def set_home_position(latitude, longitude, altitude):
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

    land()
    while(kenny.landed_state != 1):
        rospy.sleep(1)

    return flight_list

def line(lat, lon, distance, alt, direction):
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

    set_mode("RTL")

    land()
    while(uav.landed_state != 1):
        rospy.sleep(1)

    return flight_list
    # return None
    
def circle(lat, lon, raio, alt):
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

    land()
    while(uav.landed_state != 1):
        rospy.sleep(1)

    return flight_list
    
def eight_waypoints(center_lat, center_lon, min_altitude, max_altitude, radius):
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

    land()
    while(uav.landed_state != 1):
        rospy.sleep(1)

    return flight_list
 

# ------------ MAIN
def listener():

    rospy.init_node('genarate_flight_data', anonymous=True)
    kenny = UAV()
    flight_list = []

    ## quantity of each flight will be executed 
    # qtd_good_exe = 15
    qtd_good_exe = 0
    qtd_fault_exe = 5

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

    file_path = "flight_data_gauss_mild.json"


    # print(kenny.sequential)
    # print(kenny.armed)
    # print(kenny.vtol_state)
    j = 110
    for flight in table:
        for i in range(0, qtd_exe):
            flag_fault = i <= qtd_fault_exe

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

            with open(file_path, "a") as json_file:
                    json.dump(flight_data, json_file)
                    json_file.write('\n')
            j= j + 1


    print(flight_list)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
