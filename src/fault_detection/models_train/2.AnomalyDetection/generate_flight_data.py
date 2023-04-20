#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import NavSatFix, Imu, BatteryState


# --- 
from libs.anomalyDetection import findAnomaly, clusters, checkValues
from libs import anomalyDetection,clustering,loadData, NoiseGenerator
import time
import math

table = [
    {"Type": "Takeoff and landing", "Takeoff Alt": 5},
    {"Type": "Takeoff and landing", "Takeoff Alt": 15},
    {"Type": "Takeoff and landing", "Takeoff Alt": 30},
    {"Type": "Hovering", "Hovering Alt": 5},
    {"Type": "Hovering", "Hovering Alt": 15},
    {"Type": "Hovering", "Hovering Alt": 30},
    {"Type": "Forward flight", "Distance": 10, "Start Alt": 15, "End Alt": 15},
    {"Type": "Forward flight", "Distance": 10, "Start Alt": 15, "End Alt": 10},
    {"Type": "Forward flight", "Distance": 15, "Start Alt": 50, "End Alt": 30},
    {"Type": "Forward flight", "Distance": 5, "Start Alt": 7, "End Alt": 5},
    {"Type": "Backward flight", "Distance": 10, "Start Alt": 15, "End Alt": 10},
    {"Type": "Backward flight", "Distance": 15, "Start Alt": 50, "End Alt": 30},
    {"Type": "Backward flight", "Distance": 5, "Start Alt": 7, "End Alt": 5},
    {"Type": "Sideways flight", "Direction": "L", "Distance": 10, "Start Alt": 15, "End Alt": 10},
    {"Type": "Sideways flight", "Direction": "L", "Distance": 15, "Start Alt": 50, "End Alt": 30},
    {"Type": "Sideways flight", "Direction": "L", "Distance": 5, "Start Alt": 7, "End Alt": 5},
    {"Type": "Sideways flight", "Direction": "D", "Distance": 10, "Start Alt": 15, "End Alt": 10},
    {"Type": "Sideways flight", "Direction": "D", "Distance": 15, "Start Alt": 50, "End Alt": 30},
    {"Type": "Sideways flight", "Direction": "D", "Distance": 5, "Start Alt": 7, "End Alt": 5},
    {"Type": "Circular flight", "Radius": 5, "Wind speed": None},
    {"Type": "Circular flight", "Radius": 15, "Wind speed": None},
    {"Type": "Circular flight", "Radius": 20, "Wind speed": None},
    {"Type": "8", "Starting location (x,y,z)": None, "Obstacle position (x,y,z)": None, "Obstacle size": None, "Velocity": None},
]


def get_harpia_root_dir():
    return rospy.get_param("/harpia_home", default=os.path.expanduser("~/harpia"))

'''
    behaivors 
'''

def circle(from_wp, raio, alt):
    point = Point(from_wp.geo.longitude, from_wp.geo.latitude)
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

def takeoff(alt):
    mavros_cmd(
        '/mavros/cmd/takeoff',
        CommandTOL,
        error_msg="Takeoff failed",
        altitude=alt, latitude=0, longitude=0, min_pitch=0, yaw=0
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

def sequential_noise(data):
    sequential = {'roll':[],
                          'pitch':[],
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
        self.sub_pose   = rospy.Subscriber('/drone_info/pose'        , DronePose    , self.pose_callback)
        self.sub_ex_state = rospy.Subscriber('/mavros/extended_state', ExtendedState, self.ex_state_callback)
        self.sub_ex_state = rospy.Subscriber('/mavros/state'         , ExtendedState, self.state_callback)
        self.sequential = {'roll':-math.inf,
                          'pitch':-math.inf,
                          'yaw' : -math.inf,
                          'heading':-math.inf, 
                          'rollRate':-math.inf,
                          'pitchRate':-math.inf,
                          'yawRate':-math.inf,
                          'groundSpeed':-math.inf,
                          'climbRate':-math.inf, 
                          'altitudeRelative':-math.inf,
                          'throttlePct':-math.inf}
        self.armed = None
        self.land_ex = -math.inf
        self.mode =  None
        # self.weightedCluster = []

    
    def pose_callback(self, data): 
        self.sequential['roll'] = data.roll
        self.sequential['pitch'] = data.pitch
        self.sequential['yaw'] = data.yaw
        self.sequential['heading'] = data.heading
        self.sequential['rollRate'] = data.rollRate
        self.sequential['pitchRate'] = data.pitchRate
        self.sequential['yawRate'] = data.yawRate
        self.sequential['groundSpeed'] = data.groundspeed
        self.sequential['throttlePct'] = data.throttle
        self.sequential['altitudeRelative'] = data.altRelative


# ------------ BEHAIVORS

def takeoff_land(kenny:UAV, alt:int, fight_time:int,flag_fault:bool) -> list:
    flight_list = []
    start = time.time()

    while(time.time()-start < flight_time):
# while(!drone.current_state.armed && drone.ex_current_state.landed_state != 2)
#                 {
#                     set_loiter();
#                     arm();
#                     takeoff(drone);
#                 }

        rospy.sleep(1)
        flight_list.append(kenny.sequential)

    return flight_list



# ------------ MAIN
def listener():

    rospy.init_node('genarate_flight_data', anonymous=True)
    kenny = UAV()
    flight_list = []

    ## quantity of each flight will be executed 
    qtd_good_exe = 15
    qtd_fault_exe = 5

    fight_time = 180 #s

    qtd_exe = qtd_fault_exe + qtd_good_exe

    for flight in table:
        for i in range(0, qtd_fault_exe):
            
            flag_fault = i <= qtd_good_exe

            if flight["Type"] == "Takeoff and landing":
               flight_list.append(takeoff_land(kenny, flight["Takeoff Alt"], fight_time,flag_fault))
        


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
