#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import NavSatFix, Imu, BatteryState


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.pose)

# Classes

class UAV(object):
    def __init__(self):
        self.sub_pose     = rospy.Subscriber('/mavros/local_position/pose'  , PoseStamped              , self.pose_callback)
        self.sub_imu      = rospy.Subscriber('/mavros/imu/data'             , Imu                      , self.imu_callback)
        self.sub_vfr_hud  = rospy.Subscriber('/mavros/vfr_hud'              , VFR_HUD  , self.vfr_hud_callback)
        self.sub_g_pos    = rospy.Subscriber('/mavros/global_position/local', Odometry , self.globalp_callback)
        self.sequential = {'roll':[],
                          'pitch':[],
                          'heading':[], #yaw
                          'rollRate':[],
                          'pitchRate':[],
                          'yawRate':[],
                          'groundSpeed':[],
                          'climbRate':[], # ?
                          'altitudeRelative':[],
                          'throttlePct':[]}
        # self.weightedCluster = []

    
    def pose_callback(self, data): 
        self.sequential['roll'] = data.pose.position.x
        self.sequential['pitch'] = data.pose.position.y
        self.sequential['heading'] = data.pose.position.z

    def imu_callback(self, data): 
        self.sequential['rollRate'] = data.angular_velocity.x
        self.sequential['pitchRate'] = data.angular_velocity.y
        self.sequential['yawRate'] = data.angular_velocity.z

    def vfr_hud_callback(self, data): 
        self.sequential['groundSpeed'] = data.groundspeed
        self.sequential['throttlePct'] = data.throttle

    def globalp_callback(self, data): 
        self.sequential['altitudeRelative'] = data.pose.pose.position.z

    # def unsubscribe(self):
    #     self.sub.unregister()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    kenny = UAV()

    for i in range(30):
        print(kenny.sequential)
        rospy.sleep(1)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
