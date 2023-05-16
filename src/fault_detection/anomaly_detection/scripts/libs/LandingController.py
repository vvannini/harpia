"""
testing offboard positon control with a simple takeoff script
"""

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, Twist, TwistStamped
import math
import numpy
from std_msgs.msg import Header
from PID import PID




class LandingController:
    target = PoseStamped()
    output = TwistStamped()
    velocity = None

    def __init__(self):
        self.X = PID()
        self.Y = PID()
        self.Z = PID()
        self.lastTime = rospy.get_time()
        self.target = None
        self.descentVelocity = -0.5
        self.landingDescent = False
        self.descending = False
        self.distTolerance = 2
        self.velTolerance = 0.1

    # Send target to land at
    def setTarget(self, target):
        self.landingDescent = False
        self.descending = False
        self.target = target

    def update(self, state, velocity):
        self.velocity = velocity
        if (self.target is None):
            rospy.logwarn("Target position for landing controller is none.")
            return None
        # simplify variables a bit
        time = state.header.stamp.to_sec()
        position = state.pose.position
        orientation = state.pose.orientation
        # create output structure
        output = TwistStamped()
        output.header = state.header
        # check if we're at target altitude to initiate descent
        distToTarget = math.pow(position.x - self.target.position.x, 2) + math.pow(position.y - self.target.position.y, 2) + math.pow(position.z - self.target.position.z, 2)
        if (not self.landingDescent and distToTarget < self.distTolerance):
            self.landingDescent = True
            print "Started landing descent"
        # output velocities
        linear = Vector3()
        angular = Vector3()
        # Control in X vel
        linear.x = self.X.update(self.target.position.x, position.x, time)
        # Control in Y vel
        linear.y = self.Y.update(self.target.position.y, position.y, time)
        # Control in Z vel
        if(self.landingDescent):
            linear.z = self.descentVelocity
            if (not self.descending and velocity.twist.linear.z < (1-self.velTolerance)*self.descentVelocity):
                self.descending = True
                print "Descending"
        else:
            linear.z = self.Z.update(self.target.position.z, position.z, time)
        # Control yaw (no x, y angular)
        # TODO
        output.twist = Twist()
        output.twist.linear = linear
        return output


    def landed(self):
        return self.descending and (self.velocity.twist.linear.z > -0.2)

    def stop(self):
        setTarget(self.current)
        update(self.current)
