#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/Imu.h>
#include "mavros_msgs/WaypointList.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/VFR_HUD.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <bits/stdc++.h>
#include <harpia_msgs/DronePose.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "actionlib_msgs/GoalID.h"

#include <tf/transform_datatypes.h>

#include <signal.h>
#include<math.h>

#include <fstream>
#include<iomanip>

#include <cstdlib>
using namespace std;
std::string homepath = getenv("HOME");

// change GeoPoint to geographic_msgs/geopoint
struct GeoPoint{
	string  name;
	double longitude;
	double latitude;
	double altitude;
};

class Drone
{
	public:
		harpia_msgs::DronePose pose;
		void chatterCallback_localPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void chatterCallback_imu(const sensor_msgs::Imu::ConstPtr& msg);
		void chatterCallback_vfr_hud(const mavros_msgs::VFR_HUD::ConstPtr& msg);
		void chatterCallback_golbalp(const nav_msgs::Odometry::ConstPtr& msg);
		void chatterCallback_compass(const std_msgs::Float64::ConstPtr& msg);
};



void Drone::chatterCallback_localPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // do conversions
	float linearposx=msg->pose.position.x;
   	float linearposy=msg->pose.position.y;
   	double quatx= msg->pose.orientation.x;
   	double quaty= msg->pose.orientation.y;
   	double quatz= msg->pose.orientation.z;
   	double quatw= msg->pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose.roll = roll;
    pose.pitch = pitch;
    pose.yaw = yaw;
}

void Drone::chatterCallback_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
    pose.rollRate = msg->angular_velocity.x;
    pose.pitchRate = msg->angular_velocity.y;
    pose.yawRate = msg->angular_velocity.z;
}

void Drone::chatterCallback_vfr_hud(const mavros_msgs::VFR_HUD::ConstPtr& msg)
{
    pose.groundSpeed = msg->groundspeed;
    pose.climbRate   = msg->climb;
    pose.throttle    = msg->throttle;
}

void Drone::chatterCallback_golbalp(const nav_msgs::Odometry::ConstPtr& msg)
{
    pose.altRelative = msg->pose.pose.position.z;
}

void Drone::chatterCallback_compass(const std_msgs::Float64::ConstPtr& msg)
{
    pose.heading = msg->data;
}


/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "drone_info", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    // signal(SIGINT, mySigintHandler);
    
    Drone drone;

    ros::Subscriber GPS  = nh.subscribe("/mavros/local_position/pose"			,1, &Drone::chatterCallback_localPose,&drone);
    ros::Subscriber IMU  = nh.subscribe("/mavros/imu/data"						,1, &Drone::chatterCallback_imu		 ,&drone);
    ros::Subscriber VFR  = nh.subscribe("/mavros/vfr_hud"						,1, &Drone::chatterCallback_vfr_hud	 ,&drone);
    ros::Subscriber GPOS = nh.subscribe("/mavros/global_position/local"			,1, &Drone::chatterCallback_golbalp	 ,&drone);
    ros::Subscriber HDG  = nh.subscribe("/mavros/global_position/compass_hdg"	,1, &Drone::chatterCallback_compass	 ,&drone);

    ros::Publisher  PUB  = nh.advertise<harpia_msgs::DronePose>("pose", 1000);

    ros::Rate loop_rate(10);

    ROS_INFO("ROS INFO NODE");

    while(ros::ok)
    {
    	// ROS_INFO("%s", drone.pose.c_str());
    	PUB.publish(drone.pose);
    	ros::spinOnce();
    	loop_rate.sleep();
    }
    


    return 0;
}
