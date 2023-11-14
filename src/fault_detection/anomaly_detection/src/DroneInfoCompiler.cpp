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

/**
 * @brief The Drone class represents a drone entity and contains functionalities
 *        to handle different ROS messages related to drone data.
 */
class Drone
{
	public:
        /**
        * @brief Represents the pose information of the drone.
        */
		harpia_msgs::DronePose pose;
		void chatterCallback_localPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void chatterCallback_imu(const sensor_msgs::Imu::ConstPtr& msg);
		void chatterCallback_vfr_hud(const mavros_msgs::VFR_HUD::ConstPtr& msg);
		void chatterCallback_golbalp(const nav_msgs::Odometry::ConstPtr& msg);
		void chatterCallback_compass(const std_msgs::Float64::ConstPtr& msg);
};



/**
 * @brief Callback function for processing local pose messages.
 *
 * This function extracts relevant information from the received PoseStamped message,
 * performs necessary conversions, and updates the Drone's pose member variable.
 *
 * @param msg Const pointer to the received PoseStamped message.
 */
void Drone::chatterCallback_localPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Extract linear position and quaternion components from the received message
    float linearposx = msg->pose.position.x;
    float linearposy = msg->pose.position.y;
    double quatx = msg->pose.orientation.x;
    double quaty = msg->pose.orientation.y;
    double quatz = msg->pose.orientation.z;
    double quatw = msg->pose.orientation.w;

    // Convert quaternion to roll, pitch, and yaw
    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Update the Drone's pose with the converted values
    pose.roll = roll;
    pose.pitch = pitch;
    pose.yaw = yaw;
}


/**
 * @brief Callback function for processing IMU (Inertial Measurement Unit) messages.
 *
 * This function extracts angular velocity components from the received Imu message
 * and updates the Drone's pose member variable with the corresponding roll, pitch, and yaw rates.
 *
 * @param msg Const pointer to the received Imu message.
 */
void Drone::chatterCallback_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Update the Drone's pose with angular velocity components from the IMU message
    pose.rollRate = msg->angular_velocity.x;
    pose.pitchRate = msg->angular_velocity.y;
    pose.yawRate = msg->angular_velocity.z;
}


/**
 * @brief Callback function for processing VFR_HUD (Vehicle-Flown-Report Heads Up Display) messages.
 *
 * This function extracts relevant flight information from the received VFR_HUD message
 * and updates the Drone's pose member variable with ground speed, climb rate, and throttle values.
 *
 * @param msg Const pointer to the received VFR_HUD message.
 */
void Drone::chatterCallback_vfr_hud(const mavros_msgs::VFR_HUD::ConstPtr& msg)
{
    // Update the Drone's pose with ground speed, climb rate, and throttle values from the VFR_HUD message
    pose.groundSpeed = msg->groundspeed;
    pose.climbRate   = msg->climb;
    pose.throttle    = msg->throttle;
}


/**
 * @brief Callback function for processing Odometry messages with global position information.
 *
 * This function extracts the relative altitude information from the received Odometry message
 * and updates the Drone's pose member variable with the relative altitude value.
 *
 * @param msg Const pointer to the received Odometry message.
 */
void Drone::chatterCallback_golbalp(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Update the Drone's pose with the relative altitude value from the Odometry message
    pose.altRelative = msg->pose.pose.position.z;
}


/**
 * @brief Callback function for processing compass heading information.
 *
 * This function extracts the compass heading data from the received Float64 message
 * and updates the Drone's pose member variable with the heading information.
 *
 * @param msg Const pointer to the received Float64 message containing compass heading data.
 */
void Drone::chatterCallback_compass(const std_msgs::Float64::ConstPtr& msg)
{
    // Update the Drone's pose with the heading information from the Float64 message
    pose.heading = msg->data;
}



/*-------------*/
/* Main method */
/*-------------*/

/**
 * @brief Main function for the drone_info ROS node.
 *
 * This function initializes the ROS node, creates a Drone object, subscribes to various
 * topics (GPS, IMU, VFR, Global Position, Heading), and publishes the drone's pose information.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line argument strings.
 * @return int Exit status.
 */
int main(int argc, char **argv) {
    // Initialize the ROS node with an anonymous name
    ros::init(argc, argv, "drone_info", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    
    // Create a Drone object
    Drone drone;

    // Subscribe to ROS topics
    ros::Subscriber GPS  = nh.subscribe("/mavros/local_position/pose", 1, &Drone::chatterCallback_localPose, &drone);
    ros::Subscriber IMU  = nh.subscribe("/mavros/imu/data", 1, &Drone::chatterCallback_imu, &drone);
    ros::Subscriber VFR  = nh.subscribe("/mavros/vfr_hud", 1, &Drone::chatterCallback_vfr_hud, &drone);
    ros::Subscriber GPOS = nh.subscribe("/mavros/global_position/local", 1, &Drone::chatterCallback_golbalp, &drone);
    ros::Subscriber HDG  = nh.subscribe("/mavros/global_position/compass_hdg", 1, &Drone::chatterCallback_compass, &drone);

    // Advertise the pose information to a ROS topic
    ros::Publisher PUB = nh.advertise<harpia_msgs::DronePose>("pose", 1000);

    // Set the loop rate
    ros::Rate loop_rate(10);

    // Display a ROS info message
    ROS_INFO("ROS INFO NODE");

    // Main loop
    while (ros::ok()) {
        // Publish the drone's pose information
        PUB.publish(drone.pose);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

