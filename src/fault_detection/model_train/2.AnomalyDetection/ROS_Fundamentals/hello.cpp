//-------------------------------------------------------------
// This is a ROS version of the standard "hello world" program
//-------------------------------------------------------------

// This header defines the standard ROS classes.
#include <ros/ros.h>

int main(int argc, char **argv){

	// initialize the ROS system.
	ros::init(argc, argv, "hello_ros"); //-> the last parameter 
	                                    //   is a string containing 
	                                    //   the default name of our node
										//   (can even be overriden by a launch file)


	// Establish this program as a ROS node.
	ros::NodeHandle nh;


	// Send some output as a log message.
	ROS_INFO_STREAM("Hello, ROS!");

}