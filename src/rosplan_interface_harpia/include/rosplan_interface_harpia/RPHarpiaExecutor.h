#ifndef RP_HARPIA_EXECUTOR
#define RP_HARPIA_EXECUTOR

#include <ros/ros.h>
#include <vector>

#include <rosplan_action_interface/RPActionInterface.h>

namespace KCL_rosplan {

	/**
     * @class RPHarpiaExecutor
     * @brief A class that defines an action interface for executing missions in the Harpia UAV system.
     *
     * This class is responsible for processing action dispatch messages and executing corresponding missions.
     */
	class RPHarpiaExecutor: public RPActionInterface
	{

	private:

	public:

        /**
         * @brief Constructor for the RPHarpiaExecutor class.
         * @param nh A ROS NodeHandle for the interface to communicate with ROS.
         */
		RPHarpiaExecutor(ros::NodeHandle &nh);

		/**
         * @brief Callback function to listen and process action dispatch messages.
         * @param msg A pointer to the received ActionDispatch message.
         * @return True if the action was executed successfully, false otherwise.
         */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
