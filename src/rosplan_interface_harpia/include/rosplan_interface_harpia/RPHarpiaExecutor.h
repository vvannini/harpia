#ifndef RP_HARPIA_EXECUTOR
#define RP_HARPIA_EXECUTOR

#include <ros/ros.h>
#include <vector>

#include <rosplan_action_interface/RPActionInterface.h>

namespace KCL_rosplan {

	class RPHarpiaExecutor: public RPActionInterface
	{

	private:

	public:

		/* constructor */
		RPHarpiaExecutor(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
