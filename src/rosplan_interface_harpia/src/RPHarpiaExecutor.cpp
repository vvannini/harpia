#include <iostream>
#include "rosplan_interface_harpia/RPHarpiaExecutor.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/WaypointList.h"
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <bits/stdc++.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <harpia_msgs/Mission.h>
#include <harpia_msgs/ChangeMission.h>
#include <harpia_msgs/UAV.h>
#include <harpia_msgs/Map.h>
#include <harpia_msgs/Goal.h>
#include <harpia_msgs/RegionPoint.h>
#include <harpia_msgs/MissionPlannerAction.h>
#include <harpia_msgs/MissionFaultMitigation.h>
#include <harpia_msgs/RegionPoint.h>
#include <harpia_msgs/PathPlanning.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "harpia_msgs/MissionPlannerActionGoal.h"
#include "actionlib_msgs/GoalID.h"

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
	GeoPoint position;
	mavros_msgs::State current_state;
	mavros_msgs::ExtendedState ex_current_state;
	void chatterCallback_GPS(const sensor_msgs::NavSatFix::ConstPtr& msg);
	void chatterCallback_currentState(const mavros_msgs::State::ConstPtr& msg);
	void chatterCallback_currentStateExtended(const mavros_msgs::ExtendedState::ConstPtr& msg);
};

void Drone::chatterCallback_GPS(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	position.longitude = msg->longitude;
	position.latitude = msg->latitude;
	position.altitude = msg->altitude;
}

void Drone::chatterCallback_currentState(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void Drone::chatterCallback_currentStateExtended(const mavros_msgs::ExtendedState::ConstPtr& msg){
    ex_current_state = *msg;
}

// detach route from mission
class Mission
{
	public:
		int WPqtd; // waypoint qtd
		int currentWP;
		int IDGoal;
		bool Ended = true;
		bool Cancelled = false;
		harpia_msgs::Mission hMission;
		// decision_support::newMission missionWP;
		void send_mission();
		void chatterCallback_wpqtd(const mavros_msgs::WaypointList::ConstPtr& msg);
		void chatterCallback_current(const mavros_msgs::WaypointReached::ConstPtr& msg);
		void chatterCallback_harpiaMission(const harpia_msgs::Mission::ConstPtr& msg);
		void chatterCallback_IDGoal(const harpia_msgs::MissionPlannerActionGoal::ConstPtr& msg);
		void chatterCallback_cancelGoal(const harpia_msgs::ChangeMission::ConstPtr& msg);

		Mission();


};

Mission::Mission(void)
{
	Ended = true;
}


void Mission::chatterCallback_wpqtd(const mavros_msgs::WaypointList::ConstPtr& msg)
{
	WPqtd = msg->waypoints.size()-1;
}

void Mission::chatterCallback_IDGoal(const harpia_msgs::MissionPlannerActionGoal::ConstPtr& msg)
{
	IDGoal = atoi(msg->goal_id.id.c_str());
	// ROS_INFO("id Goal: %i", IDGoal);
}

void Mission::chatterCallback_current(const mavros_msgs::WaypointReached::ConstPtr& msg)
{
	if(currentWP!=msg->wp_seq)
	{
		currentWP = msg->wp_seq;
		ROS_INFO("Waypoint: %i", msg->wp_seq+1);
	}
	if(WPqtd == msg->wp_seq)
	{
		Ended = true;
	}
	else
		Ended = false;

}

void Mission::chatterCallback_cancelGoal(const harpia_msgs::ChangeMission::ConstPtr& msg)
{

	if(msg->op != 0)
	{
		Cancelled = true; 
	}
	else
		Cancelled = false;
		// ROS_INFO("%s", msg.goals[0]);
		// ROS_INFO("%li", msg.op);

	// ROS_INFO("msg: %li", sizeof(msg));
}

void Mission::chatterCallback_harpiaMission(const harpia_msgs::Mission::ConstPtr& msg)
{
	hMission.uav = msg->uav;
	hMission.map = msg->map;
	hMission.goals = msg->goals;
}

Mission mission;
Drone drone;


void land(Drone drone)
{
	ros::NodeHandle n;
	ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 0;
    srv_land.request.latitude = drone.position.latitude;//0;
    srv_land.request.longitude = drone.position.longitude;//0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if(land_cl.call(srv_land))
    {
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }
    else
    {
        ROS_ERROR("Failed Land");
    }
}

void set_loiter()
{
	ros::NodeHandle n;
	ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    //srv_setMode.request.custom_mode = "AUTO.LOITER";
    srv_setMode.request.custom_mode = "AUTO.LOITER";
    if(cl.call(srv_setMode))
    {
        //ROS_INFO("AUTO.LOITER");
        ROS_INFO("LOITER");
    }
    else
    {
        ROS_ERROR("Failed SetMode");
    }
}

void set_auto()
{
	ros::NodeHandle n;
	ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "AUTO.MISSION";
    //srv_setMode.request.custom_mode = "AUTO";
    if(cl.call(srv_setMode))
    {
        //ROS_INFO("AUTO.MISSION");
        ROS_INFO("AUTO");
    }
    else
    {
        ROS_ERROR("Failed SetMode");
    }
}

void arm()
{
	ros::NodeHandle n;
	ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_INFO("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }
}

void takeoff(Drone drone)
{
	ros::NodeHandle n;
	ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

    mavros_msgs::CommandTOL srv_takeoff;
    // ROS_INFO("Takeoff at %f, at %f", drone.position.altitude+5, drone.position.altitude);
    srv_takeoff.request.altitude = 15;
    srv_takeoff.request.latitude = drone.position.latitude;//-12.82046769976293;
    srv_takeoff.request.longitude = drone.position.longitude;//-50.33633513165995;
    //srv_takeoff.request.latitude = 0;
    //srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if(takeoff_cl.call(srv_takeoff)){
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
    }
}

harpia_msgs::RegionPoint getGeoPoint(GeoPoint geo, harpia_msgs::Map mapa)
{
	int qtd_regions = mapa.roi.size();
	int i;
	for(i = 0; i<qtd_regions; i++)
		if(strcmp(mapa.roi[i].name.c_str(), geo.name.c_str()) == 0 )
		{
			ROS_INFO("%s", mapa.roi[i].name.c_str());
			return mapa.roi[i].center;
		}
	qtd_regions = mapa.bases.size();
	for(int i = 0; i<qtd_regions; i++)
		if(strcmp(mapa.bases[i].name.c_str(), geo.name.c_str()) == 0 )
		{
			ROS_INFO("%s", mapa.bases[i].name.c_str());
			return mapa.bases[i].center;
		}
	harpia_msgs::RegionPoint null;
	return null;


}

int getRadius(string region)
{
	string command = "python3 ~/drone_arch/drone_ws/src/ROSPlan/src/rosplan/rosplan_planning_system/src/ActionInterface/getRadius.py "+region+" >> ~/drone_arch/Data/out.txt";
	system(command.c_str());
	// cout << result;
	string line;
  	ifstream myfile ((homepath + "/drone_arch/Data/out.txt").c_str());
  	if (myfile.is_open())
 	{
 		cout << "file opened" << endl;
 		getline (myfile,line);
 		int radius = stod(line);
    	myfile.clear();
    	myfile.close();

    	return radius;
  	}
  	else
  		cout << "Unable to open file";
  		return 10.0;
}

mavros_msgs::WaypointList calcRoute(harpia_msgs::RegionPoint from, harpia_msgs::RegionPoint to, string name_from, string name_to, harpia_msgs::Map map) // TODO VERONICA arrumar chamada da funcao
{
	ros::NodeHandle n;
 	ros::ServiceClient client = n.serviceClient<harpia_msgs::PathPlanning>("harpia/path_planning");
  	harpia_msgs::PathPlanning srv;
  	srv.request.r_from = from;
  	srv.request.r_to = to;
  	srv.request.name_from = name_from;
  	srv.request.name_to = name_to;
  	srv.request.op = 0;
  	srv.request.map = map;

  	if (client.call(srv))
  	{
    	return srv.response.waypoints;
  	}
  	else
  	{
    	ROS_ERROR("Failed to call service harpia/path_planning");
    	mavros_msgs::WaypointList null;
    	return null;
  	}
}

mavros_msgs::WaypointList calcRoute_pulverize(harpia_msgs::RegionPoint at, harpia_msgs::Map map)
{
	ros::NodeHandle n;
 	ros::ServiceClient client = n.serviceClient<harpia_msgs::PathPlanning>("harpia/path_planning");
  	harpia_msgs::PathPlanning srv;
  	srv.request.r_from = at;
  	srv.request.r_to = at;
  	srv.request.op = 1;
  	srv.request.name_from = "at";
  	srv.request.name_to = "pulverize_region";
  	srv.request.map = map;

  	if (client.call(srv))
  	{
    	return srv.response.waypoints;
  	}
  	else
  	{
    	ROS_ERROR("Failed to call service harpia/path_planning");
    	mavros_msgs::WaypointList null;
    	return null;
  	}
}

mavros_msgs::WaypointList calcRoute_picture(harpia_msgs::RegionPoint at, harpia_msgs::Map map)
{
	// string command = "python3 ~/harpia/path_planners/simple-behaivors/square.py "+to_string(at.longitude)+" "+to_string(at.latitude)+" "+to_string(at.altitude)+ " 250";
	// system(command.c_str());

	ros::NodeHandle n;
 	ros::ServiceClient client = n.serviceClient<harpia_msgs::PathPlanning>("harpia/path_planning");
  	harpia_msgs::PathPlanning srv;
  	srv.request.r_from = at;
  	srv.request.r_to = at;
  	srv.request.op = 2;
  	srv.request.name_from = "at";
  	srv.request.name_to = "take_picture";
  	srv.request.map = map;

  	if (client.call(srv))
  	{
    	return srv.response.waypoints;
  	}
  	else
  	{
    	ROS_ERROR("Failed to call service harpia/path_planning");
    	mavros_msgs::WaypointList null;
    	return null;
  	}
}

int sendWPFile(mavros_msgs::WaypointList mission_wp)
{
	ros::NodeHandle p;
	GeoPoint geo;
	string line;
	int wp_count = 1 ;
	//mavros_msgs::Waypoint* mission_wp = NULL;
	mavros_msgs::WaypointPush wp_push_srv;
	mavros_msgs::WaypointClear wp_clear_srv;


	ros::ServiceClient wp_srv_client = p.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
	ros::ServiceClient wp_clear_client = p.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");


 	wp_clear_srv.request = {};

  	if (wp_clear_client.call(wp_clear_srv))
	{
	    ROS_INFO("Waypoint list was cleared");
	}
	else
	{
	    ROS_ERROR("Waypoint list couldn't been cleared");
	}

  	wp_push_srv.request.start_index = 0;
  	wp_count = mission_wp.waypoints.size();
  	cout << wp_count << endl;
  	for(int n=0; n<wp_count; n++)
  		wp_push_srv.request.waypoints.push_back(mission_wp.waypoints[n]);

  	cout << wp_srv_client.call(wp_push_srv) << endl;

  	if(wp_srv_client.call(wp_push_srv))
  	{
 	   ROS_INFO("Success:%d", (bool)wp_push_srv.response.success);
 	   remove((homepath + "/drone_arch/Data/route.txt").c_str());
 	}
 	else
 	{
  	  ROS_ERROR("Waypoint couldn't been sent");
  	  ROS_ERROR("Success:%d", (bool)wp_push_srv.response.success);
 	  remove((homepath + "/drone_arch/Data/route.txt").c_str());
  	  return 0;
  	}


  	return 1;
}

void reset_mission()
{
	ros::NodeHandle nh;
	ros::ServiceClient set_current_client = nh.serviceClient<mavros_msgs::WaypointSetCurrent>("mavros/mission/set_current");
	mavros_msgs::WaypointSetCurrent set_current_srv;

	set_current_srv.request.wp_seq = 0;

  	if (set_current_client.call(set_current_srv))
	{
	    ROS_INFO("Reset Mission");
	}
	else
	{
	    ROS_ERROR("Reset couldn't been done");
	}

}

void callRoute(GeoPoint from, GeoPoint to)
{
	ROS_INFO("Sending WP file for route %s _ %s.wp", from.name.c_str(), to.name.c_str());
	string command = "rosrun mavros mavwp load ~/drone_arch/Data/Rotas/wp/"+from.name+"_"+to.name+".wp";
	ROS_INFO("%s", command.c_str());
	system(command.c_str());
}

geometry_msgs::Point convert_goe_to_cart(geographic_msgs::GeoPoint p, geographic_msgs::GeoPoint home)
{
	geometry_msgs::Point point;
	double pi = 2*acos(0.0);
	point.x = (p.longitude - home.longitude) * (6400000.0 * (cos(home.latitude * pi / 180) * 2 * pi / 360));
	point.y =(p.latitude - home.latitude)  * (10000000.0 / 90);

	return point;
}

harpia_msgs::RegionPoint create_RegionPoint(GeoPoint point, harpia_msgs::Map map)
{
	harpia_msgs::RegionPoint region_point;

	region_point.geo.latitude = point.latitude;
	region_point.geo.longitude = point.longitude;
	region_point.geo.altitude = point.altitude;

	region_point.cartesian = convert_goe_to_cart(region_point.geo, map.geo_home);


	return region_point;
}

/*--------------------------------------------*/
void mySigintHandler(int sig)
{
	// Do some custom action.
	set_loiter();
	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}
/*--------------------------------------------*/


namespace KCL_rosplan {
	RPHarpiaExecutor::RPHarpiaExecutor(ros::NodeHandle &nh) {}

	/* action dispatch callback */
	bool RPHarpiaExecutor::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ros::NodeHandle n;
	 	ros::ServiceClient client = n.serviceClient<harpia_msgs::MissionFaultMitigation>("harpia/mission_fault_mitigation");
	  	harpia_msgs::MissionFaultMitigation srv;
	  	srv.request.uav = mission.hMission.uav;
	  	srv.request.action_id = msg->action_id;


	  	int replan, cancelled;

	  	if (client.call(srv) && msg->action_id != 0)
	  	{
	    	replan = srv.response.replan;
	  	}
	  	else
	  	{
	    	ROS_INFO("BN not called");
	    	replan = 0;
	  	}
	 //  	if(mission.Cancelled)
		// {
		// 	ROS_ERROR("Preempted %s", msg->name.c_str());
		// 	set_loiter();
		// 	mission.Cancelled = false;
		// 	return false;
		// }
	  	if(replan != 1)
		{
            // There was no need for replan.

            // The action implementation goes here.
			string str = msg->name.c_str();
			string str1 = "go_to";
			size_t found = str.find(str1);
			ROS_INFO("%s", msg->name.c_str());
	    	if (found != string::npos)
			{
                // We are on the way and there is no need for replan.

				mission.Ended = false;
				GeoPoint from, to;
				harpia_msgs::RegionPoint r_from, r_to;
				mavros_msgs::WaypointList route;

				//get coordinates
				from.name = msg->parameters[0].value.c_str();
				to.name = msg->parameters[1].value.c_str();
				ROS_INFO("go_to %s -> %s", from.name.c_str(), to.name.c_str());

				//getGeoPoint(&from);
				from.latitude = drone.position.latitude;
				from.longitude = drone.position.longitude;
				from.altitude = 15;

				r_from = create_RegionPoint(from, mission.hMission.map);
				r_to = getGeoPoint(to, mission.hMission.map);



				ROS_INFO("GEO GeoPoint %f %f %f -> %f %f %f", r_from.geo.latitude, r_from.geo.longitude, r_from.geo.altitude, r_to.geo.latitude, r_to.geo.longitude, r_to.geo.altitude);


				//calc route
				route = calcRoute(r_from, r_to, from.name, to.name, mission.hMission.map);

				//is flying?
					//is armed
				while(!drone.current_state.armed && drone.ex_current_state.landed_state != 2)
				{
					set_loiter();
					arm();
					takeoff(drone);
				}
				ros::Duration(10).sleep();

				//send route
				if(!sendWPFile(route))
					callRoute(from, to);
				ros::Duration(20).sleep();


				set_auto();

				//while not at the end, wait
				while(!mission.Ended){
					ros::Duration(10).sleep();
				}

				set_loiter();
				ros::Duration(10).sleep();

				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;

			}
			else if (strcmp(msg->name.c_str(), "pulverize_region") == 0)
			{
				// string region = msg->parameters[1].value.c_str();

				// ROS_INFO("pulverize_region %s", region.c_str());

				ROS_INFO("pulverize_region");
				mavros_msgs::WaypointList route;
				// string region = msg->parameters[1].value.c_str();
				mission.Ended = false;

				//get coordinates
				//int radius = getRadius(region);

				//calc route
				GeoPoint at;
				at.latitude = drone.position.latitude;
				at.longitude = drone.position.longitude;
				at.altitude = 15;
				harpia_msgs::RegionPoint r_at;
				r_at = create_RegionPoint(at, mission.hMission.map);
				route = calcRoute_pulverize(r_at, mission.hMission.map);


				//is flying?
					//is armed
				while(!drone.current_state.armed && drone.ex_current_state.landed_state != 2)
				{
					set_loiter();
					arm();
					takeoff(drone);
				}
				ros::Duration(20).sleep();

				//send route
				if(!sendWPFile(route))
					ROS_ERROR("Error call route pulverize_region");
				ros::Duration(20).sleep();

				//change to auto
				set_auto();

				while(!mission.Ended){
					ros::Duration(10).sleep();
				}
				//while not at the end, wait
				set_loiter();
				ros::Duration(10).sleep();

				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "take_image") == 0)
			{
				ROS_INFO("take_image");
				string region = msg->parameters[0].value.c_str();
				mission.Ended = false;
				mavros_msgs::WaypointList route;

				//get coordinates
				//int radius = getRadius(region);

				//calc route
				GeoPoint at;
				at.latitude = drone.position.latitude;
				at.longitude = drone.position.longitude;
				at.altitude = 15;
				harpia_msgs::RegionPoint r_at;
				r_at = create_RegionPoint(at, mission.hMission.map);
				route = calcRoute_picture(r_at, mission.hMission.map);

				//is flying?
					//is armed
				while(!drone.current_state.armed && drone.ex_current_state.landed_state != 2)
				{
					set_loiter();
					arm();
					takeoff(drone);
				}
				ros::Duration(20).sleep();

				//send route
				if(!sendWPFile(route))
					ROS_ERROR("Error call route pulverize_region");
				ros::Duration(20).sleep();

				//change to auto
				set_auto();

				//while not at the end, wait
				while(!mission.Ended){
					ros::Duration(10).sleep();
				}

				set_loiter();
				ros::Duration(10).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "recharge_input") == 0)
			{
				ROS_INFO("recharge_input");

				if(drone.current_state.mode != "AUTO.LAND")
					land(drone);

				while(drone.ex_current_state.landed_state != 1)
				{
					ROS_INFO("landing... %d", drone.ex_current_state.landed_state);
					ros::Duration(10).sleep();
				}


				ros::Duration(msg->duration).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "clean_camera") == 0)
			{
				ROS_INFO("clean_camera");
				if(drone.current_state.mode != "AUTO.LAND")
					land(drone);

				while(drone.ex_current_state.landed_state != 1)
				{
					ROS_INFO("landing... %d", drone.ex_current_state.landed_state);
					ros::Duration(10).sleep();
				}
				ros::Duration(msg->duration).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "recharge_battery") == 0)
			{
				ROS_INFO("recharge_battery");
				if(drone.current_state.mode != "AUTO.LAND")
					land(drone);

				while(drone.ex_current_state.landed_state != 1)
				{
					ROS_INFO("landing... %d", drone.ex_current_state.landed_state);
					ros::Duration(10).sleep();
				}
				ros::Duration(msg->duration).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "has_all_goals_achived") == 0)
			{
				ROS_INFO("has-all-goals-achived");

				ros::Duration(1).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "need_battery") == 0)
			{
				ROS_INFO("need-battery");

				ros::Duration(msg->duration).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "need_input") == 0)
			{
				ROS_INFO("need-input");

				ros::Duration(msg->duration).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}

			if(mission.Cancelled)
			{
				ROS_ERROR("Preempted %s", msg->name.c_str());
				set_loiter();
				mission.Cancelled = false;
				return false;
			}

			return true;
		}
		else
		{
			ROS_INFO("NEED TO REPLAN");
			return false;
		}
		// verify if has cancelled current mission

		

	}
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_interface_harpia", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    signal(SIGINT, mySigintHandler);

    ros::Subscriber GPS 				= nh.subscribe("/mavros/global_position/global", 		1, &Drone::chatterCallback_GPS, 				&drone);
    ros::Subscriber state 	 			= nh.subscribe("/mavros/state", 				 		1, &Drone::chatterCallback_currentState, 		&drone);
    ros::Subscriber state_ext 			= nh.subscribe("/mavros/extended_state", 		 		1, &Drone::chatterCallback_currentStateExtended,&drone);
    ros::Subscriber global 				= nh.subscribe("/mavros/mission/waypoints", 	 		1, &Mission::chatterCallback_wpqtd, 			&mission);
    ros::Subscriber current 			= nh.subscribe("/mavros/mission/reached", 		 		1, &Mission::chatterCallback_current, 			&mission);
    ros::Subscriber harpia_mission 		= nh.subscribe("/harpia/mission", 				 		1, &Mission::chatterCallback_harpiaMission, 	&mission);
    ros::Subscriber harpia_goalId 		= nh.subscribe("/harpia/mission_goal_manager/goal", 	1, &Mission::chatterCallback_IDGoal, 			&mission);
    ros::Subscriber harpia_goalCancel 	= nh.subscribe("/harpia/ChangeMission", 				1, &Mission::chatterCallback_cancelGoal, 		&mission);

    // create PDDL action subscriber
    KCL_rosplan::RPHarpiaExecutor rpti(nh);

    rpti.runActionInterface();

    return 0;
}
