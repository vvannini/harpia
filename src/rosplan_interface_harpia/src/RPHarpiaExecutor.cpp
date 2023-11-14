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

/**
 * @class Drone
 * @brief Represents a class for managing drone-related information.
 *
 * The `Drone` class is designed to store and update information related to a drone's
 * position and state within a larger system.
 */
class Drone
{
public:
    GeoPoint position;                       // Geographic position information of the drone.
    mavros_msgs::State current_state;         // Current state of the drone.
    mavros_msgs::ExtendedState ex_current_state; // Extended state of the drone.

    /**
     * @brief Callback function for updating the drone's GPS position.
     *
     * This function is used as a callback to update the drone's geographic position
     * based on the data received from a `sensor_msgs::NavSatFix` message.
     *
     * @param msg A pointer to the `sensor_msgs::NavSatFix` message containing GPS data.
     */
    void chatterCallback_GPS(const sensor_msgs::NavSatFix::ConstPtr& msg);

    /**
     * @brief Callback function for updating the drone's current state.
     *
     * This function is used as a callback to update the drone's current state
     * based on the data received from a `mavros_msgs::State` message.
     *
     * @param msg A pointer to the `mavros_msgs::State` message containing state information.
     */
    void chatterCallback_currentState(const mavros_msgs::State::ConstPtr& msg);

    /**
     * @brief Callback function for updating the drone's extended state.
     *
     * This function is used as a callback to update the drone's extended state
     * based on the data received from a `mavros_msgs::ExtendedState` message.
     *
     * @param msg A pointer to the `mavros_msgs::ExtendedState` message containing extended state information.
     */
    void chatterCallback_currentStateExtended(const mavros_msgs::ExtendedState::ConstPtr& msg);
};

/**
 * @brief Updates the drone's position with GPS data.
 *
 * This function is called when GPS data is received. It extracts the longitude, latitude, and altitude
 * information from the provided `NavSatFix` message and updates the `position` member accordingly.
 *
 * @param msg A pointer to the `sensor_msgs::NavSatFix` message containing GPS data.
 */
void Drone::chatterCallback_GPS(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    position.longitude = msg->longitude;
    position.latitude = msg->latitude;
    position.altitude = msg->altitude;
}

/**
 * @brief Updates the drone's current state.
 *
 * This function is called to update the drone's current state based on the data
 * received from the provided `State` message. It copies the message data to the `current_state` member.
 *
 * @param msg A pointer to the `mavros_msgs::State` message containing state information.
 */
void Drone::chatterCallback_currentState(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

/**
 * @brief Updates the drone's extended state.
 *
 * This function is called to update the drone's extended state based on the data
 * received from the provided `ExtendedState` message. It copies the message data to the `ex_current_state` member.
 *
 * @param msg A pointer to the `mavros_msgs::ExtendedState` message containing extended state information.
 */
void Drone::chatterCallback_currentStateExtended(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
    ex_current_state = *msg;
}

// detach route from mission

/**
 * @class Mission
 * @brief Represents a class for managing and controlling mission-related data.
 *
 * The `Mission` class is responsible for managing mission-related information, such as
 * waypoints, mission progress, and cancellation status. It provides functions to send missions
 * and updates mission-related variables based on incoming messages.
 */
class Mission
{
public:
    int WPqtd;                             // Stores the number of waypoints in the mission.
    int currentWP;                         // Stores the current waypoint being executed.
    int IDGoal;                            // Stores the ID of the current mission goal.
    bool Ended = true;                    // Flag indicating if the mission has ended.
    bool Cancelled = false;               // Flag indicating if the mission has been cancelled.
    harpia_msgs::Mission hMission;         // Stores a message representing a mission.
    
    /**
     * @brief Constructor for the Mission class.
     *
     * Initializes the `Ended` flag to `true` when an instance of the class is created.
     */
    Mission();

    /**
     * @brief Sends the mission to be executed.
     *
     * This function is used to send the mission represented by the `hMission` member.
     */
    void send_mission();

    /**
     * @brief Callback function to update the number of waypoints in the mission.
     *
     * This function is called when a `mavros_msgs::WaypointList` message is received,
     * and it updates the `WPqtd` member based on the number of waypoints in the message.
     *
     * @param msg A pointer to the `mavros_msgs::WaypointList` message containing the list of waypoints.
     */
    void chatterCallback_wpqtd(const mavros_msgs::WaypointList::ConstPtr& msg);

    /**
     * @brief Callback function to update the ID of the current goal.
     *
     * This function is called when a `harpia_msgs::MissionPlannerActionGoal` message is received,
     * and it updates the `IDGoal` member based on the goal ID in the message.
     *
     * @param msg A pointer to the `harpia_msgs::MissionPlannerActionGoal` message containing goal information.
     */
    void chatterCallback_IDGoal(const harpia_msgs::MissionPlannerActionGoal::ConstPtr& msg);

    /**
     * @brief Callback function to update the current waypoint being executed and mission status.
     *
     * This function is called when a `mavros_msgs::WaypointReached` message is received. It updates
     * the `currentWP` member based on the current waypoint sequence and the `Ended` flag based on the
     * mission progress.
     *
     * @param msg A pointer to the `mavros_msgs::WaypointReached` message containing waypoint information.
     */
    void chatterCallback_current(const mavros_msgs::WaypointReached::ConstPtr& msg);

    /**
     * @brief Callback function to handle mission cancellation.
     *
     * This function is called when a `harpia_msgs::ChangeMission` message is received. It updates
     * the `Cancelled` flag based on the operation code in the message.
     *
     * @param msg A pointer to the `harpia_msgs::ChangeMission` message containing mission change information.
     */
    void chatterCallback_cancelGoal(const harpia_msgs::ChangeMission::ConstPtr& msg);

    /**
     * @brief Callback function to update the mission information.
     *
     * This function is called when a `harpia_msgs::Mission` message is received. It updates
     * the `hMission` member based on the data in the message.
     *
     * @param msg A pointer to the `harpia_msgs::Mission` message containing mission data.
     */
    void chatterCallback_harpiaMission(const harpia_msgs::Mission::ConstPtr& msg);
};

/**
 * @brief Constructor for the Mission class.
 *
 * Initializes the `Ended` flag to `true` when an instance of the class is created.
 */
Mission::Mission(void)
{
    Ended = true;
}


/**
 * @brief Callback function to update the number of waypoints in the mission.
 *
 * This function is called when a `mavros_msgs::WaypointList` message is received,
 * and it updates the `WPqtd` member based on the number of waypoints in the message.
 *
 * @param msg A pointer to the `mavros_msgs::WaypointList` message containing the list of waypoints.
 */
void Mission::chatterCallback_wpqtd(const mavros_msgs::WaypointList::ConstPtr& msg)
{
    WPqtd = msg->waypoints.size() - 1;
}

/**
 * @brief Callback function to update the ID of the current goal.
 *
 * This function is called when a `harpia_msgs::MissionPlannerActionGoal` message is received,
 * and it updates the `IDGoal` member based on the goal ID in the message.
 *
 * @param msg A pointer to the `harpia_msgs::MissionPlannerActionGoal` message containing goal information.
 */
void Mission::chatterCallback_IDGoal(const harpia_msgs::MissionPlannerActionGoal::ConstPtr& msg)
{
    IDGoal = atoi(msg->goal_id.id.c_str());
    // ROS_INFO("id Goal: %i", IDGoal);
}

/**
 * @brief Callback function to update the current waypoint being executed and mission status.
 *
 * This function is called when a `mavros_msgs::WaypointReached` message is received. It updates
 * the `currentWP` member based on the current waypoint sequence and the `Ended` flag based on the
 * mission progress.
 *
 * @param msg A pointer to the `mavros_msgs::WaypointReached` message containing waypoint information.
 */
void Mission::chatterCallback_current(const mavros_msgs::WaypointReached::ConstPtr& msg)
{
    if (currentWP != msg->wp_seq)
    {
        currentWP = msg->wp_seq;
        ROS_INFO("Waypoint: %i", msg->wp_seq + 1);
    }
    if (WPqtd == msg->wp_seq)
    {
        Ended = true;
    }
    else
        Ended = false;
}

/**
 * @brief Callback function to handle mission cancellation.
 *
 * This function is called when a `harpia_msgs::ChangeMission` message is received. It updates
 * the `Cancelled` flag based on the operation code in the message.
 *
 * @param msg A pointer to the `harpia_msgs::ChangeMission` message containing mission change information.
 */
void Mission::chatterCallback_cancelGoal(const harpia_msgs::ChangeMission::ConstPtr& msg)
{
    if (msg->op != 0)
    {
        Cancelled = true;
    }
    else
        Cancelled = false;
    // ROS_INFO("%s", msg.goals[0]);
    // ROS_INFO("%li", msg.op);

    // ROS_INFO("msg: %li", sizeof(msg));
}

/**
 * @brief Callback function to update the mission information.
 *
 * This function is called when a `harpia_msgs::Mission` message is received. It updates
 * the `hMission` member based on the data in the message.
 *
 * @param msg A pointer to the `harpia_msgs::Mission` message containing mission data.
 */
void Mission::chatterCallback_harpiaMission(const harpia_msgs::Mission::ConstPtr& msg)
{
    hMission.uav = msg->uav;
    hMission.map = msg->map;
    hMission.goals = msg->goals;
}

// Create instances of the Mission and Drone classes
Mission mission;
Drone drone;



/**
 * @brief Initiates a landing procedure for the drone at its current position.
 *
 * This function sends a command to the drone to initiate the landing procedure.
 * It communicates with the `mavros` package to perform the landing action.
 *
 * @param drone The instance of the `Drone` class representing the drone for which the landing command is issued.
 */
void land(Drone drone)
{
    ros::NodeHandle n;
    
    // Create a service client for sending the landing command
    ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    
    mavros_msgs::CommandTOL srv_land;
    
    // Set parameters for the landing command
    srv_land.request.altitude = 0;
    srv_land.request.latitude = drone.position.latitude;
    srv_land.request.longitude = drone.position.longitude;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    
    // Send the landing command
    if (land_cl.call(srv_land))
    {
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }
    else
    {
        ROS_ERROR("Failed Land");
    }
}


/**
 * @brief Sets the drone to the loiter mode, causing it to hover in its current location.
 *
 * This function sends a command to the drone to switch to the "LOITER" mode, which instructs the drone to
 * hover in its current location while maintaining altitude and position.
 *
 * @note This function communicates with the `mavros` package to set the mode.
 */
void set_loiter()
{
    ros::NodeHandle n;
    
    // Create a service client for setting the mode
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    
    // Set the base mode to 0 (meaning no change in the base mode)
    srv_setMode.request.base_mode = 0;
    
    // Set the custom mode to "AUTO.LOITER" to switch to the loiter mode
    srv_setMode.request.custom_mode = "AUTO.LOITER";
    
    // Send the mode change command
    if (cl.call(srv_setMode))
    {
        // Print a success message when the mode is set to "LOITER"
        ROS_INFO("LOITER");
    }
    else
    {
        // Print an error message if setting the mode fails
        ROS_ERROR("Failed SetMode");
    }
}


/**
 * @brief Sets the drone to the auto mode, enabling it to follow a predefined mission.
 *
 * This function sends a command to the drone to switch to the "AUTO" mode or "AUTO.MISSION" mode,
 * which allows the drone to follow a predefined mission, typically a sequence of waypoints or commands.
 *
 * @note This function communicates with the `mavros` package to set the mode.
 */
void set_auto()
{
    ros::NodeHandle n;
    
    // Create a service client for setting the mode
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    
    // Set the base mode to 0 (meaning no change in the base mode)
    srv_setMode.request.base_mode = 0;
    
    // Set the custom mode to "AUTO" or "AUTO.MISSION" to enable auto mode
    srv_setMode.request.custom_mode = "AUTO";
    
    // Send the mode change command
    if (cl.call(srv_setMode))
    {
        // Print a success message when the mode is set to "AUTO"
        ROS_INFO("AUTO");
    }
    else
    {
        // Print an error message if setting the mode fails
        ROS_ERROR("Failed SetMode");
    }
}


/**
 * @brief Arms the drone, enabling the propellers for flight.
 *
 * This function sends a command to the drone to arm its propellers, allowing it to take off and enter flight mode.
 *
 * @note This function communicates with the `mavros` package to perform the arming action.
 */
void arm()
{
    ros::NodeHandle n;
    
    // Create a service client for arming the drone
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    
    // Set the request value to true, indicating the intent to arm the drone
    srv.request.value = true;
    
    // Send the arming command to the drone
    if (arming_cl.call(srv))
    {
        // Print a success message when the arming command is sent successfully
        ROS_INFO("ARM send ok %d", srv.response.success);
    }
    else
    {
        // Print an error message if arming or disarming fails
        ROS_ERROR("Failed arming or disarming");
    }
}


/**
 * @brief Initiates the takeoff procedure for the drone to ascend to a specified altitude.
 *
 * This function communicates with the `mavros` package to command the drone to take off and reach a target altitude.
 *
 * @param drone The drone object that provides information about its current position.
 *
 * @note The drone's position (latitude and longitude) should be set before calling this function.
 */
void takeoff(Drone drone)
{
    ros::NodeHandle n;

    // Create a service client for initiating takeoff
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

    mavros_msgs::CommandTOL srv_takeoff;

    // Set the target altitude for the drone's takeoff (e.g., 15 meters)
    srv_takeoff.request.altitude = 15;
    srv_takeoff.request.latitude = drone.position.latitude;
    srv_takeoff.request.longitude = drone.position.longitude;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;

    // Send the takeoff command to the drone
    if (takeoff_cl.call(srv_takeoff))
    {
        // Print a success message when the takeoff command is sent successfully
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
    }
    else
    {
        // Print an error message if the takeoff procedure fails
        ROS_ERROR("Failed Takeoff");
    }
}


/**
 * @brief Retrieve the geographical center point of a specific region or base from a map.
 *
 * This function searches for a region or base with a matching name in the provided map and returns its center point.
 *
 * @param geo The `GeoPoint` object containing the name of the region or base to look for.
 * @param mapa The `harpia_msgs::Map` object representing the map containing regions and bases.
 *
 * @return A `harpia_msgs::RegionPoint` object representing the center point of the found region or base. If no matching region or base is found, a default `null` `harpia_msgs::RegionPoint` is returned.
 *
 * @note Ensure that the provided `GeoPoint` object contains a valid name for the region or base to be searched.
 */
harpia_msgs::RegionPoint getGeoPoint(GeoPoint geo, harpia_msgs::Map mapa)
{
    int qtd_regions = mapa.roi.size();
    int i;
    
    // Search for a matching region by comparing names
    for(i = 0; i < qtd_regions; i++)
    {
        if (strcmp(mapa.roi[i].name.c_str(), geo.name.c_str()) == 0)
        {
            // Print the found region's name and return its center point
            ROS_INFO("%s", mapa.roi[i].name.c_str());
            return mapa.roi[i].center;
        }
    }
    
    qtd_regions = mapa.bases.size();
    
    // Search for a matching base by comparing names
    for (int i = 0; i < qtd_regions; i++)
    {
        if (strcmp(mapa.bases[i].name.c_str(), geo.name.c_str()) == 0)
        {
            // Print the found base's name and return its center point
            ROS_INFO("%s", mapa.bases[i].name.c_str());
            return mapa.bases[i].center;
        }
    }
    
    // Return a default null RegionPoint if no match is found
    harpia_msgs::RegionPoint null;
    return null;
}


/**
 * @brief Retrieve the radius of a specified region from an external Python script.
 *
 * This function runs an external Python script to calculate the radius of a region specified by its name. The Python script takes the region name as an argument and appends the result to an output file.
 *
 * @param region A string representing the name of the region for which the radius needs to be determined.
 *
 * @return An integer value representing the calculated radius of the specified region. If the region is not found or an error occurs, a default radius of 10.0 is returned.
 *
 * @note This function relies on an external Python script to perform the radius calculation. Ensure the Python script is correctly configured and available in the specified path.
 */
int getRadius(string region)
{
    // Construct the command to run an external Python script with the specified region name
    string command = "python3 ~/drone_arch/drone_ws/src/ROSPlan/src/rosplan/rosplan_planning_system/src/ActionInterface/getRadius.py " + region + " >> ~/drone_arch/Data/out.txt";
    system(command.c_str());

    // Read the calculated radius from the output file
    string line;
    ifstream myfile((homepath + "/drone_arch/Data/out.txt").c_str());

    if (myfile.is open())
    {
        cout << "file opened" << endl;
        getline(myfile, line);
        int radius = stod(line);
        myfile.clear();
        myfile.close();
        
        return radius;
    }
    else
    {
        cout << "Unable to open file";
        return 10.0; // Return a default radius if an error occurs or the file cannot be opened
    }
}


/**
 * @brief Calculate a route between two region points using a path planning service.
 *
 * This function calculates a route between two region points (from and to) using a path planning service. It sends a service request to the path planning service, specifying the source region, destination region, region names, and map information. The service response includes a list of waypoints that make up the calculated route.
 *
 * @param from The starting region point for the route.
 * @param to The destination region point for the route.
 * @param name_from The name of the starting region.
 * @param name_to The name of the destination region.
 * @param map The map information that includes region data.
 *
 * @return A mavros_msgs::WaypointList containing a list of waypoints that form the calculated route. If the service call fails, an empty mavros_msgs::WaypointList is returned.
 *
 * @note The function communicates with a path planning service to determine the route. Ensure that the path planning service is correctly configured and available.
 */
mavros_msgs::WaypointList calcRoute(harpia_msgs::RegionPoint from, harpia_msgs::RegionPoint to, string name_from, string name_to, harpia_msgs::Map map)
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
        return srv.response.waypoints; // Return the calculated route waypoints
    }
    else
    {
        ROS_ERROR("Failed to call service harpia/path_planning");
        mavros_msgs::WaypointList null;
        return null; // Return an empty waypoint list if the service call fails
    }
}


/**
 * @brief Calculate a route to the pulverize region using a path planning service.
 *
 * This function calculates a route to the pulverize region based on a specified region point. It sends a service request to the path planning service, specifying the source region (same as the destination region), region names, and map information. The service response includes a list of waypoints that make up the calculated route to the pulverize region.
 *
 * @param at The region point to start the route calculation.
 * @param map The map information that includes region data.
 *
 * @return A mavros_msgs::WaypointList containing a list of waypoints that form the calculated route to the pulverize region. If the service call fails, an empty mavros_msgs::WaypointList is returned.
 *
 * @note The function communicates with a path planning service to determine the route. Ensure that the path planning service is correctly configured and available.
 */
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
        return srv.response.waypoints; // Return the calculated route waypoints
    }
    else
    {
        ROS_ERROR("Failed to call service harpia/path_planning");
        mavros_msgs::WaypointList null;
        return null; // Return an empty waypoint list if the service call fails
    }
}


/**
 * @brief Calculate a route to take pictures of a region using a path planning service.
 *
 * This function calculates a route to a specified region to capture pictures. It sends a service request to the path planning service, specifying the destination region, region names, and map information. The service response includes a list of waypoints that make up the calculated route to the "take_picture" region.
 *
 * @param at The region point from which to calculate the route for taking pictures.
 * @param map The map information that includes region data.
 *
 * @return A mavros_msgs::WaypointList containing a list of waypoints that form the calculated route to the "take_picture" region. If the service call fails, an empty mavros_msgs::WaypointList is returned.
 *
 * @note The function communicates with a path planning service to determine the route. Ensure that the path planning service is correctly configured and available. The commented-out code was intended for an alternative approach and is currently disabled.
 */
mavros_msgs::WaypointList calcRoute_picture(harpia_msgs::RegionPoint at, harpia_msgs::Map map)
{
    // Alternative approach: An example of using a Python script to calculate the route.
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
        return srv.response.waypoints; // Return the calculated route waypoints
    }
    else
    {
        ROS_ERROR("Failed to call service harpia/path_planning");
        mavros_msgs::WaypointList null;
        return null; // Return an empty waypoint list if the service call fails
    }
}

/**
 * @brief Send a list of waypoints to the drone's flight controller.
 *
 * This function sends a list of waypoints, specified in the `mission_wp` parameter, to the drone's flight controller through the MAVLink protocol. It clears the existing mission waypoints and then pushes the new set of waypoints to the controller using ROS services.
 *
 * @param mission_wp A mavros_msgs::WaypointList containing a list of waypoints to be sent to the drone's flight controller.
 *
 * @return An integer result indicating the success of sending waypoints. Returns 1 if the waypoints were successfully sent, and 0 in case of failure.
 *
 * @note The function clears the existing waypoints in the flight controller's mission list and pushes the new waypoints specified in `mission_wp`. It provides feedback about the success or failure of the operation and removes any temporary waypoint files.
 */
int sendWPFile(mavros_msgs::WaypointList mission_wp)
{
    ros::NodeHandle p;
    GeoPoint geo;
    string line;
    int wp_count = 1;
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
        ROS_ERROR("Waypoint list couldn't be cleared");
    }

    wp_push_srv.request.start_index = 0;
    wp_count = mission_wp.waypoints.size();
    cout << wp_count << endl;

    for (int n = 0; n < wp_count; n++)
        wp_push_srv.request.waypoints.push_back(mission_wp.waypoints[n]);

    cout << wp_srv_client.call(wp_push_srv) << endl;

    if (wp_srv_client.call(wp_push_srv))
    {
        ROS_INFO("Success:%d", (bool)wp_push_srv.response.success);
        remove((homepath + "/drone_arch/Data/route.txt").c_str());
    }
    else
    {
        ROS_ERROR("Waypoints couldn't be sent");
        ROS_ERROR("Success:%d", (bool)wp_push_srv.response.success);
        remove((homepath + "/drone_arch/Data/route.txt").c_str());
        return 0; // Return 0 on failure
    }

    return 1; // Return 1 on successful waypoint sending
}


/**
 * @brief Reset the current waypoint mission to the first waypoint.
 *
 * This function sends a request to reset the current waypoint mission to the first waypoint in the mission list of the drone's flight controller through the MAVLink protocol. It utilizes ROS services to set the current waypoint to the beginning of the mission, effectively restarting the mission execution.

 * @note The function should be used when the mission execution needs to be reset to the initial waypoint.

 */
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
        ROS_ERROR("Reset couldn't be done");
    }
}


/**
 * @brief Call a predefined route for the drone from one location to another.
 *
 * This function loads a predefined waypoint (WP) file that specifies a route for the drone to follow from one location (GeoPoint) to another. It uses the `rosrun mavros` command to load the WP file into the drone's flight controller through the MAVLink protocol.

 * @param from The starting location (GeoPoint) of the route.
 * @param to The destination location (GeoPoint) of the route.

 * @note The function assumes that a WP file with a specific naming convention exists for the given route.

 */
void callRoute(GeoPoint from, GeoPoint to)
{
    ROS_INFO("Sending WP file for route %s _ %s.wp", from.name.c_str(), to.name.c_str());
    string command = "rosrun mavros mavwp load ~/drone_arch/Data/Rotas/wp/" + from.name + "_" + to.name + ".wp";
    ROS_INFO("%s", command.c_str());
    system(command.c_str());
}


/**
 * @brief Convert a geographic point to Cartesian coordinates relative to a reference home point.
 *
 * This function takes a geographic point and a reference home point and converts the geographic point to Cartesian coordinates, providing the relative position with respect to the home point in meters. It uses the Haversine formula to perform the conversion.

 * @param p The geographic point to be converted to Cartesian coordinates.
 * @param home The reference geographic point (home) used as the origin for the Cartesian coordinates.

 * @return A `geometry_msgs::Point` representing the Cartesian coordinates relative to the reference home point.

 * @note This function assumes that the Earth's radius is approximately 6,400,000 meters and that the latitude and longitude are given in degrees.

 */
geometry_msgs::Point convert_goe_to_cart(geographic_msgs::GeoPoint p, geographic_msgs::GeoPoint home)
{
    geometry_msgs::Point point;
    double pi = 2 * acos(0.0);
    point.x = (p.longitude - home.longitude) * (6400000.0 * (cos(home.latitude * pi / 180) * 2 * pi / 360));
    point.y = (p.latitude - home.latitude) * (10000000.0 / 90);

    return point;
}


/**
 * @brief Create a harpia_msgs::RegionPoint from a GeoPoint and a Map.
 *
 * This function creates a `harpia_msgs::RegionPoint` by taking a `GeoPoint` and a `Map` as input. It sets the geographic and Cartesian coordinates of the `RegionPoint` based on the provided `GeoPoint` and `Map`. The Cartesian coordinates are calculated relative to the map's home point.

 * @param point The `GeoPoint` to be used for creating the `RegionPoint`.
 * @param map The `Map` that contains the reference home point used for the Cartesian coordinate conversion.

 * @return A `harpia_msgs::RegionPoint` representing the geographic and Cartesian coordinates of the point.

 * @note This function assumes that the Earth's radius is approximately 6,400,000 meters and that latitude and longitude are given in degrees.

 */
harpia_msgs::RegionPoint create_RegionPoint(GeoPoint point, harpia_msgs::Map map)
{
    harpia_msgs::RegionPoint region_point;

    // Set geographic coordinates from the provided GeoPoint.
    region_point.geo.latitude = point.latitude;
    region_point.geo.longitude = point.longitude;
    region_point.geo.altitude = point.altitude;

    // Calculate Cartesian coordinates relative to the map's home point.
    region_point.cartesian = convert_goe_to_cart(region_point.geo, map.geo_home);

    return region_point;
}


/*--------------------------------------------*/
/**
 * @brief Custom signal handler for handling SIGINT (Ctrl+C) signals.
 *
 * This function is a custom signal handler that gets called when a SIGINT (Ctrl+C) signal is received. It performs the following actions:
 * 1. Sets the flight mode to "LOITER" by invoking the `set_loiter()` function.
 * 2. Initiates a graceful shutdown of the ROS node using `ros::shutdown()`.

 * @param sig The signal number, typically SIGINT, indicating the user's interruption request.

 * @note This signal handler allows for clean handling of Ctrl+C signals in your ROS node.

 */
void mySigintHandler(int sig)
{
    // Do some custom action.
    set_loiter();

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

/*--------------------------------------------*/


namespace KCL_rosplan {
	/**
     * @brief RPHarpiaExecutor class handles the execution of ROSPlan actions for Harpia.
     *
     * The RPHarpiaExecutor class is responsible for executing ROSPlan actions in the context of Harpia UAV missions. It includes the action dispatch callback function for handling specific action types.

     * @details
     * This class manages action execution for a Harpia UAV. It processes various action types, including mission fault mitigation, waypoint generation, and specific mission-related tasks. Additionally, it provides feedback on action success or failure.

     * @param nh A ROS node handle for interacting with ROSPlan and managing actions.
     */
	RPHarpiaExecutor::RPHarpiaExecutor(ros::NodeHandle &nh) {}

	/**
	* @brief Action dispatch callback for handling ROSPlan actions.
	*
	* The concreteCallback function is invoked in response to ROSPlan action dispatch messages. It manages the execution of various actions, handling mission replanning, waypoint generation, and other mission-specific tasks. The function provides feedback on action success or failure.

	* @param msg A pointer to the action dispatch message specifying the action to be executed.

	* @return True if the action is successfully completed; otherwise, it returns false.
	*
	* @note This function handles different scenarios, including mission replanning, action execution, and handling preemption due to mission cancellation.

	* @see RPHarpiaExecutor
	*/
	bool RPHarpiaExecutor::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // Create a ROS node handle and service client for mission fault mitigation
		ros::NodeHandle n;
	 	ros::ServiceClient client = n.serviceClient<harpia_msgs::MissionFaultMitigation>("harpia/mission_fault_mitigation");
	  	harpia_msgs::MissionFaultMitigation srv;
	  	srv.request.uav = mission.hMission.uav;
	  	srv.request.action_id = msg->action_id;


	  	int replan, cancelled;

        // Call the mission fault mitigation service
	  	if (client.call(srv) && msg->action_id != 0)
	  	{
	    	replan = srv.response.replan;
	  	}
	  	else
	  	{
	    	ROS_INFO("BN not called");
	    	replan = 0;
	  	}
	  	
	  	if(replan != 1)
		{
            // There was no need for replan. All action implementation goes here.

			string str = msg->name.c_str();
			string str1 = "go_to";
			size_t found = str.find(str1);
			ROS_INFO("%s", msg->name.c_str());
	    	if (found != string::npos)
			{
				// Handle 'go_to' action

				mission.Ended = false;
				GeoPoint from, to;
				harpia_msgs::RegionPoint r_from, r_to;
				mavros_msgs::WaypointList route;

                // Extract coordinates
				from.name = msg->parameters[0].value.c_str();
				to.name = msg->parameters[1].value.c_str();
				ROS_INFO("go_to %s -> %s", from.name.c_str(), to.name.c_str());

                // Set coordinates for 'from' point
				from.latitude = drone.position.latitude;
				from.longitude = drone.position.longitude;
				from.altitude = 15;

                // Create RegionPoint objects for 'from' and 'to' points
				r_from = create_RegionPoint(from, mission.hMission.map);
				r_to = getGeoPoint(to, mission.hMission.map);



				ROS_INFO("GEO GeoPoint %f %f %f -> %f %f %f", r_from.geo.latitude, r_from.geo.longitude, r_from.geo.altitude, r_to.geo.latitude, r_to.geo.longitude, r_to.geo.altitude);


                // Calculate the route
				route = calcRoute(r_from, r_to, from.name, to.name, mission.hMission.map);

                // Check if the UAV is flying, armed, and take off if needed
				while(!drone.current_state.armed && drone.ex_current_state.landed_state != 2)
				{
					set_loiter();
					arm();
					takeoff(drone);
				}
				ros::Duration(10).sleep();

                // Send the route to the UAV
				if(!sendWPFile(route))
					callRoute(from, to);
				ros::Duration(20).sleep();


				set_auto();

                // Wait until the mission is completed
				while(!mission.Ended){
					ros::Duration(10).sleep();
				}

				set_loiter();
				ros::Duration(10).sleep();

				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());

			}
			else if (strcmp(msg->name.c_str(), "pulverize_region") == 0)
			{
                // Handle 'pulverize_region' action
				ROS_INFO("pulverize_region");
				mavros_msgs::WaypointList route;
				// string region = msg->parameters[1].value.c_str();
				mission.Ended = false;

				//get coordinates
				//int radius = getRadius(region);

                // Create a RegionPoint for the current position
				GeoPoint at;
				at.latitude = drone.position.latitude;
				at.longitude = drone.position.longitude;
				at.altitude = 15;
				harpia_msgs::RegionPoint r_at;
				r_at = create_RegionPoint(at, mission.hMission.map);

                // Calculate the route for pulverization
				route = calcRoute_pulverize(r_at, mission.hMission.map);


                // Check if the UAV is flying, armed, and take off if needed
				while(!drone.current_state.armed && drone.ex_current_state.landed_state != 2)
				{
					set_loiter();
					arm();
					takeoff(drone);
				}
				ros::Duration(20).sleep();

                // Send the route to the UAV
				if(!sendWPFile(route))
					ROS_ERROR("Error call route pulverize_region");
				ros::Duration(20).sleep();

				set_auto();

                // Wait until the mission is completed
				while(!mission.Ended){
					ros::Duration(10).sleep();
				}

				set_loiter();
				ros::Duration(10).sleep();

				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
			}
			else if (strcmp(msg->name.c_str(), "take_image") == 0)
			{
                // Handle 'take_image' action
				ROS_INFO("take_image");
				string region = msg->parameters[0].value.c_str();
				mission.Ended = false;
				mavros_msgs::WaypointList route;

				//get coordinates
				//int radius = getRadius(region);

                // Create a RegionPoint for the current position
				GeoPoint at;
				at.latitude = drone.position.latitude;
				at.longitude = drone.position.longitude;
				at.altitude = 15;
				harpia_msgs::RegionPoint r_at;
				r_at = create_RegionPoint(at, mission.hMission.map);
				route = calcRoute_picture(r_at, mission.hMission.map);

                // Check if the UAV is flying, armed, and take off if needed
				while(!drone.current_state.armed && drone.ex_current_state.landed_state != 2)
				{
					set_loiter();
					arm();
					takeoff(drone);
				}
				ros::Duration(20).sleep();

                // Send the route to the UAV
				if(!sendWPFile(route))
					ROS_ERROR("Error call route pulverize_region");
				ros::Duration(20).sleep();

				set_auto();

                // Send the route to the UAV
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
                // Handle 'recharge_input' action
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
                // Handle 'clean_camera' action
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
                // Handle 'recharge_battery' action
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
                // Handle 'has_all_goals_achieved' action
				ROS_INFO("has-all-goals-achived");

				ros::Duration(1).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "need_battery") == 0)
			{
                // Handle 'need_battery' action
				ROS_INFO("need-battery");

				ros::Duration(msg->duration).sleep();
				// complete the action
				ROS_INFO("KCL: (%s) HarpiaExecutor Action completing.", msg->name.c_str());
				// return true;
			}
			else if (strcmp(msg->name.c_str(), "need_input") == 0)
			{
                // Handle 'need_input' action
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

/**
 * @brief Main function of the ROS-based application for Harpia mission execution.
 *
 * This function initializes a ROS node, sets up subscribers for various topics, and runs the PDDL action interface
 * using the `RPHarpiaExecutor` class.
 *
 * @param argc Number of command-line arguments.
 * @param argv Command-line arguments.
 * @return 0 upon successful execution.
 */
int main(int argc, char **argv) {
    // Initialize the ROS node with the name "rosplan_interface_harpia" and specify an anonymous name.
    ros::init(argc, argv, "rosplan_interface_harpia", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // Register a signal handler to handle SIGINT (Ctrl+C) for graceful shutdown.
    signal(SIGINT, mySigintHandler);

    // Create ROS subscribers to listen to various topics such as GPS data, state information, mission waypoints, etc.
    ros::Subscriber GPS = nh.subscribe("/mavros/global_position/global", 1, &Drone::chatterCallback_GPS, &drone);
    ros::Subscriber state = nh.subscribe("/mavros/state", 1, &Drone::chatterCallback_currentState, &drone);
    ros::Subscriber state_ext = nh.subscribe("/mavros/extended_state", 1, &Drone::chatterCallback_currentStateExtended, &drone);
    ros::Subscriber global = nh.subscribe("/mavros/mission/waypoints", 1, &Mission::chatterCallback_wpqtd, &mission);
    ros::Subscriber current = nh.subscribe("/mavros/mission/reached", 1, &Mission::chatterCallback_current, &mission);
    ros::Subscriber harpia_mission = nh.subscribe("/harpia/mission", 1, &Mission::chatterCallback_harpiaMission, &mission);
    ros::Subscriber harpia_goalId = nh.subscribe("/harpia/mission_goal_manager/goal", 1, &Mission::chatterCallback_IDGoal, &mission);
    ros::Subscriber harpia_goalCancel = nh.subscribe("/harpia/ChangeMission", 1, &Mission::chatterCallback_cancelGoal, &mission);

    // Create an instance of the RPHarpiaExecutor class to run the action interface.
    KCL_rosplan::RPHarpiaExecutor rpti(nh);
    rpti.runActionInterface();

    return 0; // Return 0 to indicate successful execution.
}
