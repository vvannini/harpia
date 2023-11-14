#!/usr/bin/env python3

import rospy
import actionlib
import sys
import math
import json
import time
import os
import rosnode, psutil
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import *
from rosplan_dispatch_msgs.srv import *
from diagnostic_msgs.msg import KeyValue

from std_msgs.msg import String

from harpia_msgs.msg import ChangeMission, MissionPlannerAction, MissionPlannerGoal, MissionPlannerFeedback, MissionPlannerResult, Mission, MissionPlannerActionGoal
from harpia_msgs.srv import PathPlanning


from mavros_msgs.msg import *
from mavros_msgs.srv import *

from sensor_msgs.msg import *
from geographic_msgs.msg import *
from geometry_msgs.msg import *

from actionlib_msgs.msg import GoalStatusArray, GoalID

KB_UPDATE_ADD_KNOWLEDGE = 0
KB_UPDATE_RM_KNOWLEDGE = 2
KB_UPDATE_ADD_GOAL = 1
KB_UPDATE_RM_GOAL = 3
KB_UPDATE_ADD_METRIC = 4

KB_ITEM_INSTANCE = 0
KB_ITEM_FACT = 1
KB_ITEM_FUNCTION = 2
KB_ITEM_EXPRESSION = 3
KB_ITEM_INEQUALITY = 4

OP_UPDATE_KNOWLEDGE_BASE = 0
OP_REPLAN				 = 1
OP_ADD_RM_GOALS			 = 2
def control_callback(data):
    """
    Callback function for stop the mission.

    Args:
        data (std_msgs.String): ROS message containing control command information.

    Returns:
        None

	Note:
		need fix
    """
    # Assuming data is of type std_msgs/String
    if data.data == 'kill':
        rospy.signal_shutdown("Received kill command")

'''
	Classes to subscribe and publish services
'''

class Drone(object):
    def __init__(self):
        """
        Initializes a Drone object.

        Subscribes to relevant ROS topics for receiving global position, battery state, and mission reached information.

        Attributes:
            sub_position (rospy.Subscriber): ROS subscriber for global position updates (NavSatFix).
            sub_battery (rospy.Subscriber): ROS subscriber for battery state updates (BatteryState).
            sub_mission (rospy.Subscriber): ROS subscriber for mission reached updates (WaypointReached).
            latitude (float): Current latitude of the drone.
            longitude (float): Current longitude of the drone.
            battery (float): Current battery percentage of the drone.
            current (int): Current waypoint reached in the mission.
        """
        self.sub_position = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.global_position_callback)
        self.sub_battery  = rospy.Subscriber('mavros/battery', BatteryState, self.battery_state_callback)
        self.sub_mission  = rospy.Subscriber('mavros/mission/reached', WaypointReached, self.reached_callback)
        self.latitude     = None
        self.longitude    = None
        self.battery      = None
        self.current      = None

    def global_position_callback(self, data):
        """
        Callback function for updating the drone's global position.

        Args:
            data (NavSatFix): ROS message containing global position information.

        Returns:
            None
        """
        self.latitude = data.latitude
        self.longitude = data.longitude

    def battery_state_callback(self, data):
        """
        Callback function for updating the drone's battery state.

        Args:
            data (BatteryState): ROS message containing battery state information.

        Returns:
            None
        """
        self.battery = data.percentage * 100

    def reached_callback(self, data):
        """
        Callback function for updating the drone's current waypoint in the mission.

        Args:
            data (WaypointReached): ROS message containing information about the reached waypoint.

        Returns:
            None
        """
        self.current = data.wp_seq + 1

class ActionServer():
    """
    The main class that runs the action server for mission planning.

    Attributes:
        a_server (actionlib.SimpleActionServer): ROS action server for handling mission planning goals.
        uav (Drone): Instance of the Drone class representing the UAV.
        new_goals (List[Goal] or None): List of new goals received from the ChangeMission topic.
        Mission_Sub (rospy.Subscriber): ROS subscriber for receiving mission updates.
        Goals_Sub (rospy.Subscriber): ROS subscriber for receiving new goals from the ChangeMission topic.
        mission (Mission or None): Current mission received from the MissionPlannerActionGoal topic.
        change_goals (int): Operation code for changing goals received from the ChangeMission topic.

    Methods:
        mission_callback(data): Callback function for updating the mission attribute.
        new_goal_callback(data): Callback function for updating the new_goals and change_goals attributes.
        run(): Initiates the ROS node and starts the mission goal manager service.
        register_cancel_callback(cancel_cb): Registers a callback function for handling mission cancellation.
        abort(): Sets the mission status to aborted.
        succeed(): Sets the mission status to succeeded.
        execute_cb(data): Callback function for handling mission planning goals.

    """

    def __init__(self):
        """
        Initializes the ActionServer object.
        """
        self.a_server = actionlib.SimpleActionServer(
            "harpia/mission_goal_manager",
            MissionPlannerAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.uav = Drone()
        self.new_goals = None
        self.Mission_Sub = rospy.Subscriber('/harpia/mission_goal_manager/goal', MissionPlannerActionGoal, self.mission_callback)
        self.Goals_Sub = rospy.Subscriber('/harpia/ChangeMission', ChangeMission, self.new_goal_callback)
        self.mission = None
        self.change_goals = None

    def mission_callback(self, data):
        """
        Callback function for updating the mission attribute.

        Args:
            data (MissionPlannerActionGoal): ROS message containing the mission goal.

        Returns:
            None
        """
        self.mission = data.goal.mission

    def new_goal_callback(self, data):
        """
        Callback function for updating the new_goals and change_goals attributes.

        Args:
            data (ChangeMission): ROS message containing new goals and the operation code.

        Returns:
            None
        """
        try:
            self.change_goals = data.op
            self.new_goals = data.goals
        except:
            self.change_goals = 0

    def run(self):
        """
        Initiates the ROS node and starts the mission goal manager service.

        Returns:
            None
        """
        rospy.init_node("mission_goal_manager_server")
        self.a_server.start()
        rospy.loginfo("Mission Goal Manager Service Ready")
        rospy.spin()

    def register_cancel_callback(self, cancel_cb):
        """
        Registers a callback function for handling mission cancellation.

        Args:
            cancel_cb: Callback function for mission cancellation.

        Returns:
            None
        """
        self.a_server.set_aborted(MissionPlannerResult())

    def abort(self):
        """
        Sets the mission status to aborted.

        Returns:
            None
        """
        self.a_server.set_aborted(MissionPlannerResult())

    def succeed(self):
        """
        Sets the mission status to succeeded.

        Returns:
            None
        """
        self.a_server.set_succeeded(MissionPlannerResult())

    def execute_cb(self, data):
        """
        Callback function for handling mission planning goals.

        Args:
            data (MissionPlannerAction): ROS message containing the mission planning goal.

        Returns:
            None
        """
        rospy.loginfo(f"EXECUTE - MissionGoalManager with op {data.op}")

        if data.op == OP_UPDATE_KNOWLEDGE_BASE:
            update_knowledge_base(data.mission.uav, data.mission.map, data.mission.goals, "base_1")
        elif data.op == OP_REPLAN:
            rospy.loginfo("REPLAN - MissionGoalManager")
            replan(data.mission)
        elif data.op == OP_ADD_RM_GOALS:
            rospy.logerr("ADD/RM GOALS - MissionGoalManager")
            replan(data.mission)
            return

        feedback_msg = MissionPlannerFeedback()

        while not call_mission_planning() or self.change_goals:
            if self.change_goals:
                print('MISSION GOAL Manager CANCEL')
                feedback_msg.status = 1
                self.a_server.publish_feedback(feedback_msg)
                op = self.change_goals
                self.change_goals = None
                replan(self.mission, self.uav, None, self.new_goals, op)
            elif not wait_until(lambda: self.uav.battery is not None, msg="Waiting for UAV battery..."):
                self.abort()
                return

            if self.uav.battery <= 20:
                base = go_to_base(data.mission, self.uav)
                replan(self.mission, self.uav, base, None, 0)
            else:
                replan(self.mission, self.uav, None, None, 0)
            feedback_msg.status = 0
            self.a_server.publish_feedback(feedback_msg)
            self.change_goals = 0

        land()
        rospy.sleep(30)
        self.succeed()

def wait_until(check, msg=None, rate=1):
    """
    Waits until a given condition is met.

    Args:
        check (callable): A function that returns a boolean. The function is called repeatedly until it returns True.
        msg (str, optional): A message to log while waiting. Defaults to None.
        rate (float, optional): Rate at which to check the condition in Hz. Defaults to 1.

    Returns:
        bool: True if the condition is met, False if ROS is shut down during waiting.

    Example:
        ```python
        # Wait until a condition is met with a log message
        result = wait_until(lambda: some_condition(), "Waiting for some_condition to be True...")

        # Wait until a condition is met with a specific rate
        result = wait_until(lambda: some_condition(), rate=2)
        ```

    """
    rate = rospy.Rate(rate)

    while not check():
        if rospy.is_shutdown():
            return False
        if msg is not None:
            rospy.loginfo(msg)
        rate.sleep()

    return True


'''
	Callers for ROSPlan Services
'''
def try_call_srv(topic, msg_ty=Empty):
    """
    Attempts to call a ROS service and returns True if successful, False otherwise.

    Args:
        topic (str): The topic of the ROS service.
        msg_ty (Message, optional): The message type to be sent to the service. Defaults to Empty.

    Returns:
        bool: True if the service call is successful, False otherwise.

    Example:
        ```python
        # Try calling a service with default Empty message type
        result = try_call_srv('/some_service')

        # Try calling a service with a specific message type
        result = try_call_srv('/another_service', SomeMessageType)
        ```

    Raises:
        rospy.ServiceException: If the service call fails.

    """
    rospy.wait_for_service(topic)
    try:
        query_proxy = rospy.ServiceProxy(topic, msg_ty)
        query_proxy()
        return True
    except rospy.ServiceException as e:
        rospy.logerr(f"MissionGoalManager Service call to {topic} failed: {e}")
        return False


def call_problem_generator() -> bool:
    """
    Calls the ROS service for problem generation.

    Returns:
        bool: True if the service call is successful, False otherwise.
    """
    return try_call_srv('/rosplan_problem_interface/problem_generation_server', Empty)


def call_plan_generator() -> bool:
    """
    Calls the ROS service for planning.

    Returns:
        bool: True if the service call is successful, False otherwise.
    """
    return try_call_srv('/rosplan_planner_interface/planning_server', Empty)


def call_parser() -> bool:
    """
    Calls the ROS service for plan parsing.

    Returns:
        bool: True if the service call is successful, False otherwise.
    """
    return try_call_srv('/rosplan_parsing_interface/parse_plan', Empty)


def call_dispatch() -> bool:
    """
    Calls the ROS service for plan dispatching.

    Returns:
        bool: True if the service call is successful, False otherwise.
    """
    return try_call_srv('/rosplan_plan_dispatcher/dispatch_plan', Empty)


def cancel_dispatch() -> bool:
    """
    Calls the ROS service to cancel plan dispatching.

    Returns:
        bool: True if the service call is successful, False otherwise.
    """
    return try_call_srv('/rosplan_plan_dispatcher/cancel_dispatch', Empty)


def call_mission_planning() -> bool:
    """
    Calls the ROS service for mission planning.

    Returns:
        bool: True if the service call is successful, False otherwise.
    """
    return try_call_srv('/harpia/mission_planning', Empty)

'''
	Fuctions to calc the regios distances
'''
def calc_distances(regions):
	"""
    Calculate distances between regions.

    Args:
        regions (List[Region]): List of regions.

    Returns:
        List[Function]: List of functions representing distances between regions.
    """
	out = []
	for x in regions:
		for y in regions:
			if x == y: continue

			out.append(
				create_function(
					"distance",
					euclidean_distance(x.center.cartesian, y.center.cartesian),
					[KeyValue("region", x.name), KeyValue("region", y.name)]
				)
			)

	return out

def euclidean_distance(a, b):
	"""
    Calculate the Euclidean distance between two points.

    Args:
        a (Point): First point.
        b (Point): Second point.

    Returns:
        float: Euclidean distance between points.
    """
	return math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)

def geo_to_cart(geo_point, geo_home):
	"""
    Convert geographical coordinates to Cartesian coordinates.

    Args:
        geo_point (GeoPoint): Geographical coordinates.
        geo_home (GeoPoint): Home coordinates.

    Returns:
        Point: Cartesian coordinates.
    """

	def calc_y(lat, lat_):
		return (lat - lat_) * (10000000.0 / 90)

	def calc_x(longi, longi_, lat_):
		return (longi - longi_) * (
			6400000.0 * (math.cos(lat_ * math.pi / 180) * 2 * math.pi / 360)
		)

	x = calc_x(geo_point.longitude, geo_home.longitude, geo_home.latitude)
	y = calc_y(geo_point.latitude, geo_home.latitude)

	return Point(x, y, geo_point.altitude)

'''
	Functions to manipulate Knowledge base
'''

def try_update_knowledge(item, update_type):
	"""
    Try to update knowledge in the ROSPlan knowledge base.

    Args:
        item: The item to update.
        update_type: The type of update.

    Returns:
        bool: True if the update was successful, False otherwise.
    """
	rospy.wait_for_service('/rosplan_knowledge_base/update')
	try:
		query_proxy = rospy.ServiceProxy('rosplan_knowledge_base/update', KnowledgeUpdateService)
		query_proxy(update_type, item)
		return True
	except rospy.ServiceException as e:
		rospy.logerr(f"MissionGoalManager Service call to {topic} failed: {e}")
		return False

def call_clear():
    """
    Call the ROSPlan knowledge base service to clear all knowledge.

    Returns:
        bool: True if the service call was successful, False otherwise.
    """
    return try_call_srv('/rosplan_knowledge_base/clear', Empty)

def add_instance(item):
    """
    Add an instance to the ROSPlan knowledge base.

    Args:
        item: The item to add.

    Returns:
        bool: True if the update was successful, False otherwise.
    """
    return try_update_knowledge(item, KB_UPDATE_ADD_KNOWLEDGE)

def remove_instance(item):
    """
    Remove an instance from the ROSPlan knowledge base.

    Args:
        item: The item to remove.

    Returns:
        bool: True if the update was successful, False otherwise.
    """
    return try_update_knowledge(item, KB_UPDATE_RM_KNOWLEDGE)

def add_goal(item):
    """
    Add a goal to the ROSPlan knowledge base.

    Args:
        item: The item to add.

    Returns:
        bool: True if the update was successful, False otherwise.
    """
    return try_update_knowledge(item, KB_UPDATE_ADD_GOAL)

def remove_goal(item):
    """
    Remove a goal from the ROSPlan knowledge base.

    Args:
        item: The item to remove.

    Returns:
        bool: True if the update was successful, False otherwise.
    """
    return try_update_knowledge(item, KB_UPDATE_RM_GOAL)

def add_metric(item):
    """
    Add a metric to the ROSPlan knowledge base.

    Args:
        item: The item to add.

    Returns:
        bool: True if the update was successful, False otherwise.
    """
    return try_update_knowledge(item, KB_UPDATE_ADD_METRIC)

def get_knowledge(name, topic):
    """
    Get knowledge from the ROSPlan knowledge base.

    Args:
        name: The name of the knowledge item to retrieve.
        topic: The ROSPlan knowledge base service topic.

    Returns:
        KnowledgeItem: The knowledge item retrieved from the knowledge base.
    """
    rospy.wait_for_service(topic)
    try:
        query_proxy = rospy.ServiceProxy(topic, GetAttributeService)
        return query_proxy(name)
    except rospy.ServiceException as e:
        rospy.logerr(f"MissionGoalManager Service call to {topic} failed: {e}")
        return KnowledgeItem()

def get_function(function_name):
    """
    Get a function from the ROSPlan knowledge base.

    Args:
        function_name: The name of the function to retrieve.

    Returns:
        KnowledgeItem: The function retrieved from the knowledge base.
    """
    return get_knowledge(function_name, '/rosplan_knowledge_base/state/functions')

def get_goal(goal_name):
    """
    Get a goal from the ROSPlan knowledge base.

    Args:
        goal_name: The name of the goal to retrieve.

    Returns:
        KnowledgeItem: The goal retrieved from the knowledge base.
    """
    return get_knowledge(goal_name, '/rosplan_knowledge_base/state/goals')

def get_predicate(predicate_name):
    """
    Get a predicate from the ROSPlan knowledge base.

    Args:
        predicate_name: The name of the predicate to retrieve.

    Returns:
        KnowledgeItem: The predicate retrieved from the knowledge base.
    """
    return get_knowledge(predicate_name, '/rosplan_knowledge_base/state/propositions')

def create_object(item_name, item_type):
    """
    Create a knowledge item representing an object or instance.

    Args:
        item_name: The name of the object or instance.
        item_type: The type of the object or instance.

    Returns:
        KnowledgeItem: The knowledge item representing the object or instance.
    """
    instance = KnowledgeItem()
    instance.knowledge_type = KnowledgeItem.INSTANCE
    instance.instance_type = item_type
    instance.instance_name = item_name

    return instance

def create_function(attribute_name, function_value, values=[]):
    """
    Create a knowledge item representing a function.

    Args:
        attribute_name: The name of the function attribute.
        function_value: The value of the function.
        values: List of KeyValue pairs representing the function arguments.

    Returns:
        KnowledgeItem: The knowledge item representing the function.
    """
    instance = KnowledgeItem()

    instance.knowledge_type = KnowledgeItem.FUNCTION
    instance.attribute_name = attribute_name
    instance.values = values
    instance.function_value = function_value

    return instance


def create_predicate(attribute_name, values=[], is_negative=False):
    """
    Create a knowledge item representing a predicate.

    Args:
        attribute_name: The name of the predicate attribute.
        values: List of KeyValue pairs representing the predicate arguments.
        is_negative: Boolean indicating whether the predicate is negative.

    Returns:
        KnowledgeItem: The knowledge item representing the predicate.
    """
    instance = KnowledgeItem()

    instance.knowledge_type = KnowledgeItem.FACT
    instance.attribute_name = attribute_name
    instance.values = values
    instance.is_negative = is_negative

    return instance


def create_metric(optimization, item):
    """
    Create a knowledge item representing a metric.

    Args:
        optimization: The optimization direction (e.g., minimize, maximize).
        item: The item for the metric.

    Returns:
        KnowledgeItem: The knowledge item representing the metric.
    """
    instance = KnowledgeItem()

    instance.knowledge_type = KnowledgeItem.EXPRESSION
    instance.optimization = optimization
    instance.expr.tokens = item

    return instance


def set_distances(map, goals, latitude, longitude):
    """
    Set distances in the knowledge base based on the current location.

    Args:
        map: The map information.
        goals: List of goals with regions.
        latitude: Current latitude.
        longitude: Current longitude.
    """
    geo_home = map.geo_home

    regions_name = [g.region for g in goals]

    # Get current latitude and longitude
    cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), geo_home)

    for base in map.bases:
        d = euclidean_distance(base.center.cartesian, cart_location)
        obj = create_function(
            "distance",
            round(d, 2),
            [KeyValue("region", "aux"), KeyValue("region", base.name)]
        )
        add_instance(obj)

    for region in map.roi:
        if region.name in regions_name:
            d = euclidean_distance(cart_location, region.center.cartesian)
            obj = create_function(
                "distance",
                [KeyValue("region", "aux"), KeyValue("region", region.name)],
                round(d, 2)
            )
            add_instance(obj)

def regions_to_perform_action(goals, action):
    """
    Get regions associated with a specific action.

    Args:
        goals: List of goals with regions.
        action: Action to filter regions.

    Returns:
        List of regions associated with the specified action.
    """
    return [step.region for step in goals if step.action == action]

def update_knowledge_base(uav, map, goals, at):
    """
    Update the knowledge base with information related to the UAV, map, and goals.

    Args:
        uav (UAV): UAV object containing information about the drone.
        map (Map): Map object containing information about the environment.
        goals (List[Goal]): List of goals to be achieved by the UAV.
        at (str): The initial location of the UAV.

    Returns:
        None
    """
    regions = map.roi
    bases = map.bases
    pulverize = regions_to_perform_action(goals, 'pulverize')
    photo = regions_to_perform_action(goals, 'take_picture')
    end = regions_to_perform_action(goals, 'end')

    # In this version, goals are only added at the initial location
    # To prevent repetition, a set is used to store unique regions in goals
    goals_regions = set(pulverize + photo)
    regions_obj = [r for r in regions if r.name in goals_regions] + bases

    # Clear the knowledge base
    call_clear()

    # Set drone initial state

    # Adding objects to knowledge base
    add_instance(create_predicate("at", [KeyValue("region", at)]))
    add_instance(create_function("battery-capacity", uav.battery.capacity))
    add_instance(create_function("velocity", uav.frame.efficient_velocity))
    add_instance(create_function("battery-amount", 100))
    add_instance(create_function("discharge-rate-battery", uav.battery.discharge_rate))
    add_instance(create_function("input-amount", 0))
    add_instance(create_function("mission-length", 0))
    add_instance(create_function("input-capacity", uav.input_capacity))

    for r in regions:
        add_instance(create_object(str(r.name), "region"))

    for b in bases:
        add_instance(create_object(str(b.name), "base"))

    for fact in calc_distances(regions_obj):
        add_instance(fact)

    total_goals = 0

    for i in pulverize:
        add_instance(create_predicate("pulverize-goal", [KeyValue("region", i)]))
        # add_instance(create_function("pulverizePathLen", 314, [KeyValue("region", i)]))
        add_goal(create_predicate("pulverized", [KeyValue("region", i)]))
        total_goals += 1

    for i in photo:
        add_instance(create_predicate("picture-goal", [KeyValue("region", i)]))
        # add_instance(create_function("picturePathLen", 1000, [KeyValue("region", i)]))
        add_goal(create_predicate("taken-image", [KeyValue("region", i)]))
        total_goals += 1

    for i in end:
        add_goal(create_predicate("at", [KeyValue("region", i)]))

    add_metric(create_metric("minimize (mission-length)", []))

def find_at(map, goals, latitude, longitude):
    """
    Finds a base that is within a 50m radius. If none is found, try to find a region which has a center
    within the same range. If none is found, return `None`. If there is one of these waypoints in range,
    the drone is considered at that waypoint.

    Args:
        map (Map): Map object containing information about the environment.
        goals (List[Goal]): List of goals to be achieved by the UAV.
        latitude (float): Current latitude of the UAV.
        longitude (float): Current longitude of the UAV.

    Returns:
        Union[Base, Region, None]: Returns a Base or Region object if found within a 50m radius,
        otherwise returns None.
    """
    # Get current latitude and longitude
    cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), map.geo_home)

    # Check if any base is within a 50m radius
    for base in map.bases:
        d = euclidean_distance(base.center.cartesian, cart_location)
        if d <= 50:
            return base

    # Get region names from goals
    region_names = {g.region for g in goals}

    # Check if any region has a center within a 50m radius
    for region in map.roi:
        if region.name in region_names:
            d = euclidean_distance(cart_location, region.center.cartesian)
            if d <= 50:
                return region

    return None


def find_nearest_base(map, latitude, longitude):
    """
    Finds the nearest base to the given latitude and longitude.

    Args:
        map (Map): Map object containing information about the environment.
        latitude (float): Latitude of the location.
        longitude (float): Longitude of the location.

    Returns:
        Base: Nearest Base object.
    """
    # Get current latitude and longitude
    cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), map.geo_home)

    # Find the nearest base using euclidean distance
    return min(map.bases, key=lambda base: euclidean_distance(base.center.cartesian, cart_location))

def call_path_planning(r_from, r_to, map):
    """
    Calls the path planning service to calculate the path from one region to another.

    Args:
        r_from (Region): Starting region.
        r_to (Region): Destination region.
        map (Map): Map object containing information about the environment.

    Returns:
        PathPlanningResponse or None: Response from the path planning service or None if the service call fails.
    """
    rospy.wait_for_service("/harpia/path_planning")
    try:
        query_proxy = rospy.ServiceProxy("/harpia/path_planning", PathPlanning)
        resp = query_proxy(r_from.center, r_to.center, r_from.name, r_to.name, 3, map)
        return resp
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None


def replan(mission, uav, base, goals, op):
	"""
    Replans the mission based on the given operation.

    Args:
        mission (Mission): Mission object containing information about the mission.
        uav (Drone): UAV object containing information about the UAV.
        base (Base or None): Base object if the operation is to return home, else None.
        goals (list): List of goals to be added or removed from the mission.
        op (int): Operation code (0: continue mission, 1: add goals, 2: remove goals).

    Returns:
        None
    """
	rospy.loginfo("CANCEL DISPATCH - MissionGoalManager")

	if not wait_until(lambda: mission.goals != []	  , msg="Waiting for Regions..."): return
	if not wait_until(lambda: uav.latitude is not None, msg="Waiting for position..."): return

	if goals != None:
		pulverize = regions_to_perform_action(goals, 'pulverize')
		photo	  = regions_to_perform_action(goals, 'take_picture')

	if base == None:
		# Here we want to continue the mission.
		# at_move = get_predicate("at-move")
		at = get_predicate("at")

		# rospy.loginfo(f"at={at}")
		# rospy.loginfo(f"len at_={len(at.attributes)}")

		if len(at.attributes) == 0:
			# if len(at_move.attributes) > 0:
				# remove_instance(at_move.attributes[0])

			#codigo achar at
			base_or_region_nearby = find_at(mission.map, mission.goals, uav.latitude, uav.longitude)
			if base_or_region_nearby != None:
				# We are at a base or a region.
				rospy.loginfo(f'at = {base_or_region_nearby}')
				at = create_predicate("at", [KeyValue("region", base_or_region_nearby.name)])
			else:
				# for updates on the mission while on the move, I created a new auxiliar region to start the plan
				# add new region to problem
				add_instance(create_object("aux", "region"))
				add_instance(create_predicate("its-not-base", [KeyValue("region", "aux")]))
				set_distances(mission.map, mission.goals, uav.latitude, uav.longitude)

				at = create_predicate("at", [KeyValue("region", "aux")])
		else:
			at = at.attributes[0]
	else:
		# We want to go home
		at_move =  get_predicate("at-move")
		if len(at_move.attributes) > 0:
			remove_instance(at_move.attributes[0])

		at_kb = get_predicate("at")
		rospy.loginfo(at_kb)
		if len(at_kb.attributes) > 0:
				remove_instance(at_kb.attributes[0])

		rospy.loginfo(f"base = {base}")
		at = create_predicate("at", [KeyValue("base", base.name)])

	add_instance(at)

	# saving the current total goals to manage goals
	# f = get_function("total-goals")
	#total_goals = f.attributes[0].function_value
	#remove_instance(f.attributes[0])

	bat = get_function("battery-amount")
	remove_instance(bat.attributes[0])

	if not wait_until(lambda: uav.battery is not None, msg="Waiting for UAV battery..."): return

	obj = create_function("battery-amount", uav.battery)
	add_instance(obj)
	# has_end is a variable to verify if a mission has already a designed end
	has_end = False
	if (op == 0) :
		print("Op 0")
		# getting current goals and verifying if it already done
		for goal in get_goal('').attributes:
			if goal.attribute_name != "at":
				for goal_achived in get_predicate(goal.attribute_name).attributes:
					if goal.values[0].value == goal_achived.values[0].value:
						# f.attributes[0].function_value = f.attributes[0].function_value - 1
						remove_goal(goal)
			else:
				has_end =  True
		# add_instance(create_function("total-goals", total_goals))

	elif (op == 1):
		# add
		print("Op 1")
		if pulverize:
			add_instance(create_predicate("has-pulverize-goal"))

		if photo:
			add_instance(create_predicate("has-picture-goal"))

		for i in pulverize:
			add_instance(create_predicate("pulverize-goal", [KeyValue("region", i)]))
			add_instance(create_function("pulverize-path-len", 314, [KeyValue("region", i)]))
			add_goal(create_predicate("pulverized", [KeyValue("region", i)]))
			# total_goals = total_goals + 1

		for i in photo:
			add_instance(create_predicate("picture-goal", [KeyValue("region", i)]))
			add_instance(create_function("picture-path-len", 1000, [KeyValue("region", i)]))
			add_goal(create_predicate("taken-image", [KeyValue("region", i)]))
			# total_goals = total_goals + 1

		# add_instance(create_function("total-goals", total_goals))

		# add_instance(create_function("total-goals", total_goals))
		for g in goals: mission.goals.append(g)
		regions   = mission.map.roi
		pulverize = regions_to_perform_action(mission.goals, 'pulverize')
		photo	  = regions_to_perform_action(mission.goals, 'take_picture')

		goals_regions = set(pulverize + photo)
		bases	   = mission.map.bases
		regions_obj = [r for r in regions if r.name in goals_regions] + bases

		for fact in calc_distances(regions_obj):
			add_instance(fact)

	elif (op == 2):
		print("Op 2")
		# Remove
		for goal in get_goal('').attributes:
			# print(goal)
			# if goal.attribute_name != "at":
			# testar regiao e objetivo.

			# for goal_toRemove in goals:
			# 	print(f"{goal.values[0].value} == {goal_toRemove.region}")
			# 	print(f"{goal.attribute_name} == {goal_toRemove.action}")
			if (goal.attribute_name == 'taken-image'):
				for goal_toRemove in photo:
					if (goal.values[0].value == goal_toRemove):
					# f.attributes[0].function_value = f.attributes[0].function_value - 1
						# print(goal)
						remove_goal(goal)
			elif(goal.attribute_name == 'pulverized'):
				for goal_toRemove in pulverize:
					if (goal.values[0].value == goal_toRemove):
					# f.attributes[0].function_value = f.attributes[0].function_value - 1
						# print(goal)
						remove_goal(goal)
			else:
				print('not implemented yet')

		
		# print(obj)
		add_instance(obj)


def go_to_base(mission, uav):
	"""
    Go to the nearest base immediately and land.

    Args:
        mission (Mission): Mission object containing information about the mission.
        uav (Drone): UAV object containing information about the UAV.

    Returns:
        Base or None: Base object if the drone successfully went to the nearest base, else None.
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

'''
	Callers for MAVRos Services
'''

def mavros_cmd(topic, msg_ty, error_msg="MAVROS command failed: ", **kwargs):
	"""
    Send a command to MAVROS service.

    Args:
        topic (str): ROS service topic for MAVROS command.
        msg_ty (Message Type): ROS message type for the command.
        error_msg (str, optional): Error message to log in case of failure. Defaults to "MAVROS command failed: ".
        **kwargs: Additional keyword arguments for the command.
    """
	rospy.wait_for_service(topic)
	try:
		service_proxy = rospy.ServiceProxy(topic, msg_ty)
		response = service_proxy(**kwargs)
		rospy.loginfo(response)
	except rospy.ServiceException as e:
		rospy.logerr(f"{error_msg} {e}")

def land():
    """
    Command the drone to land using MAVROS.
    """
    mavros_cmd(
        '/mavros/cmd/land',
        CommandTOL,
        error_msg="Landing failed",
        altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0
    )


def set_mode(mode):
    """
    Set the flight mode of the drone using MAVROS.
    """
    mavros_cmd(
        '/mavros/set_mode',
        SetMode,
        error_msg="Set mode failed",
        custom_mode=mode
    )


def clear_mission():
    """
    Clear the current mission in MAVROS.
    """
    mavros_cmd(
        '/mavros/mission/clear',
        WaypointClear,
        error_msg="Clear mission failed"
    )


def send_route(route):
    """
    Send a route (list of waypoints) to the MAVROS mission planner.
    """
    mavros_cmd(
        '/mavros/mission/push',
        WaypointPush,
        error_msg="Send route failed",
        start_index=route.current_seq,
        waypoints=route.waypoints
    )

if __name__ == "__main__":
	rospy.Subscriber('/harpia/control/kill_mission', String, control_callback)
	server = ActionServer()
	server.run()
