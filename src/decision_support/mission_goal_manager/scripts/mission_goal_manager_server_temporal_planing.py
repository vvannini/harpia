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



'''
	Classes to subscribe and publish services
'''

class Drone(object):
	def __init__(self):
		self.sub_position = rospy.Subscriber('mavros/global_position/global', NavSatFix, self.global_position_callback)
		self.sub_battery  = rospy.Subscriber('mavros/battery', BatteryState, self.battery_state_callback)
		self.sub_mission  = rospy.Subscriber('mavros/mission/reached', WaypointReached, self.reached_callback)
		self.latitude	  = None
		self.longitude	  = None
		self.battery	  = None
		self.current	  = None


	def global_position_callback(self, data):
		self.latitude = data.latitude
		self.longitude = data.longitude

	def battery_state_callback(self, data):
		self.battery = data.percentage * 100

	def reached_callback(self, data):
		self.current = data.wp_seq + 1


class ActionServer():
	"""
	This is the main class that runs the action server. The `run()` method
	initiates the server and blocks while the server is online.
	"""

	def __init__(self):
		self.a_server = actionlib.SimpleActionServer(
			"harpia/mission_goal_manager",
			MissionPlannerAction,
			execute_cb=self.execute_cb,
			auto_start=False
			# cancel = self.execute_cb
		)
		self.uav = Drone()
		self.new_goals    = None
		self.Mission_Sub  = rospy.Subscriber('/harpia/mission_goal_manager/goal', MissionPlannerActionGoal, self.mission_callback)
		self.Goals_Sub    = rospy.Subscriber('/harpia/ChangeMission', ChangeMission, self.new_goal_callback)
		self.mission      = None
		self.change_goals = None

	def mission_callback(self, data):
		self.mission = data.goal.mission

	def new_goal_callback(self, data):
		try:
			# print(data)
			self.change_goals = data.op
			self.new_goals = data.goals

		except:
			self.change_goals = 0

	def run(self):
		rospy.init_node("mission_goal_manager_server")
		self.a_server.start()
		rospy.loginfo("Mission Goal Manager Service Ready")

		rospy.spin()

	def register_cancel_callback(self,cancel_cb):
		self.a_server.set_aborted(MissionPlannerResult())

	def abort(self):   self.a_server.set_aborted(MissionPlannerResult())
	def succeed(self): self.a_server.set_succeeded(MissionPlannerResult())

	def execute_cb(self, data):
		rospy.loginfo(f"EXECUTE - MissionGoalManager with op {data.op}")

		if data.op == OP_UPDATE_KNOWLEDGE_BASE:
			update_knowledge_base(data.mission.uav, data.mission.map, data.mission.goals, "base_1")
		elif data.op == OP_REPLAN:
			rospy.loginfo("REPLAN - MissionGoalManager")
			replan(data.mission)
		elif data.op == OP_ADD_RM_GOALS:
			rospy.logerr("ADD/RM GOALS - MissionGoalManager not implemented yet")
			self.abort()
			return

		feedback_msg = MissionPlannerFeedback()
		

		print(self.a_server)
		while not call_mission_planning():
			if self.change_goals:
				print('MISSION GOAL Manager CANCEL')
				feedback_msg.status = 1				

				# while self.change_goals:
				self.a_server.publish_feedback(feedback_msg)

				#def replan(mission, uav, base, goals, op):
				replan(self.mission, self.uav, None, self.new_goals, self.change_goals)


			elif not wait_until(lambda: self.uav.battery is not None, msg="Waiting for UAV battery..."):
				# If the wait has failed abor the mission immediately
				self.abort()
				return

			if self.uav.battery <= 20:
				base = go_to_base(data.mission, self.uav)
				replan(self.mission, self.uav, base, None, 0)
			else:
				replan(data.mission, self.uav, None, None, 0)
			feedback_msg.status = 0
			self.a_server.publish_feedback(feedback_msg)
			self.change_goals = 0

		# Land the drone
		land()
		rospy.sleep(30)
		self.succeed()

def wait_until(check, msg=None, rate=1):
	rate = rospy.Rate(rate)

	while not check():
		if rospy.is_shutdown(): return False
		if msg is not None: rospy.loginfo(msg)
		rate.sleep()

	return True

'''
	Callers for ROSPlan Services
'''
def try_call_srv(topic, msg_ty=Empty):
	rospy.wait_for_service(topic)
	try:
		query_proxy = rospy.ServiceProxy(topic, msg_ty)
		query_proxy()
		return True
	except rospy.ServiceException as e:
		rospy.logerr(f"MissionGoalManager Service call to {topic} failed: {e}")
		return False

def call_problem_generator(): return try_call_srv('/rosplan_problem_interface/problem_generation_server', Empty)
def call_plan_generator():	  return try_call_srv('/rosplan_planner_interface/planning_server'			, Empty)
def call_parser():			  return try_call_srv('/rosplan_parsing_interface/parse_plan'				, Empty)
def call_dispach():			  return try_call_srv('/rosplan_plan_dispatcher/dispatch_plan'				, Empty)
def cancel_dispatch():		  return try_call_srv('/rosplan_plan_dispatcher/cancel_dispatch'			, Empty)
def call_mission_planning():  return try_call_srv('/harpia/mission_planning'							, Empty)

'''
	Fuctions to calc the regios distances
'''
def calc_distances(regions):
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
	return math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)

def geo_to_cart(geo_point, geo_home):
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
	rospy.wait_for_service('/rosplan_knowledge_base/update')
	try:
		query_proxy = rospy.ServiceProxy('rosplan_knowledge_base/update', KnowledgeUpdateService)
		query_proxy(update_type, item)
		return True
	except rospy.ServiceException as e:
		rospy.logerr(f"MissionGoalManager Service call to {topic} failed: {e}")
		return False

def call_clear():		   return try_call_srv('/rosplan_knowledge_base/clear', Empty)

def add_instance(item):    return try_update_knowledge(item, KB_UPDATE_ADD_KNOWLEDGE)
def remove_instance(item): return try_update_knowledge(item, KB_UPDATE_RM_KNOWLEDGE)
def add_goal(item):		   return try_update_knowledge(item, KB_UPDATE_ADD_GOAL)
def remove_goal(item):	   return try_update_knowledge(item, KB_UPDATE_RM_GOAL)
def add_metric(item):	   return try_update_knowledge(item, KB_UPDATE_ADD_METRIC)

def get_knowledge(name, topic):
	rospy.wait_for_service(topic)
	try:
		query_proxy = rospy.ServiceProxy(topic, GetAttributeService)
		return query_proxy(name)
	except rospy.ServiceException as e:
		rospy.logerr(f"MissionGoalManager Service call to {topic} failed: {e}")
		return KnowledgeItem()

def get_function(function_name):   return get_knowledge(function_name , '/rosplan_knowledge_base/state/functions')
def get_goal(goal_name):		   return get_knowledge(goal_name, '/rosplan_knowledge_base/state/goals')
def get_predicate(predicate_name): return get_knowledge(predicate_name, '/rosplan_knowledge_base/state/propositions')

def create_object(item_name, item_type):
	instance = KnowledgeItem()
	instance.knowledge_type = KnowledgeItem.INSTANCE
	instance.instance_type = item_type
	instance.instance_name = item_name

	return instance

def create_function(attribute_name, function_value, values=[]):
	instance = KnowledgeItem()

	instance.knowledge_type = KnowledgeItem.FUNCTION
	instance.attribute_name = attribute_name
	instance.values			= values
	instance.function_value = function_value

	return instance

def create_predicate(attribute_name, values=[], is_negative=False):
	instance = KnowledgeItem()

	instance.knowledge_type = KnowledgeItem.FACT
	instance.attribute_name = attribute_name
	instance.values			= values
	instance.is_negative	= is_negative

	return instance

def create_metric(optimization, item):
	instance = KnowledgeItem()

	instance.knowledge_type = KnowledgeItem.EXPRESSION
	instance.optimization	= optimization
	instance.expr.tokens	= item

	return instance

def set_distances(map, goals, latitude, longitude):

	#see distance to all regions
	geo_home = map.geo_home

	regions_name = []
	for g in goals:
		regions_name.append(g.region)

	#get current lat long
	cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), geo_home)

	distance = float('inf')
	closest_region = ''
	for base in map.bases:
		d = euclidean_distance(base.center.cartesian, cart_location)
		obj = create_function(
			"distance",
			round(d, 2),
			[KeyValue("region", "aux"), KeyValue("region", base.name)]
		)
		add_instance(obj)

	for region in map.roi:
		if(region.name in regions_name):
			d = euclidean_distance(cart_location, region.center.cartesian)
			obj = create_function(
				"distance",
				[KeyValue("region", "aux"), KeyValue("region", region.name)],
				round(d, 2)
			)
			add_instance(obj)

def regions_to_perform_action(goals, action):
		return [step.region for step in goals if step.action == action]

def update_knowledge_base(uav, map, goals, at):
	
	regions   = map.roi
	bases	   = map.bases
	pulverize = regions_to_perform_action(goals, 'pulverize')
	photo	  = regions_to_perform_action(goals, 'take_picture')
	end		  = regions_to_perform_action(goals, 'end')

	# In this version I'm only adding in the initial 
	# I had to make it as an for to prevent repetition
	goals_regions = set(pulverize + photo)
	regions_obj = [r for r in regions if r.name in goals_regions] + bases

	call_clear()

	# set drone init

	#adding objetcts to knowlege base
	add_instance(create_predicate("at", [KeyValue("region", at)]))
	add_instance(create_predicate("can-go"))
	add_instance(create_predicate("can-take-pic"))
	add_instance(create_function("battery-capacity", uav.battery.capacity))
	add_instance(create_function("velocity", uav.frame.efficient_velocity))
	add_instance(create_function("battery-amount", 100))
	add_instance(create_function("recharge-rate-battery", uav.battery.recharge_rate))
	add_instance(create_function("discharge-rate-battery", uav.battery.discharge_rate))
	add_instance(create_function("input-amount", 0))
	add_instance(create_function("missionLength", 0))
	add_instance(create_function("input-capacity", uav.input_capacity))

	for r in regions:
		add_instance(create_object(str(r.name), "region"))
		add_instance(create_predicate("its-not-base", [KeyValue("region", r.name)]))

	for b in bases:
		add_instance(create_object(str(b.name), "base"))

	for fact in calc_distances(regions_obj):
		add_instance(fact)

	if pulverize:
		add_instance(create_predicate("has-pulverize-goal"))

	if photo:
		add_instance(create_predicate("has-picture-goal"))

	total_goals = 0

	for i in pulverize:
		add_instance(create_predicate("pulverize-goal", [KeyValue("region", i)]))
		add_instance(create_function("pulverize-path-len", 314, [KeyValue("region", i)]))
		add_goal(create_predicate("pulverized", [KeyValue("region", i)]))
		total_goals += 1

	for i in photo:
		add_instance(create_predicate("picture-goal", [KeyValue("region", i)]))
		add_instance(create_function("picture-path-len", 1000, [KeyValue("region", i)]))
		add_goal(create_predicate("takenImage", [KeyValue("region", i)]))
		total_goals += 1

	for i in end:
		add_goal(create_predicate("at", [KeyValue("region", i)]))

	add_instance(create_function("total-goals", total_goals))
	add_instance(create_function("goals-achived", 0))
	# add_metric(create_metric("minimize (total-time)", []))
	add_metric(create_metric("minimize (missionLength)", []))

def find_at(map, goals, latitude, longitude):
	"""
	Finds a base that is withing a 50m radius. If none is found, try to find a region which has a center
	within the same range. If none is found, return `None`. If there is one of these waypoints in range,
	the drone is considered at that waypoint.
	"""

	#get current lat long
	cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), map.geo_home)

	for base in map.bases:
		d = euclidean_distance(base.center.cartesian, cart_location)
		# rospy.loginfo(f"base = {base.name} distance={d}")
		if d <= 50:
			return base

	region_names = { g.region for g in goals }

	for region in map.roi:
		if region.name in region_names:
			d = euclidean_distance(cart_location, region.center.cartesian)
			# rospy.loginfo(f"base = {region.name} distance={d}")
			if d <= 50:
				return region

	return None

def find_nearest_base(map, latitude, longitude):
	cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), map.geo_home)

	return min(map.bases, key=lambda base: euclidean_distance(base.center.cartesian, cart_location))

def call_path_planning(r_from, r_to, map):
	rospy.wait_for_service("/harpia/path_planning")
	try:
		query_proxy = rospy.ServiceProxy("/harpia/path_planning", PathPlanning)
		resp = query_proxy(r_from.center, r_to.center, r_from.name, r_to.name, 3, map)
		return resp
	except rospy.ServiceException as e:
		rospy.logerr("Service call failed: %s" % e)
		return None

def replan(mission, uav, base, goals, op):
	rospy.loginfo("CANCEL DISPATCH - MissionGoalManager")

	if not wait_until(lambda: mission.goals != []	  , msg="Waiting for Regions..."): return
	if not wait_until(lambda: uav.latitude is not None, msg="Waiting for position..."): return

	if goals != None:
		pulverize = regions_to_perform_action(goals, 'pulverize')
		photo	  = regions_to_perform_action(goals, 'take_picture')

	if base == None:
		# Here we want to continue the mission.
		at_move = get_predicate("at-move")
		at = get_predicate("at")

		# rospy.loginfo(f"at={at}")
		# rospy.loginfo(f"len at_={len(at.attributes)}")

		if len(at.attributes) == 0:
			if len(at_move.attributes) > 0:
				remove_instance(at_move.attributes[0])

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
	f = get_function("total-goals")
	total_goals = f.attributes[0].function_value
	remove_instance(f.attributes[0])

	bat = get_function("battery-amount")
	remove_instance(bat.attributes[0])

	if not wait_until(lambda: uav.battery is not None, msg="Waiting for UAV battery..."): return

	obj = create_function("battery-amount", uav.battery)
	add_instance(obj)
	# has_end is a variable to verify if a mission has already a designed end
	has_end = False
	if (op == 0) :
		# getting current goals and verifying if it already done
		for goal in get_goal('').attributes:
			if goal.attribute_name != "at":
				for goal_achived in get_predicate(goal.attribute_name).attributes:
					if goal.values[0].value == goal_achived.values[0].value:
						f.attributes[0].function_value = f.attributes[0].function_value - 1
						remove_goal(goal)
			else:
				has_end =  True
		add_instance(create_function("total-goals", total_goals))

	elif (op == 1):
		if pulverize:
			add_instance(create_predicate("has-pulverize-goal"))

		if photo:
			add_instance(create_predicate("has-picture-goal"))

		for i in pulverize:
			add_instance(create_predicate("pulverize-goal", [KeyValue("region", i)]))
			add_instance(create_function("pulverize-path-len", 314, [KeyValue("region", i)]))
			add_goal(create_predicate("pulverized", [KeyValue("region", i)]))
			total_goals = total_goals + 1

		for i in photo:
			add_instance(create_predicate("picture-goal", [KeyValue("region", i)]))
			add_instance(create_function("picture-path-len", 1000, [KeyValue("region", i)]))
			add_goal(create_predicate("taken-image", [KeyValue("region", i)]))
			total_goals = total_goals + 1

	elif (op == 2):
		for goal in get_goal('').attributes:
			if goal.attribute_name != "at":
				for goal_toRemove in goals:
					if goal.values[0].value == goal_achived.values[0].value:
						f.attributes[0].function_value = f.attributes[0].function_value - 1
						remove_goal(goal)
			else:
				has_end =  True

		add_instance(create_function("total-goals", total_goals))

		add_instance(create_function("total-goals", total_goals))
		print(obj)
		add_instance(obj)

		for g in goals: mission.goals.append(g)
		regions   = mission.map.roi
		pulverize = regions_to_perform_action(mission.goals, 'pulverize')
		photo	  = regions_to_perform_action(mission.goals, 'take_picture')

		goals_regions = set(pulverize + photo)
		bases	   = mission.map.bases
		regions_obj = [r for r in regions if r.name in goals_regions] + bases

		for fact in calc_distances(regions_obj):
			add_instance(fact)

	


def replan1(mission, goals, op, uav):
	rospy.loginfo("CANCEL DISPATCH - MissionGoalManager")

	pulverize = regions_to_perform_action(goals, 'pulverize')
	photo	  = regions_to_perform_action(goals, 'take_picture')

	if not wait_until(lambda: uav.latitude is not None, msg="Waiting for position..."): return
	if not wait_until(lambda: mission.goals != []	  , msg="Waiting for Regions..."): return
	# print(mission.goals)
	
	# Here we want to continue the mission.
	at_move = get_predicate("at-move")
	at = get_predicate("at")

	# rospy.loginfo(f"at={at}")
	# rospy.loginfo(f"len at_={len(at.attributes)}")

	if len(at.attributes) == 0:
		if len(at_move.attributes) > 0:
			remove_instance(at_move.attributes[0])

		#codigo achar at
		base_or_region_nearby = find_at(mission.map, mission.goals, uav.latitude, uav.longitude)
		if base_or_region_nearby != None:
			# We are at a base or a region.
			# rospy.loginfo(f'at = {base_or_region_nearby}')
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
	
	add_instance(at)

	# saving the current total goals to manage goals
	f = get_function("total-goals")
	total_goals = f.attributes[0].function_value
	print(f.attributes[0])
	#.attributes[0]
	remove_instance(f.attributes[0])

	# has_end is a variable to verify if a mission has already a designed end
	has_end = False

	bat = get_function("battery-amount")
	remove_instance(bat.attributes[0])

	if not wait_until(lambda: uav.battery is not None, msg="Waiting for UAV battery..."): return

	obj = create_function("battery-amount", uav.battery)
	add_instance(obj)

	if (op == 1):
		if pulverize:
			add_instance(create_predicate("has-pulverize-goal"))

		if photo:
			add_instance(create_predicate("has-picture-goal"))

		for i in pulverize:
			add_instance(create_predicate("pulverize-goal", [KeyValue("region", i)]))
			add_instance(create_function("pulverize-path-len", 314, [KeyValue("region", i)]))
			add_goal(create_predicate("pulverized", [KeyValue("region", i)]))
			total_goals = total_goals + 1

		for i in photo:
			add_instance(create_predicate("picture-goal", [KeyValue("region", i)]))
			add_instance(create_function("picture-path-len", 1000, [KeyValue("region", i)]))
			add_goal(create_predicate("taken-image", [KeyValue("region", i)]))
			total_goals = total_goals + 1

		add_instance(create_function("total-goals", total_goals))
		print(obj)
		add_instance(obj)

		for g in goals: mission.goals.append(g)
		regions   = mission.map.roi
		pulverize = regions_to_perform_action(mission.goals, 'pulverize')
		photo	  = regions_to_perform_action(mission.goals, 'take_picture')

		goals_regions = set(pulverize + photo)
		bases	   = mission.map.bases
		regions_obj = [r for r in regions if r.name in goals_regions] + bases

		for fact in calc_distances(regions_obj):
			add_instance(fact)



def go_to_base(mission, uav):
	"""
	Go to nearest base immediately and land.
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
	rospy.wait_for_service(topic)
	try:
		service_proxy = rospy.ServiceProxy(topic, msg_ty)
		response = service_proxy(**kwargs)
		rospy.loginfo(response)
	except rospy.ServiceException as e:
		rospy.logerr(f"{error_msg} {e}")

def land():
	mavros_cmd(
		'/mavros/cmd/land',
		CommandTOL,
		error_msg="Landing failed",
		altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0
	)

def set_mode(mode):
	mavros_cmd(
		'/mavros/set_mode',
		SetMode,
		error_msg="Set mode failed",
		custom_mode=mode
	)

def clear_mission():
	mavros_cmd(
		'/mavros/mission/clear',
		WaypointClear,
		error_msg="Clear mission failed"
	)

def send_route(route):
	mavros_cmd(
		'/mavros/mission/push',
		WaypointPush,
		error_msg="Send route failed",
		start_index=route.current_seq,
		waypoints=route.waypoints
	)

if __name__ == "__main__":
	server = ActionServer()
	server.run()
