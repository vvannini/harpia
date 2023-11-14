#!/usr/bin/env python3

import sys, traceback

import math
import os
import pandas as pd
import pickle
import json
import rospy
import time

from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import *
from rosplan_dispatch_msgs.srv import *

from sensor_msgs.msg import BatteryState


from harpia_msgs.msg import *
from mavros_msgs.msg import *
from harpia_msgs.srv import *

# Used:
# WaypointList
# Waypoint
# PathPlanningResponse
# PathPlanning

import time
import shapely.geometry

# Behaviours
from libs.Behaviours.behaviours import pulverize, picture

# Ray Casting
from libs.RayCasting.raycasting import point_in_polygon, Vector

# AG
from libs.AG.genetic import Subject, Genetic
from libs.AG.model import Mapa, Conversor, CartesianPoint
from libs.AG.visualization import vis_mapa

# RRT
from libs.RRT.rrt import RRT

# PFP
from libs.PotentialFieldPlanning.potential_field_planning import potential_field_planning

class Drone(object):
    """
    A class representing a drone and its battery state information.

    Attributes:
        sub_battery (rospy.Subscriber): A ROS subscriber for monitoring battery state.
        battery (float): The current battery percentage.
    """

    def __init__(self):
        """
        Initializes a Drone instance with a battery state subscriber and sets the battery percentage to None.
        """
        self.sub_battery = rospy.Subscriber('mavros/battery', BatteryState, self.battery_state_callback)
        self.battery = None

    def battery_state_callback(self, data):
        """
        Callback function to update the battery percentage based on received battery state information.

        Args:
            data (BatteryState): Battery state information received from the 'mavros/battery' topic.
        """
        self.battery = data.percentage * 100

# ---
# UTILS
def log(info):
    """
    Log information to a JSON log file.

    Args:
        info (dict): A dictionary containing information to be logged.

    This function opens an existing JSON log file, appends the provided 'info' dictionary to the log, and then saves the updated log back to the file.
    """

    # Get the root directory for Harpia.
    harpia_root_dir = get_harpia_root_dir()

    # Define the path to the log directory and the log file.
    log_path = os.path.join(harpia_root_dir, "results")
    log_json = os.path.join(log_path, "mission_log.json")

    # Open the log file for reading.
    with open(log_json, "r") as log_file:
        # Load the JSON content from the file.
        log_file = json.load(log_file)

    # Add the 'info' dictionary to the log.
    log_file[-1]['knn'].append(info)

    # Dump the updated log back to the log file.
    with open(log_json, 'w') as outfile:
        json.dump(log_file, outfile, indent=4)


def euclidean_distance(A, B):
    """
    Calculate the Euclidean distance between two 2D points A and B.

    Args:
        A (Point): The first point with attributes x and y.
        B (Point): The second point with attributes x and y.

    Returns:
        float: The Euclidean distance between points A and B.

    The function takes two points, A and B, and computes the Euclidean distance
    between them in a 2D space using the Pythagorean theorem.
    """
    return math.sqrt((B.x - A.x) ** 2 + (B.y - A.y) ** 2)

def calc_dist_path(path):
    """
    Calculate the total distance along a path defined by a list of 2D points.

    Args:
        path (list of tuples): A list of 2D points, where each point is represented as a tuple (x, y).

    Returns:
        float: The total Euclidean distance along the path.

    The function iterates through the list of points in the path and calculates the
    total distance by summing the Euclidean distances between consecutive points.
    """
    dist = 0
    i = 1
    for p in path:
        if p != path[-1]:
            A = CartesianPoint(p[0], p[1])
            B = CartesianPoint(path[i][0], path[i][1])
            dist += euclidean_distance(A, B)
            i += 1

    return dist

def wait_until(check, msg=None, rate=1):
     """
    Wait until a given condition is met.

    Args:
        check (callable): A callable that represents the condition to check. It should return a boolean.
        msg (str, optional): An optional message to log while waiting.
        rate (float, optional): The rate (in Hz) at which to check the condition.

    Returns:
        bool: True if the condition is met; False if ROS is shut down or the condition is not met.

    This function repeatedly checks the given condition (the 'check' parameter) until it returns True.
    It can also log a message (the 'msg' parameter) at each check. The checking rate is controlled by the 'rate' parameter.

    The function returns True if the condition is met or False if ROS is shut down or the condition is not met.
    """
    rate = rospy.Rate(rate)

    while not check():
        if rospy.is_shutdown(): return False
        if msg is not None: rospy.loginfo(msg)
        rate.sleep()

    return True

def get_harpia_root_dir():
    """
    Get the root directory path of the 'Harpia' project.

    Returns:
        str: The root directory path as a string.

    This function retrieves the root directory path of the 'Harpia' project. It does so by obtaining a parameter from the ROS parameter server. If the parameter '/harpia_home' exists, it returns its value as the root directory path. Otherwise, it falls back to a default directory path, which is the user's home directory with 'harpia' appended to it.

    Returns the root directory path as a string.
    """
    return rospy.get_param("/harpia_home", default=os.path.expanduser("~/harpia"))

def to_waypointList(route, geo_home):
    """
    Convert a list of waypoints in Cartesian coordinates to a WaypointList in geographic coordinates.

    Args:
        route (list): A list of waypoints in Cartesian coordinates.
        geo_home (GeoPoint): The home location in geographic coordinates.

    Returns:
        WaypointList: A WaypointList containing the converted waypoints in geographic coordinates.

    This function takes a list of waypoints in Cartesian coordinates and converts them into a WaypointList in geographic coordinates. It performs the conversion for each waypoint by first converting from Cartesian to geographic coordinates using the provided home location (geo_home). The resulting WaypointList includes the converted waypoints with appropriate fields set for mission planning.

     """
    geo_route = WaypointList()
    for wp in route:
        geo = Conversor.cart_to_geo(CartesianPoint(wp[0], wp[1]), geo_home)
        geo_wp = Waypoint()
        geo_wp.frame = 3
        geo_wp.command = 16
        if geo_route.waypoints == []:
            geo_wp.is_current = True
        else:
            geo_wp.is_current = False
        geo_wp.autocontinue = True
        geo_wp.param1 = 0
        geo_wp.param2 = 0
        geo_wp.param3 = 0
        geo_wp.param4 = 0
        geo_wp.x_lat = geo.latitude
        geo_wp.y_long = geo.longitude
        geo_wp.z_alt = 15
        geo_route.waypoints.append(geo_wp)

    return geo_route


def feasibility_ag(fitness_trace):
    """
    Evaluate the feasibility of an action based on a fitness trace.

    Args:
        fitness_trace (list): A fitness trace containing two fitness values.

    Returns:
        str: A string indicating the feasibility of the action.

    This function takes a fitness trace as input, which is a list containing two fitness values: the first value represents the fitness related to hitting an obstacle, and the second value represents the fitness related to the distance from the destination waypoint.

    Args:
        - fitness_trace (list): A list of two fitness values [fit_obs, fit_d].

    Returns:
        - str: A string indicating the feasibility of the action. Possible values are "feasible," "verify," or "infeasible."

    Feasibility is determined as follows:
    - If `fit_obs` (fitness related to hitting an obstacle) is greater than 0, it is considered "infeasible" (indicating the action hit an obstacle).
    - If `fit_d` (fitness related to the distance from the destination waypoint) is greater than 10 meters, it is considered "verify" (indicating the action is away from the destination waypoint).
    - If neither condition is met, the action is considered "feasible."

    The function returns a string indicating the feasibility status of the action.
    """
    if fitness_trace[1] > 0:  # Hit obstacle (fit_obs) (max 1 hit)
        return "infeasible"
    elif fitness_trace[0] > 10:  # Away from destination wp (fit_d) (max 10 meters)
        return "verify"
    else:
        return "feasible"

def feasibility(route, obstacles, destination):
  """
    Check the feasibility of a route with respect to obstacles and the distance to the destination.

    Args:
        route (list of tuple): A list of waypoints (x, y) representing the route.
        obstacles (list of list): A list of obstacle polygons, each represented as a list of vertices.
        destination (tuple): The (x, y) coordinates of the destination waypoint.

    Returns:
        tuple: A tuple containing a feasibility status and the distance to the destination.

    This function checks the feasibility of a route by considering potential obstacles and the distance from the last waypoint in the route to the destination waypoint.

    Args:
        - route (list of tuple): A list of waypoints (x, y) representing the route.
        - obstacles (list of list): A list of obstacle polygons, each represented as a list of vertices.
        - destination (tuple): The (x, y) coordinates of the destination waypoint.

    Returns:
        - tuple: A tuple containing two elements:
            1. str: A string indicating the feasibility status, which can be one of "feasible," "verify," or "infeasible."
            2. float: The distance from the last waypoint to the destination waypoint.

    The function first calculates the distance from the last waypoint in the route to the destination waypoint. If this distance is greater than a specified minimum (MIN_PRECISION), the route status is set to "verify."

    It then checks if any of the waypoints in the route fall within any of the obstacle polygons. If any waypoint is inside an obstacle, the route status is set to "infeasible."

    If none of the above conditions are met, the route status is set to "feasible."

    The function returns a tuple with the feasibility status and the distance to the destination.
    """
    # Minimum distance the last waypoint needs to be from the destination wp
    MIN_PRECISION = 10

    last_waypoint = Vector(route[-1][0], route[-1][1])
    rospy.loginfo(f"feasibility last_waypoint={last_waypoint}  destination={destination}")
    distance_to_objective = euclidean_distance(last_waypoint, destination)
    rospy.loginfo(f"feasibility distance_to_objective={distance_to_objective}")

    for obstacle in obstacles:
        for (x, y) in route:
            waypoint = Vector(x, y)
            if point_in_polygon(waypoint, obstacle):
                return "infeasible", distance_to_objective

    if distance_to_objective > MIN_PRECISION:
        return "verify", distance_to_objective

    return "feasible", distance_to_objective


def get_first_factible_route(ag):
    """
    Find the first feasible route in an agent's history based on its fitness trace.

    Args:
        ag (object): An agent with a history attribute that contains fitness traces.

    Returns:
        int or None: The birth time of the first feasible route found in the agent's history, or None if none are feasible.

    This function iterates through an agent's history and checks the feasibility of each route based on its fitness trace. It looks for the first route that is deemed "feasible" and returns the birth time of that route.

    Args:
        - ag (object): An agent with a history attribute that contains fitness traces.

    Returns:
        - int or None: The birth time of the first feasible route found in the agent's history, or None if none are feasible.
    """
    for subject in ag.history:
        feasibility_result = feasibility_ag(subject['fitness_trace'])

        if feasibility_result == "feasible":
            return subject['birth_time']

    return None

def pass_through_obstacle(obstacle_points, line_points):
     """
    Check if a line passes through an obstacle.

    Args:
        obstacle_points (list of tuples): Points representing the vertices of the obstacle polygon.
        line_points (list of tuples): Points representing the vertices of the line.

    Returns:
        bool: True if the line passes through the obstacle, False otherwise.

    This function checks if a line, represented by a sequence of points, passes through an obstacle, represented by a sequence of points. It uses the Shapely library for geometric operations to perform the intersection check.

    Args:
        - obstacle_points (list of tuples): Points representing the vertices of the obstacle polygon.
        - line_points (list of tuples): Points representing the vertices of the line.

    Returns:
        - bool: True if the line passes through the obstacle, False otherwise.
    """
    poly = shapely.geometry.Polygon(obstacle_points)
    line = shapely.geometry.LineString(line_points)
    
    return line.intersects(poly)

def count_obstacles(from_wp, to_wp, map, geo_home):
     """
    Count the number of obstacles intersecting with a line segment.

    Args:
        from_wp (Waypoint): Starting waypoint.
        to_wp (Waypoint): Ending waypoint.
        map (Map): Map containing no-fly zones.
        geo_home (GeoPoint): Home location in geographic coordinates.

    Returns:
        int: The number of obstacles intersecting with the line segment.

    This function counts the number of obstacles that intersect with a line segment between two waypoints. It uses the provided map with no-fly zones and the home location in geographic coordinates for reference.

    Args:
        - from_wp (Waypoint): Starting waypoint.
        - to_wp (Waypoint): Ending waypoint.
        - map (Map): Map containing no-fly zones.
        - geo_home (GeoPoint): Home location in geographic coordinates.

    Returns:
        - int: The number of obstacles intersecting with the line segment.
    """
    obst_qty = 0

    line = [(from_wp.cartesian.x, from_wp.cartesian.y), (to_wp.cartesian.x, to_wp.cartesian.y)]

    for nfz in map.nfz:
        poly = [(point.cartesian.x, point.cartesian.y) for point in nfz.points]
        if(pass_through_obstacle(poly, line)):
            obst_qty += 1

    return obst_qty
# ---
# PATH PLANNERS


def pfp(from_wp, to_wp, map, save_filename=None):
    """
    Perform path planning using Potential Field Planning (PFP).

    Args:
        from_wp (Waypoint): Starting waypoint.
        to_wp (Waypoint): Ending waypoint.
        map (Map): Map containing no-fly zones.
        save_filename (str): Optional filename to save the generated map image.

    Returns:
        tuple: A tuple containing waypoint list, path length, feasibility status, distance to objective, time taken, and path length.

    This function performs path planning using Potential Field Planning (PFP) to generate a safe path from a starting waypoint to an ending waypoint while avoiding obstacles.

    Args:
        - from_wp (Waypoint): Starting waypoint.
        - to_wp (Waypoint): Ending waypoint.
        - map (Map): Map containing no-fly zones.
        - save_filename (str): Optional filename to save the generated map image.

    Returns:
        - tuple: A tuple containing the following elements:
          - WaypointList: A list of waypoints representing the generated path.
          - float: The length of the generated path.
          - str: Feasibility status, which can be 'feasible', 'infeasible', or 'verify'.
          - float: Distance to the objective (destination waypoint).
          - float: Time taken to generate the path.
          - int: Length of the generated path.

    This function calculates a path using Potential Field Planning (PFP) from the provided starting waypoint to the ending waypoint, considering the specified map with no-fly zones. The generated path is returned as a list of waypoints, along with information about its feasibility, length, and the time taken for the planning process. An optional map image can be saved to the provided file.
    """
    origin = Conversor.geo_to_cart(from_wp.geo, map.geo_home)
    rospy.loginfo(f"origin={origin}")

    destination = Conversor.geo_to_cart(to_wp.geo, map.geo_home)
    rospy.loginfo(f"destination={destination}")

    # ---
    # Prepare obstacleList for check_collision_mode='ray_casting'
    obstacleList = [[Conversor.geo_to_cart(p.geo, map.geo_home) for p in area.points] for area in map.nfz]

    rc_map = Mapa(
        origin,
        destination,
        areas_n=obstacleList,
    )
    # ---

    resolution = 20

    start_time = time.time()
    route = potential_field_planning(origin, destination, obstacleList, resolution)
    time_taken = time.time() - start_time

    if route is None:
        rospy.logwarn("Cannot find route")
    else:
        rospy.loginfo("found route!!")
        rospy.loginfo(f"route={route}")

        feasibility_res, distance_to_objective = feasibility(route, obstacleList, destination)
        if save_filename:
            vis_mapa(rc_map, route=route, save=f"{save_filename}.png")

        return (
            to_waypointList(route, map.geo_home),
            calc_dist_path(route),
            feasibility_res,
            distance_to_objective,
            time_taken,
            len(route),
        )


def rrt(from_wp, to_wp, map, save_filename=None):
"""
    Perform path planning using the Rapidly-exploring Random Tree (RRT) algorithm.

    Args:
        from_wp (Waypoint): Starting waypoint.
        to_wp (Waypoint): Ending waypoint.
        map (Map): Map containing no-fly zones.
        save_filename (str): Optional filename to save the generated map image.

    Returns:
        tuple: A tuple containing waypoint list, path length, feasibility status, distance to objective, time taken, and path length.

    This function performs path planning using the Rapidly-exploring Random Tree (RRT) algorithm to generate a safe path from a starting waypoint to an ending waypoint while avoiding obstacles.

    Args:
        - from_wp (Waypoint): Starting waypoint.
        - to_wp (Waypoint): Ending waypoint.
        - map (Map): Map containing no-fly zones.
        - save_filename (str): Optional filename to save the generated map image.

    Returns:
        - tuple: A tuple containing the following elements:
          - WaypointList: A list of waypoints representing the generated path.
          - float: The length of the generated path.
          - str: Feasibility status, which can be 'feasible', 'infeasible', or 'verify'.
          - float: Distance to the objective (destination waypoint).
          - float: Time taken to generate the path.
          - int: Length of the generated path.

    This function calculates a path using the Rapidly-exploring Random Tree (RRT) algorithm from the provided starting waypoint to the ending waypoint, considering the specified map with no-fly zones. The generated path is returned as a list of waypoints, along with information about its feasibility, length, and the time taken for the planning process. An optional map image can be saved to the provided file.
    """
    origin = Conversor.geo_to_cart(from_wp.geo, map.geo_home)
    rospy.loginfo(f"origin={origin}")

    destination = Conversor.geo_to_cart(to_wp.geo, map.geo_home)
    rospy.loginfo(f"destination={destination}")

    # Prepare obstacleList for collision checking using ray casting
    obstacleList = [[Conversor.geo_to_cart(p.geo, map.geo_home) for p in area.points] for area in map.nfz]

    rc_map = Mapa(
        origin,
        destination,
        areas_n=obstacleList,
    )

    # Define the rand_area, based on origin and destination coordinates
    ps = [origin.x, origin.y, destination.x, destination.y]

    rand_area_x = math.floor(min(ps) * 1.2)
    rand_area_y = math.ceil(max(ps) * 1.2)
    rand_area = [rand_area_x, rand_area_y]

    rospy.loginfo(f"rand_area={rand_area}")

    # Calculate the distance to the goal based on the origin and destination
    dist_to_goal = euclidean_distance(origin, destination) * 1.2
    rospy.loginfo(f"dist_to_goal={dist_to_goal}")

    # Set initial parameters for the RRT algorithm
    rrt = RRT(
        start=[origin.x, origin.y],
        goal=[destination.x, destination.y],
        rand_area=rand_area,
        obstacle_list=obstacleList,
        expand_dis=25,  # minumum precision, to consider inside the goal (meters) 100
        path_resolution=1,
        goal_sample_rate=50,
        max_iter=5000,
        check_collision_mode="ray_casting",
    )

    start_time = time.time()
    route = rrt.planning(animation=False)

    if route is not None:
        route = list(reversed(route))

    time_taken = time.time() - start_time

    if route is None:
        rospy.logwarn("Cannot find route")
        return None
    else:
        rospy.loginfo("found route!!")
        rospy.loginfo(f"route={route}")

        feasibility_res, distance_to_objective = feasibility(
            route, obstacleList, destination
        )

        # Draw the final route on the map and save it to a file if a filename is provided
        if save_filename:
            vis_mapa(rc_map, route=route, save=f"{save_filename}.png")

        return (
            to_waypointList(route, map.geo_home),
            calc_dist_path(route),
            feasibility_res,
            distance_to_objective,
            time_taken,
            len(route),
        )


def ag(from_wp, to_wp, map, save_filename=None):
"""
    Perform path planning using the Genetic Algorithm (Algoritmo genetico, AG in pt-br).

    Args:
        from_wp (Waypoint): Starting waypoint.
        to_wp (Waypoint): Ending waypoint.
        map (Map): Map containing no-fly zones.
        save_filename (str): Optional filename to save the generated map image.

    Returns:
        tuple: A tuple containing waypoint list, path length, feasibility status, distance to objective, time taken, and path length.

    This function performs path planning using the Genetic Algorithm (AG) algorithm to generate a safe path from a starting waypoint to an ending waypoint while avoiding obstacles.

    Args:
        - from_wp (Waypoint): Starting waypoint.
        - to_wp (Waypoint): Ending waypoint.
        - map (Map): Map containing no-fly zones.
        - save_filename (str): Optional filename to save the generated map image.

    Returns:
        - tuple: A tuple containing the following elements:
          - WaypointList: A list of waypoints representing the generated path.
          - float: The length of the generated path.
          - str: Feasibility status, which can be 'feasible', 'infeasible', or 'verify'.
          - float: Distance to the objective (destination waypoint).
          - float: Time taken to generate the path.
          - int: Length of the generated path.

    This function calculates a path using the Genetic Algorithm (AG) algorithm from the provided starting waypoint to the ending waypoint, considering the specified map with no-fly zones. The generated path is returned as a list of waypoints, along with information about its feasibility, length, and the time taken for the planning process. An optional map image can be saved to the provided file.
    """
    # Convert map to ag_map
    nfzs = [[point.cartesian for point in area.points] for area in map.nfz]

    origin = Conversor.geo_to_cart(from_wp.geo, map.geo_home)
    destination = Conversor.geo_to_cart(to_wp.geo, map.geo_home)

    # Create an AG map structure
    ag_map = Mapa(
        origin,
        destination,
        areas_n=nfzs,
        # inflation_rate gives a sense of security, expanding the limits of each obstacle.  Defaults to 0.1.
        inflation_rate=0.1,
    )

    # Create an AG instance for path planning
    ag = Genetic(
        Subject,
        ag_map,
        px0=ag_map.origin.x,  # CartesianPoint
        py0=ag_map.origin.y,  # CartesianPoint
        taxa_cross=5,
        population_size=20,
        max_exec_time=120,
        C_d=1000,
        C_obs=1000000,
        C_con=1,
        C_cur=10,
        C_t=10,
        C_dist=100,
        C_z_bonus=0,
        v_min=-3.0,
        v_max=3.0,
        e_min=-3,
        e_max=3,
        a_min=-3.0,
        a_max=3.0,
        T_min=5,
        T_max=15,
        delta_T=10,
        min_precision=10,
        mutation_prob=0.7,
        gps_imprecision=1,
    )

    # Run the AG algorithm and get the best route found
    best = ag.run(info=True)
    route = best.get_route()

    rospy.loginfo("Best route found:")

    # Check if route is feasible
    feasibility_res = feasibility_ag(best.fitness_trace)
    rospy.loginfo(f"Ag generated an <{feasibility_res}> route")

    distance_path = best.fitness_trace[5]  # (fit_dist)
    distance_to_objective = best.fitness_trace[0]  # (fit_d)

    # Get the time when the first feasible route was found
    first_factible_route_found_time = get_first_factible_route(ag)

    # Draw the final route on the map and save it to a file if a filename is provided
    if save_filename:
        vis_mapa(ag_map, route=route, save=f"{save_filename}.png")

    return (
        to_waypointList(route, map.geo_home),
        distance_path,
        feasibility_res,
        distance_to_objective,
        first_factible_route_found_time,
        len(route),
    )

# ---
# SERVER

def select_planner(obstacles_qty, distance, battery):
 """
    Select a path planning algorithm based on input features.

    Args:
        obstacles_qty (int): The number of obstacles in the environment.
        distance (float): The distance to the destination.
        battery (float): The remaining battery percentage.

    Returns:
        str: The selected path planning algorithm ("rrt" or another planner).

    This function takes input features, including the number of obstacles (`obstacles_qty`), the distance to the destination (`distance`), and the remaining battery percentage (`battery`). It uses a K-Nearest Neighbors (KNN) classifier to predict the best path planning algorithm based on these features.

    Returns the selected path planning algorithm, which can be "rrt" or another planner based on the KNN prediction.
    """
    # Uncomment the next line to always return "rrt"
    # return "rrt"

    global KNN

    # Predict the best planner based on the input features
    return KNN.predict([[obstacles_qty, distance, battery]])

def run_path_planning(from_wp, to_wp, map, obstacles_qty):
    """
    Run path planning based on specified waypoints, map, and obstacle quantity.

    Args:
        from_wp (Waypoint): The starting waypoint.
        to_wp (Waypoint): The destination waypoint.
        map (Map): The map containing obstacle information.
        obstacles_qty (int): The quantity of obstacles in the environment.

    Returns:
        list of Waypoint or None: The generated path as a list of waypoints or None if no feasible path is found.

    This function calculates the distance between the starting and destination waypoints, retrieves UAV battery information, and selects a path planning algorithm based on the obstacles quantity. It logs the selected planner and execution time.

    If "ag," "rrt," or "pfp" is selected, the respective path planning algorithm is executed, and the results are logged. If the selected planner fails to find a feasible path, the function iteratively attempts alternative planners from a list (RRT, AG, PFP) and logs each attempt. The function continues until a feasible path is found.

    The generated path (route) is returned as a list of waypoints if a feasible path is found. Otherwise, None is returned.

    Note: The function rotates between planners to increase the likelihood of finding a feasible path when the initial selection fails.
    """
    distance = euclidean_distance(from_wp.cartesian, to_wp.cartesian)/1000

    uav = Drone()

    if not wait_until(lambda: uav.battery is not None, msg="Waiting for UAV battery..."):
        return None

    selected_planner = select_planner(obstacles_qty, distance, uav.battery)
    rospy.loginfo(f"selected_planner={selected_planner}")

    result = None

    if selected_planner == "ag":
        rospy.loginfo("Using ag")
        start = time.time()    
        result = ag(from_wp, to_wp, map)
        end = time.time()
        log(["ag",obstacles_qty, distance, uav.battery, round((end-start),3), result[1], result[5]])

    elif selected_planner == "rrt":
        rospy.loginfo("Using rrt")
        start = time.time() 
        result = rrt(from_wp, to_wp, map)
        end = time.time()
        log(["RRT",obstacles_qty, distance, uav.battery, round((end-start),3), result[1], result[5]])

    elif selected_planner == "pfp":
        rospy.loginfo("Using pfp")
        start = time.time() 
        result = pfp(from_wp, to_wp, map)
        end = time.time()
        log(["pfp",obstacles_qty, distance, uav.battery, round((end-start),3), result[1], result[5]])


    else:
        rospy.logerr("ERROR!!! Select or choose a different algorithm")
        return None

    if result is None:
        return None

    (route, _, feasibility_res, _, _, _) = result

    # Fica em loop chamando os diferentes planners ate encontrar uma rota "feasible"

    # sem aspas mesmo se quiser, reduzir para somente uma das opcoes, pra forçar algum planner
    # especifico, mas acho que assim, rotacionando tem mais chance de encontrar
    # uma rota em menos iterações

    backup_planners = [rrt, ag, pfp]
    i = 0
    while feasibility_res != 'feasible':
        start = time.time() 
        result = backup_planners[i % 3](from_wp, to_wp, map)
        (route, _, feasibility_res, _, _, _) = result
        end = time.time()
        log([str(backup_planners[i % 3]),obstacles_qty, distance, uav.battery, round((end-start),3), result[1], result[5]])
        i += 1

    return route

class PathPlanningOp():
    PLAN_PATH     = 0
    PULVERIZE     = 1
    TAKE_PICTURE  = 2
    PLAN_PATH_RRT = 3

def path_planning(data):
    """
    Perform path planning operations based on the specified data.

    Args:
        data (PathPlanningData): The data object containing the operation and related information.

    Returns:
        PathPlanningResponse or None: A response object with the generated route or None if no feasible path is found.

    This function performs various path planning operations based on the given `data` object. The operation is determined by the `op` field within the data object.

    - If the operation is `PLAN_PATH`, path planning is executed using the `run_path_planning` function. The number of obstacles is estimated using the `count_obstacles` function. The generated route is returned as a `PathPlanningResponse` object.

    - If the operation is `PULVERIZE`, the `pulverize` function is called to generate a route. The resulting route is returned as a `PathPlanningResponse` object.

    - If the operation is `TAKE_PICTURE`, a route for capturing a picture is generated using the `picture` function. The resulting route is returned as a `PathPlanningResponse` object.

    - If the operation is `PLAN_PATH_RRT`, path planning is executed using the `rrt` function, and the generated route is returned as a `PathPlanningResponse` object.

    If a feasible route is found, it is returned as a `PathPlanningResponse` object. If no feasible path is found, None is returned.

    Note: This function handles various path planning scenarios based on the specified operation in the `data` object.
    """
    route = WaypointList()

    # Organize inputs
    from_wp = data.r_from
    to_wp = data.r_to
    map = data.map

    if data.op == PathPlanningOp.PLAN_PATH:
        obstacles_qty = count_obstacles(from_wp, to_wp, map, map.geo_home)
        # print(obstacles_qty)
        # OBSTACLES
        # mapa_obstacles = pd.read_csv(f"{CSV_PATH}/mapa{data.map.id}_obstacles.csv", index_col=0)
        # if data.name_from == "aux":
        #     obstacles_qty = 0  
        # else:
        #     obstacles_qty = mapa_obstacles[data.name_from][data.name_to]

        route = run_path_planning(from_wp, to_wp, map, obstacles_qty)

    elif data.op == PathPlanningOp.PULVERIZE:
        route = pulverize(from_wp)

    elif data.op == PathPlanningOp.TAKE_PICTURE:
        distance = 10
        route = picture(from_wp, distance)

    elif data.op == PathPlanningOp.PLAN_PATH_RRT:
        result = rrt(from_wp, to_wp, map)
        (route, _, _, _, _, _) = result

    if route is None:
        # This will cause a `ServiceError` in rospy which is correct, since we
        # couldn't do the action.
        return None
    else:
        return PathPlanningResponse(route)

def path_planning_server():
    """
    Initialize and run the path planning service.

    This function initializes the ROS node for the path planning service and configures a global variable for the K-nearest neighbors (KNN) classifier. The KNN model is loaded from a pickle file, allowing it to be used across multiple service requests. The service listens for incoming path planning requests and delegates them to the `path_planning` function. Once the service is ready, it enters the ROS service loop.

    Note: The service runs indefinitely to accept path planning requests.

    """
    rospy.init_node("path_planning_server")

    harpia_root_dir = get_harpia_root_dir()

    # Configure a global variable for the KNN classifier. This may not be the best way of
    # doing it, but it loads the model only once and is able to use ROS parameters.
    global KNN

    knn_pickle_file = os.path.join(harpia_root_dir, "src/path_planning/scripts/libs/KNN/models/knn5w.pkl")

    # Load the model from disk a single time
    KNN = pickle.load(open(knn_pickle_file, "rb"))

    global CSV_PATH

    CSV_PATH = os.path.join(harpia_root_dir, "csv")

    srv = rospy.Service('harpia/path_planning', PathPlanning, path_planning)

    rospy.loginfo("Path Planning Service Ready")
    srv.spin()

if __name__ == "__main__":
    path_planning_server()
