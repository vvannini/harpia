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
    def __init__(self):
        self.sub_battery = rospy.Subscriber('mavros/battery',  BatteryState, self.battery_state_callback)
        self.battery     = None

    def battery_state_callback(self, data): 
        self.battery = data.percentage * 100

# ---
# UTILS
def log(index):
    harpia_root_dir = get_harpia_root_dir()

    # log path
    log_path = os.path.join(harpia_root_dir, "results")
    log_json = os.path.join(log_path, "mission_log.json")

    # open log
    with open(log_json, "r") as log_file:
        log_file = json.load(log_file)

    # add replan
    log_file[-1]['knn'][index] += 1

    # dump log
    with open(log_json, 'w') as outfile:
        json.dump(log_file, outfile, indent=4)

def euclidean_distance(A, B):
    return math.sqrt((B.x - A.x) ** 2 + (B.y - A.y) ** 2)

def calc_dist_path(path):
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
    rate = rospy.Rate(rate)

    while not check():
        if rospy.is_shutdown(): return False
        if msg is not None: rospy.loginfo(msg)
        rate.sleep()

    return True

def get_harpia_root_dir():
    return rospy.get_param("/harpia_home", default=os.path.expanduser("~/harpia"))

def to_waypointList(route, geo_home):
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
    if fitness_trace[1] > 0:  # Hit obstacle (fit_obs) (max 1 hit)
        return "infeasible"
    elif fitness_trace[0] > 10:  # Away from destination wp (fit_d) (max 10 meters)
        return "verify"
    else:
        return "feasible"

def feasibility(route, obstacles, destination):
    """Checks if the route is feasible

    Args:
        route ([type]): [description]
        obstacles ([type]): [description]
        destination ([type]): [description]

    Returns:
        string: 'infeasible' - the route hits an obstacle
                'verify'     - the route does not arrive at the destination, according to min precision
                'feasible'   - the route is alright
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
    for subject in ag.history:
        feasibility_result = feasibility_ag(subject['fitness_trace'])

        if feasibility_result == "feasible":
            return subject['birth_time']

    return None

def pass_through_obstacle(obstacle_points, line_points):
    poly = shapely.geometry.Polygon(obstacle_points)
    line = shapely.geometry.LineString(line_points)
    
    return line.intersects(poly)

def count_obstacles(from_wp, to_wp, map, geo_home):
    obst_qty = 0
    line = Conversor.list_geo_to_cart([origin[:-1]] + [destination[:-1]], geo_home)

    for nfz in m['nfz']:
        poly = Conversor.list_geo_to_cart(nfz['geo_points'], geo_home)
        # poly1 = list_geo_to_cart(nfz['geo_points'], geo_home)
        if(pass_through_obstacle(poly, line)):
            obst_qty += 1

    return obst_qty
# ---
# PATH PLANNERS


def pfp(from_wp, to_wp, map, save_filename=None):
    origin = Conversor.geo_to_cart(from_wp.geo, map.geo_home)
    rospy.loginfo(f"origin={origin}")

    destination = Conversor.geo_to_cart(to_wp.geo, map.geo_home)
    rospy.loginfo(f"destination={destination}")

    # ---
    # obstacleList for check_collision_mode='ray_casting'
    obstacleList = [[Conversor.geo_to_cart(p.geo, map.geo_home) for p in area.points] for area in map.nfz]

    # Creates the ag_map in a structure the AG will understand
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
        # Draw final route
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

    origin = Conversor.geo_to_cart(from_wp.geo, map.geo_home)
    rospy.loginfo(f"origin={origin}")

    destination = Conversor.geo_to_cart(to_wp.geo, map.geo_home)
    rospy.loginfo(f"destination={destination}")

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

    dist_to_goal = euclidean_distance(origin, destination) * 1.2
    rospy.loginfo(f"dist_to_goal={dist_to_goal}")

    # Set Initial parameters
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

        # Draw final route
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

    # Convert map to ag_map
    # nfzs has this structure:
    # nfzs = [
    #     [CartesianPoint(), CartesianPoint(), CartesianPoint(), CartesianPoint()],
    #     ...
    # ]
    nfzs = [[point.cartesian for point in area.points] for area in map.nfz]

    origin = Conversor.geo_to_cart(from_wp.geo, map.geo_home)
    destination = Conversor.geo_to_cart(to_wp.geo, map.geo_home)

    # Creates the ag_map in a structure the AG will understand
    ag_map = Mapa(
        origin,
        destination,
        areas_n=nfzs,
        # inflation_rate gives a sense of security, expanding the limits of each obstacle.  Defaults to 0.1.
        inflation_rate=0.1,
    )

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

    best = ag.run(info=True)

    route = best.get_route()

    rospy.loginfo("Best route found:")

    # Check if route is feasible
    feasibility_res = feasibility_ag(best.fitness_trace)
    rospy.loginfo(f"Ag generated an <{feasibility_res}> route")

    distance_path = best.fitness_trace[5]  # (fit_dist)
    distance_to_objective = best.fitness_trace[0]  # (fit_d)

    first_factible_route_found_time = get_first_factible_route(ag)

    # Draw final route
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
    """Selects a path planner based on the information received

    Args:
        obstacles_qty (int): quantity of obstacles between the origin and the destination waypoint
        distance (float): the distance in a straight line between the origin and the destination waypoint

    Returns:
        selected_planner (str): the name of the selected planner, according to the intelligence
    """

    # return "rrt"

    global KNN

    # Predict the best planner
    return KNN.predict([[obstacles_qty, distance, battery]])

def run_path_planning(from_wp, to_wp, map, obstacles_qty):
    distance = euclidean_distance(from_wp.cartesian, to_wp.cartesian)

    uav = Drone()

    if not wait_until(lambda: uav.battery is not None, msg="Waiting for UAV battery..."):
        return None

    selected_planner = select_planner(obstacles_qty, distance, uav.battery)
    rospy.loginfo(f"selected_planner={selected_planner}")

    result = None

    if selected_planner == "ag":
        rospy.loginfo("Using ag")
        log(0)
        result = ag(from_wp, to_wp, map)

    elif selected_planner == "rrt":
        rospy.loginfo("Using rrt")
        log(1)
        result = rrt(from_wp, to_wp, map)

    elif selected_planner == "pfp":
        rospy.loginfo("Using pfp")
        log(2)
        result = pfp(from_wp, to_wp, map)

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
        result = backup_planners[i % 3](from_wp, to_wp, map)
        (route, _, feasibility_res, _, _, _) = result
        i += 1

    return route

class PathPlanningOp():
    PLAN_PATH     = 0
    PULVERIZE     = 1
    TAKE_PICTURE  = 2
    PLAN_PATH_RRT = 3

def path_planning(data):
    """
    Args:
        origin_point (RegionPoint): data.req.from
        destination_point (RegionPoint): data.req.to

    Returns:
        List[Waypoint]
    """

    route = WaypointList()

    # Organize inputs
    from_wp = data.r_from
    to_wp = data.r_to
    map = data.map

    if data.op == PathPlanningOp.PLAN_PATH:
        # OBSTACLES
        mapa_obstacles = pd.read_csv(f"{CSV_PATH}/mapa{data.map.id}_obstacles.csv", index_col=0)
        if data.name_from == "aux":
            obstacles_qty = 0  
        else:
            obstacles_qty = mapa_obstacles[data.name_from][data.name_to]

        route = run_path_planning(from_wp, to_wp, map, obstacles_qty)

    elif data.op == PathPlanningOp.PULVERIZE:
        route = pulverize(from_wp)

    elif data.op == PathPlanningOp.TAKE_PICTURE:
        distance = 5
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
    rospy.init_node("path_planning_server")

    harpia_root_dir = get_harpia_root_dir()

    # Configure a global variable for the KNN classifier. This may not be the best way of
    # doing it, but it loads the model only once and is able to use ROS parameters.
    global KNN

    knn_pickle_file = os.path.join(harpia_root_dir, "src/path_planning/scripts/libs/KNN/models/knn29.pickle")

    # Load the model from disk a single time
    KNN = pickle.load(open(knn_pickle_file, "rb"))

    global CSV_PATH

    CSV_PATH = os.path.join(harpia_root_dir, "csv")

    srv = rospy.Service('harpia/path_planning', PathPlanning, path_planning)

    rospy.loginfo("Path Planning Service Ready")
    srv.spin()

if __name__ == "__main__":
    path_planning_server()
