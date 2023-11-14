#!/usr/bin/env python3
#@authors Veronica Vannini

# Import libraries
import pyAgrum as gum
import sys
import rospy
import math
import rosnode, psutil
from std_srvs.srv import Empty

from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import *
from rosplan_dispatch_msgs.srv import *

from harpia_msgs.msg import *
from harpia_msgs.srv import *

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import NavSatFix, Imu, BatteryState

from pylab import *
import matplotlib.pyplot as plt
import os
import json

from std_msgs.msg import String



# Classes

class Plan(object):
    def __init__(self):
        """
        Initializes an instance of the Plan class.

        This class is designed to handle Harpia/ROSPlan plan data by subscribing to a specific topic and providing
        a callback function for processing the received data.

        Attributes:
            sub (rospy.Subscriber): ROS subscriber for the "rosplan_parsing_interface/complete_plan" topic.
            plan (CompletePlan): Instance of the CompletePlan class to store the received plan data.
        """
        self.sub = rospy.Subscriber("rosplan_parsing_interface/complete_plan", CompletePlan, self.plan_callback)
        self.plan = CompletePlan()

    def plan_callback(self, data):
        """
        Callback function for processing the received plan data.

        Args:
            data (CompletePlan): Received plan data of type CompletePlan.
        """
        self.plan = data
        self.unsubscribe()

    def unsubscribe(self):
        """
        Unsubscribes the ROS subscriber to stop receiving plan data.

        This method is called after processing the plan data to ensure that the subscriber is
        deactivated and stops listening to the topic.
        """
        self.sub.unregister()


class oMission(object):
    def __init__(self):
        """
        Initializes an instance of the oMission class.

        This class is designed to handle ROS mission data by subscribing to a specific topic and providing
        a callback function for processing the received data. It also contains methods for calculating distances
        between regions in the mission map.

        Attributes:
            sub (rospy.Subscriber): ROS subscriber for the "harpia/mission" topic.
            map (Mission): Instance of the Mission class to store the received mission data.
        """
        self.sub = rospy.Subscriber("harpia/mission", Mission, self.mission_callback)
        self.map = None

    def mission_callback(self, data):
        """
        Callback function for processing the received mission data.

        Args:
            data (Mission): Received mission data of type Mission.
        """
        self.map = data.map
        self.unsubscribe()

    def calculate_distances(self):
        """
        Calculates distances between regions in the mission map.

        Returns:
            distances (dict): A dictionary containing distances between different regions in the mission map.
        """
        distances = {}
        while self.map is None:
            print("Waiting for Map...")
            rospy.sleep(1)

        all_regions = self.map.bases + self.map.roi

        for region1 in all_regions:
            region1_name = region1.name
            region1_center = region1.center.cartesian

            if region1_name not in distances:
                distances[region1_name] = {}

            for region2 in all_regions:
                region2_name = region2.name
                region2_center = region2.center.cartesian

                if region1_name == region2_name:
                    continue  # Skip calculating distance with itself

                distance = self.euclidean_distance(region1_center, region2_center)
                distances[region1_name][region2_name] = distance

        return distances

    @staticmethod
    def euclidean_distance(point1, point2):
        """
        Calculates the Euclidean distance between two points.

        Args:
            point1 (Point): The first point with x and y coordinates.
            point2 (Point): The second point with x and y coordinates.

        Returns:
            distance (float): The Euclidean distance between the two points.
        """
        return math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2)

    def unsubscribe(self):
        """
        Unsubscribes the ROS subscriber to stop receiving mission data.

        This method is called after processing the mission data in the mission_callback method to ensure that
        the subscriber is deactivated and stops listening to the topic.
        """
        self.sub.unregister()


class Drone(object):
    def __init__(self):
        """
        Initializes an instance of the Drone class.

        This class is designed to handle ROS data related to the drone, such as global position and battery state,
        by subscribing to specific topics and providing callback functions for processing the received data.

        Attributes:
            sub (rospy.Subscriber): ROS subscriber for the "mavros/global_position/global" topic.
            sub1 (rospy.Subscriber): ROS subscriber for the "mavros/battery" topic.
            latitude (float): Current latitude of the drone.
            longitude (float): Current longitude of the drone.
            battery (float): Current battery percentage of the drone.
        """
        self.sub = rospy.Subscriber('mavros/global_position/global',  NavSatFix, self.global_position_callback)  
        self.sub1 = rospy.Subscriber('mavros/battery',  BatteryState, self.battery_state_callback)  
        self.latitude = float("inf")
        self.longitude = float("inf")
        self.battery =  float("inf")
    
    def global_position_callback(self, data): 
        """
        Callback function for processing the received global position data.

        Args:
            data (NavSatFix): Received global position data of type NavSatFix.
        """
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.unsubscribe()

    def battery_state_callback(self, data): 
        """
        Callback function for processing the received battery state data.

        Args:
            data (BatteryState): Received battery state data of type BatteryState.
        """
        self.battery = data.percentage * 100
        # self.unsubscribe()

    def unsubscribe(self):
        """
        Unsubscribes the ROS subscribers to stop receiving data.

        This method is called after processing the data in the callback methods to ensure that
        the subscribers are deactivated and stop listening to the topics.
        """
        self.sub.unregister()
        # self.sub1.unregister()

# Classe responsável pela ações do sistema.


class Region:
    def __init__(self, idi, name, geo_points, cart_points, geo_center, cart_center):
        """
        Initializes an instance of the Region class.

        This class represents a region in the mission map, containing information such as region ID, name,
        geographical points, Cartesian points, geographical center, and Cartesian center.

        Args:
            idi (int): Region ID.
            name (str): Region name.
            geo_points (List[GeoPoint]): List of geographical points defining the region.
            cart_points (List[Point]): List of Cartesian points defining the region.
            geo_center (GeoPoint): Geographical center of the region.
            cart_center (Point): Cartesian center of the region.
        """
        self.idi = idi
        self.name = name
        self.geo_points = geo_points
        self.points = cart_points
        self.geo_center = geo_center
        self.cart_center = cart_center

class CartesianPoint:
    def __init__(self, x, y, z=0):
        """
        Initializes an instance of the CartesianPoint class.

        This class represents a point in Cartesian coordinates, consisting of x, y, and optional z coordinates.

        Args:
            x (float): X-coordinate.
            y (float): Y-coordinate.
            z (float, optional): Z-coordinate (default is 0).
        """
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        """
        Returns a string representation of the CartesianPoint.

        Returns:
            str: String representation of the CartesianPoint.
        """
        return f"[{self.x}, {self.y}, {self.z}]"


class GeoPoint:
    def __init__(self, latitude, longitude, altitude):
        """
        Initializes an instance of the GeoPoint class.

        This class represents a geographical point with latitude, longitude, and altitude.

        Args:
            latitude (float): Latitude coordinate.
            longitude (float): Longitude coordinate.
            altitude (float): Altitude coordinate.
        """
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude


def dist_prob(battery):
    """
    Calculates the probability of reaching a region given the remaining battery.

    Args:
        battery (float): Remaining battery level.

    Returns:
        float: Probability of reaching a region based on the given battery level.
    """
    if 5 <= battery < 15:
        return 0.20
    elif 15 <= battery < 30:
        return 0.50
    elif 30 <= battery < 60:
        return 0.75
    elif 60 <= battery < 80:
        return 0.85
    elif 80 <= battery < 100:
        return 0.95
    else:
        return 0


def calc_probabilities(battery_init, consumption, ArrayActions, prob, distances):
    """
    Calculate probabilities based on Bayesian Network.

    Args:
        battery_init (float): Initial battery level.
        consumption (float): Battery consumption rate per second.
        ArrayActions (list): List of Action objects representing the sequence of actions.
        prob (list): List of calculated probabilities.
        distances (dict): Dictionary containing distances between regions.

    Returns:
        list: List of probabilities for each action.
    """
    battery = [battery_init]

    # Indicates whether it is the first region or not
    FLAG = 0
    print("\n")
    print("---------------------- PROBABILITY INFO --------------------------")
    print("\n")

    # For each region
    for obj in ArrayActions[:-1]:
        # If the region is for recharging
        if obj.name == "recharge_battery" or obj.name == "recharge_input":
            battery.append(100)
            print("Recharged Battery")
            prob.append(1)
            print()
        else:
            # Calculate the probability and remaining battery to reach the next region
            if obj.name == "go_to":
                r_from = obj.parameters[0].value 
                r_to   = obj.parameters[1].value

                d = distances[r_from][r_to]
                
                battery.append(battery[-1] - consumption * (d / 5))
                prob.append(dist_prob(battery[-1]))
            else:
                battery.append(battery[-1] - consumption * (50 / 5))
                prob.append(dist_prob(battery[-1]))
            print("Probability" + get_DecisionString(obj) + " : " + str(prob[-1]))
            print("Battery remain = " + str(battery[-1]))
            print()
    return prob


def value(probability, action):
    """
    Determine if the action occurs or not based on the given probability.

    Args:
        probability (float): Probability value.
        action (str): Name of the action.

    Returns:
        int: 1 if the action occurs, 0 otherwise.
    """
    threshold = 0.2 if action in ['go_to_base', 'clean_camera', 'recharge_input'] else 0.5

    if probability >= threshold:
        return 1
    else:
        return 0


def get_decision_string(obj):
    """
    Generate a decision string based on the given action object.

    Args:
        obj: Action object containing information about the action.

    Returns:
        str: Decision string based on the action.
    """
    if "go_to" in obj.name:
        return 'go_to_' + str(obj.parameters[1].value)
    else:
        return obj.name + '_' + str(obj.parameters[0].value)


def log(replan, bn):
    """
    Log information about the mission, including the replan status and Bayesian Network (BN).

    Args:
        replan (int): Replan status (1 for true, 0 for false).
        bn (Bayesian Network): The Bayesian Network to be logged.
    """
    # Log path
    LOG_PATH = "~/harpia/results/"
    LOG_PATH = os.path.expanduser(LOG_PATH)
    log_json = LOG_PATH + "mission_log.json"
    log_net = LOG_PATH + "net/"

    # Open log
    with open(log_json, "r") as log_file:
        log_data = json.load(log_file)

    # Add replan
    log_data[-1]['replan'] = log_data[-1]['replan'] + replan

    i = 0
    while os.path.exists(log_net + f"{i}.net"):
        i += 1

    # Close any open file streams before saving BN
    bn_file_path = log_net + f"{i}.net"
    with open(bn_file_path, "w") as bn_file:
        gum.saveBN(bn, bn_file_path)

    log_data[-1]['bn_net'].append(i)
    
    # Dump log
    with open(log_json, 'w') as outfile:
        json.dump(log_data, outfile, indent=4)


def mission_fault_mitigation(req):
    """
    Mitigate faults during a mission.

    Args:
        req: Request object containing information about the mission and UAV.

    Returns:
        MissionFaultMitigationResponse: Response object indicating the success of fault mitigation.
    """
    # Starting objects
    p = Plan()
    uav = Drone()
    mission = oMission()
    distances = mission.calculate_distances()

    # Waiting for plan
    while p.plan.plan ==  []:
        print("Waiting for Complete Plan...")
        rospy.sleep(1)

    ArrayActions = p.plan.plan[req.action_id:]

    # Terminal log about the plan
    print("\n")
    print("------------------------- PATH INFO ------------------------------")
    for obj in ArrayActions[:-1]:
        print(get_DecisionString(obj), obj.duration,  sep = " ")

    while uav.battery ==  float("inf"):
            print("Waiting for UAV battery...")
            rospy.sleep(1)

    # Terminal log about the hardware
    print("\n")
    print("----------------------- HARDWARE INFO ----------------------------")

    bateria_init = uav.battery
    consumo = req.uav.battery.discharge_rate    
    recarga = req.uav.battery.recharge_rate  

    print("Discharge rate battery = " + str(consumo))
    print("Battery ammount = " + str(bateria_init)) 

    # creating BN
    bn = gum.BayesNet('FaultDetection')
    print(bn)

    #Define models and probability
    prob = []
    model = []
    # lista de inferencia
    # ie = []

    #Replan flag - 0: dont need replan
    FLAG = 0

    #Count for region / segregate first region for the rest
    count = 0

    # Get prob
    prob = calc_probabilities(bateria_init, consumo, ArrayActions, prob, distances)

    dict_evidences = {}

    print("\n")
    print("---------------------- PREDICTION INFO ---------------------------")

    relaxed_action = ['go_to_base', 'clean_camera', 'recharge_input']

    # Running BN 
    for obj in ArrayActions[:-1]:
        # Creating actions
        # StringA = 'actions_' + str(count)
        StringDecisao = str(count)+"_"+get_DecisionString(obj)
        # action = gum.LabelizedVariable(StringA,'.',2)
        action = gum.LabelizedVariable(StringDecisao,'.',2)
        action.changeLabel(0,"Replan") # false
        action.changeLabel(1, "Do_Action") # true
        node = bn.add(action)
        model.append(action)
        if(count):
            bn.addArc(node-1, node)
            bn.cpt(node)[:]=[ [1,.0], [abs(1 - prob[count]),abs(prob[count])]]
        else:
            bn.cpt(node).fillWith([abs(1 - prob[count]),abs(prob[count])])

        ie=gum.LazyPropagation(bn)
        # ie = gum.InfluenceDiagramInference(model[count])

        ie.setEvidence(dict_evidences)

        # Making inference
        ie.makeInference()
        if(count):
            inf = ie.posterior(node)
            ie_list = inf.tolist()
            # print(ie.H(count))
            # if(ie.H(count) < 0.15):
            # if(ie_list[0] == 1):

            # Check if the mission can continue
            if((obj.name in relaxed_action and ie_list[1] >= .2) or ie_list[1] >= ie_list[0]):
                print()
                print('--------------------- Action '+str(count+1)+' ---------------------')
                print('Probabilidade: '+str(ie_list[1]))
                dict_evidences[StringDecisao] = 1 #value(prob[count], obj.name)
            else:
                print("\n------------------------- REPLANNING -----------------------------\n")
                FLAG = 1
                log(1, bn)
                return MissionFaultMitigationResponse(FLAG)
        else:
            ie.eraseAllEvidence()


        count += 1

        
    # gnb.showInference(bn, evs={})
    # gum.saveBN(bn,"WaterSprinkler.net")
    # with open("WaterSprinkler.net","r") as out:
    #   print(out.read())
    log(0, bn) 
    return MissionFaultMitigationResponse(FLAG)

def mission_fault_mitigation_server():
    """
    Initialize and run the mission fault mitigation server.

    This function initializes a ROS node, sets up a service for mission fault mitigation,
    and runs a loop while waiting for requests.

    Returns:
        None
    """
    rospy.init_node('mission_fault_mitigation_server')
    s = rospy.Service('harpia/mission_fault_mitigation', MissionFaultMitigation, mission_fault_mitigation)
    rospy.loginfo("Mission Fault Mitigation Ready")
    
    while not rospy.is_shutdown():
        continue


if __name__ == '__main__':
    mission_fault_mitigation_server()
    