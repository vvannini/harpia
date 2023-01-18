#!/usr/bin/env python3
#@authors Pedro Natali and Veronica Vannini

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
import os
import json

from std_msgs.msg import String



# Classes

class Plan(object):
    def __init__(self):
        self.sub  = rospy.Subscriber("rosplan_parsing_interface/complete_plan", CompletePlan, self.plan_callback)   
        self.plan = CompletePlan()
    
    def plan_callback(self, data): 
        self.plan = data
        self.unsubscribe()

    def unsubscribe(self):
        self.sub.unregister()

class Drone(object):
    def __init__(self):
        self.sub = rospy.Subscriber('mavros/global_position/global',  NavSatFix, self.global_position_callback)  
        self.sub1 = rospy.Subscriber('mavros/battery',  BatteryState, self.battery_state_callback)  
        self.latitude = float("inf")
        self.longitude = float("inf")
        self.battery =  float("inf")
    
    def global_position_callback(self, data): 
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.unsubscribe()

    def battery_state_callback(self, data): 
        self.battery = data.percentage * 100
        # self.unsubscribe()

    def unsubscribe(self):
        self.sub.unregister()

# Classe responsável pela ações do sistema.


class Region:
    def __init__(self, idi, name, geo_points, cart_points, geo_center, cart_center):
        self.idi = idi
        self.name = name
        self.geo_points = geo_points
        self.points = cart_points
        self.geo_center = geo_center
        self.cart_center = cart_center


class CartesianPoint:
    def __init__(self, x, y, z=0):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"[{self.x}, {self.y}, {self.z}]"


class GeoPoint:
    def __init__(self, latitude, longitude, altitude):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

def get_harpia_root_dir():
    return rospy.get_param("/harpia_home", default=os.path.expanduser("~/harpia"))

#Calcula a probabilidade de chegar em uma regiao X dado que a bateria restante nela é Y (lembrando que a bateria é inconsistente abaixo de 15%)
#Esse datacu eu tirei das vozes da minha cabeca depois de cheirar lolo???

def dist_prob(bateria):
    if(5 <= bateria and bateria < 15):
        return 0.20
    elif(15 <= bateria and bateria < 30):
        return 0.50
    elif(30 <= bateria and bateria < 60):
        return 0.75
    elif(60 <= bateria and bateria < 80):
        return 0.85
    elif(80 <= bateria and bateria < 100):
        return 0.95
    else:
        return 0

#Rede Bayesiana definida no sistema que utiliza:
## Bateria inicial, consumo definido à partir da bateria,
## ArrayActions (vetor de objeto da classe ação) e as probabilidades definidas
def calc_probabilities(bateria_init, consumo, ArrayActions, prob) :
    bateria = []
    bateria.append(bateria_init)

    # Indica se é a primeira região ou não
    FLAG = 0
    print("\n")
    print("---------------------- PROBABILITY INFO --------------------------")
    print("\n")
    
    # Para cada regiao
    for obj in ArrayActions:
        if(obj.name == "recharge_battery"): #or obj.name == "recharge_input"):
            bateria.append(100)
            print("Bateria recarregada")
            prob.append(1)
            print()
        else:
            bateria.append(bateria[len(bateria)-1] - consumo*float(obj.duration))
            prob.append(dist_prob(bateria[len(bateria) - 1]))
            print("Probabilidade de " + get_DecisionString(obj) + " : " + str(prob[len(prob)-1]))
            print("bateria restante = " + str(bateria[len(bateria)-1]))
            print()
    return prob

#Essa funcao determina se a ação acontece ou não, se seu valor for 0.1, significa que ações com 0.1 de probabilidade iriam contar como (SIM)
def value(valor, action):
    if(action == 'go_to_base' or action == 'clean_camera' or action == 'recharge_input'):
        if(valor >= 0.2):
            return 1
        else:
            return 0
    else:
        if(valor >= 0.5):
            return 1;
        else:
            return 0;

def get_DecisionString(obj):
    if "go_to" in obj.name:
        return  'go_to_'+ str(obj.parameters[1].value)
    else:
        return  obj.name +'_'+ str(obj.parameters[0].value)

def log(replan, bn):
    harpia_root_dir = get_harpia_root_dir()

    # log path
    log_json = os.path.join(harpia_root_dir, "results/mission_log.json")
    log_net = os.path.join(harpia_root_dir, "results/net/")

    # open log
    with open(log_json, "r") as log_file:
        log_file = json.load(log_file)

    # add replan
    log_file[-1]['replan'] = log_file[-1]['replan'] + replan

    i = 0
    while os.path.exists(log_net+"%s.net" % i):
        i += 1
    # gum.saveBN(bn, log_net+"%s.net" % i)
    # with open("WaterSprinkler.net","r") as out:
    #   print(out.read())

    log_file[-1]['bn_net'].append(i)
    # dump log
    with open(log_json, 'w') as outfile:
        json.dump(log_file, outfile, indent=4)
    outfile.close()

def fault_detection(req):
    p = Plan()
    uav = Drone()
    while p.plan.plan ==  []:
        print("Waiting for Complete Plan...")
        rospy.sleep(1)

    ArrayActions = p.plan.plan[req.action_id:]

    print("\n")
    print("------------------------- PATH INFO ------------------------------")
    for obj in ArrayActions[:-1]:
        print(get_DecisionString(obj), obj.duration,  sep = " ")

    while uav.battery ==  float("inf"):
            print("Waiting for UAV battery...")
            rospy.sleep(1)

    print("\n")
    print("----------------------- HARDWARE INFO ----------------------------")
    #define bateria e consumo inicial
    bateria_init = uav.battery
    consumo = req.uav.battery.discharge_rate 
    recarga = req.uav.battery.recharge_rate   
    print("Discharge rate battery = " + str(consumo))
    print("Battery ammount = " + str(bateria_init)) 

    #define os conjuntos de modelos e probabilidades
    prob = []
    model = []

    #flag do sistema -> 0 se não precisa replan
    FLAG = 0

    #Cria um contador para as regioes / separa região inicial do resto
    count = 0

    #Pega os valores das probabilidades definidas na rede
                         #  current batt, uav discharge rate, plan, prob
    prob = calc_probabilities(bateria_init, consumo, ArrayActions[:-1], prob)

    dict_evidences = {}

    final_prob = 1

    print("\n")
    print("---------------------- PREDICTION INFO ---------------------------")

    relaxed_action = ['go_to_base', 'clean_camera', 'recharge_input']

    for p, obj in zip(prob, ArrayActions[:-1]):
        
        if (((obj.name in relaxed_action) and p >= 0.2) or (p >= 0.5)):
            print('--------------------- Atualmente em '+str(count+1)+' ---------------------')
            print('Probabilidade: '+str(p))
            print('Probabilidade Final: '+str(final_prob))
            # print(obj)
        else:
            print("\n------------------------- REPLANNING -----------------------------\n")
            print('Probabilidade: '+str(p))
            print('Probabilidade Final: '+str(final_prob))
            # print(obj)
            # print(obj in relaxed_action)
            FLAG = 1
            return FaultDetectionResponse(FLAG)


        count += 1

    # gnb.showInference(bn, evs={})
    # gum.saveBN(bn,"WaterSprinkler.net")
    # with open("WaterSprinkler.net","r") as out:
    #   print(out.read())
    # log(0, bn)
    return FaultDetectionResponse(FLAG)

def fault_detection_server():
    rospy.init_node('fault_detection_server')
    s = rospy.Service('harpia/fault_detection', FaultDetection, fault_detection)
    print("Fault Detecion Ready")
    rospy.spin()

if __name__ == '__main__':
    fault_detection_server()

