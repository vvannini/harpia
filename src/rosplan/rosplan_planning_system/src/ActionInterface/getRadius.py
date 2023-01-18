import sys
import rospy
import math
import json
import time
from math import radians, cos, sin, asin, sqrt

PATH = '/home/vannini/drone_arch/Data/' 

def haversine(lon1, lat1, lon2, lat2):

    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371
    return c * r

def get_radius(mapa_json, name):
	radius =0
	min_dist = 9999
	for region in mapa_json["regions"]:
		if(region["name"] == name):
			for geo1 in region["geo_points"]:
				for geo2 in region["geo_points"]:
					if(geo1[0] != geo2[0] and geo1[1] != geo2[1]):
						dist = haversine(geo1[0], geo1[1], geo2[0], geo2[1])
						if(dist < min_dist):
							min_dist = dist
			return min_dist*1000


mapa_filename = PATH + "mapa.json"
mission_filename = PATH + "missao.json"
with open(mission_filename, "r") as mission_file:
		mission_file = json.load(mission_file)
		mission = mission_file[0]

mapa_id = 0

with open(mapa_filename, "r") as mapa_file:
		mapa_file = json.load(mapa_file)
		mapa = mapa_file[mapa_id]


longi = 0
lat =0
name = sys.argv[1]

radius = get_radius(mapa, name)
# if(name.find('base') != -1):
# 	lat, longi = get_base(mapa, name)
# else:
# 	lat, longi = get_region(mapa, name)


print(int(radius))
