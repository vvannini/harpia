import sys
import rospy
import math
import json
import time

PATH = '/home/vannini/drone_arch/Data/' 


def get_region(mapa_json, name):
	longi = 0
	lat =0
	for step in mapa_json["regions"]:
		if(step["name"] == name):
			lat = step["center"][1]
			longi = step["center"][0]
	return lat, longi



def get_base(mapa_json, name):
	longi = 0
	lat =0
	for base in mapa_json["bases"]:
		if(base["name"] == name):
			lat = base["center"][1]
			longi = base["center"][0]
	return lat, longi


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
if(name.find('base') != -1):
	lat, longi = get_base(mapa, name)
else:
	lat, longi = get_region(mapa, name)


print(lat)
print(longi)