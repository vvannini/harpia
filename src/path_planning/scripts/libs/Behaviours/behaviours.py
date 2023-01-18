import math
import pyproj
from functools import partial
from shapely.geometry import Point, mapping
from shapely.ops import transform

from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import *
from rosplan_dispatch_msgs.srv import *

from harpia_msgs.msg import *
from mavros_msgs.msg import *
from harpia_msgs.srv import *


def pulverize(from_wp):
    point = Point(from_wp.geo.longitude, from_wp.geo.latitude)
    alt = 10
    raio = 50
    local_azimuthal_projection = f"+proj=aeqd +R=6371000 +units=m +lat_0={point.y} +lon_0={point.x}"

    wgs84_to_aeqd = partial(
        pyproj.transform,
        pyproj.Proj('+proj=longlat +datum=WGS84 +no_defs'),
        pyproj.Proj(local_azimuthal_projection),
    )

    aeqd_to_wgs84 = partial(
        pyproj.transform,
        pyproj.Proj(local_azimuthal_projection),
        pyproj.Proj('+proj=longlat +datum=WGS84 +no_defs'),
    )

    point_transformed = transform(wgs84_to_aeqd, point)

    buffer = point_transformed.buffer(raio)

    buffer_wgs84 = transform(aeqd_to_wgs84, buffer)

    # print(buffer_wgs84)
    coord = mapping(buffer_wgs84)
    # print(coord)

    # Create polygon from lists of points
    x = []
    y = []

    some_poly = buffer_wgs84
    # Extract the point values that define the perimeter of the polygon
    x, y = some_poly.exterior.coords.xy

    route = WaypointList()
    for i, j in zip(x, y):
        geo_wp = Waypoint()
        geo_wp.frame = 3
        geo_wp.command = 16
        if route.waypoints == []:
            geo_wp.is_current = True
        else:
            geo_wp.is_current = False
        geo_wp.autocontinue = True
        geo_wp.param1 = 0
        geo_wp.param2 = 0
        geo_wp.param3 = 0
        geo_wp.param4 = 0
        geo_wp.x_lat = j
        geo_wp.y_long = i
        geo_wp.z_alt = 10
        route.waypoints.append(geo_wp)

    return route


def picture(from_wp, distance):
    longitude = from_wp.geo.longitude
    latitude = from_wp.geo.latitude
    alt = 10
    # distance = 250

    theta = 135

    geo_route = WaypointList()

    for i in range(4):
        dx = distance* math.cos(math.radians(theta))  # theta measured clockwise from due east
        dy = distance* math.sin(math.radians(theta))  # dx, dy same units as R

        delta_longitude = dx / (111320 * math.cos(latitude))  # dx, dy in meters
        delta_latitude = dy / 110540  # result in degrees long/lat

        longitude = longitude + delta_longitude
        latitude = latitude + delta_latitude

        theta = theta + 90

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
        geo_wp.x_lat = latitude
        geo_wp.y_long = longitude
        geo_wp.z_alt = alt
        geo_route.waypoints.append(geo_wp)

    return geo_route
