"""

Potential Field based path planner

author: Atsushi Sakai (@Atsushi_twi)

Ref:
https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf

"""
import sys
import os
import math
nb_dir = os.getcwd()
if nb_dir not in sys.path:
    sys.path.append(nb_dir)
# print(sys.path)

from collections import deque
import numpy as np
import matplotlib.pyplot as plt

from libs.AG.utils import euclidean_distance
from libs.ChanceConstraint.chance_constraint import chance_constraint, CartesianPoint


# Potential Field
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 3

# Chance Constraint
# The GPS imprecision
UNCERTAINTY = 5


def get_XY(area, loop=False):
    X = []
    Y = []
    for p in area:
        X.append(p.x)
        Y.append(p.y)

    if loop:
        X.append(X[0])
        Y.append(Y[0])

    return X, Y


def calc_attractive_potential(point, destination, max_dist):
    # Closer to the objective, higher the potential
    return euclidean_distance(point, destination) / max_dist


def calc_repulsive_potential(point, obstacles):
    # Considers the chance constraint over all obstacles as the repulsive potential
    gps_imprecision = 5
    rp = 0
    for obstacle in obstacles:
        rp += chance_constraint(point, obstacle, UNCERTAINTY)
    return rp * 2


def get_minimax(origin, destination):

    x_min = min(origin.x, destination.x)
    x_max = max(origin.x, destination.x)
    y_min = min(origin.y, destination.y)
    y_max = max(origin.y, destination.y)

    x_dif = x_max - x_min
    x_max += x_dif
    x_min -= x_dif

    y_dif = y_max - y_min
    y_max += y_dif
    y_min -= y_dif

    return x_min, x_max, y_min, y_max


def calc_potential_field(origin, destination, obstacles, discretization=10):

    max_dist = euclidean_distance(origin, destination)

    x_min, x_max, y_min, y_max = get_minimax(origin, destination)

    xw = int(math.ceil((x_max - x_min) / discretization))
    yw = int(math.ceil((y_max - y_min) / discretization))

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * discretization + x_min

        for iy in range(yw):
            y = iy * discretization + y_min
            ug = calc_attractive_potential(CartesianPoint(x, y), destination, max_dist)
            uo = calc_repulsive_potential(CartesianPoint(x, y), obstacles)
            uf = ug + uo
            pmap[ix][iy] = uf

    # --- Visualization
    # Setting up input values
    x = np.arange(x_min, x_max, discretization)
    y = np.arange(y_min, y_max, discretization)
    X, Y = np.meshgrid(x, y)

    vis_pmap = np.swapaxes(np.array(pmap), 0, 1)

    im = plt.pcolormesh(X, Y, vis_pmap, shading='nearest', cmap=plt.cm.Purples)
    # im = plt.imshow(potential_map, cmap=plt.cm.Purples, extent=(-2, 11, -2, 11), interpolation='bilinear', origin='lower')
    plt.colorbar(im)
    plt.axis('scaled')
    # Plot the obstacles limits
    for obs in obstacles:
        X_obs,Y_obs = get_XY(obs, loop=True)
        plt.plot(X_obs,Y_obs, color='green')
    plt.title('Chance Constraint over a No-Fly Zone')
    plt.savefig('out_potentialfield.png')
    # ---

    return pmap, x_min, y_min


def get_motion_model():
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion


def oscillations_detection(previous_ids, ix, iy):
    previous_ids.append((ix, iy))

    if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
        previous_ids.popleft()

    # check if contains any duplicates by copying into a set
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False


def potential_field_planning(origin, destination, obstacles, resolution):
    print("Starting Potential Field Planning")

    # Calculate potential field
    # pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, resolution, rr, sx, sy)
    pmap, minx, miny = calc_potential_field(origin, destination, obstacles, discretization=resolution)

    print('    Generated potential field')
    # print(f"pmap.shape={pmap.shape}")
    # print(f"len(pmap)={len(pmap)}")
    # print(f"len(pmap[0])={len(pmap[0])}")
    # print(f'pmap={pmap}')
    sx = origin.x
    sy = origin.y
    gx = destination.x
    gy = destination.y

    # search path
    # d = np.hypot(sx - gx, sy - gy)
    d = euclidean_distance(origin, destination)
    ix = round((sx - minx) / resolution)
    iy = round((sy - miny) / resolution)
    gix = round((gx - minx) / resolution)
    giy = round((gy - miny) / resolution)

    rx, ry = [sx], [sy]
    motion = get_motion_model()
    previous_ids = deque()

    while d >= resolution:
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                p = float("inf")  # outside area
                # print("outside potential!")
            else:
                p = pmap[inx][iny]
                # print(f'else: p={p}, pmap[inx][iny] inx={inx} iny={iny}')
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
                # print(f'if minp>p: minp={minp} minix={inx} miniy={iny}')
        ix = minix
        iy = miniy
        xp = ix * resolution + minx
        yp = iy * resolution + miny
        # d = np.hypot(gx - xp, gy - yp)
        d = euclidean_distance(CartesianPoint(xp, yp), destination)
        # print(f'd={d}')
        rx.append(xp)
        ry.append(yp)
        # print(f'xp yp = {xp} {yp}')

        if (oscillations_detection(previous_ids, ix, iy)):
            print("Oscillation detected at ({},{})!".format(ix, iy))
            break

    # print("Goal!!")

    waypoints = convert_output_to_wp(rx, ry)

    return waypoints


def convert_output_to_wp(rx, ry):
    waypoints = []
    for x, y in zip(rx, ry):
        wp = [x, y]
        waypoints.append(wp)

    return waypoints


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)
