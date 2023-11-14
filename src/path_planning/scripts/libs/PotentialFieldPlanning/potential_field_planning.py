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
    """
    Calculate the attractive potential between a point and a destination.

    Parameters:
    - `point`: Current point.
    - `destination`: Destination point.
    - `max_dist`: Maximum distance for normalization.

    Returns:
    - Attractive potential value.

    This function calculates the attractive potential between a given point (`point`) and a destination point (`destination`). The potential is higher when the point is closer to the destination.

    Parameters:
    - `point`: Current point for which the potential is calculated.
    - `destination`: The target destination point.
    - `max_dist`: Maximum distance for normalization.

    Returns:
    - Attractive potential value, normalized by the maximum distance.

    """
    # Closer to the objective, higher the potential
    return euclidean_distance(point, destination) / max_dist


def calc_repulsive_potential(point, obstacles):
    """
    Calculate the repulsive potential based on chance constraints over all obstacles.

    Parameters:
    - `point`: Current point.
    - `obstacles`: List of obstacles.

    Returns:
    - Repulsive potential value.

    This function calculates the repulsive potential based on chance constraints over all obstacles. The repulsive potential is influenced by the chance constraints evaluated for each obstacle.

    Parameters:
    - `point`: Current point for which the repulsive potential is calculated.
    - `obstacles`: List of obstacles in the environment.

    Returns:
    - Repulsive potential value.

    """
    # Considers the chance constraint over all obstacles as the repulsive potential
    gps_imprecision = 5
    rp = 0
    for obstacle in obstacles:
        rp += chance_constraint(point, obstacle, UNCERTAINTY)
    return rp * 2


def get_minimax(origin, destination):
    """
    Calculate the minimax bounding box for a given line segment.

    Parameters:
    - `origin`: Starting point of the line segment.
    - `destination`: Ending point of the line segment.

    Returns:
    - Tuple containing the coordinates of the minimax bounding box: (x_min, x_max, y_min, y_max).

    This function calculates the minimax bounding box for a given line segment defined by the origin and destination points. The minimax bounding box contains the minimal and maximal x and y coordinates of the line segment, ensuring that it completely covers the segment.

    Parameters:
    - `origin`: Starting point of the line segment.
    - `destination`: Ending point of the line segment.

    Returns:
    - Tuple containing the coordinates of the minimax bounding box: (x_min, x_max, y_min, y_max).
    """
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

    """
    Calculate the potential field for path planning using attractive and repulsive potentials.

    Parameters:
    - `origin`: Starting point of the path.
    - `destination`: Ending point of the path.
    - `obstacles`: List of obstacle areas represented by their boundaries.
    - `discretization`: Grid spacing for potential field calculation.

    Returns:
    - Tuple containing the potential field map, x-axis minimum, and y-axis minimum.

    This function calculates the potential field for path planning by combining attractive and repulsive potentials. It uses a discretized grid to represent the field.

    Parameters:
    - `origin`: Starting point of the path.
    - `destination`: Ending point of the path.
    - `obstacles`: List of obstacle areas represented by their boundaries.
    - `discretization`: Grid spacing for potential field calculation.

    Returns:
    - Tuple containing the potential field map, x-axis minimum, and y-axis minimum.

    Additional Visualization:
    - The function generates a visual representation of the potential field with obstacle boundaries.

    Example:
    ```
    potential_field, x_min, y_min = calc_potential_field(origin, destination, obstacles, discretization=10)
    ```
    """
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
     """
    Get a motion model for path planning.

    Returns:
    - List of possible motion directions represented as [dx, dy] pairs.

    This function returns a motion model, which is a list of possible motion directions. Each direction is represented as a pair [dx, dy], indicating the change in x and y coordinates.

    Example:
    ```
    motion_model = get_motion_model()
    ```
    """
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
    """
    Detect oscillations in path planning.

    Args:
    - previous_ids (deque): A deque containing previous indices (ix, iy) in the path.
    - ix (int): Current x-index in the path.
    - iy (int): Current y-index in the path.

    Returns:
    - bool: True if oscillations are detected, False otherwise.

    This function detects oscillations in path planning by keeping track of previous indices in a deque. It checks for duplicates in the deque, indicating a repetitive pattern in the path.

    Example:
    ```
    previous_indices = deque(maxlen=5)
    is_oscillating = oscillations_detection(previous_indices, current_x, current_y)
    ```
    """
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
    """
    Perform potential field path planning from origin to destination.

    Args:
    - origin (CartesianPoint): The starting point of the path.
    - destination (CartesianPoint): The destination point of the path.
    - obstacles (list): List of obstacle coordinates.
    - resolution (float): The resolution for potential field calculations.

    Returns:
    - WaypointList: The generated path as a list of waypoints.

    This function performs potential field path planning using the given origin, destination, obstacles, and resolution. It calculates the potential field, searches for a path, and returns the generated path as a list of waypoints.

    Example:
    ```
    start_point = CartesianPoint(0, 0)
    end_point = CartesianPoint(10, 10)
    obstacle_list = [(5, 5), (6, 6), (7, 7)]
    resolution = 1.0

    path = potential_field_planning(start_point, end_point, obstacle_list, resolution)
    ```
    """
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
     """
    Convert the output path coordinates to a list of waypoints.

    Args:
    - rx (list): List of x-coordinates of the path.
    - ry (list): List of y-coordinates of the path.

    Returns:
    - WaypointList: The generated path as a list of waypoints.

    This function takes lists of x and y coordinates of a path (`rx` and `ry`) and converts them into a list of waypoints (`WaypointList`). Each waypoint is represented as a list `[x, y]`.

    Example:
    ```
    x_coordinates = [0, 1, 2, 3]
    y_coordinates = [0, 1, 4, 9]

    waypoints = convert_output_to_wp(x_coordinates, y_coordinates)
    ```
    """
    waypoints = []
    for x, y in zip(rx, ry):
        wp = [x, y]
        waypoints.append(wp)

    return waypoints


def draw_heatmap(data):
     """
    Draw a heatmap from the given data.

    Args:
    - data (list or np.array): The input data for the heatmap.

    This function takes a list or NumPy array of data and draws a heatmap using the `pcolor` function from Matplotlib. The color intensity represents the values in the input data.

    Example:
    ```
    data_matrix = [
        [1, 2, 3],
        [4, 5, 6],
        [7, 8, 9]
    ]

    draw_heatmap(data_matrix)
    ```
    """
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)
