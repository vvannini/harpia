import itertools
import math

from collections import namedtuple
from scipy.stats import norm

import matplotlib.pyplot as plt
import numpy as np

from libs.AG.visualization import plot_map


CartesianPoint = namedtuple('CartesianPoint', 'x y')
Vector = namedtuple('Vector', 'x y')


def pairwise_circle(iterable):
    """
    Generates pairs of elements from an iterable in a circular manner.

    Parameters:
    - iterable (iterable): Any iterable (e.g., list, tuple, string) from which pairs are generated.

    Returns:
    - itertools.zip_longest iterator: An iterator yielding pairs of elements from the input iterable.
      The last pair will be composed of the last element and the first element of the iterable.

    Example:
    >>> list(pairwise_circle([1, 2, 3, 4]))
    [(1, 2), (2, 3), (3, 4), (4, 1)]

    >>> list(pairwise_circle("abcde"))
    [('a', 'b'), ('b', 'c'), ('c', 'd'), ('d', 'e'), ('e', 'a')]
    """
    "s -> (s0,s1), (s1,s2), (s2, s3), ... (s<last>,s0)"
    a, b = itertools.tee(iterable)
    first_value = next(b, None)
    return itertools.zip_longest(a, b,fillvalue=first_value)

def prob_collision(distance, uncertainty):
    # Survival function (also defined as 1 - cdf, but sf is sometimes more accurate).
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.norm.html

    # mi    : média         : location (loc)
    # sigma : desvio padrão : scale

    return norm.sf(distance, loc=0, scale=uncertainty)

def distance_point_line(P, A, B, return_normal=False):
    '''Calculates the distance between the point P and the line that crosses the points A and B'''

    # Director vector of line
    D = Vector((A.x - B.x), (A.y - B.y))

    # Normal vector of line
    #N = Vector(-D.y, D.x)

    # Normalized normal vector of line AB
    aux = (math.sqrt(D.y**2+D.x**2))
    N = Vector( (D.y/aux), ((-D.x)/aux) )


    b = N.x * A.x + N.y * A.y

    distance = P.x * N.x + P.y * N.y - b

    if return_normal:
        return distance, N

    return distance



def chance_constraint(P, obs, uncertainty=1):
    
    """
    Computes the probability of collision for a point in the presence of obstacles.

    Parameters:
    - P (tuple): Coordinates of the point in the form (x, y).
    - obs (iterable): List of obstacle points or vertices defining the obstacle boundary.
    - uncertainty (float, optional): GPS imprecision representing uncertainty. Default is 1.

    Returns:
    - float: Probability of collision for the given point in the presence of obstacles.

    Notes:
    - The function uses the `distance_point_line` function to calculate distances from the point to lines
      formed by consecutive pairs of vertices in a circular manner (last vertex connected to the first).
    - The maximum distance is used to compute the probability of collision using the `prob_collision` function.

    Example:
    >>> chance_constraint((0, 0), [(1, 1), (2, 2), (3, 3)])
    0.25

    >>> chance_constraint((5, 5), [(1, 1), (2, 2), (3, 3)], uncertainty=2)
    0.012345678901234

    >>> chance_constraint((0, 0), [(1, 1), (2, 2), (3, 3)], uncertainty=0.5)
    0.5
    """
    
    # Uncertainty is the GPS imprecision
    distances = []

    for A, B in pairwise_circle(obs):
        aux_distance = distance_point_line(P, A, B)
        distances.append(aux_distance)


    distance = max(distances)

    chance = prob_collision(distance, uncertainty)
    return chance



# ----------------
# Visualization