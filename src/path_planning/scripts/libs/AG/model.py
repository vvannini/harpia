import collections
import math

from libs.AG.utils import pairwise_circle, _normal, _eq_line, _eq_intersection_point

CartesianPoint = collections.namedtuple("CartesianPoint", "x y")
GeoPoint = collections.namedtuple("GeoPoint", "latitude, longitude, altitude")
Vector = collections.namedtuple("Vector", "x y")
Version = collections.namedtuple("Version", "major, minor")


class Mapa:
    def __init__(
        self, origin, destination, areas_n=None, areas_b=None, inflation_rate=0.1
    ):
    """
        Represents a map with origin, destination, navigable and non-navigable areas.

        Parameters:
        - origin (CartesianPoint): The starting point of the route.
        - destination (CartesianPoint): The destination point of the route.
        - areas_n (list of list of CartesianPoint, optional): Non-navigable areas defined by lists of vertices.
        - areas_b (list of list of CartesianPoint, optional): Bonifying areas defined by lists of vertices.
        - inflation_rate (float, optional): The inflation rate for non-navigable areas. Default is 0.1.

        Attributes:
        - origin (CartesianPoint): The starting point of the route.
        - destination (CartesianPoint): The destination point of the route.
        - areas_b (list of list of CartesianPoint): Bonifying areas defined by lists of vertices.
        - areas_n (list of list of CartesianPoint): Non-navigable areas defined by lists of vertices.
        - areas_n_inf (list of list of CartesianPoint): Inflated non-navigable areas.

        Examples:
        >>> origin = CartesianPoint(0, 0)
        >>> destination = CartesianPoint(5, 5)
        >>> areas_n = [[CartesianPoint(1, 1), CartesianPoint(2, 2), CartesianPoint(3, 1)]]
        >>> areas_b = [[CartesianPoint(2, 3), CartesianPoint(3, 4), CartesianPoint(4, 3)]]
        >>> mapa = Mapa(origin, destination, areas_n, areas_b)
        """
        self.origin = origin  # CartesianPoint : Define o ponto de partida da rota
        self.destination = destination  # CartesianPoint : Define o ponto de destino da rota
        self.areas_b = areas_b  # [area, ...] : Areas bonificadoras
        self.areas_n = areas_n  # [area, ...] : Areas nao-navegaveis
        #                       # area = [CartesianPoint(),...]
        self.areas_n_inf = [self._inflate_area(area, inflation_rate=inflation_rate) for area in areas_n]

    def _inflate_area(self, area, inflation_rate):

        """
        Inflates a non-navigable area defined by a list of vertices.

        Parameters:
        - area (list of CartesianPoint): List of vertices defining the non-navigable area.
        - inflation_rate (float): The inflation rate for the non-navigable area.

        Returns:
        - list of CartesianPoint: Inflated non-navigable area defined by a list of vertices.

        Examples:
        >>> area = [CartesianPoint(1, 1), CartesianPoint(2, 2), CartesianPoint(3, 1)]
        >>> inflation_rate = 0.1
        >>> inflated_area = _inflate_area(area, inflation_rate)
        """
        lines = []

        for V1, V2 in pairwise_circle(area):
            N = _normal(V1, V2)

            NV1 = CartesianPoint(
                V1.x + N.x * inflation_rate, V1.y + N.y * inflation_rate
            )
            NV2 = CartesianPoint(
                V2.x + N.x * inflation_rate, V2.y + N.y * inflation_rate
            )

            a, b, c = _eq_line(NV1, NV2)
            lines.append((a, b, c))

            # if verbose:
            #     print(f"V1:{NV1} V2:{NV2}  ->  L:({a}x + {b}y + {c} = 0)")

        new_area = []

        for L1, L2 in pairwise_circle(lines):
            x, y = _eq_intersection_point(L1[0], L1[1], L1[2], L2[0], L2[1], L2[2])
            new_area.append(CartesianPoint(x, y))

            # if verbose:
            #     print(
            #         f"L1:({L1[0]}x + {L1[1]}y + {L1[2]} = 0) L2:({L2[0]}x + {L2[1]}y + {L2[2]} = 0)  ->  V=({x},{y})"
            #     )

        return new_area


class Conversor:
    def list_geo_to_cart(l, geo_home):
        """
        Converts a list of GeoPoint objects to CartesianPoint objects relative to a home GeoPoint.

        Parameters:
        - l (list of GeoPoint): List of GeoPoint objects to convert.
        - geo_home (GeoPoint): The reference GeoPoint.

        Yields:
        - CartesianPoint: Converted CartesianPoint objects.

        Example:
        >>> geo_points = [GeoPoint(37.7749, -122.4194, 0), GeoPoint(37.7749, -122.4194, 0)]
        >>> home_point = GeoPoint(37.7749, -122.4194, 0)
        >>> cartesian_points = Conversor.list_geo_to_cart(geo_points, home_point)
        >>> list(cartesian_points)
        [CartesianPoint(...), CartesianPoint(...)]
        """
        for i in l:
            yield Conversor.geo_to_cart(i, geo_home)

    def list_cart_to_geo(l, geo_home):
        """
        Converts a list of CartesianPoint objects to GeoPoint objects relative to a home GeoPoint.

        Parameters:
        - l (list of CartesianPoint): List of CartesianPoint objects to convert.
        - geo_home (GeoPoint): The reference GeoPoint.

        Yields:
        - GeoPoint: Converted GeoPoint objects.

        Example:
        >>> cartesian_points = [CartesianPoint(0, 0, 0), CartesianPoint(1, 1, 0)]
        >>> home_point = GeoPoint(37.7749, -122.4194, 0)
        >>> geo_points = Conversor.list_cart_to_geo(cartesian_points, home_point)
        >>> list(geo_points)
        [GeoPoint(...), GeoPoint(...)]
        """
        for i in l:
            yield Conversor.cart_to_geo(i, geo_home)

    def geo_to_cart(geo_point, geo_home):
        """
        Converts a GeoPoint object to a CartesianPoint object relative to a home GeoPoint.

        Parameters:
        - geo_point (GeoPoint): The GeoPoint to convert.
        - geo_home (GeoPoint): The reference GeoPoint.

        Returns:
        - CartesianPoint: Converted CartesianPoint object.

        Example:
        >>> geo_point = GeoPoint(37.7749, -122.4194, 0)
        >>> home_point = GeoPoint(37.7749, -122.4194, 0)
        >>> cartesian_point = Conversor.geo_to_cart(geo_point, home_point)
        >>> print(cartesian_point)
        CartesianPoint(...)
        """

        def calc_y(lat, lat_):
            return (lat - lat_) * (10000000.0 / 90)

        def calc_x(longi, longi_, lat_):
            return (longi - longi_) * (
                6400000.0 * (math.cos(lat_ * math.pi / 180) * 2 * math.pi / 360)
            )

        x = calc_x(geo_point.longitude, geo_home.longitude, geo_home.latitude)
        y = calc_y(geo_point.latitude, geo_home.latitude)

        # return CartesianPoint(x, y, geo_point.altitude)
        return CartesianPoint(x, y)

    def cart_to_geo(cartesian_point, geo_home):
        """
        Converts a CartesianPoint object to a GeoPoint object relative to a home GeoPoint.

        Parameters:
        - cartesian_point (CartesianPoint): The CartesianPoint to convert.
        - geo_home (GeoPoint): The reference GeoPoint.

        Returns:
        - GeoPoint: Converted GeoPoint object.

        Example:
        >>> cartesian_point = CartesianPoint(0, 0, 0)
        >>> home_point = GeoPoint(37.7749, -122.4194, 0)
        >>> geo_point = Conversor.cart_to_geo(cartesian_point, home_point)
        >>> print(geo_point)
        GeoPoint(...)
        """
        def calc_latitude_y(lat_, y):
            return ((y * 90) / 10000000.0) + lat_

        def calc_longitude_x(lat_, longi_, x):
            return ((x * 90) / (10008000 * math.cos(lat_ * math.pi / 180))) + longi_

        longitude_x = calc_longitude_x(
            geo_home.latitude, geo_home.longitude, cartesian_point.x
        )
        latitude_y = calc_latitude_y(geo_home.latitude, cartesian_point.y)

        # return GeoPoint(longitude_x, latitude_y, cartesian_point.z)
        # return GeoPoint(longitude_x, latitude_y, 10)
        return GeoPoint(latitude_y, longitude_x, 10)
