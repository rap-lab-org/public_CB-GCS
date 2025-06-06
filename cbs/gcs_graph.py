from typing import TypedDict
from shapely import Polygon, Point, difference
from polygon_generator import Polygon_obj

import sys
import os

_current = os.path.dirname(os.path.realpath(__file__))
# directory reach
_parent = os.path.dirname(_current)
if _parent not in sys.path:
    sys.path.append(_parent)
if _current not in sys.path:
    sys.path.append(_current)
import cbs.type_stubs as TP
from polygon_generator import GeneratedPolygons


NodeInfo = TypedDict("NodeInfo", {"Layer": float, "Convex Set": TP.Vert | Polygon_obj})
class EdgeUsed:
    def __init__(self, var: tuple):
        self.u: NodeInfo = var[0]
        self.v: NodeInfo = var[1]
        self.y: float = var[2]
        self.zx: float = var[3]
        self.zy: float = var[4]
        self.zxp: float = var[5]
        self.zyp: float = var[6]
        self.l: float = var[7]

    def __str__(self) -> str:
        dat = {
            "u": str(self.u),
            "v": str(self.v),
            "y": str(self.y),
            "zx": str(self.zx),
            "zy": str(self.zy),
            "zxp": str(self.zxp),
            "zyp": str(self.zyp),
            "l": str(self.l),
        }
        import json

        return json.dumps(dat, indent=2)

    def __repr__(self):
        return self.__str__()


def extended_polygon(
    coords: list[TP.Vert], vmax: float, dt: float, obsts: GeneratedPolygons
) -> Polygon:
    """
    Extends the border of a shapely polygon or a point by a set speed.

    Parameters:
        geo: The list of coordinates for the original polygon or point.
        vmax: The speed that the object can travel.
        dt: The amount of time between the current and next timestep
    Returns:
        Polygon or Point: A shapely Polygon or Point that is extended by the object speed.
    """

    # Check the length of the input list
    if len(coords) == 1:
        # If the length is 1, treat it as a point
        freePoly = Point(coords[0])
    else:
        # Otherwise, treat it as a polygon
        freePoly = Polygon(coords)

    # Extend the shape by the object speed
    extedFreePoly: Polygon = freePoly.buffer(vmax * dt)

    obsPolys = Polygon([])
    for obs in obsts.polygons:
        obsPolys = obsPolys.union(Polygon(obs.vertices))
    shape = difference(extedFreePoly, obsPolys)
    # if the extended polygon is divided into multiple components by obstacles
    if shape.geom_type == "MultiPolygon":
        for poly in shape.geoms:
            # return the components contains the original polygon
            if poly.contains(freePoly):
                return poly
        return freePoly
    return extedFreePoly


def neighbor_checker(
    extedPoly: Polygon, freePolys: GeneratedPolygons
) -> tuple[list[TP.Poly], list[int]]:
    """
    Optimize the intersection check between a Shapely Polygon and a list of polygons.

    Parameters:
    - extedPoly: The small Shapely Polygon for which intersections are checked.
    - polygons: A GeneratedPolygon object which contains the set of polygons we wih to check.
    """

    # Check for actual intersections with the small polygon
    intersectPolys: list[TP.Poly] = []
    intersectIdxes: list[int] = []
    # for i in potential_intersect_indices:
    for i in range(0, len(freePolys.polygons)):
        all_set = freePolys.polygons[i].vertices
        large_polygon_coords = all_set
        large_polygon = Polygon(large_polygon_coords)
        # Check if the extended polygon intersects with the large polygon and exclude touching edges
        if extedPoly.intersects(
            large_polygon
        ):  # and not extended_polygon.touches(large_polygon):
            intersectPolys.append(large_polygon.exterior.coords[:-1])
            intersectIdxes.append(i)

    return intersectPolys, intersectIdxes


def find_containing_list(lists_of_points, point):
    # Convert the point to a Shapely Point object
    point = Point(point)

    # Iterate over each list of points
    indices = []
    for i, points in enumerate(lists_of_points):
        # Construct a polygon from the points
        polygon = Polygon(points)

        # Check if the point is within the polygon or on its boundary
        if polygon.contains(point) or polygon.intersects(point):
            indices.append(i)
            return indices
    return indices


def find_graph_vertices_edges(
    s: TP.Vert,
    d: TP.Vert,
    vmax: float,
    freePolys: list[GeneratedPolygons],
    obsPolys: dict[int, GeneratedPolygons],
    ts: TP.TimeSteps,
):
    """
    Create a graph from given sets of polygons.

        s:        The starting point for the vehicle.
        d:        The desitnation point for the agent.
        vmax:     The max speed the vehicle can travel
        freePoly: free space polygons for each timestep.
        obsPoly:  obstacles polygons for each timestep
        ts:       List of timesteps.

    Returns:
        Tuple: (graph_nodes, graph_edges)
            graph_nodes: List of nodes in the graph.
            graph_edges: List of edges in the graph.
    """

    # Given start point, find all sets in the next time step it can reach
    dt = ts[1] - ts[0]
    ex_poly = extended_polygon([s], vmax, dt, obsPolys[1])

    # Find neighbors and indices for the start point
    curNgbrs, idxes = neighbor_checker(ex_poly, freePolys[1])
    # Store the starting data to later build a graph
    gNodes = [[0], idxes]
    gEdges = [[[0, idxes]]]

    # Iterate over timesteps
    if len(ts) > 2:
        for i in range(2, len(ts)):
            new_nodes = []
            new_edges = []
            dt = ts[i] - ts[i - 1]
            # Iterate over current_neighbors
            for j in range(0, len(curNgbrs)):
                # Use the neighbors found in the previous set as the nodes to find edges for
                ex_poly = extended_polygon(curNgbrs[j], vmax, dt, obsPolys[i])
                _, temp_indices = neighbor_checker(ex_poly, freePolys[i])
                new_nodes.extend(temp_indices)
                new_edges.append([idxes[j], temp_indices])

            # Remove duplicates to get a list of all nodes for the following timestep
            new_nodes = list(set(new_nodes))

            # Append new nodes and edges
            gNodes.append(new_nodes)
            gEdges.append(new_edges)

            # Set the new_nodes as the current_neighbors to find edges for the next set
            curNgbrs = freePolys[i].find_vertices(new_nodes)
            idxes = new_nodes

    # For the final node we need to find which node contains the end point
    final_sets = freePolys[-1].find_vertices(gNodes[-1])
    final_nodes = find_containing_list(final_sets, d)
    # #Remove all nodes that do not contain the end point
    for i in range(0, len(gEdges[-1])):
        corrected_edges = gEdges[-1][i][1]
        corrected_edges = [x for x in corrected_edges if x in final_nodes]
        gEdges[-1][i][1] = corrected_edges
        # print(graph_edges[-1])
    fixed_edges = [sublist for sublist in gEdges[-1] if sublist[1]]
    gEdges[-1] = fixed_edges
    gNodes[-1] = final_nodes

    return gNodes, gEdges
