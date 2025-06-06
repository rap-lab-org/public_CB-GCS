from shapely.geometry import Polygon, Point
from rtree import index

def extended_polygon(original_coords, object_speed, delta_t):
    """
    Extends the border of a shapely polygon or a point by a set speed.

    Parameters:
        original_coords (list): The list of coordinates for the original polygon or point.
        object_speed (int/float): The speed that the object can travel.
        delta_t (int/float): The amount of time between the current and next timestep
    Returns:
        Polygon or Point: A shapely Polygon or Point that is extended by the object speed.
    """
    # Check the length of the input list
    if len(original_coords) == 1:
        # If the length is 1, treat it as a point
        original_shape = Point(original_coords[0])
    else:
        # Otherwise, treat it as a polygon
        original_shape = Polygon(original_coords)
    
    # Extend the shape by the object speed
    extended_shape = original_shape.buffer(object_speed*delta_t)
    
    return extended_shape


def neighbor_checker(extended_polygon, polygons):
    """
    Optimize the intersection check between a Shapely Polygon and a list of polygons.

    Parameters:
    - checked_polygon (Polygon): The small Shapely Polygon for which intersections are checked.
    - polygons (GeneratedPolygon): , A GeneratedPolygon object which contains the set of polygons we wih to check.

    Returns:
    list: A list of Shapely Polygons representing the polygons from polygon_list that intersect with the extended_polygon.
    """
    all_sets = []
    for i in range(0, len(polygons.polygons)):
        all_sets.append(polygons.polygons[i].vertices)
    # # Create an R-tree index
    # idx = index.Index()

    # # Insert each large polygon into the index with its bounding box
    # for i, polygon_coords in enumerate(all_sets):
    #     large_polygon = Polygon(polygon_coords)
    #     idx.insert(i, large_polygon.bounds)

    # # Query the index for potential intersecting polygons
    # potential_intersect_indices = list(idx.intersection(extended_polygon.bounds))

    # Check for actual intersections with the small polygon
    intersecting_polygons = []
    intersecting_indices = []
    # for i in potential_intersect_indices:
    for i in range(0, len(polygons.polygons)):
        large_polygon_coords = all_sets[i]
        large_polygon = Polygon(large_polygon_coords)
        # Check if the extended polygon intersects with the large polygon and exclude touching edges
        if extended_polygon.intersects(large_polygon):# and not extended_polygon.touches(large_polygon):
            intersecting_polygons.append(large_polygon.exterior.coords[:-1])
            intersecting_indices.append(i)

    return intersecting_polygons, intersecting_indices
