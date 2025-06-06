from obstacle_position import DynamicPolygon
from generate_linear_obstacles import generate_dynamic_polygon_info
from polygon_generator import Polygon_obj, GeneratedPolygons
from neighbor_checker import extended_polygon, neighbor_checker

from shapely import Point, Polygon

def find_centroid(points):
    n = len(points)
    if n == 0:
        return None  # Handle empty list case

    # Sum up x and y coordinates separately
    sum_x = sum(point[0] for point in points)
    sum_y = sum(point[1] for point in points)

    # Calculate the centroid
    centroid_x = sum_x / n
    centroid_y = sum_y / n

    return (centroid_x, centroid_y)

def create_zero_matrix(x, y):
    matrix = [[0 for _ in range(y)] for _ in range(x)]
    return matrix

def create_dynamic_polygons(obstacle_start_pos, obstacle_start_centroids, obstacle_velocities, timesteps):
    """
    Create dynamic polygons based on initial positions, velocities, and time steps.

    Parameters:
    - obstacle_start_pos (list of list of tuples): List containing initial positions of obstacles.
    - obstacle_velocities (list of tuples): List containing velocities of obstacles.
    - timesteps (list): List of time steps.

    Returns:
    - dynamic_polygons (list): List of DynamicPolygon objects representing dynamic obstacles.
    """

    dynamic_polygons = []

    # Iterate over each obstacle
    for i in range(len(obstacle_start_pos)):

        # Generate information about the dynamic polygon
        temp_vertices, temp_centroids = generate_dynamic_polygon_info(
            obstacle_start_pos[i], obstacle_start_centroids[i], obstacle_velocities[i], timesteps)
        
        # Create a DynamicPolygon object
        temp_dynamic_polygon = DynamicPolygon(
            timesteps=timesteps,
            centroids=temp_centroids,
            vertices_set=temp_vertices)
        
        # Append the DynamicPolygon object to the list
        dynamic_polygons.append(temp_dynamic_polygon)
    
    return dynamic_polygons

def create_timed_obstacle_list(obstacle_start_pos, timesteps, bounds, dynamic_polygons):
    """
    Creates a list of lists of polygonal obstacles over time.

    Parameters:
        obstacle_start_pos (list): List of starting positions for each obstacle.
        timesteps (list): List of time steps.
        bounds (tuple): Tuple containing the bounds of the environment (min_x, min_y, max_x, max_y).
        dynamic_polygons (list): List of dynamic polygons representing obstacles over time.

    Returns:
        list: A list of GeneratedPolygons objects representing the polygonal obstacles over time.
    """
    # Get the number of obstacles and timesteps
    obs = len(obstacle_start_pos)
    steps = len(timesteps)
    
    # Create a matrix to store polygonal obstacles for each obstacle at each timestep
    polygons_list = create_zero_matrix(steps, obs)
    
    # Iterate over each obstacle and timestep to create polygonal obstacles
    for i in range(0, obs):
        for j in range(0, steps):
            # Create a temporary polygon object with dynamic vertices
            temp_poly = Polygon_obj(len(dynamic_polygons[i].polygons[j].vertices),
                                     bounds, None, dynamic_polygons[i].polygons[j].vertices)
            # Store the temporary polygon in the matrix
            polygons_list[j][i] = temp_poly

    # Create a list to store sets of polygonal obstacles for each timestep
    polygonal_obstacles = []
    for i in range(0, steps):
        # Create a GeneratedPolygons object with bounds and polygonal obstacles for the timestep
        temp_sets = GeneratedPolygons(bounds)
        temp_sets.polygons = polygons_list[i]
        # Add the generated set to the list
        polygonal_obstacles.append(temp_sets)

    return polygonal_obstacles

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

def find_graph_vertices_edges(start_point, end_point, speed, polygonal_sets, timesteps):
    """
    Create a graph from given sets of polygons.

    Args:
        start_point: The starting point for the vehicle.
        speed: The max speed the vehicle can travel
        polygonal_sets: List of GeneratedPolygon objects corresponding to the free space for each timestep.
        timesteps: List of timesteps.
        end_point: The end point  for the vehicle.

    Returns:
        Tuple: (graph_nodes, graph_edges)
            graph_nodes: List of nodes in the graph.
            graph_edges: List of edges in the graph.
    """

    # Given start point, find all sets in the next time step it can reach
    delta_t = timesteps[1]-timesteps[0]
    ex_poly = extended_polygon([start_point], speed, delta_t)
    
    # Find neighbors and indices for the start point
    current_neighbors, indices = neighbor_checker(ex_poly, polygonal_sets[1])
    # Store the starting data to later build a graph
    graph_nodes = [[0], indices]
    graph_edges = [[[0, indices]]]
    # print(len(current_neighbors))
    # print(len(indices))
    # print(current_neighbors)
    # Iterate over timesteps
    if len(timesteps)>2:
        for i in range(2, len(timesteps)):
            new_nodes = []
            new_edges = []
            delta_t = timesteps[i] - timesteps[i-1]
            # Iterate over current_neighbors
            for j in range(0, len(current_neighbors)):
                # Use the neighbors found in the previous set as the nodes to find edges for
                # in the current set
                # print(current_neighbors[j])
                # print(i, j)
                ex_poly = extended_polygon(current_neighbors[j], speed, delta_t)
                temp_neighbors, temp_indices = neighbor_checker(ex_poly, polygonal_sets[i])
                # print(temp_neighbors, temp_indices)
                # if j == 5:
                #     print(i, j, temp_indices)
                new_nodes.extend(temp_indices)
                new_edges.append([indices[j], temp_indices])
            
            # Remove duplicates to get a list of all nodes for the following timestep
            new_nodes = list(set(new_nodes))
            
            # Append new nodes and edges 
            graph_nodes.append(new_nodes)
            graph_edges.append(new_edges)
            
            # Set the new_nodes as the current_neighbors to find edges for the next set 
            current_neighbors = polygonal_sets[i].find_vertices(new_nodes)
            indices = new_nodes
    
    # For the final node we need to find which node contains the end point
    final_sets = polygonal_sets[-1].find_vertices(graph_nodes[-1])
    final_nodes = find_containing_list(final_sets, end_point)
    # print('node output', final_nodes)
    # #Remove all nodes that do not contain the end point
    # print(graph_edges[-1])
    for i in range(0, len(graph_edges[-1])):
        corrected_edges = graph_edges[-1][i][1]
        corrected_edges = [x for x in corrected_edges if x in final_nodes]
        graph_edges[-1][i][1] = corrected_edges
        # print(graph_edges[-1])
    fixed_edges = [sublist for sublist in graph_edges[-1] if sublist[1]]
    graph_edges[-1] = fixed_edges

    graph_nodes[-1] = final_nodes

    return graph_nodes, graph_edges
