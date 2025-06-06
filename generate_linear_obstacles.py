from obstacle_position import DynamicPolygon

def generate_dynamic_polygon_info(initial_vertices, centroid, velocity, timesteps):
    """
    Generate information for a DynamicPolygon obstacle based on initial vertices, centroid, velocity, and a list of timesteps.

    Parameters:
    - initial_vertices: Initial positions of the polygon vertices (list: [tuple: (x, y)]).
    - centroid: Initial centroid of the polygon (tuple: (x, y)).
    - velocity: Velocity of the polygon (tuple: (vx, vy)).
    - timesteps: List of timesteps for which to move the polygon.

    Returns:
    - vertices_set: The vertices of the polygon for each timestep (list: [list: [tuple: (x, y)]]).
    - centroids: The centroid of the polygon for each timestep (list: [tuple: (x, y)]).
    """
    dynamic_polygon_vertices = []
    dynamic_polygon_centroids = []
    for timestep in timesteps:
        current_timestep_vertices = []
        current_timestep_centroid = (centroid[0] + velocity[0] * timestep, centroid[1] + velocity[1] * timestep)
        for vertex in initial_vertices:
            new_x = vertex[0] + velocity[0] * timestep
            new_y = vertex[1] + velocity[1] * timestep

            current_timestep_vertices.append((new_x, new_y))

        dynamic_polygon_vertices.append(current_timestep_vertices)
        dynamic_polygon_centroids.append(current_timestep_centroid)
    
    return dynamic_polygon_vertices, dynamic_polygon_centroids

if __name__ == 'main':

    # Example usage:
    initial_vertices = [(0,0), (0,1), (1,1), (1,0)]
    centroid = (1/2, 1/2)
    velocity = (1, 1)
    timesteps = [0, 1, 3, 5]

    dynamic_polygon_vertices, dynamic_polygon_centroids = generate_dynamic_polygon_info(initial_vertices, centroid, velocity, timesteps)

    dynamic_polygon = DynamicPolygon(
        timesteps=timesteps,
        centroids=dynamic_polygon_centroids,
        vertices_set=dynamic_polygon_vertices)