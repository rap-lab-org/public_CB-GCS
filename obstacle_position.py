class StaticPolygon:
    """
    Initialize a static polygon with a centroid and vertices.

    Parameters:
    - centroid (tuple): The centroid of the polygon in the form (x,y).
    - vertices (list of tuples): The vertices of the polygon in the form [(x1,y1), (x2,y2)...].
    """
    def __init__(self, centroid, vertices):
        self.centroid = centroid
        self.vertices = vertices

class DynamicPolygon:
    """
    Initialize a dynamic polygon with multiple time steps.

    Parameters:
    - timesteps (list): A list of time steps in the form [t1, t2, t3...]
    - centroids (list of tuples): List of centroids for each time step in the form [(x1,y1), (x2,y2)...].
    - vertices_set (list of lists of tuples): List of sets of vertices for each time step in the form
        [[(x1t1,y1t1), (x2t1,y2t1)...], [(x1t2,y1t2), (x2t2,y2t2)...]].

    Attributes:
    - polygon (list of objects): A list of StaticPolygon objects that contain:
        - centroid (tuple): The centroid of the polygon in the form (x,y).
        - vertices (list of tuples): The vertices of the polygon in the form [(x1,y1), (x2,y2)...].
    """
    def __init__(self, timesteps, centroids, vertices_set):
        self.timesteps = timesteps
        self.polygons = []

        for i in range(0, len(timesteps)):
            polygon = StaticPolygon(centroids[i], vertices_set[i])
            self.polygons.append(polygon)