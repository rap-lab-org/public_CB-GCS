import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
from shapely import get_coordinates
from scipy.spatial import ConvexHull
from copy import deepcopy

class Polygon_obj:
    """A class representing a polygon object."""

    def __init__(self, num_vertices, bounds, max_size, vertices=None):
        """
        Initialize a Polygon_obj instance.

        Parameters:
        - num_vertices (int): The number of vertices of the polygon.
        - bounds (tuple): Tuple containing bounds for x and y coordinates.
        - max_size (int): Maximum size of the polygon.
        - vertices (list, optional): List of vertices for the polygon.
        """
        self.x = bounds[0]
        self.y = bounds[1]
        self.num_vertices = num_vertices
        self.centroid_x_range = (1, self.x[1])
        self.centroid_y_range = (1, self.y[1])
        self.distance_range = (1, max_size)
        self.max_size = max_size
        self.vertices = vertices
        if type(self.vertices) == type(None):
            # If we define vertices beforehand we also generate the equations
            # Otherwise we call the generate_polygon method to generate them
            self.equations = None
        else:
            vertices = Polygon(self.vertices).exterior.coords[:]
            x, y = list(zip(*vertices))
            self.equations = self.calculate_equations(x, y)

    def is_convex(self, x, y):
        """
        Check if the polygon is convex.

        Parameters:
        - x (list): List of x coordinates of vertices.
        - y (list): List of y coordinates of vertices.

        Returns:
        - bool: True if the polygon is convex, False otherwise.
        """
        points = np.column_stack((x, y))
        hull = ConvexHull(points)
        return len(hull.vertices) == len(x)

    def generate_polygon(self):
        """
        Generate a random polygon for the polygon object.
        """
        while True:
            # Generate random coordinates for the centroid within the specified ranges
            centroid_x = np.random.uniform(*self.centroid_x_range)
            centroid_y = np.random.uniform(*self.centroid_y_range)

            # Generate random distances from the centroid to each vertex
            distances = np.random.uniform(*self.distance_range, self.num_vertices)

            # Calculate the angles between the vertices
            angles = 2 * np.pi * np.arange(self.num_vertices) / self.num_vertices

            # Calculate the coordinates of the vertices
            x = centroid_x + distances * np.cos(angles)
            y = centroid_y + distances * np.sin(angles)

            # Apply a constraint to keep vertices within the desired range
            x = np.clip(x, self.centroid_x_range[0], self.centroid_x_range[1])
            y = np.clip(y, self.centroid_y_range[0], self.centroid_y_range[1])

            # Round the coordinates to the nearest integer
            x = np.round(x).astype(int)
            y = np.round(y).astype(int)

            if self.is_convex(x, y):
                # Calculate the equations for the polygon
                x = np.append(x, x[0])
                y = np.append(y, y[0])
                self.vertices = list(zip(x, y))
                self.equations = self.calculate_equations(x, y)
                break

    def generate_rectangle(self):
        """
        Generate a random rectangle for the polygon object.

        Returns:
        - list: List of vertices of the generated rectangle.
        """
        x_min, x_max = (1, self.x[1])
        y_min, y_max = (1, self.y[1])

        # Generate random coordinates for the bottom-left corner
        x1 = np.random.uniform(x_min, x_max)
        y1 = np.random.uniform(y_min, y_max)
        # Generate random dimensions for the rectangle
        width = np.random.uniform(1, min(x_max - x1, self.max_size))
        height = np.random.uniform(1, min(y_max - y1, self.max_size))
        # Calculate the coordinates of the other three corners
        x2, y2 = x1 + width, y1
        x3, y3 = x1, y1 + height
        x4, y4 = x1 + width, y1 + height

        self.vertices = [(x1, y1), (x2, y2), (x4, y4), (x3, y3), (x1, y1)]
        x = [point[0] for point in self.vertices]
        y = [point[1] for point in self.vertices]
        self.equations = self.calculate_equations(x, y)
        return self.vertices

    def __str__(self):
        """
        Return a string representation of the polygon object.

        Returns:
        - str: String representation of the polygon object.
        """
        return f"{self.num_vertices} Sided Polygon"


    def calculate_equations(self, x, y):
        """
        Calculate equations for the polygon edges.

        Parameters:
        - x (list): List of x coordinates of vertices.
        - y (list): List of y coordinates of vertices.

        Returns:
        - list: List of equations for polygon edges.
        """
        # shapely Polygon used to check if the inequality is >= or <=0 
        temp_polygon = Polygon(self.vertices)
        equations = []
        for i in range(len(x) - 1):
            x1, y1, x2, y2 = x[i], y[i], x[i + 1], y[i + 1]
            # Calculate the coefficients a, b, and c for the equation a*x + b*y + c <= 0
            bounds = [(x1, y1), (x2, y2)]
            if x1 == x2:
                # Vertical line, slope is zero, but we can calculate the x-intercept
                m = 0
                q = x1  # or x2, both are the same for a vertical line
            elif y1 == y2:
                # Horizontal line, slope is undefined, but y-intercept is known
                m = None
                q = y1
            else:
                m = (y2 - y1) / (x2 - x1)
                q = y1 - m * x1
             
            # print(m,q, 'm,q')
            # print((x1,y1), (x2,y2))
            if type(m) == type(None):
                # print('horizontal line')
                
                midpoint = Point([(x1 + x2)/2, (y1 + y2)/2+.0001])
                # print(midpoint)
                if temp_polygon.intersects(midpoint):
                    # print('intersection!')
                    sign = 'geq'
                    a, b, c = (0, 1, -q)
                else:
                    # print('no intersection!')
                    sign = 'leq'
                    a, b, c = (0, 1, -q)
            elif m == 0:
                # print('vertical line')
                midpoint = Point([(x1 + x2)/2+.0001, (y1 + y2)/2])
                # print(midpoint)
                if temp_polygon.intersects(midpoint):
                    # print('intersection!')
                    sign = 'geq'
                    a, b, c = (1, 0, -q)
                else:
                    # print('no intersection!')
                    sign = 'leq'
                    a, b, c = (1, 0, -q)
            else:
                # print('sloped')
                midpoint = Point([(x1 + x2)/2, (y1 + y2)/2+.0001])
                # print(midpoint)
                if temp_polygon.intersects(midpoint):
                    # print('intersection!')
                    sign = 'geq'
                    a, b, c = (-m, 1, -q)
                else:
                    # print('no intersection!')
                    sign = 'leq'
                    a, b, c = (-m, 1, -q)
            if sign == 'geq':
                a *= -1
                b *= -1
                c *=-1
            equations.append([(a, b, c), bounds])
        return equations



class GeneratedPolygons:
    """A class representing a collection of generated polygons."""

    def __init__(self, bounds):
        """
        Initialize a GeneratedPolygons instance.

        Parameters:
        - bounds (tuple): Tuple containing bounds for x and y coordinates.
        """
        self.x = bounds[0]
        self.y = bounds[1]
        self.polygons: list[Polygon_obj] = []

    def make_polygons(self, number_of_polygons, max_size, max_vertices):
        """
        Generate a set of polygons.

        Parameters:
        - number_of_polygons (int): Number of polygons to generate.
        - max_size (int): Maximum size of the polygons.
        - max_vertices (int): Maximum number of vertices for the polygons.
        """
        for _ in range(number_of_polygons):
            num_vertices = np.random.randint(3, high=max_vertices + 1, dtype=int)
            temp_polygon = Polygon_obj(
                num_vertices,
                [self.x, self.y],
                max_size)
            temp_polygon.generate_polygon()
            self.polygons.append(temp_polygon)

    def generate_non_overlapping_rectangles(self, m, max_size):
        """
        Generate a set of non-overlapping rectangles.

        Parameters:
        - m (int): Number of rectangles to generate.
        - max_size (int): Maximum size of the rectangles.

        Returns:
        - list: List of generated non-overlapping rectangles.
        """
        num_vertices = 4
        for _ in range(m):
            random_rectangle = Polygon_obj(
                num_vertices,
                [self.x, self.y],
                max_size)
            
            new_rect = random_rectangle.generate_rectangle()

            # Check for overlap with existing rectangles
            while self.check_overlap(new_rect):
                new_rect = random_rectangle.generate_rectangle()

            # Append the RandomRectangle instance
            self.polygons.append(random_rectangle)

        return self.polygons

    def check_overlap(self, new_rect):
        """
        Check if a new rectangle overlaps with existing rectangles.

        Parameters:
        - new_rect (list): List of vertices of the new rectangle.

        Returns:
        - bool: True if there is overlap, False otherwise.
        """
        for existing_rect in self.polygons:
            if self.rectangles_overlap(new_rect, existing_rect.vertices):
                return True
        return False

    @staticmethod
    def rectangles_overlap(rect1, rect2):
        """
        Check if two rectangles overlap.

        Parameters:
        - rect1 (list): List of vertices of the first rectangle.
        - rect2 (list): List of vertices of the second rectangle.

        Returns:
        - bool: True if there is overlap, False otherwise.
        """
        x1, y1, x2, y2 = rect1[0][0], rect1[0][1], rect1[2][0], rect1[2][1]
        x3, y3, x4, y4 = rect2[0][0], rect2[0][1], rect2[2][0], rect2[2][1]

        return not (x2 < x3 or x4 < x1 or y2 < y3 or y4 < y1)

    def plot_polygons(self, figax=None, idxes: list[int] | None=None):
        """
        Plot the polygons.

        Parameters:
        - ax (matplotlib.axes.Axes, optional): Axes object for plotting.

        Returns:
        - matplotlib.axes.Axes: Axes object with plotted polygons.
        """
        if figax is None:
            fig, ax = plt.subplots()
        else:
            fig, ax = figax

        # Define a color map for unique edges
        edge_colors = plt.cm.get_cmap('tab10', len(self.polygons) + 1)

        for i, polygon in enumerate(self.polygons):

            if idxes is not None and i not in idxes:
                continue

            vertices = Polygon(polygon.vertices).exterior.coords[:]
            # print(vertices)
            x, y = list(zip(*vertices))

            # Plot the polygon
            ax.fill(x, y, alpha=0.5, color=edge_colors(i))

            # Plot the unique edges with a distinct color
            for edge in polygon.equations:
                (x1, y1), (x2, y2) = edge[1]
                ax.plot([x1, x2], [y1, y2], color=edge_colors(i), linewidth=2)

        ax.set_xlim([0, self.x[1] + 1])
        ax.set_ylim([0, self.y[1] + 1])
        return fig, ax

    def merge_rectangle(self):
        """
        Merge overlapping rectangles and remove contained rectangles.

        Returns:
        - list: List of merged rectangles.
        """
        if len(self.polygons) <2:
            return self.polygons
        open_queue = self.polygons
        closed_queue = []

        #Removes a polygon if it fully lies within another
        while len(open_queue) > 0:
            for j in range(1, len(open_queue)):
                p1 = Polygon(open_queue[0].vertices)
                p2 = Polygon(open_queue[j].vertices)
                if p1.contains(p2):
                    open_queue.pop(j)
                    break
                elif p2.contains(p1):
                    open_queue.pop(0)
                    break
            if len(open_queue) == 1:
                closed_queue.append(open_queue.pop(0))
            if j == len(open_queue) - 1:
                closed_queue.append(open_queue.pop(0))


        open_queue = deepcopy(closed_queue)
        closed_queue = []

        while len(open_queue) > 0:
            i = 0
            # j = 1
            for j in range(1, len(open_queue)):
                p1 = Polygon(open_queue[i].vertices)
                p2 = Polygon(open_queue[j].vertices)
                if p1.overlaps(p2):
                    # print(list(p1.exterior.coords), list(p2.exterior.coords))
                    p_new = unary_union([p1, p2])
                    # print(p_new)
                    coords = list(p_new.exterior.coords)
                    open_queue.pop(j)
                    open_queue.pop(i)
                    open_queue.append(Polygon_obj(len(coords), [self.x, self.y], None, vertices=coords))
                    break
            if len(open_queue) == 1:
                closed_queue.append(open_queue.pop(i))
            if j == len(open_queue) - 1:
                closed_queue.append(open_queue.pop(i))
        self.polygons = closed_queue

        return closed_queue
    
    def merge_rectangles(self):
        prev_output = self.merge_rectangle()
        curr_output = []
        while len(prev_output) != len(curr_output):
            curr_output = self.merge_rectangle()
            prev_output = curr_output
        return curr_output

    def merge_polygons(self):
        """
        Merge overlapping polygons and remove contained polygons.

        Returns:
        - list: List of merged polygons.
        """
        open_queue = deepcopy(self.polygons)
        closed_queue = []
        while len(open_queue) > 0:
            i = 0
            j = 1
            for j in range(1, len(open_queue)):
                p1 = Polygon(open_queue[i].vertices[0:-1])
                p2 = Polygon(open_queue[j].vertices[0:-1])
                if p1.overlaps(p2):
                    p_new = unary_union([p1, p2])
                    coords = list(p_new.exterior.coords)
                    open_queue.pop(j)
                    open_queue.pop(i)
                    open_queue.append(Polygon_obj(len(coords), [self.x, self.y], None, vertices=coords))
                    break
            if len(open_queue) == 1:
                closed_queue.append(open_queue.pop(i))
            if j == len(open_queue) - 1:
                closed_queue.append(open_queue.pop(i))

        # Need to do it twice to catch if the final two intersect?
        open_queue = closed_queue
        closed_queue = []
        while len(open_queue) > 0:
            i = 0
            j = 1
            for j in range(1, len(open_queue)):
                p1 = Polygon(open_queue[i].vertices[0:-1])
                p2 = Polygon(open_queue[j].vertices[0:-1])
                if p1.overlaps(p2):
                    p_new = unary_union([p1, p2])
                    coords = list(p_new.exterior.coords)
                    open_queue.pop(j)
                    open_queue.pop(i)
                    open_queue.append(Polygon_obj(len(coords), [self.x, self.y], None, vertices=coords))
                    break
            if len(open_queue) == 1:
                closed_queue.append(open_queue.pop(i))
            if j == len(open_queue) - 1:
                closed_queue.append(open_queue.pop(i))

        # Removes a polygon if it fully lies within another
        open_queue = closed_queue
        closed_queue = []
        while len(open_queue) > 0:
            for j in range(1, len(open_queue)):
                p1 = Polygon(open_queue[0].vertices[0:-1])
                p2 = Polygon(open_queue[j].vertices[0:-1])
                if p1.contains(p2):
                    open_queue.pop(j)
                    break
                elif p2.contains(p1):
                    open_queue.pop(0)
                    break
            if len(open_queue) == 1:
                closed_queue.append(open_queue.pop(0))
            if j == len(open_queue) - 1:
                closed_queue.append(open_queue.pop(0))

        return closed_queue
    
    def find_vertices(self, index_numbers):
        indexed_vertices = []
        for i in index_numbers:
            indexed_vertices.append(self.polygons[i].vertices)
        return indexed_vertices



if __name__ == '__main__':

    # Example usage
    polygons_generator = GeneratedPolygons(bounds=[(0, 10), (0, 10)])
    polygons_generator.make_polygons(number_of_polygons=5, max_vertices=3, max_size=3)

    # # Plot the original polygons
    # fig, ax = plt.subplots()
    # for i in range(len(polygons_generator.polygons)):
    #     x, y = list(zip(*polygons_generator.polygons[i].vertices))
    #     ax.plot(x, y, label=f'Polygon {i + 1}')
    ax = polygons_generator.plot_polygons()
    ax.set_xlim([0, 10])
    ax.set_ylim([0, 10])
    # plt.legend()
    plt.title('Original Polygons')
    plt.show()

    inside_vertices = polygons_generator.merge_polygons()

    for i in range(0, len(inside_vertices)):
        polygons_generator.polygons.append(inside_vertices[i])
    polygons_generator.plot_polygons()

    rectangle_generator = GeneratedPolygons(bounds=[(0, 10), (0, 10)])
    rectangle_generator.generate_non_overlapping_rectangles(5, 2)

    ax = rectangle_generator.plot_polygons()
    ax.set_xlim([0, 11])
    ax.set_ylim([0, 11])
    # plt.legend()
    plt.title('Original Polygons')
    plt.show()

    inside_vertices = rectangle_generator.merge_polygons()

    for i in range(0, len(inside_vertices)):
        rectangle_generator.polygons.append(inside_vertices[i])
    rectangle_generator.plot_polygons()
