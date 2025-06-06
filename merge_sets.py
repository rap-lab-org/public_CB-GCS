from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import get_coordinates
from polygon_generator import Polygon_obj
from copy import deepcopy

def vertical_merge_polygon(polygon_set):
    """Given a list of free space polygons. Iterate through all polygons
    vertically to see if any can be merged."""

    set1 = polygon_set

    open_queue_1 = deepcopy(set1)
    # print(open_queue_1)
    closed_queue_1 = []

    i = 0
    while len(open_queue_1)>1:
        # print(i)
        # print('Queue length', len(open_queue_1))
        for i in range(0, len(open_queue_1)-1): 
            # print('Current Queue', open_queue_1)
            #list_of_rectangles[unique_rectangles_1[j]].vertices
            p1 = Polygon(open_queue_1[i].vertices)
            p2 = Polygon(open_queue_1[i+1].vertices)
            if p1.intersects(p2):
                union_result = unary_union([p1, p2])

                # Calculate the convex hull of the unary union
                convex_hull_union = union_result.convex_hull

                # Check if the convex hull is equal to the unary union
                if convex_hull_union.equals(union_result):
                    # print('They do intersect!')
                    exterior_coords = get_coordinates(union_result.exterior)
                    temp_poly = Polygon(exterior_coords)
                    check = []
                    new_poly = get_coordinates(union_result.exterior)
                    open_queue_1.pop(i+1)
                    open_queue_1.pop(i)
                    open_queue_1.insert(i, Polygon_obj(len(new_poly), [(0,0),(0,0)], 2, vertices = new_poly))
                    break
                else:
                    # print('Polygons do not intersect, remove Polygon')
                    closed_queue_1.append(open_queue_1[i])
                    open_queue_1.pop(i)
                    break
    closed_queue_1.append(open_queue_1[0])
    open_queue_1.pop(0)

    return closed_queue_1

def horizontal_merge_polygon(polygon_sets):

    set1 = polygon_sets[0]
    set2 = polygon_sets[1]

    size1 = len(set1)
    size2 = len(set2)
    open_queue_1 = deepcopy(set1)
    open_queue_2 = deepcopy(set2)
    # print(open_queue_1)
    closed_queue_1 = []
    i = 0
    j = 0
    while i < size1-1 and j < size2-1:
        size1 = len(open_queue_1)
        size2 = len(open_queue_2)
        # print('while loop', i,j, size1, size2)

        for i in range(0, size1):
            size1 = len(open_queue_1)
            size2 = len(open_queue_2)
            # print('for loop i', i,j, size1, size2)
            
            for j in range(0, size2):
                size1 = len(open_queue_1)
                size2 = len(open_queue_2)
                # print(i,j, size1, size2)
                p1 = Polygon(open_queue_1[i].vertices)
                p2 = Polygon(open_queue_2[j].vertices)
                if p1.intersects(p2):
                    union_result = unary_union([p1, p2])

                    # Calculate the convex hull of the unary union
                    convex_hull_union = union_result.convex_hull

                    # Check if the convex hull is equal to the unary union
                    jaccard_similarity = convex_hull_union.intersection(union_result).area / convex_hull_union.union(union_result).area

                    # Define a threshold based on your needs
                    threshold = 0.99  # Adjust as needed

                    if jaccard_similarity > threshold:

                        new_poly = get_coordinates(union_result.exterior)
                        open_queue_1.pop(i)
                        open_queue_2.pop(j)
                        open_queue_2.insert(j, Polygon_obj(len(new_poly), [(0,0),(0,0)], 2, vertices = new_poly))
                        break
            else:
                # Continue if the inner loop wasn't broken.
                continue
            # Inner loop was broken, break the outer.
            break
    return open_queue_1, open_queue_2
