from polygon_generator import Polygon_obj, GeneratedPolygons
from linear_solver import *
import matplotlib.pyplot as plt
from merge_sets import *
from shapely import Point, Polygon

def point_on_any_edge(point_1, point_2, list_of_polygons, unique_polygons):
    
    any_edge = []
    point = Point([(point_1[0] + point_2[0])/2,(point_1[1] + point_2[1])/2])
    for j in range(0, len(unique_polygons)):
        temp_poly = Polygon(list_of_polygons[unique_polygons[j]].vertices)
        if temp_poly.boundary.contains(point):
            any_edge.append(True)
    if len(any_edge)>0:
        return True
    else:
        return False


def generate_rectangluar_sets(ranges, lines, list_of_rectangles, max_size):
    # Sort vertical lines and pair them
    start = ranges[0][0]
    end = ranges[0][1] + 1
    real_lines = sorted(lines)

    # Find all lines and their paired intersections
    all_lines, line_pairs = find_pairs(real_lines, start, end)

    # Find intersections of lines with polygons
    intersections = []
    intersecting_indices = []
    no_start_intersections = []
    # Iterate through all lines to find intersections
    for i in range(len(all_lines)):
        temp_indices = check_multiline_bounds(all_lines[i], list_of_rectangles)
        temp_intersections = find_intersections(all_lines[i], temp_indices, list_of_rectangles)
        no_start_intersections.append(sorted(set(temp_intersections)))
        temp_intersections.append(start)
        temp_intersections.append(end)
        temp_intersections = sorted(set(temp_intersections))
        intersecting_indices.append(temp_indices)
        intersections.append(temp_intersections)

    # Initialize a list to store pre-merged polygons
    pre_merged_polygons = []
    # Iterate through lines to list intersecting polygons for each line

    for k in range(len(all_lines) - 1):
        unique_polygons = list(set(item[0] for item in intersecting_indices[k]))
        unique_polygons_2 = list(set(item[0] for item in intersecting_indices[k]))
        unique_polygons.extend(unique_polygons_2)
        unique_polygons = list(set(unique_polygons))

        line_1 = all_lines[k]
        line_2 = all_lines[k+1]
        intersections_1 = sorted(list(set(intersections[k])))
        intersections_2 = sorted(list(set(intersections[k + 1])))
        pairs = []
        if len(no_start_intersections[k]) == 0 or len(no_start_intersections[k+1]) == 0:
            bot_touch1 = 1
            bot_touch2 = 2
        else:
            bot_touch1 = no_start_intersections[k][0]
            bot_touch2 = no_start_intersections[k+1][0]

        if bot_touch1 == bot_touch2 == 0:
            rect_touches_bot = True
            start_point = 1
        else:
            start_point = 0

        # if list_of_rectangles[0].vertices[0][1]
        i = 0
        j = start_point

        for i in range(start_point, len(intersections_1)):
            current_j = start_point
            for j in range(current_j, len(intersections_2)):
                if intersections_1[i] == intersections_2[j]:
                    if intersections_1[i] == start \
                    or intersections_1[i] == end:
                        # print('Top or bot', i, j)
                        pairs.append([i,j])
                        current_j = j
                        break
                    else:
                        test = point_on_any_edge((line_1,intersections_1[i]), (line_2, intersections_2[j]),
                                                    list_of_rectangles, unique_polygons)
                        if test == True:
                            pairs.append([i,j])
                            current_j = j
                            break
        
        # Now that we have the break points we can split them into rectangles (2 points from either list)
        rectangles = []
        i = 0
        while i<= len(pairs)-2:
            rectangle = [(line_1, intersections_1[pairs[i][0]]), (line_1, intersections_1[pairs[i+1][0]]),
                        (line_2, intersections_2[pairs[i+1][1]]), (line_2, intersections_2[pairs[i][1]])]
            
            i+=2
            rectangle_obj = Polygon_obj(len(rectangle), ranges, max_size, vertices=rectangle)
            rectangles.append(rectangle_obj)
        
        pre_merged_polygons.append(rectangles)

    flat_polys = [x for xs in pre_merged_polygons for x in xs]

    # Initialize an object to store pre-merge sets
    pre_merge_sets = GeneratedPolygons(ranges)
    pre_merge_sets.polygons = flat_polys

    # Merge adjacent polygons horizontally
    for i in range(len(pre_merged_polygons) - 1):
        pre_merged_polygons[i], pre_merged_polygons[i + 1] = horizontal_merge_polygon(
            [pre_merged_polygons[i], pre_merged_polygons[i + 1]])

    # Flatten the list of pre-merged polygons
    flat_polys = [x for xs in pre_merged_polygons for x in xs]

    # Initialize an object to store post-merge sets
    post_merge_sets = GeneratedPolygons(ranges)
    post_merge_sets.polygons = flat_polys

    return post_merge_sets
