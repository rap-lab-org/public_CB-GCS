from polygon_generator import Polygon_obj, GeneratedPolygons
from shapely import Point, Polygon
from copy import deepcopy
from linear_solver import *
import matplotlib.pyplot as plt
import math

def point_inside_polygon(point, polygon_vertices):
    # Check if a point is inside a polygon.

    # Parameters:
    # - point: Tuple (x, y) representing the point.
    # - polygon_vertices: List of tuples representing the vertices of the polygon.

    # Returns:
    # - True if the point is inside the polygon, False otherwise.

    # Create a Shapely Polygon object
    polygon = Polygon(polygon_vertices)

    # Create a Shapely Point object
    point = Point(point)

    # Check if the point is inside the polygon
    return point.within(polygon)

def point_on_edge(point, polygon_vertices):
    # Check if a point is inside a polygon.

    # Parameters:
    # - point: Tuple (x, y) representing the point.
    # - polygon_vertices: List of tuples representing the vertices of the polygon.

    # Returns:
    # - True if the point intersects the polygon edges, False otherwise.

    # Create a Shapely Polygon object
    polygon = Polygon(polygon_vertices)

    # Create a Shapely Point object
    point = Point(point)

    # Check if the point intersects the polygon edges
    return point.intersects(polygon)

def midpoint(point_1, point_2):
    middle = []
    for i in range(0, len(point_1)):
        temp = (point_1[i]+point_2[i])/2
        middle.append(temp)
    return middle

def split_list_at_corner_indices(original_list, corner_indices):
    result = []
    start_index = 0

    for index in corner_indices:
        if index > 0 and index < len(original_list) - 1:
            result.append(original_list[start_index:index + 1])
            start_index = index

    result.append(original_list[start_index:])
    return result


def split_list_at_interior_index(original_list, result, interior_index):
    final_result = []
    if type(interior_index) != int:
        return result
    indices = []
    temp_value = original_list[interior_index]

    for i, inner_list in enumerate(result):
        for j, element in enumerate(inner_list):
            if element == temp_value:
                indices.append(i)
                first_half = result[i][:j+1]
                second_half = result[i][j+1:]
    for index in indices:

        if 0 <= index < len(result):
            for i, inner_list in enumerate(result):
                if i != index:
                    final_result.append(inner_list)
    final_result.append(first_half)
    final_result.append(second_half)

    return final_result

def split_list_at_interior_indices(original_list, result, interior_indices):
    final_results = deepcopy(result)
    for i in range(0, len(interior_indices)):
        final_results = split_list_at_interior_index(original_list, final_results, interior_indices[i])
    return final_results

# middle = midpoint([line_1,intersections_1[i]], [line_1, intersections_1[i+1]])
def split_intersections(intersecting_indices, intersections, line, list_of_polygons, number):
    try:
        unique_polygons = list(set(item[0] for item in intersecting_indices))
    except Exception as e:
        unique_polygons = None

    split_indices = []
    is_corner = []
    not_corner = []
    for i in range(0, len(intersections)-1):
        print('i' ,i)
        # Shifts slightly to left/right to deal with vertical lines not causing breaks
        # print(f'Current Intersections {intersections[i]}, {intersections[i+1]}')
        if number == 0:
            middle = midpoint([line+.0000001, intersections[i]], [line+.0000001, intersections[i+1]])
            # corner_middle = midpoint([line-.0000001, intersections[i]], [line-.0000001, intersections[i+1]])
            p1_x = [line+.0000001, intersections[i]]
            p1_top = [line, intersections[i]+.000001]
            p1_bot = [line, intersections[i]-.000001]
            p2_x = [line+.0000001, intersections[i+1]]
            p2_top = [line, intersections[i+1]+.000001]
            p2_bot = [line, intersections[i+1]-.000001]
        elif number == 1:
            middle = midpoint([line-.0000001, intersections[i]], [line-.0000001, intersections[i+1]])
            # corner_middle = midpoint([line+.0000001, intersections[i]], [line+.0000001, intersections[i+1]])
            p1_x = [line-.0000001, intersections[i]]
            p1_top = [line, intersections[i]+.000001]
            p1_bot = [line, intersections[i]-.000001]
            p2_x = [line-.0000001, intersections[i+1]]
            p2_top = [line, intersections[i+1]+.000001]
            p2_bot = [line, intersections[i+1]-.000001]
            print(p1_x)
        if type(unique_polygons) != type(None):
            polygon_test = []
            corner_test = []
            
            for j in range(0, len(unique_polygons)):
                test = point_inside_polygon(middle, list_of_polygons[unique_polygons[j]].vertices)

                if test == True:
                    polygon_test.append(test)
            
            if any(polygon_test):
                print(f'This is a split case!')
                split_indices.append(i)
                not_corner.append(i+1)
            else:
                print('Not a split case!')
                for j in range(0, len(unique_polygons)):    
                    middle_c = midpoint([line, intersections[i]], [line, intersections[i+1]])
                    temp_edge = point_on_edge(middle_c, list_of_polygons[unique_polygons[j]].vertices)

                    if temp_edge == True:
                        print(f'Vertical Line, so not corner!')
                        not_corner.append(i)
                        # not_corner.append(i+1)
                        break
                    else:
                        print(f'Not vertical? {j}')
                        tp1 = []
                        tp2 = []
                        if point_on_edge([line, intersections[i]], list_of_polygons[unique_polygons[j]].vertices) == True:
                            print(f'point 1 lies on polygon {j}')
                            tp1_x =  point_inside_polygon(p1_x, list_of_polygons[unique_polygons[j]].vertices)
                            tp1_top = point_inside_polygon(p1_top, list_of_polygons[unique_polygons[j]].vertices)
                            tp1_bot = point_inside_polygon(p1_bot, list_of_polygons[unique_polygons[j]].vertices)
                            print(p1_x, p1_top, p1_bot)
                            tp1 = [tp1_x, tp1_top, tp1_bot]
                        elif point_on_edge([line, intersections[i+1]], list_of_polygons[unique_polygons[j]].vertices) == True:
                            print(f'point 2 lies on polygon {j}')
                            tp2_x =  point_inside_polygon(p2_x, list_of_polygons[unique_polygons[j]].vertices)
                            tp2_top = point_inside_polygon(p2_top, list_of_polygons[unique_polygons[j]].vertices)
                            tp2_bot = point_inside_polygon(p2_bot, list_of_polygons[unique_polygons[j]].vertices)
                            print(p2_x, p2_top, p2_bot)
                            tp2 = [tp2_x, tp2_top, tp2_bot]

                        print(f'Testing for point {tp1}, {tp2}')
                        if any(tp1) == True or any(tp2) == True:
                            print(f'Point not inside polygon at any point, so must be corner')
                            corner_test.append(True)
                            
            if len(corner_test)>0:
                if any(corner_test) == True:
                    is_corner.append(i)
                else:
                    print('Not a corner either!')
            else:
                print('Not a corner either!')
    print(f'corners {is_corner}')
    print(f'split indices {split_indices}')
    is_corner = list(filter(lambda a: a != 0, is_corner))
    for i in range(0, len(split_indices)):
        is_corner = list(filter(lambda a: a != split_indices[i], is_corner))
    for i in range(0, len(not_corner)):
        is_corner = list(filter(lambda a: a != not_corner[i], is_corner))
    is_corner = list(set(is_corner))
    print(f'corners {is_corner}')
    print(f'split indices {split_indices}')
    result = split_list_at_corner_indices(intersections, is_corner)
    final_result = split_list_at_interior_indices(intersections, result, split_indices)
    final_result = sort_points_by_distance(final_result)
    return final_result



def distance_from_origin(point):
    return math.sqrt(sum(x**2 for x in point))

def sort_points_by_distance(points_list):
    return sorted(points_list, key=distance_from_origin)

def prune_singles(s1, s2, line_1, line_2, list_of_polygons, unique_polygons):
    # Some singles are extraneous and correspond to a point
    #  with no free area surrounding it. Need to remove this
    # If its a single point and we shift towards the center of
    # our bounding lines we can eliminate them

    s1_pop = []
    for i in range(0, len(s1)):
        if len(s1[i]) == 1:
            test_interior = []
            for j in range(0, len(unique_polygons)):
                temp = point_inside_polygon((line_1+0.0000001, s1[i][0]), list_of_polygons[unique_polygons[j]].vertices)
                
                test_interior.append(temp)
            if any(test_interior) == True:
                s1_pop.append(i)
    s1_pop.sort(reverse=True)
    for index in s1_pop:
        s1.pop(index)

    s2_pop = []
    for i in range(0, len(s2)):
        if len(s2[i]) == 1:
            test_interior = []
            for j in range(0, len(unique_polygons)):
                temp = point_inside_polygon((line_2-0.0000001, s2[i][0]), list_of_polygons[unique_polygons[j]].vertices)
                test_interior.append(temp)
            if any(test_interior) == True:
                s2_pop.append(i)
    s2_pop.sort(reverse=True)
    for index in s2_pop:
        s2.pop(index)

    return s1, s2

def convex_set(split_line_1, split_line_2, line_1, line_2, intersecting_indices, line_number):
    # Given a split of lines, split free space into convex polygons
    # Return a RandomConvexPolygon object
    polygons = []
    length_1 = len(split_line_1)
    length_2 = len(split_line_2)
    # unique_polygons_1 = list(set(item[0] for item in intersecting_indices[line_number]))
    # unique_polygons_2 = list(set(item[0] for item in intersecting_indices[line_number+1]))
    # unique_polygons_1.extend(unique_polygons_2)
    # unique_polygons = list(set(unique_polygons_1))
    if length_1 == length_2:

        for i in range(0, length_1):
            left_line = [[line_1, entry] for entry in split_line_1[i]]
            right_line = [[line_2, entry] for entry in split_line_2[i]][::-1]
            left_line.extend(right_line)
            print(f'left line {left_line}')
            polygons.append(RandomConvexPolygon(length_1*2, vertices=left_line))

    else:
        print('Not Even??', split_line_1, split_line_2)
    return polygons


def test_sample_polygons(test_polygon, unique_polygons):
    """Tests a test polygon over a set of unique polygons.
       Returns True if there are no overlaps"""
    verts = len(test_polygon)
    test_poly = RandomConvexPolygon(verts, vertices=test_polygon)
    p1 = Polygon(test_poly.vertices)
    for i in range(0, len(unique_polygons)):
        p2 = Polygon(unique_polygons[i].vertices)
        any_overlaps = p1.overlaps(p2)
        if p1.overlaps(p2) == True:
            break
    if any_overlaps == True:
        return False
    else:
        return True


if __name__ == 'main':
    ranges = [(0, 10),(0, 10)]
    number_of_polygons = 4
    max_vertices = 4
    max_size = 5
    polygons = GeneratedPolygons(ranges)
    polygons.make_polygons(number_of_polygons, max_vertices, max_size)
    print('Done!')
    fig, ax = plt.subplots()
    ax = polygons.plot_polygons(ax=ax)

    # Merged Polygons Graph
    inside_vertices = polygons.merge_polygons()

    for i in range(0, len(inside_vertices)):
        polygons.polygons.append(inside_vertices[i])

    fig2, ax2 = plt.subplots()
    ax2 = polygons.plot_polygons(ax=ax2)

    plt.show()

    # Graph of Merged Polygons and vert/horizontal lines
    fig2, ax2 = plt.subplots()
    ax2 = polygons.plot_polygons(ax=ax2)
    list_of_polygons = polygons.polygons
    lines = get_x_values(list_of_polygons)

    for line in lines:
            ax2.plot([line, line], [0,10+1])

    #plot x_lines
    ax2.plot([ranges[0][0], ranges[0][1]],  [ranges[0][1]+1, ranges[0][1]+1])
    ax2.plot([ranges[0][0], ranges[0][1]],  [ranges[1][1]+1, ranges[1][1]+1])

    #plot y_lines
    ax2.plot([ranges[0][0], ranges[0][0]],  [ranges[0][1]+1, ranges[1][1]+1])
    ax2.plot([ranges[0][1], ranges[0][1]],  [ranges[0][1]+1, ranges[1][1]+1])

    plt.show()

    # Sort vertical lines and pair them
    # including the start and end bounds
    start = ranges[0][0]
    end = ranges[0][1]+1
    real_lines = sorted(lines)


    all_lines, line_pairs = find_pairs(real_lines, start, end)

    # Now that we have lines, we need to find where 
    # the lines intersect with the polygons, intersections
    intersections = []
    # intersecting_indices keeps track of which 
    # polygon and line equations intersect
    intersecting_indices = []

    for i in range(0, len(all_lines)):
        temp_indices = check_multiline_bounds(all_lines[i], list_of_polygons)
        temp_intersections = find_intersections(all_lines[i], temp_indices, list_of_polygons)
        temp_intersections.append(start)
        temp_intersections.append(end)
        temp_intersections = sorted(set(temp_intersections))
        intersecting_indices.append(temp_indices)
        intersections.append(temp_intersections)

    # Now that we have a list of intersections we need to group them
    # giving us our first iterations of our final groups
    final_polygons = []
    for k in range(0, len(real_lines)-1):
        unique_polygons = list(set(item[0] for item in intersecting_indices[k]))
        unique_polygons_2 = list(set(item[0] for item in intersecting_indices[k]))
        unique_polygons.extend(unique_polygons_2)
        unique_polygons = list(set(unique_polygons))

        intersections_1 = sorted(list(set(intersections[k])))
        intersections_2 = sorted(list(set(intersections[k+1])))
        line_1 = all_lines[k]
        line_2 = all_lines[k+1]
        s1 = split_intersections(intersecting_indices[k], intersections_1, line_1, list_of_polygons, 0)
        s2 = split_intersections(intersecting_indices[k+1], intersections_2, line_2, list_of_polygons, 1)
        print(s1)
        print(s2)
        s1, s2 = prune_singles(s1, s2, line_1, line_2, list_of_polygons, unique_polygons)
        print(s1)
        print(s2)
        temp_polygons = convex_set(s1, s2, line_1, line_2, intersecting_indices, k)
        print('temp polys', temp_polygons)
        final_polygons.extend(temp_polygons)

    final_sets = GeneratedPolygons(ranges)
    final_sets.polygons = final_polygons
    final_sets.plot_polygons()