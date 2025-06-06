def get_x_values(list_of_polygons):
    # Takes in the GeneratedPolygons objects and gives
    # all UNIQUE x values needed to draw vertical lines.
    lines = []
    for object in list_of_polygons:
        for vertex in object.vertices:
            # print(vertex ,vertex[0])
            lines.append(vertex[0])

    return list(set(lines))

#First find x values needed, then check lines
# set of bounds is polygons.polygons[i].equations[j]
def check_line_bounds(line, set_of_bounds):
    # Takes in a value for a vertical line i.e. 5
    # and a set of x,y bounds of the form [(x1, y1), (x2, y2)]
    # then outputs whether the given vertical line intersects the
    # polygons edge.
    # print(set_of_bounds)
    x1 = min(set_of_bounds[0][0], set_of_bounds[1][0])
    x2 = max(set_of_bounds[0][0], set_of_bounds[1][0])
    if line >= x1 and line <=  x2:
        return True
    else:
        return False
    
def check_multiline_bounds(line, list_of_polygons):
    # Takes in a line value as well as all sets of bounds
    # to determine which polygon edges intersect the line.
    # of the form [polygon_index, equation_index]

    intersecting_indices = []
    for i in range(0, len(list_of_polygons)):
        equations = list_of_polygons[i].equations
        for j in range(0, len(equations)):
            # print(equations[j][1])
            check = check_line_bounds(line, equations[j][1])
            # print(equations[j][1])
            if check == True:
                intersecting_indices.append([i,j])
    
    return intersecting_indices

def unique_within_tolerance(list, tolerance=1e-6):
    unique_values = set()
    result = []

    for num in list:
        # Check if a similar value is already in the set within tolerance
        is_unique = all(abs(num - existing) > tolerance for existing in unique_values)

        if is_unique:
            unique_values.add(num)
            result.append(num)

    return result

def find_intersections(line, intersecting_indices, list_of_polygons):
    # Takes in a line value and the indices to determine y
    # values of intersections to determine bounds of the
    # convex set
    x = line
    intersections = []
    for i, j in intersecting_indices:
        a, b, c = list_of_polygons[i].equations[j][0]
        if b == 0:
            y1 = list_of_polygons[i].equations[j][1][0][1]
            y2 = list_of_polygons[i].equations[j][1][1][1]
            intersections.append(y1)
            intersections.append(y2)
            # print(y1, y2)
        else:
            y=-a*x/b-c/b
            intersections.append(y)
            # print(y)
    return unique_within_tolerance(intersections)

def plot_vline(ax, x_value, y1, y2):
    # Takes in x values and bounds
    # and plots all vertical lines
    
    ax.plot([x_value, x_value], [y1, y2])

def find_pairs(lines, start, end):
    # Given a set of SORTED lines, return all
    # vertical lines in a sorted list including start/end
    # as well as all pairs of lines
    all_lines = lines
    all_lines.append(start)
    all_lines.append(end)
    all_lines = sorted(list(set(all_lines)))

    line_pairs = []
    for i in range(0, len(all_lines)-1):
        temp = [all_lines[i], all_lines[i+1]]
        line_pairs.append(temp)

    return all_lines, line_pairs