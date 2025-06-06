from relax_GCS import shortest_path_convex_sets
# from relax_GCS_Modified import shortest_path_convex_sets
from generate_timesteps_convex_sets import create_dynamic_polygons, create_timed_obstacle_list, find_centroid

import networkx as nx
import numpy as np
import gurobipy as gp
from gurobipy import GRB
from copy import deepcopy

def extractEdgeFlows(fract_sol):
    flows = {}
    for edges in fract_sol:
        flows[edges] = fract_sol[edges][2]
    return flows

def greedy_edge_selector(edges_used):
    """Selects the highest flow edges from a fractional solution

    Args:
        edges_used: Dictionary of all edges with their flow values

    Returns:
        edge_paths: List of edges with maximum flow
    """
    edge_data = extractEdgeFlows(edges_used)
    result = {}

    for key, value in edge_data.items():
        if key[0] not in result:
            result[key[0]] = {key[1]: value}
        else:
            result[key[0]][key[1]] = value

    max_keys = {}
    for outer_key, inner_dict in result.items():
        inner_max = max(inner_dict.values())
        max_key = None
        for k, v in inner_dict.items():
            if v == inner_max:
                max_key = k
                break  # Stop iterating as soon as the first key with max value is found
        max_keys[outer_key] = max_key

    final_node = max([max(i,j) for i, j in edges_used])
    outer_key = 0
    inner_key = 0

    edge_paths = []

    while inner_key<final_node:
        inner_key = max_keys[outer_key]
        edge_paths.append([outer_key, inner_key])
        outer_key = inner_key

    return edge_paths

def max_greedy_edge_selector(edges_used):
    """Finds the maximum path length and returns its edges

    Args:
        edges_used: Dictionary of all edges with their flow values

    Returns:
        edge_paths_max: List of edges with maximum path length
    """
    nodes = list(edges_used.keys())
    nodes = list(set([item for sublist in nodes for item in sublist]))

    G_max_greed = nx.DiGraph()
    G_max_greed.add_nodes_from(nodes)
    edges = extractEdgeFlows(edges_used)

    for i, j in edges:
        temp_weight=edges[i,j]
        G_max_greed.add_edge(i,j, weight = temp_weight)

    longest_path = nx.dag_longest_path(G_max_greed, weight='weight')
    #max_weight = nx.dag_longest_path_length(G_max_greed, weight='weight')

    edge_paths_max = [[longest_path[i], longest_path[i+1]] for i in range(len(longest_path)-1)]

    return edge_paths_max

def random_edge_selector(edges_used):

    def get_random_edge(candidate_edges, flows):
        probabilities = np.array(flows)/sum(np.array(flows))
        return candidate_edges[np.random.choice(len(candidate_edges), p=probabilities)]

    edge_data = extractEdgeFlows(edges_used)
    result = {}

    for key, value in edge_data.items():
        if key[0] not in result:
            result[key[0]] = {key[1]: value}
        else:
            result[key[0]][key[1]] = value

    max_keys = {}
    for outer_key, inner_dict in result.items():
        #inner_max = max(inner_dict.values())
        max_key = None
        candidate_edges = []
        flows = []
        for k, v in inner_dict.items():
            candidate_edges.append(k)
            flows.append(v)
        flows = np.abs(flows)
        selected_edge = get_random_edge(candidate_edges, flows)
        max_keys[outer_key] = selected_edge

    final_node = max([max(i,j) for i, j in edges_used])
    outer_key = 0
    inner_key = 0

    edge_paths = []

    while inner_key<final_node:
        inner_key = max_keys[outer_key]
        edge_paths.append([outer_key, inner_key])
        outer_key = inner_key

    return edge_paths

def graph_reconstruction(G, edge_paths):
    """_summary_

    Args:
        G (networkx graph): _description_
        edge_paths (_type_): _description_

    Returns:
        _type_: _description_
    """
    G_new = deepcopy(G)
    keeper_nodes = list(set([item for sublist in edge_paths for item in sublist]))
    nodes_to_remove = [node for node in G_new.nodes() if node not in keeper_nodes]
    G_new.remove_nodes_from(nodes_to_remove)
    return G_new

def solve_fractional_GCS(v_0, G, T):

    """
    Summary: Feb 11, 2024
    Formulate and solve the GCS problem on the multilayer graph given
    a set of equations stating the bounds of each convex set
    """
    start_node = 0
    end_node = max(list(G.nodes))

    env = gp.Env(empty=True)
    env.setParam("TimeLimit", 300)
    env.setParam("LogToConsole", 0)
    env.setParam("LogFile", "./gurobi.log")
    env.start()
    m = gp.Model(env=env) # Initiate model.

    # Define the decision variables.
    x = gp.tupledict()
    y = gp.tupledict()
    l = gp.tupledict()
    l_x = gp.tupledict()
    l_y = gp.tupledict()

    for u in G.nodes:
        x[u] = m.addVar(lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS)
        y[u] = m.addVar(lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS)

    for (u,v) in G.edges:
        l[u,v] = m.addVar(vtype=GRB.CONTINUOUS)
        l_x[u,v] = m.addVar(lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS)
        l_y[u,v] = m.addVar(lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS)

    # Define the objective.

    sum_dist = gp.quicksum(l[u,v] for (u,v) in G.edges)
    m.setObjective(sum_dist, GRB.MINIMIZE)

    # Define the constraints.

    for (u,v) in G.edges:
        m.addConstr(l_x[u,v]==(x[v]-x[u]))
        m.addConstr(l_y[u,v]==(y[v]-y[u]))
        m.addConstr(l_x[u,v]**2 + l_y[u,v]**2 <= l[u,v]**2)

    for layer, (u,v) in enumerate(G.edges):
        m.addConstr(l[u,v] <= v_0*(T[layer+1]-T[layer]))\

    for u in G.nodes:

        if u == start_node:
            s_x, s_y = G.nodes[start_node]['Convex Set']
            m.addConstr(x[u] == s_x)
            m.addConstr(y[u] == s_y)
        elif u == end_node:
            d_x, d_y = G.nodes[end_node]['Convex Set']
            m.addConstr(x[u] == d_x)
            m.addConstr(y[u] == d_y)
        else:
            for (a,b,c), bounds in G.nodes[u]['Convex Set'].equations:
                m.addConstr(x[u]*a + y[u]*b + c <= 0)

    # Define the parameters and optimize.
    
    # m.Params.NonConvex = 2
    #m.Params.TimeLimit = 300
    # m.Params.LogToConsole = False
    # m.Params.DualReductions = 0
    m.optimize()

    if m.status != GRB.OPTIMAL:
        return {}, m,  np.inf
    
    points_selected = [(x[u].x,y[u].x) for u in G.nodes]
    
    return points_selected, m, m.objVal
