
# Authors: Allen and Richard.

# Incorporate GCS into continuous MAPF.

import numpy as np
import networkx as nx
import gurobipy as gp
from gurobipy import GRB
from generate_timesteps_convex_sets import *

def gcs_eqns_solver(v_0, G, T, is_binary, minT: bool=False, timeLimit: float=300,
                    gap: float=0.05):

    """
    Summary: Feb 11, 2024
    Formulate and solve the GCS problem on the multilayer graph given
    a set of equations stating the bounds of each convex set
    """
    start_node = 0
    end_node = len(G)-1
    # print(G.nodes)
    # print(G.edges)
    env = gp.Env(empty=True)
    if gap > 0:
        env.setParam("MIPGap", gap)
    # env.setParam("OutputFlag", 0)
    env.setParam("TimeLimit", timeLimit)
    env.setParam("LogToConsole", 0)
    env.setParam("LogFile", "./gurobi.log")
    env.start()
    m = gp.Model(env=env) # Initiate model.

    # Define the parameters and optimize.
    
    # m.Params.NonConvex = 2
    # m.setParam("LogToConsole", 0)
    # m.Params.DualReductions = 0

    # Define the decision variables.

    y = gp.tupledict()
    z_x = gp.tupledict()
    z_y = gp.tupledict()
    z_x_p = gp.tupledict()
    z_y_p = gp.tupledict()
    l = gp.tupledict()
    l_x = gp.tupledict()
    l_y = gp.tupledict()

    for (u,v) in G.edges:
        if is_binary:
            y[u,v] = m.addVar(lb=0,ub=1,vtype=GRB.BINARY, name=f'{u}_{v}_binary')
        else:
            y[u,v] = m.addVar(lb=0,ub=1,vtype=GRB.CONTINUOUS)
        z_x[u,v] = m.addVar(lb=-GRB.INFINITY,ub=GRB.INFINITY,vtype=GRB.CONTINUOUS)
        z_y[u,v] = m.addVar(lb=-GRB.INFINITY,ub=GRB.INFINITY,vtype=GRB.CONTINUOUS)
        z_x_p[u,v] = m.addVar(lb=-GRB.INFINITY,ub=GRB.INFINITY,vtype=GRB.CONTINUOUS)
        z_y_p[u,v] = m.addVar(lb=-GRB.INFINITY,ub=GRB.INFINITY,vtype=GRB.CONTINUOUS)
        l[u,v] = m.addVar(lb=0,ub=GRB.INFINITY,vtype=GRB.CONTINUOUS)
        l_x[u,v] = m.addVar(lb=-GRB.INFINITY,ub=GRB.INFINITY,vtype=GRB.CONTINUOUS)
        l_y[u,v] = m.addVar(lb=-GRB.INFINITY,ub=GRB.INFINITY,vtype=GRB.CONTINUOUS)


    sum_dist = gp.quicksum(l[u,v] for (u,v) in G.edges)
    if minT:
        tx, ty = G.nodes[end_node]['Convex Set']
        p = gp.tupledict()
        px = gp.tupledict()
        py = gp.tupledict()
        for (u, v) in G.edges:
            p[u,v] = m.addVar(lb=0,ub=GRB.INFINITY,vtype=GRB.CONTINUOUS)
            px[u,v] = m.addVar(lb=-GRB.INFINITY,ub=GRB.INFINITY,vtype=GRB.CONTINUOUS)
            py[u,v] = m.addVar(lb=-GRB.INFINITY,ub=GRB.INFINITY,vtype=GRB.CONTINUOUS)
        for (u, v) in G.edges:
            m.addConstr(px[u,v]==(tx-z_x[u,v]), name = f'{u}_{v}_x_penalty')
            m.addConstr(py[u,v]==(ty-z_y[u,v]), name = f'{u}_{v}_y_penalty')
            m.addConstr(px[u,v]**2 + py[u,v]**2 <= p[u,v]**2, name = f'{u}_{v}_square_penalty')
        sum_penalty = gp.quicksum(p[u, v] for (u, v) in G.edges)
        m.setObjective(sum_dist + sum_penalty * 1e-3 / len(T), GRB.MINIMIZE)
    else:
        # Define the objective.
        m.setObjective(sum_dist, GRB.MINIMIZE)

    # Define the constraints.

    for (u,v) in G.edges:
        m.addConstr(l_x[u,v]==(z_x_p[u,v]-z_x[u,v]), name = f'{u}_{v}_x_constr')
        m.addConstr(l_y[u,v]==(z_y_p[u,v]-z_y[u,v]), name = f'{u}_{v}_y_constr')
        m.addConstr(l_x[u,v]**2 + l_y[u,v]**2 <= l[u,v]**2, name = f'{u}_{v}_square_constr')

    flow_out_s = gp.quicksum(y[start_node, v] for v in G.successors(start_node))
    flow_in_d = gp.quicksum(y[v, end_node] for v in G.predecessors(end_node))
    m.addConstr(flow_out_s == 1, name=f'start_in_out_flow')
    m.addConstr(flow_in_d == 1, name=f'end_in_out_flow')

    for u in G.nodes:
        if u not in [start_node, end_node]:
            flow_out_u = gp.quicksum(y[u,v] for v in G.successors(u))
            flow_in_u = gp.quicksum(y[v,u] for v in G.predecessors(u))
            sum_x_out = gp.quicksum(z_x[u,v] for v in G.successors(u))
            sum_y_out = gp.quicksum(z_y[u,v] for v in G.successors(u))
            sum_x_in = gp.quicksum(z_x_p[v,u] for v in G.predecessors(u))
            sum_y_in = gp.quicksum(z_y_p[v,u] for v in G.predecessors(u))
            # m.addConstr(flow_out_u <= 1, name=f'{u}_out_flow')
            m.addConstr(flow_out_u == flow_in_u, name=f'{u}_in_out_flow')
            m.addConstr(sum_x_out == sum_x_in, name=f'{u}_x_pos')
            m.addConstr(sum_y_out == sum_y_in, name=f'{u}_y_pos')
        
    for i in range(1, len(T)-2):
        
        # finds all nodes, u, in a layer
        current_layer = []
        for j in G.nodes:
            if G.nodes[j]['Layer'] == T[i]:
                current_layer.append(j)

        dist_traversed = gp.quicksum(l[u,v] for u in current_layer for v in G.successors(u)) # if v != "d"
        max_traversible_dist = v_0*(T[i+1]-T[i])
        # print(max_traversible_dist, T[i+1], T[i])
        m.addConstr(dist_traversed <= max_traversible_dist, name=f'{T[i]}_speed')

    dist_from_s = gp.quicksum(l[start_node,v] for v in G.successors(start_node)) # if v != "d"
    max_dist_from_s = v_0*(T[1]-T[0])
    # print(max_dist_from_s, T[1], T[0])
    m.addConstr(dist_from_s <= max_dist_from_s, name=f'start_speed')


    dist_to_d = gp.quicksum(l[u,end_node] for u in G.predecessors(end_node))
    max_dist_to_d = v_0*(T[-1]-T[-2])
    # print(max_dist_to_d, T[-1], T[-2])
    m.addConstr(dist_to_d <= max_dist_to_d, name=f'end_speed')

    for (u,v) in G.edges:

        if u == start_node:
            s_x, s_y = G.nodes[start_node]['Convex Set']
            m.addConstr(z_x[u,v] - y[u,v]*s_x == 0)
            m.addConstr(z_y[u,v] - y[u,v]*s_y == 0)
        else:
            for (a,b,c), bounds in G.nodes[u]['Convex Set'].equations:
                m.addConstr(z_x[u,v]*a + z_y[u,v]*b + y[u,v]*c <= 0)

        if v == end_node:
            d_x, d_y = G.nodes[end_node]['Convex Set']
            m.addConstr(z_x_p[u,v] - y[u,v]*d_x == 0)
            m.addConstr(z_y_p[u,v] - y[u,v]*d_y == 0)
        else:
            for (a,b,c), bounds in G.nodes[v]['Convex Set'].equations:
                m.addConstr(z_x_p[u,v]*a + z_y_p[u,v]*b + y[u,v]*c <= 0)
    m.optimize()

    if m.SolCount == 0:
        return {}, np.inf, m

    # Retrieve the solution.
    
    edges_used = {(u,v):(G.nodes[u],G.nodes[v],y[u,v].x,z_x[u,v].x,z_y[u,v].x,z_x_p[u,v].x,z_y_p[u,v].x,l[u,v].x) 
                for (u,v) in G.edges}
    # print("sol =", edges_used)
    
    return edges_used, m.objVal, m

def shortest_path_convex_sets(start, end, speed, bounds, timed_obstacles, timesteps, is_binary = False):
    """Finds either a binary or fractional solution to the gcs problem.

    Args:
        start (tuple(float, float)): start point
        end (tuple(float, float)): end point
        speed (float)): speed of agent
        bounds ([tuple(float, float), tuple(float, float)]): bounds of area to evaluate
        timed_obstacles (list of GeneratedPolygon's): List of GeneratedPolygon objects corresponding to each timestep stating the restricted areas
        timesteps (list(float)): List of timesteps to evaluate
        is_binary (bool, optional): If set to true returns a binary flow solution, otherwise a fractional solution. Defaults to False.

    Returns:
        edges_used : list of all edges with their flow values
        obj_value : Value given for the solution cost
        m : Gurobi Model
        G : Graph the model solves over
    """
    max_size = None
    rectangular_sets = []
    for i in range(0, len(timed_obstacles)):
        #Create convex sets for every timestep
        list_of_rectangles = timed_obstacles[i].polygons
        lines = get_x_values(list_of_rectangles)
    
        post_merge_sets = generate_rectangluar_sets(bounds, lines, list_of_rectangles, max_size)
        rectangular_sets.append(post_merge_sets)
    
    graph_nodes, graph_edges = find_graph_vertices_edges(start, end, speed, rectangular_sets, timesteps)

    G = nx.DiGraph()
    G.add_node(0)
    nx.set_node_attributes(G, {0: timesteps[0]}, 'Layer')
    nx.set_node_attributes(G, {0: start}, 'Convex Set')
    node_numbers = deepcopy(graph_nodes)
    for i in range(1, len(graph_nodes)):
        for j in range(0, len(graph_nodes[i])):
            current_node = len(G)
            G.add_node(current_node)
            node_numbers[i][j] = current_node
            nx.set_node_attributes(G, {current_node: timesteps[i]}, 'Layer')
            convex_set = rectangular_sets[i].polygons[graph_nodes[i][j]]
            nx.set_node_attributes(G, {current_node: convex_set}, 'Convex Set')

    #for a given layer
    for i in range(0, len(graph_edges)):
        #then a given node
        for j in range(0, len(graph_edges[i])):
            old_node1_name = graph_edges[i][j][0]
            temp_index1 = graph_nodes[i].index(old_node1_name)
            node1 = node_numbers[i][temp_index1]

            for k in range(0, len(graph_edges[i][j][1])):
                
                old_node2_name = graph_edges[i][j][1][k]
                temp_index2 = graph_nodes[i+1].index(old_node2_name)
                node2 = node_numbers[i+1][temp_index2]

                G.add_edge(node1, node2)

    for i in range(0, len(G)):
                if G.nodes[i]['Layer'] == timesteps[-1]:
                    G.nodes[i]['Convex Set'] = end

    edges_used, obj_value, m = gcs_eqns_solver(speed, G, timesteps, is_binary)

    return edges_used, obj_value, m, G
