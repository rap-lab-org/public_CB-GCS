from typing import Any, NamedTuple
from bisect import bisect_right
import numpy as np
from gurobipy import GRB
import time
import networkx as nx
from copy import deepcopy
from polygon_generator import Polygon_obj
import sys
import os

_current = os.path.dirname(os.path.realpath(__file__))
# directory reach
_parent = os.path.dirname(_current)
if _parent not in sys.path:
    sys.path.append(_parent)
if _current not in sys.path:
    sys.path.append(_current)
import cbs.type_stubs as TP
import cbs.utils as ut
from cbs.utils import CbsConstraint, GCSSol, Env
import relax_GCS as gcs
import cbs.gcs_graph as gra
from polygon_generator import GeneratedPolygons, Polygon_obj


def gen_static_poly_obstacles(
    env: Env,
) -> gcs.GeneratedPolygons:
    res = gcs.GeneratedPolygons([env.xbound(), env.ybound()])
    for obs in env.inflated_obsts:
        polyObj = Polygon_obj(len(obs), (env.xbound(), env.ybound()), np.inf, obs)
        res.polygons.append(polyObj)
    return res


def gen_timed_poly_obstacles(
    cstrs: list[CbsConstraint],
    static_obsts: GeneratedPolygons,
    xbound: tuple[float, float],
    ybound: tuple[float, float],
    T: TP.TimeSteps,
) -> dict[int, gcs.GeneratedPolygons]:
    """
    generate space-time obstacles based on given CBS constraints
    """
    res: dict[int, gcs.GeneratedPolygons] = {}

    for t in range(len(T)):
        res[t] = deepcopy(static_obsts)

    for cstr in cstrs:
        t = bisect_right(T, cstr.ta) - 1
        if t not in res:
            res[t] = gcs.GeneratedPolygons([xbound, ybound])
        x0, y0 = cstr.va
        x1, y1 = cstr.vb
        corners: list[TP.Vert] = [(x0, y0), (x0, y1), (x1, y1), (x1, y0)]
        polyObj = Polygon_obj(4, (xbound, ybound), np.inf, corners)
        res[t].polygons.append(polyObj)
    return res


def _build_multi_layer_graph(
    start: TP.Vert,
    target: TP.Vert,
    T: TP.TimeSteps,
    nodes: list[list[int]],
    edges: list[list[Any]],
    rects: list[GeneratedPolygons],
) -> nx.DiGraph:
    G = nx.DiGraph()
    G.add_node(0)
    nx.set_node_attributes(G, {0: T[0]}, "Layer")
    nx.set_node_attributes(G, {0: start}, "Convex Set")
    node_numbers = deepcopy(nodes)
    for i in range(1, len(nodes)):
        for j in range(0, len(nodes[i])):
            current_node = len(G)
            G.add_node(current_node)
            node_numbers[i][j] = current_node
            nx.set_node_attributes(G, {current_node: T[i]}, "Layer")
            convex_set = rects[i].polygons[nodes[i][j]]
            nx.set_node_attributes(G, {current_node: convex_set}, "Convex Set")

    # for a given layer
    for i in range(0, len(edges)):
        # then a given node
        for j in range(0, len(edges[i])):
            old_node1_name = edges[i][j][0]
            temp_index1 = nodes[i].index(old_node1_name)
            node1 = node_numbers[i][temp_index1]
            for k in range(0, len(edges[i][j][1])):
                old_node2_name = edges[i][j][1][k]
                temp_index2 = nodes[i + 1].index(old_node2_name)
                node2 = node_numbers[i + 1][temp_index2]
                G.add_edge(node1, node2)

    for i in range(0, len(G)):
        if G.nodes[i]["Layer"] == T[-1]:
            G.nodes[i]["Convex Set"] = target

    return G


def build_graph(
    sx: float,
    sy: float,
    tx: float,
    ty: float,
    cstrs: list[CbsConstraint],
    vmax: float,
    T: TP.TimeSteps,
    env: Env,
) -> nx.DiGraph:
    xbound = env.xbound(True)
    ybound = env.ybound(True)
    static_obsts = gen_static_poly_obstacles(env)
    timed_obsts = gen_timed_poly_obstacles(cstrs, static_obsts, xbound, ybound, T)
    return _build_graph(sx, sy, tx, ty, timed_obsts, vmax, xbound, ybound, T)


def random_rounding(
    edges, G, vmax, ts, LB=np.inf, GAP=1e-3, N=1000
) -> tuple[list[TP.Vert], float]:
    bestObj: float = np.inf
    bestPath: list[TP.Vert] = []
    import fractional_solution as fs

    np.random.seed(0)
    for i in range(N):
        rnd_edges = fs.random_edge_selector(edges)
        G_rnd = fs.graph_reconstruction(G, rnd_edges)
        try:
            path, _, obj = fs.solve_fractional_GCS(vmax, G_rnd, ts)
        except Exception as e:
            import pickle

            dat = dict(vmax=vmax, G=G_rnd, T=ts)
            pickle.dump(dat, open("debug-rnd.dump", "wb"))
            raise e
        if bestObj > obj:
            bestObj = obj
            bestPath = path  # type: ignore
            print(
                f"\n    --- rnd rounding, obj:{obj:.2f} gap:{(bestObj - LB) / LB * 100.0:.2f}%",
                end="",
            )
        if (bestObj - LB) / LB < GAP:
            return bestPath, bestObj
        print(f"    --- rnd rounding {i} iteration\r", end="")
    print("")
    return bestPath, bestObj


def gen_rect_sets(
    xbound: TP.Vert, ybound: TP.Vert, obsts: list[Polygon_obj]
) -> GeneratedPolygons:
    """
    obsts: list of rectangular polygons
    """
    res = GeneratedPolygons((xbound, ybound))
    xs: list[float] = list(xbound)
    ys: list[float] = list(ybound)

    # rectObsts: list[TP.Poly] = []
    class RectInfo(NamedTuple):
        h: float
        w: float
        # bot left vertex
        xlb: float
        ylb: float
        xub: float
        yub: float

    rectObsts: list[RectInfo] = []
    for obst in obsts:
        if type(obst.vertices) is not list:
            assert False, "polygon obj has no vertex"
        vs = [(float(x), float(y)) for x, y in obst.vertices]
        _xs, _ys = zip(*vs)
        xs.extend(_xs)
        ys.extend(_ys)
        _xmin, _xmax = min(_xs), max(_xs)
        _ymin, _ymax = min(_ys), max(_ys)
        rectObsts.append(RectInfo(h=_ymax - _ymin, w=_xmax - _xmin, xlb=_xmin, ylb=_ymin, xub=_xmax, yub=_ymax))
        # rectObsts.append(vs)

    # remove duplicated values
    xs = sorted(list(set(xs)))
    ys = sorted(list(set(ys)))
    rectObsts.sort(key=lambda rect: (rect.xlb, rect.ylb))

    def gen_rect_poly_obj(xlb: float, xub: float, ylb: float, yub: float):
        return Polygon_obj(4, ((xlb, xub), (ylb, yub)), None, vertices=[
            (xlb, ylb), (xlb, yub), (xub, yub), (xub, ylb)
        ])

    polys = []

    for xid in range(1, len(xs)):
        xlb, xub = xs[xid - 1], xs[xid]
        # process rects in [r.x, r.x+r.w]
        yilb = 0
        rects = [r for r in rectObsts if r.xlb <= xlb and r.xub>=xub]
        rects.sort(key=lambda rect: rect.ylb)
        for i, r in enumerate(rects):
            # invariant: rects in [pl, pr) are strictly increase in y
            if i > 0:
                assert r.ylb >= rects[i - 1].ylb
            # precond: yilb is strictly below bottom of r
            if ys[yilb] >= r.ylb:
                continue

            # precond: empty set [yilb, yilb), as rect[i] and rect[i+1] may touched
            yiub = yilb
            while yiub < len(ys) and ys[yiub] <= r.ylb:
                yiub += 1

            # find a convex set [xlb, xub] [ys[yilb], ys[yiub-1]]
            polys.append(gen_rect_poly_obj(xlb, xub, ys[yilb], ys[yiub-1]))

            # postcond: yilb is at top of r
            yilb = yiub
            while yilb + 1 < len(ys) and ys[yilb + 1] < r.yub:
                yilb += 1

        # [xlb, xub], ys[yilb: -1] is also a convex rect
        if yilb+1 < len(ys):
            polys.append(gen_rect_poly_obj(xlb, xub, ys[yilb], ys[-1]))

    res.polygons = polys
    return res


def _build_graph(
    sx: float,
    sy: float,
    tx: float,
    ty: float,
    timed_obsts: dict[int, GeneratedPolygons],
    vmax: float,
    xbound: tuple[float, float],
    ybound: tuple[float, float],
    T: TP.TimeSteps,
):
    all_convex_sets: list[GeneratedPolygons] = []
    for tid in range(len(T)):
        obsts = timed_obsts[tid].polygons if tid in timed_obsts else []
        # lines = gcs.get_x_values(obsts)
        # convex_sets_t = gcs.generate_rectangluar_sets( (xbound, ybound), lines, obsts, None)
        convex_sets_t = gen_rect_sets(xbound, ybound, obsts)
        all_convex_sets.append(convex_sets_t)

    g_nodes, g_edges = gra.find_graph_vertices_edges(
        (sx, sy), (tx, ty), vmax, all_convex_sets, timed_obsts, T
    )

    # g_nodes, g_edges = gcs.find_graph_vertices_edges(
    #     (sx, sy), (tx, ty), vmax, all_convex_sets, T
    # )

    G = _build_multi_layer_graph(
        (sx, sy), (tx, ty), T, g_nodes, g_edges, all_convex_sets
    )
    return G


def _solve_min_time(
    sx: float,
    sy: float,
    tx: float,
    ty: float,
    timed_obsts: dict[int, GeneratedPolygons],
    vmax: float,
    xbound: tuple[float, float],
    ybound: tuple[float, float],
    T: TP.TimeSteps,
    rndRound: bool = True,
):
    # using MIP to find minimum time
    # EPS = 1e-3
    EPS_R = 1e-2
    res = GCSSol(([], T), np.inf)
    G = _build_graph(sx, sy, tx, ty, timed_obsts, vmax, xbound, ybound, T)
    es, _, mip = gcs.gcs_eqns_solver(vmax, G, T, False)
    if mip.Status == GRB.INFEASIBLE:
        return res
    # edges = [ut.EdgeUsed(e) for e in es]
    _mintID = len(T) - 1
    tLB = mip.objVal / vmax
    print(f">>>> mip Obj: {mip.ObjVal}")
    while _mintID - 1 > 0 and T[_mintID - 1] >= tLB:
        G = _build_graph(
            sx, sy, tx, ty, timed_obsts, vmax, xbound, ybound, T[: _mintID - 1]
        )

        # ensure has target
        hasTarget = False
        for i in range(len(G)):
            if G.nodes[i]["Layer"] == T[: _mintID - 1][-1]:
                hasTarget = True
                break
        if not hasTarget:
            break

        _es, _, _mip = gcs.gcs_eqns_solver(vmax, G, T[: _mintID - 1], False)
        diff = abs(_mip.ObjVal - mip.ObjVal) if _mip.Status == GRB.OPTIMAL else np.inf

        # path, obj = random_rounding(_es, G, vmax, T[: _mintID - 1], LB=mip.ObjVal)
        if diff / mip.ObjVal < EPS_R:
            print(
                f" --- sub Obj, topk:{_mintID-1}, obj: {_mip.ObjVal} diff:{diff} gap:{diff / mip.ObjVal * 100.0:.2}% \r",
                end="",
            )
            _mintID -= 1
        else:
            print()
            break

    if not rndRound:
        print(f"\n   --- running binary prog with topk:{_mintID} ... ")
        for t in range(_mintID, len(T) + 1):
            G = _build_graph(sx, sy, tx, ty, timed_obsts, vmax, xbound, ybound, T[:t])
            es, _, m = gcs.gcs_eqns_solver(vmax, G, T[:t], True)
            if m.Status == GRB.OPTIMAL:
                diff = abs(mip.ObjVal - m.ObjVal)
                print(
                    f"   --- binary prog, topk:{t}, obj:{m.ObjVal} diff:{diff} gap: {diff / mip.ObjVal * 100.0:.2}%---\r",
                    end="",
                )
                pts = [(sx, sy)]
                _threshold = 1e-3
                for u, v in es:
                    if es[u, v][2] > _threshold:
                        pts.append((es[u, v][5], es[u, v][6]))
                for _ in range(t, len(T)):
                    pts.append((tx, ty))
                assert len(pts) == len(T)
                res = GCSSol((pts, T), m.ObjVal)
                break
            else:
                print(f"   --- binary prog, topk:{t} INFEASIBLE ---\r", end="")
    else:
        for t in range(_mintID, len(T) + 1):
            print(f"   --- running rnd rounding with topk:{t} ... \r", end="")
            G = _build_graph(sx, sy, tx, ty, timed_obsts, vmax, xbound, ybound, T[:t])
            _es, _, m = gcs.gcs_eqns_solver(vmax, G, T[:t], False)
            path, obj = random_rounding(_es, G, vmax, T[:t], LB=mip.ObjVal)
            if obj != np.inf:
                for _ in range(t, len(T)):
                    path.append((tx, ty))
                assert len(path) == len(T)
                res = GCSSol((path, T), obj)
                break
        print()

    return res


def solve_min_time(
    sx: float,
    sy: float,
    tx: float,
    ty: float,
    cstrs: list[CbsConstraint],
    vmax: float,
    T: TP.TimeSteps,
    env: Env,
    rndRound: bool = True,
) -> GCSSol:
    # using MIP to find minimum time
    # EPS = 1e-3
    xbound = env.xbound()
    ybound = env.ybound()
    static_obsts = gen_static_poly_obstacles(env)
    timed_obsts = gen_timed_poly_obstacles(cstrs, static_obsts, xbound, ybound, T)
    res = _solve_min_time(
        sx, sy, tx, ty, timed_obsts, vmax, xbound, ybound, T, rndRound
    )
    return res


def solve(
    sx: float,
    sy: float,
    tx: float,
    ty: float,
    cstrs: list[CbsConstraint],
    vmax: float,
    T: TP.TimeSteps,
    env: Env,
    verify: bool = True,
    is_binary: bool = True,
    minT: bool = True,
    timeLimit: float = 300,
    gap: float = 0.05,
    **kws,
) -> GCSSol:
    """
    (sx, sy): start xy-coord of a robot
    (tx, ty): target xy-coord of a robot
    cstrs   : list of constraints, each of them defines
              a untraversable rectangle in space-time
    vmax    : max speed
    T       : timesteps
    """
    # if min_time:
    #     res = solve_min_time(sx, sy, tx, ty, cstrs, vmax, T, env, rndRound=False)
    # else:
    b_start = time.perf_counter()
    G = build_graph(sx, sy, tx, ty, cstrs, vmax, T, env)
    build_gr_time = time.perf_counter() - b_start
    edges_used, _, m = gcs.gcs_eqns_solver(
        vmax, G, T, is_binary, minT=minT, gap=gap, timeLimit=timeLimit
    )
    gcs_solve_time = time.perf_counter() - b_start
    # print(f"Binary Runtime {runtime_b}")

    if m.SolCount == 0:
        res = GCSSol(([], T), np.inf)
        res.build_gr_time = build_gr_time
        res.gcs_sol_time = gcs_solve_time
        return res

    # get waypoints
    points: list[TP.Vert] = [(sx, sy)]
    _threshold = 1e-3
    for u, v in edges_used:
        if edges_used[u, v][2] > _threshold:
            points.append((edges_used[u, v][5], edges_used[u, v][6]))
    res = GCSSol((points, T), lb=m.ObjBound if is_binary else m.ObjVal, ub=m.ObjVal)
    res.build_gr_time = build_gr_time
    res.gcs_sol_time = gcs_solve_time

    # verify res sastisfy all constraints
    if verify:
        if not res.verify(cstrs):
            import pickle

            dat = dict(
                sx=sx,
                sy=sy,
                tx=tx,
                ty=ty,
                cstrs=cstrs,
                vmax=vmax,
                T=T,
                xbound=env.xbound(),
                ybound=env.ybound(),
                env=env,
            )
            pickle.dump(dat, open("gcs-debug.dump", "wb"))
            pickle.dump(res, open("gcs-debug-path.dump", "wb"))
            raise Exception("Violate Constraints")
    return res
