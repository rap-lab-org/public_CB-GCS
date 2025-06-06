from itertools import product
from typing import TypeAlias
from enum import Enum
import heapq as hpq
import numpy as np
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
# from cbs.conflict import calcCollisionScore

EPS = 1e-3


class ConflictMode(Enum):
    OPT = 0  # optimal
    LB = 1  # lower bound
    UB = 2  # upper bound


class SearchMethod(Enum):
    ASTAR = 0  # Best First Search
    FOCAL = 1  # Focal Search
    ANYFOCAL = 2  # Anytime Focal


class SearchStatus(Enum):
    FINISH = 0
    TIMEOUT = 1
    UNSTARTED = 1

Mod: TypeAlias = ConflictMode
SMD: TypeAlias = SearchMethod


class Env:
    def __init__(
        self,
        minx: float = 0,
        maxx: float = 0,
        miny: float = 0,
        maxy: float = 0,
        maxt: float = 0,
        d_thres: float = 1,
        obsts: list[TP.Poly] | None = None,
    ):
        self.minx: float = minx
        self.maxx: float = maxx
        self.miny: float = miny
        self.maxy: float = maxy
        self.maxt: float = maxt
        self.d_thres: float = d_thres

        if obsts is None:
            obsts = []
        self.set_obsts(obsts)

    def to_dict(self) -> dict:
        return dict(
            minx=self.minx,
            maxx=self.maxx,
            miny=self.miny,
            maxy=self.maxy,
            maxt=self.maxt,
            d_thres=self.d_thres,
            obsts=self.obsts,
        )

    def bbox(self) -> TP.Poly:
        return [
            (self.minx, self.miny),
            (self.minx, self.maxy),
            (self.maxx, self.maxy),
            (self.maxx, self.miny),
        ]

    def xbound(self, consider_footprint=False) -> tuple[float, float]:
        if consider_footprint:
            return (
                self.minx + self.d_thres / 2,
                self.maxx - self.d_thres / 2,
            )
        else:
            return (self.minx, self.maxx)

    def ybound(self, consider_footprint=False) -> tuple[float, float]:
        if consider_footprint:
            return (
                self.miny + self.d_thres / 2,
                self.maxy - self.d_thres / 2,
            )
        else:
            return (self.miny, self.maxy)

    def set_obsts(self, obsts: list[TP.Poly]):
        self.obsts = obsts
        self.inflated_obsts = [ObstacleInflation(obs, self) for obs in obsts]

    def update_inflation_buf(self, buf: float=0):
        self.inflated_obsts = [ObstacleInflation(obs, self, buf) for obs in self.obsts]


class PrioritySet(object):
    """
    priority queue, min-heap
    """

    def __init__(self):
        """
        no duplication allowed
        """
        self.heap_ = []
        self.set_ = set()

    def add(self, pri: float, d: int):
        """
        will check for duplication and avoid.
        """
        if d not in self.set_:
            hpq.heappush(self.heap_, (pri, d))
            self.set_.add(d)

    def pop(self) -> tuple[float, int]:
        """
        impl detail: return the first(min) item that is in self.set_
        """
        pri, d = hpq.heappop(self.heap_)
        while d not in self.set_:
            pri, d = hpq.heappop(self.heap_)
        self.set_.remove(d)
        return pri, d

    def top(self) -> tuple[float, int]:
        """
        return the top entry in set
        """
        while self.size() > 0:
            _, d = self.heap_[0]
            if d not in self.set_:
                hpq.heappop(self.heap_)
            else:
                break
        assert self.size() > 0
        return self.heap_[0]

    def size(self) -> int:
        return len(self.set_)

    def print(self):
        print(self.heap_)
        print(self.set_)
        return

    def has(self, d) -> bool:
        if d in self.set_:
            return True
        return False

    def remove(self, d) -> bool:
        """
        implementation: only remove from self.set_, not remove from self.heap_ list.
        """
        if d not in self.set_:
            return False
        self.set_.remove(d)
        return True


class CbsConstraint:
    """ """

    def __init__(self, i, va: TP.Vert, vb: TP.Vert, ta: float, tb: float, j=-1, flag=-1):
        """
        create a constraint, if a single point, then va=vb
        """
        self.i: int = i  # id for agent i, i<0, iff not valid
        self.va: TP.Vert = va  # bot-left corner, (minx, miny)
        self.vb: TP.Vert = vb  # top-right corner, (maxx maxy)
        self.ta: float = ta
        self.tb: float = tb
        self.j: int = j  # id for agent j, undefined by default
        self.flag = flag  # useless now

    def is_contains(self, v: TP.Vert, t: float, _EPS: float = 1e-3):
        minx, miny = self.va
        maxx, maxy = self.vb
        x, y = v
        return (
            self.ta <= t <= self.tb
            and minx + _EPS < x < maxx - _EPS
            and miny + _EPS < y < maxy - _EPS
        )

    def __str__(self):
        return (
            "{i:"
            + str(self.i)
            + ",va:"
            + str(self.va)
            + ",vb:"
            + str(self.vb)
            + ",ta:"
            + str(self.ta)
            + ",tb:"
            + str(self.tb)
            + ",j:"
            + str(self.j)
            + ",flag:"
            + str(self.flag)
            + "}"
        )

    def __repr__(self):
        return self.__str__()


class CbsCellSol:
    """
    The solution in CBS high level node. A dict of paths in cell id for all robots.
    """

    def __init__(self):
        self.paths: dict[int, TP.CIDPath] = {}
        self.costs: dict[int, float] = {}

    def __str__(self):
        return str(self.paths)

    def AddPath(self, i: int, lv: list[int], lt: TP.TimeSteps, c: float = 0.0):
        self.paths[i] = (lv, lt)
        self.costs[i] = c

    def DelPath(self, i):
        self.paths.pop(i)
        self.costs.pop(i)
        return

    def GetPath(self, i: int) -> TP.CIDPath:
        return self.paths[i]

    def GetCost(self, i: int) -> float:
        return self.costs[i]


class CbsVertSol:
    """
    The solution in CBS high level node. A dict of paths in coord for all robots.
    """

    def __init__(self):
        self.paths: dict[int, TP.VertPath] = dict()
        self.costs: dict[int, float] = dict()
        return

    def __str__(self):
        return str(self.paths)

    def AddPath(self, i: int, lv: list[TP.Vert], lt: list[float], c=0.0):
        """
        lv is a list of loc id
        lt is a list of time (adjacent element increases with a step size of one)
        """
        # add a final infinity interval
        # nlv,nlt = EnforceUnitTimePath(lv,lt)
        # print("lv = ", lv)
        # lv.append(lv[-1])
        # lt.append(np.inf)
        self.paths[i] = (lv, lt)
        self.costs[i] = c
        return

    def DelPath(self, i):
        self.paths.pop(i)
        self.costs.pop(i)
        return

    def GetPath(self, i) -> TP.VertPath:
        return self.paths[i]

    def GetCost(self, i) -> float:
        return self.costs[i]

    def TotalCost(self) -> float:
        return sum(self.costs.values())


class GCSSol:
    def __init__(self, path: TP.VertPath, lb: float = 0, ub: float = np.inf):
        self.path, self.ts = path
        self.lb = lb
        self.ub = ub
        self.build_gr_time = np.inf
        self.gcs_sol_time = np.inf

    def __str__(self) -> str:
        path_str = ""
        for i in range(len(self.path)):
            v = self.path[i]
            t = self.ts[i]
            path_str += f"({v[0]}, {v[1]}, {t}),"
        return f"lb: {self.lb}, ub: {self.ub}, path:[{path_str}]"

    def verify(self, cstrs: list[CbsConstraint]) -> bool:
        for (x, y), t in zip(self.path, self.ts):
            for c in cstrs:
                if c.is_contains((x, y), t, EPS):
                    print(f"[ERROR] GCS Path violate constrast {c} at ({x}, {y} @ {t})")
                    return False
        return True


class GcsCbsNode:
    """
    High level search tree node
    """

    def __init__(
        self,
        id0: int,
        g: float,
        sol: None | CbsVertSol = None,
        cstr: None | CbsConstraint = None,
        parent: int = -1,
        mod: Mod = Mod.UB,
    ):
        if sol is None:
            sol = CbsVertSol()
        if cstr is None:
            cstr = CbsConstraint(-1, (0, 0), (0, 0), -1, -1, -1)

        self.id: int = id0
        self.sol: CbsVertSol = sol  # The feasible path represented by cell indices.
        self.cstrs: list[CbsConstraint] = [cstr]
        self.g: float = g  # @2023-11-23, lower bound cost
        self.parent: int = parent
        self.mod = mod
        self.cScore = np.inf

        # self.aux_sol: CbsVertSol = (
        #     CbsVertSol()
        # )  # @2023-11-23 The feasible path represented by points.
        # self.aux_g: float = 0  # @2023-11-23 Feasible path cost.

        return

    def __str__(self):
        str1 = (
            "{id:" + str(self.id) + ",g:" + str(self.g) + ",par:" + str(self.parent) + "}"
        )
        return str1

    def collisionScore(self, d_thres: float) -> float:
        res: float = 0
        # from cbs.conflict import calcVilocity
        _t_reso = 100

        for ri, rj in product(self.sol.paths, self.sol.paths):
            if ri >= rj:
                continue
            p1, _ = self.sol.paths[ri]
            p2, _ = self.sol.paths[rj]
            res += calcCollisionPathScore(p1, p2, d_thres, k=_t_reso)
        self.cScore = res
        return res


def rc2i(r: int, c: int, num_col: int) -> int:
    return r * num_col + c


def i2rc(i: int, num_col: int) -> tuple[int, int]:
    r = i // num_col
    c = i % num_col
    return (r, c)


def edist(v1: TP.Vert, v2: TP.Vert):
    return np.sqrt((v1[0] - v2[0]) ** 2 + (v1[1] - v2[1]) ** 2)


def GetCorners(p: TP.Vert, delta: float = 0.5):
    """
    p: centroid xy-coord of an agent
    delta: distance from p to border
    return: corners (xy-coord) of the agent (1x1 square) in order
        bot-left,bot-right,top-right,top-left
    """
    px, py = p
    x0, x1 = px - delta, px + delta
    y0, y1 = py - delta, py + delta
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def MaxDist(pts1: list[TP.Vert], pts2: list[TP.Vert]):
    """
    pts1 = [[y1,x1],[y2,x2],...,[yn,xn]], i.e., a list of 2d points {p1,p2,...,pn}
    pts2 = [[y1',x1'],[y2',x2'],...,[ym',xm']], i.e., a list of 2d points {p1',p2',...,pm'}
    return the max euclidean dist between any pair of pk and pk'.
    """
    lmax = 0
    for p1, p2 in product(pts1, pts2):
        dist = edist(p1, p2)
        lmax = max(lmax, dist)
    return lmax


def MinDist(pts1: list[TP.Vert], pts2: list[TP.Vert]):
    """
    pts1 = [[y1,x1],[y2,x2],...,[yn,xn]], i.e., a list of 2d points {p1,p2,...,pn}
    pts2 = [[y1',x1'],[y2',x2'],...,[ym',xm']], i.e., a list of 2d points {p1',p2',...,pm'}
    return the min euclidean dist between any pair of pk and pk'.
    """
    lmax = np.inf
    for p1, p2 in product(pts1, pts2):
        dist = edist(p1, p2)
        lmax = min(lmax, dist)
    return lmax


def ObstacleInflation(obs: TP.Poly, env: Env, buf: float = 0) -> TP.Poly:
    # convert obstacle in workspace to
    # obstacles regaion for the centroid of agent, considering size
    assert len(obs) == 4  # only support rectangle now
    minx = min([v[0] for v in obs]) - (env.d_thres + buf) / 2
    maxx = max([v[0] for v in obs]) + (env.d_thres + buf) / 2
    miny = min([v[1] for v in obs]) - (env.d_thres + buf) / 2
    maxy = max([v[1] for v in obs]) + (env.d_thres + buf) / 2
    minx = max(minx, env.minx)
    maxx = min(maxx, env.maxx)
    miny = max(miny, env.miny)
    maxy = min(maxy, env.maxy)

    res = [(minx, miny), (minx, maxy), (maxx, maxy), (maxx, miny)]
    return res


def debug_dump(fname="debug.dump"):
    def decorator(func):
        def wrapper(*args, **kws):
            try:
                res = func(*args, **kws)
                return res
            except Exception as e:
                import traceback

                print(f"Traceback has been saved to file '{fname}.traceback'")
                traceback.print_exc(file=open(f"{fname}.traceback", "w"))
                import pickle

                dat = dict(args=args, kws=kws)
                pickle.dump(dat, open(fname, "wb"))
                print(f"Context has been dumped to file '{fname}'")
                raise e

        return wrapper

    return decorator


def auto_g_resolution(T: list[float], v_max: float, env: Env) -> int:
    minx, maxx = env.xbound()
    miny, maxy = env.ybound()
    dt = max(T) / float(len(T) - 1)
    gRx = (maxx - minx) / (v_max * dt)
    gRy = (maxy - miny) / (v_max * dt)
    from math import floor

    return max(floor(gRx), floor(gRy))


def scaleUpEnv(e: Env, f: float) -> Env:
    obsts = []
    for obs in e.obsts:
        scaledObs = [(x * f, y * f) for x, y in obs]
        obsts.append(scaledObs)
    res = Env(
        e.minx * f, e.maxx * f, e.miny * f, e.maxy * f, e.maxt, e.d_thres * f, obsts
    )
    return res


def calcCollisionPathScore(
    p1: list[TP.Vert], p2: list[TP.Vert], d_thres: float, k: int = 100
) -> float:
    score = 0.0
    for tid in range(1, len(p1) - 1):
        score += calcCollisionScore(
            p1[tid], p1[tid + 1], p2[tid], p2[tid + 1], d_thres, t_reso=k
        )
    return score


def calcCollisionScore(
    u1: TP.Vert,
    v1: TP.Vert,
    u2: TP.Vert,
    v2: TP.Vert,
    d_thres: float,
    t_reso: int = 100,
) -> float:
    v1x, v1y = v1[0] - u1[0], v1[1] - u1[1]
    v2x, v2y = v2[0] - u2[0], v2[1] - u2[1]
    x1, y1 = u1
    x2, y2 = u2

    res = 0
    for i in range(t_reso):
        x1p = x1 + v1x * i / t_reso
        y1p = y1 + v1y * i / t_reso

        x2p = x2 + v2x * i / t_reso
        y2p = y2 + v2y * i / t_reso

        if abs(x1p - x2p) < d_thres - EPS and abs(y1p - y2p) < d_thres - EPS:
            # res += 1.0
            d = max(abs(x1p - x2p), abs(y1p - y2p))
            res += (d_thres - d + 1e-1) / (d + 1e-1)
    return res


def calcCollisionBoundBox(
    u1: TP.Vert,
    v1: TP.Vert,
    u2: TP.Vert,
    v2: TP.Vert,
    d_thres: float,
    t_reso: int = 100,
) -> tuple[TP.Vert, TP.Vert]:
    v1x, v1y = v1[0] - u1[0], v1[1] - u1[1]
    v2x, v2y = v2[0] - u2[0], v2[1] - u2[1]
    x1, y1 = u1
    x2, y2 = u2
    minx, maxx = np.inf, 0
    miny, maxy = np.inf, 0

    def corners(x: float, y: float):
        return [
            (x - d_thres, y - d_thres),
            (x - d_thres, y + d_thres),
            (x + d_thres, y + d_thres),
            (x + d_thres, y - d_thres),
        ]

    for i in range(t_reso):
        x1p = x1 + v1x * i / t_reso
        y1p = y1 + v1y * i / t_reso

        x2p = x2 + v2x * i / t_reso
        y2p = y2 + v2y * i / t_reso

        if abs(x1p - x2p) < d_thres - EPS and abs(y1p - y2p) < d_thres - EPS:
            cs = corners(x1p, y1p) + corners(x2p, y2p)
            for x, y in cs:
                minx = min(minx, x)
                maxx = max(maxx, x)
                miny = min(miny, y)
                maxy = max(maxy, y)
    return (minx, miny), (maxx, maxy)


def ensureInBound(xticks: list[float], yticks: list[float], p: TP.Vert) -> TP.Vert:
    px, py = p
    px = max(p[0], min(xticks))
    px = min(p[0], max(xticks))
    py = max(p[1], min(yticks))
    py = min(p[1], max(yticks))
    return (px, py)
