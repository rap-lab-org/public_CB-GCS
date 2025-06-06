import sys
import os
import numpy as np
from bisect import bisect_right

_current = os.path.dirname(os.path.realpath(__file__))
# directory reach
_parent = os.path.dirname(_current)
if _parent not in sys.path:
    sys.path.append(_parent)
if _current not in sys.path:
    sys.path.append(_current)
import cbs.type_stubs as TP
from cbs.utils import Mod, CbsConstraint, EPS


def CheckConflictSharedTimeSteps(
    x_ticks: list[float],
    y_ticks: list[float],
    paths: dict[int, TP.VertPath],
    ri: int,
    rj: int,
    d_thres: float = 1,
    M: Mod = Mod.LB,
    UBlevel: int = 0,
) -> tuple[float, dict[int, list[CbsConstraint]]]:
    if M == Mod.LB:
        return _CheckConflictSharedTimeStepsLB(x_ticks, y_ticks, paths, ri, rj, d_thres)
    elif M == Mod.UB:
        if UBlevel == 0:
            return _CheckConflictSharedTimeStepsUB(
                x_ticks, y_ticks, paths, ri, rj, d_thres
            )
        elif UBlevel == 1:
            return _CheckConflictSharedTimeStepsUBLarge(
                x_ticks, y_ticks, paths, ri, rj, d_thres
            )
        else:
            return _CheckConflictSharedTimeStepsUBLarge(
                x_ticks, y_ticks, paths, ri, rj, d_thres,
                CLEARANCE_F=0.5*UBlevel,
                blockT=UBlevel-1
            )
    else:
        assert False


def _CheckConflictSharedTimeStepsUBLarge(
    x_ticks: list[float],
    y_ticks: list[float],
    paths: dict[int, TP.VertPath],
    ri: int,
    rj: int,
    d_thres: float = 1,
    CLEARANCE_F: float = 0.1,
    blockT: int = 1
):
    """
    Assuming plan of all robots are in same timesteps
    - x/y_ticks: defines conflict detectioin cells
    - paths: path of all robots
    - ri: id of robot_i
    - rj: id of robot_j
    - d_thres: minimum distance between centroid of robots
    - return: dict[robot_i] = list of constraints for robot_i

    Resolving conflict for upper bound must eliminate all possible conflicts in transition.
    """
    pathi, tsi = paths[ri]
    pathj, tsj = paths[rj]
    # sanity checking: timesteps of paths must be same
    assert len(tsi) == len(tsj)
    assert np.min(np.array(tsi) - np.array(tsj)) <= EPS

    cstrs = {}
    CLEARANCEX = CLEARANCE_F * d_thres + (x_ticks[1] - x_ticks[0])
    CLEARANCEY = CLEARANCE_F * d_thres + (y_ticks[1] - y_ticks[0])

    # def inCollision(pi: TP.Vert, vi: TP.Vert, pj: TP.Vert, vj: TP.Vert) -> bool:
    #     from cbs.utils import calcCollisionScore
    #
    #     res = calcCollisionScore(pi, vi, pj, vj, d_thres)
    #     # print ("collision:", res)
    #     return res > 0

    def BoundBox(p1: TP.Vert, p2: TP.Vert) -> tuple[TP.Vert, TP.Vert]:
        x1, y1 = p1
        x2, y2 = p2
        x = (x1 + x2) / 2
        y = (y1 + y2) / 2
        minx = x - d_thres
        miny = y - d_thres
        maxx = x + d_thres
        maxy = y + d_thres
        return (minx, miny), (maxx, maxy)

    def RelativeStaticCstr(
        rid: int,
        tid: int,
        ts: TP.TimeSteps,
        c1: TP.Vert,
        v1: TP.Vert,
        c2: TP.Vert,
        v2: TP.Vert,
    ):
        x, y = c1
        v1x, v1y = v1
        v2x, v2y = v2

        vx = v1x - v2x
        vy = v1y - v2y
        c00, c11 = BoundBox(c2, (c2[0] + v2[0], c2[1] + v2[1]))
        # add velocity margin to bounding box and ensure within workspace
        c00 = max(min(x_ticks), c00[0] - abs(vx)), max(min(y_ticks), c00[1] - abs(vy))
        c11 = min(max(x_ticks), c11[0] + abs(vx)), min(max(y_ticks), c11[1] + abs(vy))

        xid = bisect_right(x_ticks, c00[0]) - 1
        yid = bisect_right(y_ticks, c00[1]) - 1
        x00 = x_ticks[xid] - CLEARANCEX
        y00 = y_ticks[yid] - CLEARANCEY
        x11 = c11[0] + CLEARANCEX
        y11 = c11[1] + CLEARANCEY

        # constriant cannot exceed boundary of workspace
        x00 = max(x00, min(x_ticks))
        y00 = max(y00, min(y_ticks))
        x11 = min(x11, max(x_ticks))
        y11 = min(y11, max(y_ticks))

        tub = ts[tid+blockT] if tid+blockT<len(ts) else ts[-1]

        if x00 + EPS < x < x11 - EPS and y00 + EPS < y < y11 - EPS:
            return [CbsConstraint(rid, (x00, y00), (x11, y11), ts[tid], tub)]
        return None
 
    from cbs.utils import calcCollisionScore
    for tid in range(1, len(tsi) - 1):
        # if not inCollision(pathi[tid], pathi[tid + 1], pathj[tid], pathj[tid + 1]):
        #     continue
        cScore = calcCollisionScore(pathi[tid], pathi[tid + 1], pathj[tid], pathj[tid + 1], d_thres)
        if cScore <= 0.0:
            continue

        vix, viy = pathi[tid + 1][0] - pathi[tid][0], pathi[tid + 1][1] - pathi[tid][1]
        vjx, vjy = pathj[tid + 1][0] - pathj[tid][0], pathj[tid + 1][1] - pathj[tid][1]

        # regard j as an static obstacle
        cstri = RelativeStaticCstr(
            ri,
            tid,
            tsi,
            pathi[tid],
            (vix, viy),
            pathj[tid],
            (vjx, vjy),
        )
        if cstri is not None:
            cstrs[ri] = cstri

        # regard i as an static obstacle
        cstrj = RelativeStaticCstr(
            rj,
            tid,
            tsj,
            pathj[tid],
            (vjx, vjy),
            pathi[tid],
            (vix, viy),
        )
        if cstrj is not None:
            cstrs[rj] = cstrj
        if len(cstrs) > 0:
            return cScore, cstrs

    return 0, {}


def _CheckConflictSharedTimeStepsUB(
    x_ticks: list[float],
    y_ticks: list[float],
    paths: dict[int, TP.VertPath],
    ri: int,
    rj: int,
    d_thres: float = 1,
    CLEARANCE_F: float = 1e-1,
):
    """
    Assuming plan of all robots are in same timesteps
    - x/y_ticks: defines conflict detectioin cells
    - paths: path of all robots
    - ri: id of robot_i
    - rj: id of robot_j
    - d_thres: minimum distance between centroid of robots
    - return: dict[robot_i] = list of constraints for robot_i

    Resolving conflict for upper bound must eliminate all possible conflicts in transition.
    """
    pathi, tsi = paths[ri]
    pathj, tsj = paths[rj]
    # sanity checking: timesteps of paths must be same
    assert len(tsi) == len(tsj)
    assert np.min(np.array(tsi) - np.array(tsj)) <= EPS

    # def inCollision(pi: TP.Vert, vi: TP.Vert, pj: TP.Vert, vj: TP.Vert) -> bool:
    #     from cbs.utils import calcCollisionScore
    #
    #     res = calcCollisionScore(pi, vi, pj, vj, d_thres)
    #     return res > 0

    from cbs.utils import calcCollisionScore
    CLEARANCEX = CLEARANCE_F * d_thres + (x_ticks[1] - x_ticks[0])
    CLEARANCEY = CLEARANCE_F * d_thres + (y_ticks[1] - y_ticks[0])
    for tid in range(1, len(tsi) - 1):
        # if not inCollision(pathi[tid], pathi[tid + 1], pathj[tid], pathj[tid + 1]):
        #     continue
        cScore = calcCollisionScore(pathi[tid], pathi[tid + 1], pathj[tid], pathj[tid + 1], d_thres)
        if cScore <= 0.0:
            continue

        xi, yi = pathi[tid]
        xj, yj = pathj[tid]

        from cbs.utils import calcCollisionBoundBox, ensureInBound

        c00, c11 = calcCollisionBoundBox(
            pathi[tid], pathi[tid + 1], pathj[tid], pathj[tid + 1], d_thres
        )
        c00 = ensureInBound(x_ticks, y_ticks, c00)
        c11 = ensureInBound(x_ticks, y_ticks, c11)

        xid = bisect_right(x_ticks, c00[0]) - 1
        yid = bisect_right(y_ticks, c00[1]) - 1
        x00 = x_ticks[xid] - CLEARANCEX
        y00 = y_ticks[yid] - CLEARANCEY
        x11 = c11[0] + CLEARANCEX
        y11 = c11[1] + CLEARANCEY

        # constriant cannot exceed boundary of workspace
        x00, y00 = ensureInBound(x_ticks, y_ticks, (x00, y00))
        x11, y11 = ensureInBound(x_ticks, y_ticks, (x11, y11))

        if (
            x00 + EPS < xi < x11 - EPS
            and x00 + EPS < xj < x11 - EPS
            and y00 + EPS < yi < y11 - EPS
            and y00 + EPS < yj < y11 - EPS
        ):
            cstr_i = [CbsConstraint(ri, (x00, y00), (x11, y11), tsi[tid], tsi[tid])]
            cstr_j = [CbsConstraint(rj, (x00, y00), (x11, y11), tsi[tid], tsi[tid])]
            return cScore, {ri: cstr_i, rj: cstr_j}

    return 0, {}


def _CheckConflictSharedTimeStepsLB(
    x_ticks: list[float],
    y_ticks: list[float],
    paths: dict[int, TP.VertPath],
    ri: int,
    rj: int,
    d_thres: float = 1,
) -> tuple[float, dict[int, list[CbsConstraint]]]:
    """
    Assuming plan of all robots are in same timesteps
    - x/y_ticks: defines conflict detectioin cells
    - paths: path of all robots
    - ri: id of robot_i
    - rj: id of robot_j
    - d_thres: minimum distance between centroid of robots
    - return: dict[robot_i] = list of constraints for robot_i
    """
    pathi, tsi = paths[ri]
    pathj, tsj = paths[rj]
    # sanity checking: timesteps of paths must be same
    assert len(tsi) == len(tsj)
    assert np.min(np.array(tsi) - np.array(tsj)) <= EPS

    for ci, cj, t in zip(pathi, pathj, tsi):
        xi, yi = ci
        xj, yj = cj
        if (abs(xi - xj) > d_thres or abs(yi - yj) > d_thres):
            continue
        # xid = bisect_right(x_ticks, min(xi, xj) - EPS) - 1
        # yid = bisect_right(y_ticks, min(yi, yj) - EPS) - 1
        # x00 = x_ticks[xid]
        # y00 = y_ticks[yid]
        x00 = max((xi + xj) / 2 - d_thres / 2, min(x_ticks))
        y00 = max((yi + yj) / 2 - d_thres / 2, min(y_ticks))
        x11 = min(x00 + d_thres, max(x_ticks))
        y11 = min(y00 + d_thres, max(y_ticks))

        assert x00 <= xi and x00 <= xj
        assert y00 <= yi and y00 <= yj

        # (xi, yi) and (xj, yj) are in same conflict cell
        if (
            x00 + EPS < xi < x11 - EPS
            and x00 + EPS < xj < x11 - EPS
            and y00 + EPS < yi < y11 - EPS
            and y00 + EPS < yj < y11 - EPS
        ):
            cstr_i = [CbsConstraint(ri, (x00, y00), (x11, y11), t, t)]
            cstr_j = [CbsConstraint(rj, (x00, y00), (x11, y11), t, t)]
            return 0, {ri: cstr_i, rj: cstr_j}
    return 0, {}
