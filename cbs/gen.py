#!/usr/bin/env python
from copy import deepcopy
from math import ceil, floor
import sys
import os
import random
import json
from itertools import product
from typing import NamedTuple, Any

_current = os.path.dirname(os.path.realpath(__file__))
# directory reach
_parent = os.path.dirname(_current)
if _parent not in sys.path:
    sys.path.append(_parent)
if _current not in sys.path:
    sys.path.append(_current)

import cbs.utils as ut
import cbs.type_stubs as TP


class RandScenMeta(NamedTuple):
    obst_locs: set[tuple[int, int]]
    obsts: list[TP.Poly]
    slocs: list[list[TP.Vert]]
    tlocs: list[list[TP.Vert]]


def scen_10x10_midswap(nagent=4, seed=0, scen_dir="cbs-scen", nscen=1):
    maxx, maxy = 10, 10
    random.seed(seed)
    scen_name = f"n{nagent}-10x10-midswap"

    starts: list[list[tuple[int, int]]] = []
    goals: list[list[tuple[int, int]]] = []

    def update_candidate(x, y, candidates):
        candidates.remove((x, y))
        if (x + 1, y) in candidates:
            candidates.remove((x + 1, y))

        if (x - 1, y) in candidates:
            candidates.remove((x - 1, y))

        if (x, y + 1) in candidates:
            candidates.remove((x, y + 1))

        if (x, y - 1) in candidates:
            candidates.remove((x, y - 1))

    for _ in range(nscen):
        locs = []
        locs += [(0, x) for x in range(maxx)]  # top
        locs += [(y, maxx - 1) for y in range(1, maxy)]  # right
        locs += [(maxy - 1, x) for x in range(0, maxx - 1)]  # bot
        locs += [(y, 0) for y in range(1, maxy - 1)]  # left
        assert len(locs) == 36
        candidates = set(locs)
        slocs = []
        tlocs = []

        while len(slocs) != nagent:
            x1, y1 = random.sample(candidates, k=1)[0]  # type: ignore
            x2, y2 = maxx - 1 - x1, maxy - 1 - y1
            print(f"Pick ({x1}, {y1}), ({x2}, {y2}), size: {len(slocs)}", end="\r")
            if (x2, y2) in candidates:
                update_candidate(x1, y1, candidates)
                update_candidate(x2, y2, candidates)
                slocs.append((x1, y1))
                slocs.append((x2, y2))
                tlocs.append((x2, y2))
                tlocs.append((x1, y1))
        print("Done")
        starts.append(slocs)
        goals.append(tlocs)

    grid = [[0 for _ in range(maxx)] for _ in range(maxy)]
    maxt, vmax = 10, 2
    scen = create_scen(grid, starts, goals, maxt=maxt, vmax=vmax, t_reso=-1, nscen=nscen)
    scen["env"]["obsts"] = [[[4.5, 4.5], [4.5, 5.5], [5.5, 5.5], [5.5, 4.5]]]
    save_scen(scen, scen_name, scen_dir)


def scen_10x10_swap(nagent=4, seed=0, scen_dir="cbs-scen", nscen=1):
    maxx, maxy = 10, 10
    random.seed(seed)
    scen_name = f"n{nagent}-10x10-swap"

    starts: list[list[tuple[int, int]]] = []
    goals: list[list[tuple[int, int]]] = []

    def update_candidate(x, y, candidates):
        candidates.remove((x, y))
        if (x + 1, y) in candidates:
            candidates.remove((x + 1, y))

        if (x - 1, y) in candidates:
            candidates.remove((x - 1, y))

        if (x, y + 1) in candidates:
            candidates.remove((x, y + 1))

        if (x, y - 1) in candidates:
            candidates.remove((x, y - 1))

    for _ in range(nscen):
        locs = []
        locs += [(0, x) for x in range(maxx)]  # top
        locs += [(y, maxx - 1) for y in range(1, maxy)]  # right
        locs += [(maxy - 1, x) for x in range(0, maxx - 1)]  # bot
        locs += [(y, 0) for y in range(1, maxy - 1)]  # left
        assert len(locs) == 36
        candidates = set(locs)
        slocs = []
        tlocs = []

        while len(slocs) != nagent:
            x1, y1 = random.sample(candidates, k=1)[0]  # type: ignore
            x2, y2 = maxx - 1 - x1, maxy - 1 - y1
            print(f"Pick ({x1}, {y1}), ({x2}, {y2}), size: {len(slocs)}", end="\r")
            if (x2, y2) in candidates:
                update_candidate(x1, y1, candidates)
                update_candidate(x2, y2, candidates)
                slocs.append((x1, y1))
                slocs.append((x2, y2))
                tlocs.append((x2, y2))
                tlocs.append((x1, y1))
        print("Done")
        starts.append(slocs)
        goals.append(tlocs)

    grid = [[0 for _ in range(maxx)] for _ in range(maxy)]
    maxt, vmax = 10, 2
    scen = create_scen(grid, starts, goals, maxt=maxt, vmax=vmax, t_reso=-1, nscen=nscen)
    save_scen(scen, scen_name, scen_dir)


def gen_random_grid(maxx: int, maxy: int, ratio: float) -> list[list[int]]:
    g = [[0 for _ in range(maxx)] for _ in range(maxy)]

    obstnum = int(maxx * maxy * ratio)
    obstlocs = random.sample(list(product(range(1, maxx), range(1, maxy))), obstnum)
    for x, y in obstlocs:
        g[y][x] = 1
    return g


def gen_random_st(candidates: set[tuple[int, int]], n: int):
    assert len(candidates) >= 2 * n
    locs = random.sample(candidates, k=2 * n)  # type: ignore
    return locs[:n], locs[n:]


def grid2coord(g: list[list[int]]):
    maxx, maxy = len(g[0]), len(g)
    obsts_g = set([(y, x) for y, x in product(range(maxx), range(maxy)) if g[y][x] == 1])
    d = 0
    obsts: list[TP.Poly] = []

    # obsts: list[TP.Poly] = [
    #     [(x + d, y + d), (x + d, y + 1 - d), (x + 1 - d, y + 1 - d), (x + 1 - d, y + d)]
    #     for x, y in obsts_g
    # ]
    def grow_rect(x, y, xl, yl):
        while (x, y + yl) in obsts_g:
            yl += 1
        flag = True
        while flag:
            for _y in range(y, y + yl):
                if (x + xl, _y) not in obsts_g:
                    flag = False
            if flag:
                xl += 1
        return xl, yl

    while len(obsts_g) > 0:
        x, y = min(obsts_g)
        xl, yl = grow_rect(x, y, 1, 1)
        obsts.append(
            [
                (x + d, y + d),
                (x + d, y + yl - d),
                (x + xl - d, y + yl - d),
                (x + xl - d, y + d),
            ]
        )
        for i, j in product(range(x, x + xl), range(y, y + yl)):
            assert (i, j) in obsts_g
            obsts_g.remove((i, j))
    return obsts


def sparsify(
    xbound,
    ybound,
    locs: list[tuple[float, float]],
    clearance: float = 0.5,
    obsts: list[TP.Poly] = [],
    xf=1,
    yf=1,
):
    size = 0.5

    minx, maxx = xbound
    miny, maxy = ybound

    def neighbors(loc: tuple[float, float]):
        res = []
        for other in locs:
            thresh = size * 2 + clearance
            if other == loc:
                continue
            if abs(other[0] - loc[0]) < thresh and abs(other[1] - loc[1]) < thresh:
                res.append(other)
        return res

    def collideObst(loc: tuple[float, float]):
        px, py = loc
        thresh = size + clearance
        for obs in obsts:
            thresh = size + clearance
            cx = sum([x for x, _ in obs]) / len(obs)
            cy = sum([y for _, y in obs]) / len(obs)
            Dx = max([x for x, _ in obs]) - min([x for x, _ in obs])
            Dy = max([y for _, y in obs]) - min([y for _, y in obs])

            if abs(cx - px) < Dx / 2 + thresh and abs(cy - py) < Dy / 2 + thresh:
                return True
        return False

    while True:
        tmp_loc: dict[int, tuple[float, float]] = {}
        from math import sqrt

        for i, loc in enumerate(locs):
            nei = neighbors(loc)
            if len(nei) == 0 and not collideObst(loc):
                continue
            vx, vy = 0, 0
            if len(nei) > 0:
                vx = loc[0] - sum([x for x, _ in nei]) / len(nei)
                vy = loc[1] - sum([y for _, y in nei]) / len(nei)
                vn = sqrt(vx**2 + vy**2)
                if vn > 0:
                    vx /= vn
                    vy /= vn
            shift = random.uniform(-clearance, clearance)
            newx = loc[0] + vx * clearance + shift * xf
            newx = max(newx, minx)
            newx = min(newx, maxx)
            newy = loc[1] + vy * clearance + shift * yf
            newy = max(newy, miny)
            newy = min(newy, maxy)

            tmp_loc[i] = (newx, newy)
            print(f"sparsify: move {loc} to {tmp_loc[i]}")
            locs[i] = tmp_loc[i]
        if len(tmp_loc) == 0:
            break
        for i, loc in tmp_loc.items():
            locs[i] = loc

    return locs


def st2coord(s: list[tuple[int, int]], t: list[tuple[int, int]]):
    slocs: list[TP.Vert] = [(x + 0.5, y + 0.5) for x, y in s]
    tlocs: list[TP.Vert] = [(x + 0.5, y + 0.5) for x, y in t]
    return slocs, tlocs


def create_scen(
    grid: list[list[int]],
    starts: list[list[tuple[int, int]]],
    goals: list[list[tuple[int, int]]],
    t_reso=4,
    g_reso=4,
    maxt=1,
    vmax=-1,
    nscen=1,
) -> dict:
    obsts = grid2coord(grid)
    maxx, maxy = len(grid[0]), len(grid)
    # -1 means auto generate
    if vmax < 0:
        vmax = (maxx + maxy) / maxt
    if t_reso < 0:
        t_reso = vmax
    d = 0.5
    env = ut.Env(minx=-d, maxx=maxx + d, miny=-d, maxy=maxy + d, d_thres=1, maxt=maxt)
    env.set_obsts(obsts)

    def updateBound(xs, ys):
        env.minx = min(env.minx, min(xs) - 0.5)
        env.maxx = max(env.maxx, max(xs) + 0.5)
        env.miny = min(env.miny, min(ys) - 0.5)
        env.maxy = max(env.maxy, max(ys) + 0.5)

    exprs = []
    for i in range(nscen):
        slocs, tlocs = st2coord(starts[i], goals[i])
        slocs = sparsify((d, maxx - d), (d, maxy - d), slocs, 0.5, obsts, xf=0, yf=1)
        tlocs = sparsify((d, maxx - d), (d, maxy - d), tlocs, 0.5, obsts, xf=0, yf=1)
        s_x = [x for x, _ in slocs]
        s_y = [y for _, y in slocs]
        d_x = [x for x, _ in tlocs]
        d_y = [y for _, y in tlocs]
        updateBound(s_x, s_y)
        updateBound(d_x, d_y)
        exprs.append(dict(s_x=s_x, s_y=s_y, d_x=d_x, d_y=d_y))
    res = dict(env=env.to_dict(), v_max=vmax, t_reso=t_reso, g_reso=g_reso)
    res["scens"] = exprs  # type: ignore
    return res


def save_scen(scen: dict, fname: str, dname: str = "cbs-scen"):
    if not os.path.exists(f"{dname}/{fname}"):
        os.makedirs(f"{dname}/{fname}")
    json.dump(scen, open(f"{dname}/{fname}/data.json", "w"), indent=2)

    from cbs.plot import plot_scen, plt

    env = ut.Env(**scen["env"])
    for sid, scen in enumerate(scen["scens"]):
        fig, _ = plot_scen(env, scen["s_x"], scen["s_y"], scen["d_x"], scen["d_y"])
        fig.savefig(f"{dname}/{fname}/{sid}.jpg", bbox_inches="tight")
        plt.close()


def gen_gcs_expr_scens():
    dname = "gcs-expr-scen"
    nscen = 1
    for k in [2, 4, 6]:
        scen_10x10_swap(k, scen_dir=dname, nscen=nscen)
        scen_10x10_midswap(k, scen_dir=dname, nscen=nscen)


if __name__ == "__main__":
    gen_gcs_expr_scens()
