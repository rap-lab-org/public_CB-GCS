#!/usr/bin/env python
import sys
import os
import json
import pickle

_current = os.path.dirname(os.path.realpath(__file__))
# directory reach
_parent = os.path.dirname(_current)
if _parent not in sys.path:
    sys.path.append(_parent)
if _current not in sys.path:
    sys.path.append(_current)

import cbs.utils as ut
from cbs.search import GcsCbsSearch
from opts import args

GLOBAL_T_RESO: int = 4
GLOBAL_STEPSIZE: float = 0.2
GLOBAL_MAX_T: float = -1.0
GLOBAL_MAX_V: float = -1.0
GLOBAL_TLIMIT: float = 100.0


def run_scen(
    env: ut.Env,
    sx: list[float],
    sy: list[float],
    dx: list[float],
    dy: list[float],
    v_max,
    sid: int,
    dir: str,
    tLimit=300.0,
    stepsize=0.2,
    g_reso=4,
    C=ut.Mod.LB,
    S=ut.SMD.ANYFOCAL,
    minT=False,
):
    resname = f"{dir}/{sid}-{C.name}-{S.name}-UB"
    p = GcsCbsSearch(env, sx, sy, dx, dy, v_max, stepsize=stepsize, g_reso=g_reso)
    p.conflict_mode = C
    p.search_method = S
    p.lowlevel_minT = minT
    p.logpath = f"{resname}.log"

    if os.path.exists(p.logpath):
        print(f"clean existing log: [{p.logpath}]")
        os.remove(p.logpath)
    try:
        res = p.Search(tLimit)
    except Exception as e:
        import traceback

        dumpname = f"{dir}/{sid}-debug.dump"
        print(f"Traceback has been saved to file '{dumpname}.traceback'")
        traceback.print_exc(file=open(f"{dumpname}.traceback", "w"))

        pickle.dump(p, open(dumpname, "wb"))
        print(f"Context has been dumped to file '{dumpname}'")
        raise e

    # save search instance to file for future analysis
    pickle.dump(p, open(f"{resname}.dump", "wb"))
    resDict = res.to_dict()
    resDict["scale"] = 1
    json.dump(resDict, open(f"{resname}.json", "w"), indent=2)

    print(f"lb:{resDict['lb']}, runtime:{resDict['ub']}, cScore:{resDict['best_cScore']}")


def run_cbs(
    dir: str,
    tLimit: float = 300,
    mod="ub",
    scenid=-1,
    minT=False,
):
    dat = json.load(open(f"{dir}/data.json", "r"))
    v_max = dat["v_max"]
    env = ut.Env(**dat["env"])
    env.update_inflation_buf(v_max * GLOBAL_STEPSIZE / 2)
    if GLOBAL_MAX_T > 0:
        env.maxt = GLOBAL_MAX_T
    if GLOBAL_MAX_V > 0:
        v_max = GLOBAL_MAX_V
    g_reso = 0
    C = ut.Mod.UB if mod == "ub" else ut.Mod.LB
    scens = range(len(dat["scens"])) if scenid == -1 else [scenid]
    for sid in scens:
        print(f"Running {dir} in case {sid}...")
        scen = dat["scens"][sid]
        sx, sy = scen["s_x"], scen["s_y"]
        dx, dy = scen["d_x"], scen["d_y"]
        run_scen(
            env,
            sx,
            sy,
            dx,
            dy,
            v_max,
            sid,
            dir,
            # scale=scale,
            tLimit=tLimit,
            stepsize=GLOBAL_STEPSIZE,
            g_reso=g_reso,
            C=C,
            minT=minT,  # subset=set([1, 2, 3])
        )


def gcs_exprs():
    exprs = [
        "./gcs-expr-scen/n2-10x10-swap",
        "./gcs-expr-scen/n4-10x10-swap",
        "./gcs-expr-scen/n6-10x10-swap",
        "./gcs-expr-scen/n2-10x10-midswap",
        "./gcs-expr-scen/n4-10x10-midswap",
        "./gcs-expr-scen/n6-10x10-midswap",
    ]
    return exprs


def simple_lb(expr_opt="large"):
    expr_opts = {"gcs": gcs_exprs()}
    exprs = expr_opts[expr_opt]
    res = {}

    def exprlb(dir):
        import gcs_adapter as lowlevel

        dat = json.load(open(f"{dir}/data.json", "r"))
        v_max = dat["v_max"]
        env = ut.Env(**dat["env"])
        # env.update_inflation_buf(v_max*GLOBAL_STEPSIZE/2)
        lbs = {}
        gap = 0.05
        for sid in range(len(dat["scens"])):
            scen = dat["scens"][sid]
            sx, sy = scen["s_x"], scen["s_y"]
            dx, dy = scen["d_x"], scen["d_y"]
            p = GcsCbsSearch(env, sx, sy, dx, dy, v_max, stepsize=GLOBAL_STEPSIZE)
            lbs[sid] = 0.0
            for i, _ in enumerate(sx):
                res = lowlevel.solve(
                    sx[i],
                    sy[i],
                    dx[i],
                    dy[i],
                    [],
                    v_max,
                    p.T,
                    p.env,
                    gap=gap,
                    ensureFeasible=False,
                )
                lbs[sid] += res.lb * (1.0 - gap)
        print(lbs)
        return lbs

    for expr in exprs:
        res[expr] = exprlb(expr)

    json.dump(res, open("simple-lb.json", "w"), indent=2)


def main(mod="ub"):
    global GLOBAL_MAX_T, GLOBAL_STEPSIZE, GLOBAL_MAX_V, GLOBAL_TLIMIT

    GLOBAL_STEPSIZE = args.stepsize
    if args.maxt > 0:
        GLOBAL_MAX_T = args.maxt
    if args.maxv is not None:
        GLOBAL_MAX_V = args.maxv
    if args.tlimit > 0:
        GLOBAL_TLIMIT = args.tlimit
    exprs = gcs_exprs()
    for expr in exprs:
        run_cbs(expr, tLimit=GLOBAL_TLIMIT, mod=mod)


if __name__ == "__main__":
    mod = "lb" if args.lowerbound is True else "ub"
    main(mod)
