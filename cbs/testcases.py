import sys
import os

_current = os.path.dirname(os.path.realpath(__file__))
# directory reach
_parent = os.path.dirname(_current)
if _parent not in sys.path:
    sys.path.append(_parent)
if _current not in sys.path:
    sys.path.append(_current)

from cbs.utils import Mod
from cbs.search import RunCBS
import cbs.utils as ut
import cbs.search as cs
from cbs.expr import runMICPMAPF


def test0():
    """
    Single agent move from a to b
    """
    name = "test0"
    scale = 10
    env = ut.Env(maxx=3, maxy=3, maxt=4, d_thres=1)

    s_x = [0.5]
    s_y = [2.5]
    d_x = [2.5]
    d_y = [0.5]

    v_max = 1.0

    time_limit = 3600  # 300 # 5 min

    res = cs.RunCBS(
        env, s_x, s_y, d_x, d_y, v_max, time_limit, scale=scale, name=name
    )  # lower bound

    print("GCS-CBS returns: ", res)
    micpres = runMICPMAPF(
        env, s_x, s_y, d_x, d_y, v_max, time_limit, scale=scale, name=name
    )


def test1():
    env = ut.Env(maxx=3, maxy=3, maxt=8, d_thres=1)
    scale = 10
    name = "test1"
    # print("envs = ", envs)

    s_x = [0.5, 0.5]
    s_y = [2.5, 0.5]
    d_x = [2.5, 2.5]
    d_y = [0.5, 2.5]

    v_max = 1

    time_limit = 300  # 300 # 5 min

    res = cs.RunCBS(
        env,
        s_x,
        s_y,
        d_x,
        d_y,
        v_max,
        time_limit,
        t_reso=18,
        g_reso=0,
        scale=scale,
        conflict_mode=ut.Mod.UB,
        name=name,
        minT=False
    )  # lower bound

    print("GCS-CBS returns: ", res)

    # micpres = runMICPMAPF(
    #     env, s_x, s_y, d_x, d_y, v_max, time_limit, scale=scale, name=name
    # )
    # print(micpres)


def test2():
    envs = ut.Env(maxx=5, maxy=5, maxt=5, d_thres=1)

    s_x = [1, 3]
    s_y = [1, 2]
    d_x = [4, 2]
    d_y = [4, 3]

    v_max = 1

    time_limit = 1000  # 300 # 5 min

    res = RunCBS(
        envs,
        s_x,
        s_y,
        d_x,
        d_y,
        v_max,
        time_limit,
        t_reso=3,
        g_reso=10,
        conflict_mode=Mod.UB,
        scale=10,
        name="test2",
    )  # lower bound

    print("GCS-CBS returns: ", res)


def test3():
    """
    two agent swap their location in empty workspace,
    optimal dist = 4.84
    """
    envs = ut.Env(maxx=3, maxy=3, maxt=1, d_thres=1)

    s_x = [0.5, 2.5]
    s_y = [1.5, 1.5]
    d_x = [2.5, 0.5]
    d_y = [1.5, 1.5]

    v_max = 3.0

    time_limit = 300  # 300 # 5 min

    res = cs.RunCBS(
        envs,
        s_x,
        s_y,
        d_x,
        d_y,
        v_max,
        time_limit,
        t_reso=8,
        g_reso=0,
        scale=10,
        conflict_mode=ut.Mod.UB,
        minT=False,
        name="test3",
    )  # lower bound

    print("GCS-CBS returns: ", res)


def test3_2():
    """
    two agent swap their location in empty workspace,
    optimal dist = 4.84
    """
    envs = ut.Env(maxx=5, maxy=5, maxt=3, d_thres=1)

    s_x = [1, 4]
    s_y = [1, 4]
    d_x = [4, 1]
    d_y = [4, 1]

    v_max = 2.0

    time_limit = 3000  # 300 # 5 min

    res = RunCBS(
        envs,
        s_x,
        s_y,
        d_x,
        d_y,
        v_max,
        time_limit,
        t_reso=4,
        g_reso=0,
        scale=10,
        conflict_mode=Mod.UB,
        fname="test3_2",
    )  # lower bound

    print("GCS-CBS returns: ", res)


def test4():
    """
    four agent swap their location in empty workspace
    """
    env = ut.Env(maxx=5, maxy=5, maxt=3, d_thres=1)
    scale = 10
    name = "test4"

    s_x = [1, 4, 1, 4]
    s_y = [1, 4, 4, 1]
    d_x = [4, 1, 4, 1]
    d_y = [4, 1, 1, 4]

    v_max = 2

    time_limit = 300  # 300 # 5 min

    res = RunCBS(
        env,
        s_x,
        s_y,
        d_x,
        d_y,
        v_max,
        time_limit,
        t_reso=4,
        g_reso=0,
        scale=scale,
        conflict_mode=Mod.UB,
        name=name,
    )  # lower bound

    print("GCS-CBS returns: ", res)

    # micpres = runMICPMAPF(env, s_x, s_y, d_x, d_y, v_max, time_limit, scale=scale, name=name)
    # print (micpres)


def test4_2():
    """
    four agent swap their location in empty workspace
    """
    env = ut.Env(maxx=10, maxy=10, maxt=6, d_thres=1)

    s_x = [1, 9, 1, 9]
    s_y = [1, 9, 9, 1]
    d_x = [9, 1, 9, 1]
    d_y = [9, 1, 1, 9]
    scale = 10
    name = "test4_2"

    v_max = 2

    time_limit = 3000  # 300 # 5 min

    # res = RunCBS(
    #     env,
    #     s_x,
    #     s_y,
    #     d_x,
    #     d_y,
    #     v_max,
    #     time_limit,
    #     t_reso=4,
    #     g_reso=0,
    #     scale=scale,
    #     conflict_mode=Mod.UB,
    #     name=name,
    # )  # lower bound
    # print("GCS-CBS returns: ", res)

    micpres = runMICPMAPF(
        env, s_x, s_y, d_x, d_y, v_max, time_limit, scale=scale, name=name
    )
    print(micpres)


def test_obs_1():
    envs = ut.Env(maxx=4, maxy=4, maxt=1, d_thres=1)
    envs.set_obsts([[(1.5, 1), (1.5, 2), (2.5, 2), (2.5, 1)]])
    # print("envs = ", envs)

    s_x = [0.5, 3.5]
    s_y = [1.5, 1.5]
    d_x = [3.5, 0.5]
    d_y = [1.5, 1.5]

    scale = 10
    name = "obs_1"
    v_max = 5.0

    time_limit = 300  # 300 # 5 min

    res = RunCBS(
        envs,
        s_x,
        s_y,
        d_x,
        d_y,
        v_max,
        time_limit,
        scale=scale,
        name=name,
        t_reso=8,
        g_reso=3,
        conflict_mode=Mod.UB,
    )

    print("GCS-CBS returns: ", res)


    # micpres = runMICPMAPF(
    #     envs, s_x, s_y, d_x, d_y, v_max, time_limit, scale=scale, name=name
    # )
    # print(micpres)


def test_obs_2():
    envs = ut.Env(maxx=4, maxy=4, maxt=1, d_thres=1)
    envs.set_obsts([[(2, 1), (2, 2), (3, 2), (3, 1)]])
    # print("envs = ", envs)

    s_x = [0.5, 3.5]
    s_y = [1.5, 1.5]
    d_x = [3.5, 0.5]
    d_y = [1.5, 1.5]

    v_max = 5.0

    time_limit = 300  # 300 # 5 min

    res = RunCBS(envs, s_x, s_y, d_x, d_y, v_max, time_limit, t_reso=8, g_reso=3)

    print("GCS-CBS returns: ", res)


def test_obs_3():
    envs = ut.Env(maxx=4, maxy=4, maxt=1, d_thres=1)
    envs.set_obsts([[(2, 1), (2, 2), (2.5, 2), (2.5, 1)]])
    scale = 10
    name = "obs_3"
    # print("envs = ", envs)

    s_x = [0.5, 3.5]
    s_y = [1.5, 1.5]
    d_x = [3.5, 0.5]
    d_y = [1.5, 1.5]

    v_max = 5.0

    time_limit = 300  # 300 # 5 min

    # res = RunCBS(envs, s_x, s_y, d_x, d_y, v_max, time_limit, t_reso=8, g_reso=3,
    #                 name=name, scale=scale)
    #
    # print("GCS-CBS returns: ", res)

    micpres = runMICPMAPF(
        envs, s_x, s_y, d_x, d_y, v_max, time_limit, scale=scale, name=name
    )
    print(micpres)


def test_more_agent():
    envs = ut.Env(maxx=4, maxy=4, maxt=1, d_thres=1)
    envs.set_obsts([[(1.5, 1), (1.5, 2), (2.5, 2), (2.5, 1)]])
    # print("envs = ", envs)

    s_x = [0.5, 3.5, 2.0]
    s_y = [1.5, 1.5, 0.5]
    d_x = [3.5, 0.5, 2.0]
    d_y = [1.5, 1.5, 3.5]

    v_max = 5.0

    time_limit = 300  # 300 # 5 min

    res = RunCBS(envs, s_x, s_y, d_x, d_y, v_max, time_limit, t_reso=16, g_reso=3)

    print("GCS-CBS returns: ", res)


def test_rnd_obsts_20():
    import json
    dir = "./cbs-scen/n3-10x10-20"
    dat = json.load(open(f"{dir}/data.json", "r"))
    tLimit = 300
    scale = 10
    env = ut.Env(**dat["env"])
    v_max = dat["v_max"]
    t_reso = v_max
    g_reso = 0

    sid = 1
    name = f"test_rnd_obsts_{sid}"
    scen = dat["scens"][sid]
    sx, sy = scen["s_x"], scen["s_y"]
    dx, dy = scen["d_x"], scen["d_y"]

    res = RunCBS(
        env,
        sx,
        sy,
        dx,
        dy,
        v_max,
        tLimit,
        t_reso=t_reso,
        g_reso=g_reso,
        scale=scale,
        conflict_mode=Mod.UB,
        minT=True,
        name=name,
    )  # lower bound
    print("GCS-CBS returns: ", res)


if __name__ == "__main__":
    test3()
    # test_rnd_obsts_20()
