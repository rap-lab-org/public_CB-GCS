import copy
from itertools import product
from math import ceil, sqrt
import numpy as np
import time
import sys

import os

_current = os.path.dirname(os.path.realpath(__file__))
# directory reach
_parent = os.path.dirname(_current)
if _parent not in sys.path:
    sys.path.append(_parent)
if _current not in sys.path:
    sys.path.append(_current)

import cbs.utils as ut
import cbs.conflict as cf
from cbs.utils import CbsConstraint, Mod, SMD, SearchStatus
from cbs.type_stubs import Poly, VertPath

######
DEBUG_PRINT = True
######


class GcsCBSRes:
    def __init__(self):
        self.open_empty = False
        self.planner = "GCS-CBS"
        self.run_time: float = 0.0
        self.round: int = 0
        self.n_exp: int = 0
        self.sol_nid: int = 0
        self.last_nid: int = 0
        self.best_nid: int = 0
        self.best_g: float = 0
        self.best_cScore: float = 0
        self.lb_cost: float = -1
        self.lb_collision: float = np.inf
        self.ub_cost: float = np.inf
        self.ub_collision: float = np.inf
        self.all_paths: dict[int, VertPath] = {}
        self.return_status: SearchStatus = SearchStatus.UNSTARTED

    def __str__(self):
        return f"lb:({self.lb_cost:.3f}, {self.lb_collision:.3f}), ub:({self.ub_cost:.3f}, {self.ub_collision:.3f})"

    def __repr__(self):
        return self.__str__()

    def to_dict(self):
        return dict(
            round=self.round,
            n_exp=self.n_exp,
            solid=self.sol_nid,
            lastid=self.last_nid,
            best_nid=self.best_nid,
            best_g=self.best_g,
            best_cScore=self.best_cScore,
            lb=self.lb_cost,
            lb_collision=self.lb_collision,
            ub=self.ub_cost,
            ub_collision=self.ub_collision,
            paths=self.all_paths,
            return_status=self.return_status.name
        )

    def updateSol(self, nd: ut.GcsCbsNode):
        self.return_status = SearchStatus.FINISH
        self.sol_nid = nd.id
        self.all_paths = nd.sol.paths
        self.updateBound(nd)

    def updateBound(self, nd: ut.GcsCbsNode):
        if nd.mod != Mod.UB and self.lb_cost <= nd.g:
            self.lb_cost = nd.g
            self.lb_collision = min(self.lb_collision, nd.cScore)
        if nd.cScore == 0.0 and self.ub_cost >= nd.g:
            self.ub_cost = nd.g
            self.ub_collision = min(self.ub_collision, nd.cScore)


class GcsCbsSearch:
    """ """

    def __init__(
        self, env: ut.Env, s_x, s_y, d_x, d_y, v_max, stepsize: float = 0.2, g_reso: int = 1
    ):
        """
        s_x is a list of starting x-coord of all agents.
        s_y is a list of starting y-coord of all agents.

        d_x is a list of destination x-coord of all agents.
        d_y is a list of destination y-coord of all agents.

        v_max = robot speed limit.

        """
        self.env = env
        self.minx = env.minx
        self.maxx = env.maxx
        self.miny = env.miny
        self.maxy = env.maxy
        self.d_thres: float = env.d_thres
        # original obstacles
        self.obsts: list[Poly] = env.obsts
        # inflated obstacles
        self.inflated_obsts: list[Poly] = env.inflated_obsts

        numSteps = ceil(env.maxt / stepsize) + 1
        self.T: list[float] = list(np.linspace(0, env.maxt, numSteps + 1))

        if g_reso == 0:
            g_reso = ut.auto_g_resolution(self.T, v_max, env)
            print("auto set g reso to ", g_reso)
        self.x_ticks: list[float] = list(
            np.linspace(self.minx, self.maxx, int(self.maxx / self.d_thres) * g_reso + 1)
        )

        self.y_ticks: list[float] = list(
            np.linspace(self.miny, self.maxy, int(self.maxy / self.d_thres) * g_reso + 1)
        )
        self.n_col = len(self.x_ticks)
        self.num_robots = len(s_x)
        self.s_x = s_x
        self.s_y = s_y
        self.d_x = d_x
        self.d_y = d_y
        self.start_index: list[int] = []
        # TODO: add static obstacles, currently is empty

        self.v_max: float = v_max
        self.nodes: dict[int, ut.GcsCbsNode] = {}  # high level nodes
        self.open_list = ut.PrioritySet()

        self.focal_list = ut.PrioritySet()
        self.C = np.inf

        self.node_id_gen = 1
        self.resDat = GcsCBSRes()

        self.conflict_mode: Mod = Mod.LB
        self.search_method: SMD = SMD.ANYFOCAL
        # whether tie-breaking by minT
        self.lowlevel_minT: bool = False
        self.logpath: None | str = None
        self.checkInstance()

        return

    def checkInstance(self):
        """ """
        for i in range(self.num_robots):
            for j in range(i + 1, self.num_robots):
                xi, yi = self.s_x[i], self.s_y[i]
                xj, yj = self.s_x[j], self.s_y[j]
                if abs(xi - xj) < self.d_thres and abs(yi - yj) < self.d_thres:
                    print(
                        "GCS-CBS initial check, agents ",
                        i,
                        j,
                        " collision at start",
                    )
                    sys.exit()

                xi, yi = self.d_x[i], self.d_y[i]
                xj, yj = self.d_x[j], self.d_y[j]

                if abs(xi - xj) < self.d_thres and abs(yi - yj) < self.d_thres:
                    print(
                        "GCS-CBS initial check, agents ",
                        i,
                        j,
                        " collision at target",
                    )
                    sys.exit()
        print("GCS-CBS initial check passed")
        return

    def SetConflictMode(self, mode: Mod):
        """ """
        self.conflict_mode = mode

    def BacktrackCstrs(
        self, ri: int, nid: int
    ) -> tuple[list[CbsConstraint], list[CbsConstraint]]:
        """
        given a node, trace back to the root, find all constraints relavant.
        """
        node_cs: list[CbsConstraint] = list()
        swap_cs: list[CbsConstraint] = list()
        cid = nid
        while cid != -1:
            for cstr in self.nodes[cid].cstrs:
                if cstr.i != ri:  # not a constraint for robot i
                    continue
                node_cs.append(cstr)
            cid = self.nodes[cid].parent
        return node_cs, swap_cs

    def InitSearch(self):
        """
        called at the beginning of the search.
        generate first High level node.
        compute individual optimal path for each robot.
        """
        if DEBUG_PRINT:
            print("CBS init search")
        nid = self.node_id_gen
        self.nodes[nid] = ut.GcsCbsNode(nid, 0, mod=Mod.LB)
        self.node_id_gen = self.node_id_gen + 1
        self.open_list = ut.PrioritySet()

        fea_cost_sum = 0
        for ri in range(self.num_robots):  # loop over agents, plan their paths
            if DEBUG_PRINT:
                print(
                    "GCS-CBS will call GCS with inputs",
                    # ", cstrs = ",
                    # node_cs,
                    ", vmax = ",
                    self.v_max,
                    ", s_xy = ",
                    self.s_x[ri],
                    self.s_y[ri],
                    ", d_xy = ",
                    self.d_x[ri],
                    self.d_y[ri],
                    ",rid = ",
                    ri,
                )
                print("GCS-CBS conflict mode = ", self.conflict_mode)

            import gcs_adapter as lowlevel

            res = lowlevel.solve(
                self.s_x[ri],
                self.s_y[ri],
                self.d_x[ri],
                self.d_y[ri],
                [],
                self.v_max,
                self.T,
                self.env,
                minT=self.lowlevel_minT,
                timeLimit=self.time_limit / self.num_robots,
                ensureFeasible=self.conflict_mode == Mod.UB,
                # gap=-1 if self.conflict_mode == Mod.LB else 0.05,
                gap=0.05,
            )
            fea_cost_sum += res.lb
            if len(res.path) == 0:
                return False  # in this branch, the low level search fails.

            # self.nodes[nid].sol.DelPath(ri)
            self.nodes[nid].sol.AddPath(
                ri, res.path, self.T, res.lb
            )  # inf end is add here.

        self.nodes[nid].g = self.nodes[nid].sol.TotalCost()
        self.nodes[nid].collisionScore(self.d_thres)
        self.resDat.updateBound(self.nodes[nid])
        self.open_list.add(self.nodes[nid].g, self.nodes[nid].id)  # add to OPEN

        return

    def Lsearch(
        self,
        ri: int,
        cstrs: list[ut.CbsConstraint],
        curr_node: ut.GcsCbsNode,
        mod: Mod = Mod.LB,
    ) -> ut.GcsCbsNode | None:
        """
        low level search
        """
        node_cs, _ = self.BacktrackCstrs(ri, curr_node.id)

        if DEBUG_PRINT:
            print(
                "GCS-CBS will call GCS with inputs",
                # ", cstrs = ",
                # node_cs,
                ", vmax = ",
                self.v_max,
                ", s_xy = ",
                self.s_x[ri],
                self.s_y[ri],
                ", d_xy = ",
                self.d_x[ri],
                self.d_y[ri],
                ",node id = ",
                curr_node.id,
                ",rid = ",
                ri,
            )
            print("GCS-CBS conflict mode = ", self.conflict_mode)

        # create new node first
        # if low level faild, we can retrieve the new node from dump, for debug purpose
        new_id = self.node_id_gen
        self.node_id_gen = self.node_id_gen + 1
        new_nd = copy.deepcopy(curr_node)
        new_nd.id = new_id
        self.nodes[new_nd.id] = new_nd  # add to self.nodes
        new_nd.parent = curr_node.id
        new_nd.cstrs = cstrs  # a list of constraints now @2024-01

        import gcs_adapter as lowlevel

        all_cstrs = node_cs + cstrs
        # ensureFeasible = True
        # TODO: if this expansion is improving lower bound,
        # then turn off feasible
        # if curr_node.mod == Mod.LB and mod == Mod.LB:
        #     ensureFeasible = False
        minT = curr_node.cScore <= 100
        res = lowlevel.solve(
            self.s_x[ri],
            self.s_y[ri],
            self.d_x[ri],
            self.d_y[ri],
            all_cstrs,
            self.v_max,
            self.T,
            self.env,
            # minT=self.lowlevel_minT,
            minT=minT,
            # gap=-1 if mod == Mod.LB else 0.05,
            gap=0.05,
            timeLimit=self.time_limit,
            ensureFeasible=self.conflict_mode == Mod.UB,
            verify=False,
        )

        if len(res.path) == 0:
            return None  # in this branch, the low level search fails.

        new_nd.sol.DelPath(ri)
        new_nd.sol.AddPath(ri, res.path, res.ts, res.lb)  # inf end is add here.
        new_nd.g = new_nd.sol.TotalCost()  # or len(new_path)-1 ? TODO check
        new_nd.collisionScore(self.d_thres)
        return new_nd

    def FirstConflictSharedTimeSteps(
        self, nd: ut.GcsCbsNode, mod: Mod, UBlevel: int = 0
    ) -> tuple[float, dict[int, list[ut.CbsConstraint]]]:
        cScore, branches = 0.0, {}
        for r1, r2 in product(nd.sol.paths, nd.sol.paths):
            if r1 >= r2:
                continue
            tScore, tBranches = cf.CheckConflictSharedTimeSteps(
                self.x_ticks,
                self.y_ticks,
                nd.sol.paths,
                r1,
                r2,
                d_thres=self.d_thres,
                M=mod,
                UBlevel=UBlevel,
            )
            if tScore > cScore and len(tBranches) > 0:
                cScore, branches = tScore, tBranches
        return cScore, branches

    def _isTimeOut(self) -> bool:
        tnow = time.perf_counter()
        if tnow - self.tstart > self.time_limit:
            print(" GCS-CBS timeout! ")
            return True
        return False

    def _writeSolLog(self):
        if self.logpath is not None:
            t = time.perf_counter() - self.tstart
            lb = self.resDat.lb_cost
            ub = self.resDat.ub_cost
            with open(self.logpath, "a") as f:
                f.write(f"{t:.3f},{lb:.3f},{ub:.3f}\n")

    def _reachGoal(self, nd: ut.GcsCbsNode):
        self.resDat.updateSol(nd)
        self._writeSolLog()
        if DEBUG_PRINT:
            print(
                f"Find a solution, lb=({self.resDat.lb_cost:.2f}, {self.resDat.lb_collision:.2f})",
                f"ub=({self.resDat.ub_cost:.2f}, {self.resDat.ub_collision:.2f}) g={nd.g:.2f} c={nd.cScore:.2f}",
            )

    def _BestFirstSearch(self):
        while self.open_list.size() > 0:
            self.resDat.round += 1
            fval, nid = self.open_list.pop()  # pop_node = (f-value, high-level-node-id)
            curr_node = self.nodes[nid]
            if DEBUG_PRINT:
                print("### GCS-CBS popped node: ", curr_node, " priority = ", fval)

            _, branches = self.FirstConflictSharedTimeSteps(curr_node, self.conflict_mode)

            if len(branches) == 0:  # no conflict, terminates
                self._reachGoal(curr_node)
                break

            self.resDat.n_exp += 1
            for robot_id, cstrs in branches.items():
                if DEBUG_PRINT:
                    print("GCS-CBS loop over cstr:", cstrs)
                newNd = self.Lsearch(robot_id, cstrs, curr_node)
                if newNd is None:
                    continue
                self.open_list.add(newNd.g, newNd.id)  # add to OPEN
                print("GCS-CBS added to open ", newNd)

            if self._isTimeOut():
                self.resDat.return_status = SearchStatus.TIMEOUT
                break

    def _UpdateFocalList(self, fBound: float):
        """
        move all nodes with f <= fBound from open to focal
        """
        while self.open_list.size() > 0:
            f, nid = self.open_list.top()
            if f <= fBound:
                self.open_list.remove(nid)
                cScore = self.nodes[nid].collisionScore(self.env.d_thres)
                gScore = self.nodes[nid].g
                # max gvalue first then min collision
                self.focal_list.add(cScore, nid)
                print("  add to focal, nid: ", nid, "f:", gScore, "c:", cScore)
            else:
                break

    def _AnyTimeFocalSearch(self) -> bool:
        w: float = np.inf
        _EPS = 1e-2
        flag = False
        while w >= 1:
            if not (self._FocalSearch(w)) or self._isTimeOut():
                break
            flag = True
            w = max(1.0, self.resDat.ub_cost / (self.resDat.lb_cost + _EPS))
            # maintain invariant of focal list, i.e.,
            # all fval must in the new bound
            ids = copy.copy(self.focal_list.set_)
            for nid in ids:
                nd = self.nodes[nid]
                if nd.g >= self.resDat.ub_cost:
                    self.focal_list.remove(nid)
        if not flag:
            obj = np.inf
            cScore = np.inf
            nid = -1
            for n in self.nodes.values():
                if n.cScore < cScore:
                    cScore = n.cScore
                    obj = n.g
                    nid = n.id
                elif n.cScore == cScore and obj > n.g:
                    obj = n.g
                    nid = n.id
            self.resDat.updateSol(self.nodes[nid])
            self.resDat.best_nid = nid
            self.resDat.best_g = self.nodes[nid].g
            self.resDat.best_cScore = self.nodes[nid].cScore
        else:
            self.resDat.best_nid = self.resDat.sol_nid
            self.resDat.best_g = self.nodes[self.resDat.sol_nid].g
            self.resDat.best_cScore = self.nodes[self.resDat.sol_nid].cScore
        return flag

    def _FocalSetNodeMod(self, pid: int, nid: int, mod: Mod):
        # no parent, or parent mod and constraint mod are same, or constraint mod is UB
        if pid == -1 or self.nodes[pid].mod == mod or mod == Mod.UB:
            self.nodes[nid].mod = mod
        # no matter what's the constraint mod, if parent is UB then child must be UB
        elif self.nodes[pid].mod == Mod.UB:
            self.nodes[nid].mod = Mod.UB
        # then must in case: LB parent and LB constraint
        else:
            self.nodes[nid].mod = Mod.LB

    def _FocalExpd(
        self, curNd: ut.GcsCbsNode, branches: dict[int, list[CbsConstraint]], mod: Mod
    ) -> bool:
        self.resDat.n_exp += 1
        for rid, cstrs in branches.items():
            newNd = self.Lsearch(rid, cstrs, curNd, mod)
            if newNd is None or newNd.g >= self.resDat.ub_cost:
                continue
            self._FocalSetNodeMod(newNd.parent, newNd.id, mod)
            self.open_list.add(newNd.g, newNd.id)  # add to OPEN
            print("CBS added to open, nid: ", newNd.id, "f:", newNd.g, "c:", newNd.cScore)
            if newNd.cScore == 0.0:
                return True
            if self._isTimeOut():
                self.resDat.return_status = SearchStatus.TIMEOUT
                return False
        return True

    def _FocalFinish(
        self, curNd: ut.GcsCbsNode, branches: dict[int, list[CbsConstraint]]
    ) -> bool:
        if len(branches) == 0:  # no conflict
            if self.conflict_mode != Mod.LB:
                assert curNd.collisionScore(self.d_thres) <= 1e-4
            self._reachGoal(curNd)
            return True
        return False

    def _FocalFmin(self):
        fmin = np.inf
        for nid in self.focal_list.set_:
            fmin = min(fmin, self.nodes[nid].g)
        return fmin

    def _FocalSearch(self, w: float) -> bool:
        fmin = self._FocalFmin()
        while self.focal_list.size() + self.open_list.size() > 0:
            self.resDat.round += 1
            # update focal if openlist is not empty
            if self.open_list.size() > 0:
                fmin, _ = self.open_list.top()
                print("### Focal fmin:", f"{fmin:.2f}")
            # guarantee node with [fmin, w*fmin] are move from open to focal
            self._UpdateFocalList(w * fmin)
            assert self.focal_list.size() > 0

            _, nid = self.focal_list.pop()
            self.resDat.last_nid = nid
            curNd = self.nodes[nid]

            if DEBUG_PRINT:
                print(
                    f"### GCS-CBS popped node: {curNd}",
                    f" f={curNd.g:.2f}",
                    f" c={curNd.cScore:.2f}",
                )

            if self.conflict_mode == Mod.LB:
                _, branches = self.FirstConflictSharedTimeSteps(curNd, self.conflict_mode)
                if self._FocalFinish(curNd, branches):
                    return True
                # check if timeout
                if not self._FocalExpd(curNd, branches, self.conflict_mode):
                    return False
            else:
                # type1 branch: upper bound constraints - large
                _, brUB = self.FirstConflictSharedTimeSteps(curNd, Mod.UB, UBlevel=1)
                if self._FocalFinish(curNd, brUB):
                    return True
                # type1.2 branch: upper bound constraints - large with longer time block
                _, brUB = self.FirstConflictSharedTimeSteps(curNd, Mod.UB, UBlevel=4)
                if self._FocalFinish(curNd, brUB):
                    return True
                # check if timeout
                if not self._FocalExpd(curNd, brUB, Mod.UB):
                    return False
                # type 2 branch: upper bound constraints - small
                _, brUB2 = self.FirstConflictSharedTimeSteps(curNd, Mod.UB, UBlevel=0)
                if len(brUB2) > 0:
                    if not self._FocalExpd(curNd, brUB2, Mod.UB):
                        return False
                # type3 branch: lower bound constraints
                _, brLB = self.FirstConflictSharedTimeSteps(curNd, Mod.LB)
                # LB conflict might be empty
                if len(brLB) > 0:
                    # check if timeout
                    if not self._FocalExpd(curNd, brLB, Mod.LB):
                        return False
        return False

    def Search(self, time_limit):
        """
        = high level search
        """
        if DEBUG_PRINT:
            print("GCS-CBS search begin!")
        self.time_limit = time_limit
        self.tstart = time.perf_counter()
        self.resDat.__init__()
        self.InitSearch()
        # assert self.open_list.size() > 0  # otherwise -> low level fail

        self.tstart = time.perf_counter()
        if self.search_method is SMD.ASTAR:
            self._BestFirstSearch()
        elif self.search_method is SMD.FOCAL:
            self._FocalSearch(2.0)
        else:
            self._AnyTimeFocalSearch()
        if (self.open_list.size() + self.focal_list.size()) == 0:
            print("List is empty!")
            self.resDat.open_empty = True
        self.resDat.run_time = float(time.perf_counter() - self.tstart)
        return self.resDat


def RunCBS(
    env: ut.Env,
    s_x,
    s_y,
    d_x,
    d_y,
    v_max,
    time_limit,
    minT: bool = False,
    conflict_mode: Mod = Mod.LB,
    stepsize: float = 0.2,
    g_reso: int = 1,
    scale: float = 1,  # scale up coordinate, speed to avoid float error in low level
    name: str = "latest",
):
    """

    envs is a dict
    envs["x_ticks"] = a list of vertical lines.
    envs["y_ticks"] = a list of horizontal lines.
    envs["max_t"] = the max time steps. Always use [0,1,2,...,max_t] as the time list

    s_x is a list of starting x-coord of all agents.
    s_y is a list of starting y-coord of all agents.

    d_x is a list of destination x-coord of all agents.
    d_y is a list of destination y-coord of all agents.

    v_max = robot speed limit.

    time_limit is the max allowed runtime for the planner.

    """
    from os.path import join

    dir = join("cbs-scen", name)
    if not os.path.exists(dir):
        os.mkdir(dir)

    env = ut.scaleUpEnv(env, scale)
    s_x = [x * scale for x in s_x]
    s_y = [y * scale for y in s_y]
    d_x = [x * scale for x in d_x]
    d_y = [y * scale for y in d_y]
    v_max *= scale * sqrt(2)

    from cbs.plot import plot_scen

    fig, _ = plot_scen(env, s_x, s_y, d_x, d_y)
    fig.savefig(join(dir, "scen.jpg"))

    try:
        p = GcsCbsSearch(env, s_x, s_y, d_x, d_y, v_max, g_reso)
        p.SetConflictMode(conflict_mode)
        p.lowlevel_minT = minT
        res = p.Search(time_limit)

        from cbs.plot import plot_cbs_sol
        import matplotlib.pyplot as plt

        ani = plot_cbs_sol(p.nodes[res.best_nid], p)
        ani.save(join(dir, "cbs.gif"))
        plt.show()
        return res
    except Exception as e:
        # program is interrupted
        import traceback

        fname = join(dir, "cbs.traceback")
        print(f"Traceback has been saved to file '{fname}.traceback'")
        traceback.print_exc(file=open(fname, "w"))
        raise e
    finally:
        # save search object to rebuild later
        import pickle

        fname = join(dir, "cbs.dump")
        print(f"Context has been dumped to file '{fname}'")
        pickle.dump(p, open(fname, "wb"))
