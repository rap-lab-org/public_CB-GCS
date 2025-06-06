from collections.abc import Callable
from matplotlib import animation
from matplotlib import pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.figure import Figure
from matplotlib.axes import Axes
from matplotlib.patches import Rectangle, Circle
from matplotlib.text import Annotation
import os
import sys

import mpl_toolkits.mplot3d.art3d as art3d
from mpl_toolkits.mplot3d import Axes3D

_current = os.path.dirname(os.path.realpath(__file__))
# directory reach
_parent = os.path.dirname(_current)
if _parent not in sys.path:
    sys.path.append(_parent)
if _current not in sys.path:
    sys.path.append(_current)

from cbs.type_stubs import TimeSteps, Vert, Poly
from cbs.search import GcsCbsSearch
from cbs.utils import Env, GcsCbsNode, GetCorners, CbsConstraint


def plot_scen(
    env: Env,
    sx: list[float],
    sy: list[float],
    dx: list[float],
    dy: list[float],
    figsize=(15, 15),
) -> tuple[Figure, Axes]:
    subp = plt.subplots(figsize=figsize)
    fig: Figure = subp[0]
    ax: Axes = subp[1]
    ax.set_xlim(env.xbound())
    ax.set_ylim(env.ybound())
    fig.tight_layout()
    _draw_static_obstacles2D(ax, env.obsts, alpha=0.8)
    _draw_static_obstacles2D(ax, env.inflated_obsts, alpha=0.2)

    clst = list(mcolors.TABLEAU_COLORS.keys())
    colors = {rid: clst[rid % len(clst)] for rid in range(len(sx))}
    for rid in range(len(sx)):
        c00, _, _, _ = GetCorners((sx[rid], sy[rid]), env.d_thres / 2)
        rect_s = Rectangle(
            c00, env.d_thres, env.d_thres, color=colors[rid], alpha=0.3
        )
        x, y = sx[rid], sy[rid]
        ax.annotate(f"r-{rid}", xy=(x, y), xytext=(x, y), textcoords="data")
        ax.add_patch(rect_s)

        c00, _, _, _ = GetCorners((dx[rid], dy[rid]), env.d_thres / 2)
        rect_d = Rectangle(
            c00, env.d_thres, env.d_thres, color=colors[rid], linewidth=2, fill=False
        )
        ax.add_patch(rect_d)
        _draw_point(
            ax,
            (sx[rid], sy[rid]),
            0,
            True,
            color=colors[rid],
            mfc="none",
            alpha=1,
            marker="o",
            ms=10,
        )
        _draw_point(ax, (dx[rid], dy[rid]), 0, True, color=colors[rid], marker="+", ms=10)
    return fig, ax

def plot_path_static(
    agent_paths: dict[int, list[Vert]],
    env: Env,
    rects,
    circles,
    labels,
    figsize=(15, 15),
    draw_inflat=True
    ):
    subp = plt.subplots(figsize=figsize)
    fig: Figure = subp[0]
    ax: Axes = subp[1]
    ax.set_xlim(env.xbound())
    ax.set_ylim(env.ybound())
    fig.tight_layout()

    _draw_static_obstacles2D(ax, env.obsts, alpha=1)
    if draw_inflat:
        _draw_static_obstacles2D(ax, env.inflated_obsts, alpha=0.2)

    clst = list(mcolors.TABLEAU_COLORS.keys())
    colors = {rid: clst[rid % len(clst)] for rid in agent_paths.keys()}


    for rid in agent_paths.keys():
        # sx, sy = agent_paths[rid][0]
        # ax.plot([sx], [sy], alpha=1, marker="*", ms=20, color=color, markeredgecolor="black")
        gx, gy = agent_paths[rid][-1]
        ax.plot([gx], [gy], alpha=1, marker="*", ms=20, color=colors[rid], markeredgecolor="black", zorder=-1)
    for rid in agent_paths.keys():
        color = colors[rid]
        v = agent_paths[rid][0]
        c00, _, _, _ = GetCorners(v, env.d_thres / 2)
        rects[rid] = Rectangle(
            c00, env.d_thres, env.d_thres, fc=mcolors.to_rgba(color, 0.3), ec="black",
        )
        circles[rid] = Circle(v, radius=env.d_thres / 25, 
                                  fc=mcolors.to_rgba(color, 0.7), ec="black", zorder=-1)
        ax.add_patch(rects[rid])
        ax.add_patch(circles[rid])
        _draw_path2D(ax, agent_paths[rid], alpha=0.3, color=color)
    return fig, ax


def plot_path_animation(
    agent_paths: dict[int, list[Vert]],
    env: Env,
    T: TimeSteps,
    frame_delay: float,
    numframes: int = 500,
    figsize=(15, 15),
    draw_inflat=True,
    figax_style: Callable | None=None,
    drawHeader=True
):

    rects: dict[int, Rectangle] = {}
    circles: dict[int, Circle] = {}
    labels: dict[int, Annotation] = {}
    fig, ax = plot_path_static(agent_paths, env, rects, circles, labels,
                               figsize=figsize, draw_inflat=draw_inflat)
    if figax_style is not None:
        figax_style(fig, ax)

    tmax = max(T)
    dt = tmax / numframes
    header: Annotation | None = None
    if drawHeader:
        header = ax.annotate(
            f"Time: 0.00 / {tmax:.2f} ( Step: {0:<3})",
            xy=(env.maxx * 0.75, env.maxy*0.98),
            bbox={'facecolor': 'black', 'alpha': 0.5, 'pad': 2},
            ha="center", va="center",
            color="snow",
        )
    from bisect import bisect_right

    def update_plot(frame: int):
        curT: float = frame * dt
        ts: int = bisect_right(T, curT) - 1
        if header is not None:
            header.set_text(f"Time: {curT:.2f} / {tmax:.2f} ( Step: {ts:<3})")
        for rid, path in agent_paths.items():
            if ts + 1 < len(path):
                x1, y1 = path[ts]
                x2, y2 = path[ts + 1]
                vx = (x2 - x1) / (T[ts + 1] - T[ts])
                vy = (y2 - y1) / (T[ts + 1] - T[ts])
                x = x1 + vx * (curT - T[ts])
                y = y1 + vy * (curT - T[ts])
            else:
                x, y = path[ts]
            # labels[rid].set_position((x, y + 0.1))
            rects[rid].set_xy((x - env.d_thres / 2, y - env.d_thres / 2))
            circles[rid].set_center((x, y))

        return (
            [header] if header is not None else []
            # + list(labels.values())
            + list(rects.values())
            + list(circles.values())
        )

    # Create animation
    from tqdm.auto import tqdm
    import numpy as np

    ani = animation.FuncAnimation(
        fig,
        update_plot,
        frames=tqdm(np.arange(numframes), initial=1),
        interval=frame_delay,
        blit=True,
    )
    return ani


def plot_cbs_sol(plan: GcsCbsNode, ctx: GcsCbsSearch, figaxcb=None) -> animation.FuncAnimation:
    # points[i]: location of centroid of all agents at the i-th time step
    all_locs: dict[int, list[Vert]] = {}

    for rid in range(ctx.num_robots):
        all_locs[rid] = plan.sol.paths[rid][0]
    ani = plot_path_animation(all_locs, ctx.env, ctx.T, 20, figax_style=figaxcb)
    return ani


def plot_mapf_fig(datJson: str, resJson: str, scale: float = 1, figsize=(5, 5),
                  draw_inflat=True):
    import json
    from cbs.utils import scaleUpEnv

    dat = json.load(open(datJson, "r"))
    res = json.load(open(resJson, "r"))
    env = scaleUpEnv(Env(**dat["env"]), scale)
    paths = {}
    for rid in res["paths"]:
        if len(res["paths"][rid]) == 2:
            paths[int(rid)], _ = res["paths"][rid]
        else:
            paths[int(rid)] = res["paths"][rid]

    rects: dict[int, Rectangle] = {}
    circles: dict[int, Circle] = {}
    labels: dict[int, Annotation] = {}
    fig, ax = plot_path_static(paths, env, rects, circles, labels, 
                               figsize=figsize, draw_inflat=draw_inflat)
    return fig, ax


def plot_mapf_res(datJson: str, resJson: str, scale: float = 1, 
                  numframes=100, figsize=(5, 5), figaxcb=None,
                  draw_inflat=False, **kws):
    import json
    from cbs.utils import scaleUpEnv

    dat = json.load(open(datJson, "r"))
    res = json.load(open(resJson, "r"))
    env = scaleUpEnv(Env(**dat["env"]), scale)
    paths = {}
    T = res["paths"]["0"][1]
    for rid in res["paths"]:
        paths[int(rid)], _ = res["paths"][rid]
    ani = plot_path_animation(paths, env, T, 20, 
                              numframes=numframes, figsize=figsize, 
                              figax_style=figaxcb, draw_inflat=draw_inflat, **kws)
    return ani


def _draw_point(ax: Axes | Axes3D, v: Vert, t: float, is_2d=False, **kws):
    """
    kws: parameters for scatter
    """
    if is_2d:
        ax.plot([v[0]], [v[1]], **kws)
    else:
        ax.plot([v[0]], [v[1]], [t], **kws)


def _draw_path2D(ax: Axes, path: list[Vert], **kws):
    color = kws.get("color", "blue")
    alpha = kws.get("alpha", 1)

    for i, (x_u, y_u) in enumerate(path[:-1]):
        x_v, y_v = path[i + 1]
        point_x, point_y = ([x_u, x_v], [y_u, y_v])
        if i + 1 < len(path) - 1:
            ax.scatter([x_u], [y_u], ec=color, fc="none")
        ax.plot(point_x, point_y, color=color, alpha=alpha)


def _draw_path3D(ax: Axes3D, path: list[Vert], ts: TimeSteps, is_2d=False, **kws):
    color = kws.get("color", "blue")
    alpha = kws.get("alpha", 1)

    for i, (x_u, y_u) in enumerate(path[:-1]):
        t_u = ts[i]
        x_v, y_v = path[i + 1]
        t_v = ts[i + 1]
        point_x, point_y, t = ([x_u, x_v], [y_u, y_v], [t_u, t_v])
        if i != 0 and i + 1 < len(path) - 1:
            ax.scatter(point_x, point_y, t, color=color, alpha=alpha) #type: ignore
        ax.plot(point_x, point_y, t, color=color, alpha=alpha)

    _draw_point(
        ax, path[0], ts[0], is_2d, color=color, mfc="none", alpha=1, marker="o", ms=10
    )
    _draw_point(ax, path[-1], ts[-1], is_2d, alpha=1, marker="+", ms=10, color=color)


def _draw_static_obstacles2D(ax, obsts: list[Poly], alpha=0.2):
    d = 0
    for obs in obsts:
        c00, _, c11, _ = obs
        x, y = c00[0], c00[1]
        dx, dy = c11[0] - c00[0] + d, c11[1] - c00[1] + d
        patch = Rectangle((x, y), dx, dy, fill=True, color="black", alpha=alpha)
        ax.add_patch(patch)


def _draw_static_obstacles3D(ax, obstacles: list[Poly], ts: TimeSteps, alpha=0.3):
    for obstacle in obstacles:
        c00, _, c11, _ = obstacle
        x, y, z = c00[0], c00[1], ts[0]
        dx, dy, dz = c11[0] - c00[0], c11[1] - c00[1], ts[-1] - ts[0]
        ax.bar3d(x, y, z, dx, dy, dz, alpha=alpha, color="black")


def _draw_axes(ax, ts: TimeSteps, x_ticks: list[float], y_ticks: list[float]):
    import numpy as np

    def f(x, y, t):
        return 0 * x + 0 * y + t

    X, Y = np.meshgrid(x_ticks, y_ticks)
    for t in ts:
        ax.plot_wireframe(X, Y, f(X, Y, t), color="gray", alpha=0.5, linewidths=0.1)


def _draw_constraints(ax: Axes3D, cstrs: list[CbsConstraint], **kws):
    color = kws.get("color", "blue")
    alpha = kws.get("alpha", 0.3)
    for cstr in cstrs:
        minx, miny = cstr.va
        maxx, maxy = cstr.vb
        t = cstr.ta
        rect = Rectangle(
            (minx, miny),
            maxx - minx,
            maxy - miny,
            fill=True,
            alpha=alpha,
            color=color,
            edgecolor="black",
        )
        ax.add_patch(rect)
        art3d.pathpatch_2d_to_3d(rect, z=t) # type: ignore


def _draw_violation(
    ax: Axes,
    ts: TimeSteps,
    path: list[Vert],
    cstrs: list[CbsConstraint],
    ecolor: str = "red",
):
    # highlight vialation
    for (x, y), t in zip(path, ts):
        for c in cstrs:
            if c.is_contains((x, y), t):
                minx, miny = c.va
                maxx, maxy = c.vb
                t = c.ta
                rect = Rectangle(
                    (minx, miny),
                    maxx - minx,
                    maxy - miny,
                    fill=False,
                    edgecolor=ecolor,
                    linewidth=2,
                )
                ax.add_patch(rect)
                art3d.pathpatch_2d_to_3d(rect, z=t) # type: ignore


def _draw_gcs_sol(
    ts: TimeSteps,
    x_ticks: list[float],
    y_ticks: list[float],
    path: list[Vert],
    static_obsts: list[Poly],
    cstrs: list[CbsConstraint],
    **kws,
) -> tuple[Figure, Axes3D]:
    fig: Figure = plt.figure(figsize=kws.get("figsize", (10, 10)))
    ax: Axes3D = plt.axes(projection="3d") # type:ignore
    ax.set_zlabel("z")
    _draw_axes(ax, ts, x_ticks, y_ticks)
    _draw_static_obstacles3D(ax, static_obsts, ts)
    _draw_path3D(ax, path, ts)
    _draw_constraints(ax, cstrs)
    _draw_violation(ax, ts, path, cstrs)
    return fig, ax


def plot_gcs_sol(
    ctx: GcsCbsSearch,
    rid: int,
    cur: GcsCbsNode,
    cstrs: list[CbsConstraint],
    title="path3d",
) -> tuple[Figure, Axes]:
    path, ts = cur.sol.paths[rid]
    static_obsts = ctx.inflated_obsts
    fig, ax = _draw_gcs_sol(ts, ctx.x_ticks, ctx.y_ticks, path, static_obsts, cstrs)

    # draw previous path
    if cur.parent != -1:
        pre = ctx.nodes[cur.parent]
        prev_path, _ = pre.sol.paths[rid]
        _draw_path3D(ax, prev_path, ts, color="blue", alpha=0.2)
    # draw other agents path
    for i in range(ctx.num_robots):
        if i == rid:
            continue
        _path, _ = cur.sol.paths[i]
        _draw_path3D(ax, _path, ts, color="gray", alpha=0.5)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(title)

    return fig, ax
