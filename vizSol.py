#!/usr/bin/env python
from itertools import product
import json
from sys import argv

import cbs.plot as cp
from matplotlib.figure import Figure
from matplotlib.axes import Axes
import matplotlib as mp
import matplotlib.pyplot as plt

mp.rcParams["savefig.pad_inches"] = 0


def figax_style(fig: Figure, ax: Axes):
    from matplotlib.colors import to_rgba

    bg = "silver"
    alpha = 0.6

    fig.set_facecolor(to_rgba(bg, alpha))
    fig.tight_layout(pad=0)
    fig.set_frameon(False)
    fig.subplots_adjust(left=0, bottom=0, top=1, right=1, hspace=0, wspace=0)
    ax.set_facecolor(to_rgba(bg, alpha))
    ax.set_xticks([], [])
    ax.set_yticks([], [])

def viz_fig(figsize=(10, 10), efile=None, sfile=None):
    efile = "./gcs-expr-scen/n4-10x10-swap/data.json" if efile is None else efile
    sfile = "./gcs-expr-scen/n4-10x10-swap/0-mip-UB.json" if sfile is None else sfile
    fig, ax = cp.plot_mapf_fig(efile, sfile, figsize=figsize, draw_inflat=False)
    plt.show()

def viz_ani(figsize=(10, 10), efile=None, sfile=None):
    efile = "./gcs-expr-scen/n4-10x10-swap/data.json" if efile is None else efile
    sfile = "./gcs-expr-scen/n4-10x10-swap/0-mip-UB.json" if sfile is None else sfile
    ani = cp.plot_mapf_res(efile, sfile, figsize=figsize, draw_inflat=False)
    ani.save("mamp.mp4")


if __name__ == "__main__":
    viz_fig()
    viz_ani()
