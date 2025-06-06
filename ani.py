import cbs.plot as cp

sfile = "./gcs-expr-scen/n6-10x10-swap/0-kcbs.json"
efile = "./gcs-expr-scen/n6-10x10-swap/data.json"

fig, ax = cp.plot_mapf_fig(efile, sfile, figsize=(5, 5), draw_inflat=False)
ax.set_xticks([], [])
ax.set_yticks([], [])
fig.tight_layout()
fig.savefig("swap-kcbs-6.png")

sfile = "./gcs-expr-scen/n4-10x10-midswap/0-kcbs.json"
efile = "./gcs-expr-scen/n4-10x10-midswap/data.json"

fig, ax = cp.plot_mapf_fig(efile, sfile, figsize=(5, 5), draw_inflat=False)
ax.set_xticks([], [])
ax.set_yticks([], [])
fig.tight_layout()
fig.savefig("cent-kcbs.png")

sfile = "./gcs-expr-scen/n6-10x10-swap/0-UB-ANYFOCAL-GCS.json"
efile = "./gcs-expr-scen/n6-10x10-swap/data.json"

fig, ax = cp.plot_mapf_fig(efile, sfile, figsize=(5, 5), draw_inflat=False)
ax.set_xticks([], [])
ax.set_yticks([], [])
fig.tight_layout()
fig.savefig("swap-gcs-6.png")

sfile = "./gcs-expr-scen/n4-10x10-midswap/0-UB-ANYFOCAL-GCS.json"
efile = "./gcs-expr-scen/n4-10x10-midswap/data.json"

fig, ax = cp.plot_mapf_fig(efile, sfile, figsize=(5, 5), draw_inflat=False)
ax.set_xticks([], [])
ax.set_yticks([], [])
fig.tight_layout()
fig.savefig("cent-gcs.png")

sfile = "./gcs-expr-scen/n6-10x10-swap/0-mip-UB.json"
efile = "./gcs-expr-scen/n6-10x10-swap/data.json"

fig, ax = cp.plot_mapf_fig(efile, sfile, figsize=(5, 5), draw_inflat=False)
ax.set_xticks([], [])
ax.set_yticks([], [])
fig.tight_layout()
fig.savefig("swap-milp-6.png")

sfile = "./gcs-expr-scen/n4-10x10-midswap/0-mip-UB.json"
efile = "./gcs-expr-scen/n4-10x10-midswap/data.json"

fig, ax = cp.plot_mapf_fig(efile, sfile, figsize=(5, 5), draw_inflat=False)
ax.set_xticks([], [])
ax.set_yticks([], [])
fig.tight_layout()
fig.savefig("cent-milp.png")
