"""Renders a top-down schematic of room-and-table_joined.usd (tabletops +
legs, drawn from actual USD world-space bboxes) so the layout can be
visually confirmed against the requested "two tables pushed together along
their long edges" description, without needing a full raytraced render.
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from pxr import Usd, UsdGeom

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
ASSETS_USD_DIR = os.path.join(REPO_ROOT, "source", "sim_to_real_so101", "assets", "usd")
OUT_PATH = os.path.join(ASSETS_USD_DIR, "room-and-table_joined.usd")
PNG_PATH = os.path.join(SCRIPT_DIR, "joined_tables_preview.png")

stage = Usd.Stage.Open(OUT_PATH)
bbcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)

def rng_of(path):
    return bbcache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()

fig, ax = plt.subplots(figsize=(8, 8))

colors = {"Table": "#c9a876", "Table_02": "#7fb3d5"}
leg_color = "#333333"

for table, color in colors.items():
    top = rng_of(f"/World/{table}/TableTop")
    mn, mx = top.GetMin(), top.GetMax()
    ax.add_patch(patches.Rectangle((mn[0], mn[1]), mx[0]-mn[0], mx[1]-mn[1],
                                    facecolor=color, edgecolor="black", linewidth=1.5, label=f"{table} top", zorder=2))
    for leg in ["Leg1", "Leg2", "Leg3", "Leg4"]:
        lr = rng_of(f"/World/{table}/{leg}")
        lmn, lmx = lr.GetMin(), lr.GetMax()
        ax.add_patch(patches.Rectangle((lmn[0], lmn[1]), lmx[0]-lmn[0], lmx[1]-lmn[1],
                                        facecolor=leg_color, edgecolor="black", zorder=3))

# annotate seam
t1 = rng_of("/World/Table/TableTop")
t2 = rng_of("/World/Table_02/TableTop")
seam_y = (t1.GetMax()[1] + t2.GetMin()[1]) / 2
ax.annotate("SEAM (3mm)", xy=(0, seam_y), xytext=(1.3, seam_y),
            arrowprops=dict(arrowstyle="->"), fontsize=10, va="center")
ax.axhline(t1.GetMax()[1], color="red", linestyle="--", linewidth=0.8)
ax.axhline(t2.GetMin()[1], color="red", linestyle="--", linewidth=0.8)

ax.annotate("LONG EDGE\n(1.2 m)", xy=(0, t1.GetMax()[1]), xytext=(0, 1.35), ha="center", fontsize=9)
ax.annotate("", xy=(-0.6, 1.25), xytext=(0.6, 1.25), arrowprops=dict(arrowstyle="<->"))

ax.set_xlim(-2.2, 2.2)
ax.set_ylim(-2.2, 2.2)
ax.set_aspect("equal")
ax.set_xlabel("X (m) -- long axis")
ax.set_ylabel("Y (m) -- join/short axis")
ax.set_title("Top-down view: room-and-table_joined.usd\nTwo tables joined LONG EDGE to LONG EDGE")
ax.legend(loc="upper right")
ax.grid(True, linestyle=":", alpha=0.5)

# room outline
floor = rng_of("/World/Floor")
fmn, fmx = floor.GetMin(), floor.GetMax()
ax.add_patch(patches.Rectangle((fmn[0], fmn[1]), fmx[0]-fmn[0], fmx[1]-fmn[1],
                                facecolor="none", edgecolor="gray", linewidth=1, linestyle="--", zorder=1))

fig.savefig(PNG_PATH, dpi=150, bbox_inches="tight")
print(f"Saved {PNG_PATH}")
