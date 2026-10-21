"""Dump full mesh geometry (points, faceVertexCounts/Indices, extent) of
/World/Table/TableTop in real-to-sim.usd, to figure out how to author UVs
for a non-trivial (72-point) mesh.

Run with Isaac Sim's own Python:
    C:\\Isaac-Sim\\python.bat usd-composer-stages\\inspect_tabletop_geometry.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom

STAGE_PATH = (
    r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop\source"
    r"\sim_to_real_so101\demo\real-to-sim.usd"
)

stage = Usd.Stage.Open(STAGE_PATH)
top_prim = stage.GetPrimAtPath("/World/Table/TableTop")
mesh = UsdGeom.Mesh(top_prim)

pts = mesh.GetPointsAttr().Get()
fvc = mesh.GetFaceVertexCountsAttr().Get()
fvi = mesh.GetFaceVertexIndicesAttr().Get()
extent = mesh.GetExtentAttr().Get()

print("num points:", len(pts))
print("extent:", extent)
xs = [p[0] for p in pts]
ys = [p[1] for p in pts]
zs = [p[2] for p in pts]
print("x range:", min(xs), max(xs))
print("y range:", min(ys), max(ys))
print("z range:", min(zs), max(zs))

print("\nfaceVertexCounts:", list(fvc))
print("\nnum faces:", len(fvc))

print("\nfirst 20 points:")
for i, p in enumerate(pts[:20]):
    print(" ", i, p)

# group points by z to understand shape (top surface, bottom surface, sides, bevels)
from collections import defaultdict
z_groups = defaultdict(list)
for i, p in enumerate(pts):
    z_groups[round(p[2], 5)].append(i)
print("\nz-levels (rounded) and point counts:")
for z, idxs in sorted(z_groups.items()):
    print(f"  z={z}: {len(idxs)} points")

simulation_app.close()
