"""Validates room-and-table_joined.usd against room-and-table.usd:
existence, dimensions/rotation/elevation match, seam correctness, no overlap,
leg preservation/non-intersection, floor level match, material bindings,
and that no unrelated /World prims changed.
"""
import os

from pxr import Usd, UsdGeom

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
ASSETS_USD_DIR = os.path.join(REPO_ROOT, "source", "sim_to_real_so101", "assets", "usd")
SRC_PATH = os.path.join(ASSETS_USD_DIR, "room-and-table.usd")
OUT_PATH = os.path.join(ASSETS_USD_DIR, "room-and-table_joined.usd")

errors = []

def check(cond, msg):
    status = "OK" if cond else "FAIL"
    print(f"[{status}] {msg}")
    if not cond:
        errors.append(msg)

src = Usd.Stage.Open(SRC_PATH)
out = Usd.Stage.Open(OUT_PATH)
bbcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)

# --- existence ---------------------------------------------------------------
for path in ["/World/Table", "/World/Table_02"]:
    for child in ["", "/TableTop", "/Leg1", "/Leg2", "/Leg3", "/Leg4"]:
        p = out.GetPrimAtPath(path + child)
        check(p.IsValid(), f"{path}{child} exists in output")

# --- original Table unchanged in output (matches source) --------------------
src_top_rng = bbcache.ComputeWorldBound(src.GetPrimAtPath("/World/Table/TableTop")).ComputeAlignedRange()
out_t1_top_rng = bbcache.ComputeWorldBound(out.GetPrimAtPath("/World/Table/TableTop")).ComputeAlignedRange()
check(src_top_rng.GetMin() == out_t1_top_rng.GetMin() and src_top_rng.GetMax() == out_t1_top_rng.GetMax(),
      "Table (original) TableTop world bbox unchanged in output")

# --- Table_02 dims/rotation/elevation match Table ----------------------------
out_t2_top_rng = bbcache.ComputeWorldBound(out.GetPrimAtPath("/World/Table_02/TableTop")).ComputeAlignedRange()
t1min, t1max = out_t1_top_rng.GetMin(), out_t1_top_rng.GetMax()
t2min, t2max = out_t2_top_rng.GetMin(), out_t2_top_rng.GetMax()
t1dims = t1max - t1min
t2dims = t2max - t2min
dims_close = all(abs(a - b) < 1e-9 for a, b in zip(t1dims, t2dims))
check(dims_close, f"Table_02 TableTop dims match Table's ({tuple(t1dims)} vs {tuple(t2dims)})")
check(t1min[2] == t2min[2] and t1max[2] == t2max[2], f"Coplanar / same elevation (Z range {t1min[2]}-{t1max[2]} vs {t2min[2]}-{t2max[2]})")
check(t1dims[1] == t2dims[1], "Same Y depth (rotation preserved, not rotated 90deg)")

xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
t1_mat = xform_cache.GetLocalToWorldTransform(out.GetPrimAtPath("/World/Table"))
t2_mat = xform_cache.GetLocalToWorldTransform(out.GetPrimAtPath("/World/Table_02"))
def rotation_part(m):
    return [[m[i][j] for j in range(3)] for i in range(3)]
check(rotation_part(t1_mat) == rotation_part(t2_mat), "Table_02 rotation matches Table (same 3x3 rotation block)")

# --- seam ----------------------------------------------------------------
# Long axis = X (1.2 m edges run along X); join/short axis = Y (0.7 m).
# Table_02 is translated along +Y, so the seam is measured along Y.
seam = t2min[1] - t1max[1]
print(f"Measured seam (Table_02.min.y - Table.max.y): {seam}")
check(1.5e-3 <= seam <= 6e-3, f"Seam within tolerance of 2-5mm target: {seam*1000:.3f} mm")
check(seam > 0, "Tops do not overlap (positive seam)")
check(abs((t1max[0] - t1min[0]) - (t2max[0] - t2min[0])) < 1e-9, "X (long) dims match, tops fully aligned/parallel on long axis")
check(t1min[0] == t2min[0] and t1max[0] == t2max[0], "Tops aligned on X (no lateral offset along the long edge)")

# --- legs preserved and no unintended intersection ---------------------------
leg_ranges_1 = {}
leg_ranges_2 = {}
for leg in ["Leg1", "Leg2", "Leg3", "Leg4"]:
    r1 = bbcache.ComputeWorldBound(out.GetPrimAtPath(f"/World/Table/{leg}")).ComputeAlignedRange()
    r2 = bbcache.ComputeWorldBound(out.GetPrimAtPath(f"/World/Table_02/{leg}")).ComputeAlignedRange()
    leg_ranges_1[leg] = r1
    leg_ranges_2[leg] = r2
    check(r1.GetMin() != r2.GetMin() or r1.GetMax() != r2.GetMax(), f"{leg}: Table_02 leg is at a different position than Table's")

def ranges_overlap(r1, r2):
    for i in range(3):
        if r1.GetMax()[i] <= r2.GetMin()[i] or r2.GetMax()[i] <= r1.GetMin()[i]:
            return False
    return True

any_leg_collision = False
for l1, r1 in leg_ranges_1.items():
    for l2, r2 in leg_ranges_2.items():
        if ranges_overlap(r1, r2):
            any_leg_collision = True
            print(f"  COLLISION: Table/{l1} intersects Table_02/{l2}")
check(not any_leg_collision, "No leg-to-leg intersection between the two tables")

# --- both tables on same floor level ------------------------------------
table1_bbox = bbcache.ComputeWorldBound(out.GetPrimAtPath("/World/Table")).ComputeAlignedRange()
table2_bbox = bbcache.ComputeWorldBound(out.GetPrimAtPath("/World/Table_02")).ComputeAlignedRange()
check(table1_bbox.GetMin()[2] == table2_bbox.GetMin()[2], f"Same floor level (Z min {table1_bbox.GetMin()[2]} vs {table2_bbox.GetMin()[2]})")

# --- material bindings intact (both had none originally; confirm still none / consistent) ---
from pxr import UsdShade
for path in ["/World/Table/TableTop", "/World/Table_02/TableTop"]:
    prim = out.GetPrimAtPath(path)
    bapi = UsdShade.MaterialBindingAPI(prim)
    mat, rel = bapi.ComputeBoundMaterial()
    check(mat is None or not mat.GetPrim().IsValid(), f"{path}: material binding state consistent with source (none)")

# --- no unrelated /World prims changed --------------------------------------
src_world_children = {c.GetName() for c in src.GetPrimAtPath("/World").GetChildren()}
out_world_children = {c.GetName() for c in out.GetPrimAtPath("/World").GetChildren()}
expected_new = out_world_children - src_world_children
check(expected_new == {"Table_02"}, f"Only Table_02 added to /World (got new: {expected_new})")
check(src_world_children.issubset(out_world_children), "All original /World children still present")

for name in ["Floor", "Walls", "RobotMount", "SkyLight", "KeyLight"]:
    p_src = src.GetPrimAtPath(f"/World/{name}")
    p_out = out.GetPrimAtPath(f"/World/{name}")
    r_src = bbcache.ComputeWorldBound(p_src).ComputeAlignedRange()
    r_out = bbcache.ComputeWorldBound(p_out).ComputeAlignedRange()
    check(r_src.GetMin() == r_out.GetMin() and r_src.GetMax() == r_out.GetMax(), f"/World/{name} unchanged")

# --- composition sanity -------------------------------------------------
check(out.GetPseudoRoot().IsValid(), "Output stage opens without error")
check(out.GetDefaultPrim().GetPath() == "/World", "defaultPrim still /World")

print("\n=== Summary ===")
if errors:
    print(f"{len(errors)} FAILURE(S):")
    for e in errors:
        print(" -", e)
else:
    print("All checks passed.")
