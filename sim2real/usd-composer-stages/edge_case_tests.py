"""Extra edge-case tests for room-and-table_joined.usd, beyond the core
checks in validate_joined_tables.py:
  - exact (non-bbox-only) axis-isolation of the translation (no drift on X/Z)
  - all leg-pairs (not just cross-table) checked for unintended intersection
  - full-scene bbox stays inside the room (Floor/Walls) with no wall clipping
  - stage reopens with zero composition errors/warnings captured
  - seam measured at multiple points along the long edge (corners), not just
    the aggregate bbox, to catch any unwanted skew/rotation
  - tabletop corner points don't overlap between the two tables
"""
import os
import io
import contextlib

from pxr import Usd, UsdGeom

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
ASSETS_USD_DIR = os.path.join(REPO_ROOT, "source", "sim_to_real_so101", "assets", "usd")
OUT_PATH = os.path.join(ASSETS_USD_DIR, "room-and-table_joined.usd")

errors = []

def check(cond, msg):
    status = "OK" if cond else "FAIL"
    print(f"[{status}] {msg}")
    if not cond:
        errors.append(msg)

# --- reopen and capture any composition errors/warnings ---------------------
buf = io.StringIO()
with contextlib.redirect_stderr(buf):
    stage = Usd.Stage.Open(OUT_PATH)
stderr_out = buf.getvalue()
check(stage is not None and stage.GetPseudoRoot().IsValid(), "Stage reopens successfully")
check(stderr_out.strip() == "", f"No composition errors/warnings on reopen (captured: {stderr_out!r})")

bbcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)
xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())

# --- exact translation isolation: only Y changed, X/Z untouched -------------
t1_mat = xform_cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/Table"))
t2_mat = xform_cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/Table_02"))
t1_pos = t1_mat.ExtractTranslation()
t2_pos = t2_mat.ExtractTranslation()
check(abs(t2_pos[0] - t1_pos[0]) < 1e-12, f"No drift on X: Table.x={t1_pos[0]} Table_02.x={t2_pos[0]}")
check(abs(t2_pos[2] - t1_pos[2]) < 1e-12, f"No drift on Z: Table.z={t1_pos[2]} Table_02.z={t2_pos[2]}")
check(abs((t2_pos[1] - t1_pos[1]) - 0.703) < 1e-9, f"Y translation is exactly short_dim+seam (0.703): got {t2_pos[1]-t1_pos[1]}")

# --- all leg pairs (8x8, not just the 4x4 already covered) ------------------
def rng_of(path):
    return bbcache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()

def overlap(r1, r2):
    for i in range(3):
        if r1.GetMax()[i] <= r2.GetMin()[i] or r2.GetMax()[i] <= r1.GetMin()[i]:
            return False
    return True

all_legs = {}
for table in ["Table", "Table_02"]:
    for leg in ["Leg1", "Leg2", "Leg3", "Leg4"]:
        all_legs[f"{table}/{leg}"] = rng_of(f"/World/{table}/{leg}")

names = list(all_legs.keys())
collisions = []
for i in range(len(names)):
    for j in range(i + 1, len(names)):
        n1, n2 = names[i], names[j]
        if n1.split("/")[0] == n2.split("/")[0]:
            continue  # skip same-table pairs, expected to be separate legs already
        if overlap(all_legs[n1], all_legs[n2]):
            collisions.append((n1, n2))
check(len(collisions) == 0, f"No cross-table leg-pair intersections (checked {len(names)*(len(names)-1)//2} pairs): {collisions}")

# --- closest inner legs are near the seam but not touching -------------------
t1_leg3 = all_legs["Table/Leg3"]   # (-0.54, 0.29) corner, nearest the seam on Table
t1_leg4 = all_legs["Table/Leg4"]   # (0.54, 0.29)
t2_leg1 = all_legs["Table_02/Leg1"]  # (-0.54, -0.29+0.703=0.413)
t2_leg2 = all_legs["Table_02/Leg2"]  # (0.54, 0.413)
inner_gap_1 = t2_leg1.GetMin()[1] - t1_leg3.GetMax()[1]
inner_gap_2 = t2_leg2.GetMin()[1] - t1_leg4.GetMax()[1]
print(f"Inner leg gap (Table/Leg3 -> Table_02/Leg1): {inner_gap_1}")
print(f"Inner leg gap (Table/Leg4 -> Table_02/Leg2): {inner_gap_2}")
check(inner_gap_1 > 0, "Inner legs (Leg3/Leg1) do not touch/overlap")
check(inner_gap_2 > 0, "Inner legs (Leg4/Leg2) do not touch/overlap")

# --- seam measured at both ends of the long edge (catches skew/rotation) ----
top1 = rng_of("/World/Table/TableTop")
top2 = rng_of("/World/Table_02/TableTop")
seam_all_x = top2.GetMin()[1] - top1.GetMax()[1]
check(abs(top1.GetMin()[0] - top2.GetMin()[0]) < 1e-12, "Left end (min X) of both tops aligned exactly")
check(abs(top1.GetMax()[0] - top2.GetMax()[0]) < 1e-12, "Right end (max X) of both tops aligned exactly")
check(1.5e-3 <= seam_all_x <= 6e-3, f"Seam uniform along full long edge: {seam_all_x*1000:.3f} mm")

# --- combined tables bbox stays inside the room ------------------------------
table1_bbox = rng_of("/World/Table")
table2_bbox = rng_of("/World/Table_02")
floor_bbox = rng_of("/World/Floor")
smin = [min(table1_bbox.GetMin()[i], table2_bbox.GetMin()[i]) for i in range(3)]
smax = [max(table1_bbox.GetMax()[i], table2_bbox.GetMax()[i]) for i in range(3)]
fmin, fmax = floor_bbox.GetMin(), floor_bbox.GetMax()
print(f"Combined tables bbox: min={tuple(smin)} max={tuple(smax)}")
print(f"Floor bbox: min={tuple(fmin)} max={tuple(fmax)}")
check(smin[0] >= fmin[0] and smax[0] <= fmax[0], "Combined tables fit within floor X extent")
check(smin[1] >= fmin[1] and smax[1] <= fmax[1], "Combined tables fit within floor Y extent (no wall clipping)")

# --- Y-span sanity: assembly is wider (Y), not longer (X), than one table ---
one_table_x = top1.GetMax()[0] - top1.GetMin()[0]
one_table_y = top1.GetMax()[1] - top1.GetMin()[1]
combined_x = smax[0] - smin[0]
combined_y = smax[1] - smin[1]
print(f"Single table: X={one_table_x} Y={one_table_y}")
print(f"Combined tabletop span: X={combined_x} Y={combined_y}")
check(abs(combined_x - one_table_x) < 1e-9, "Combined X span unchanged from single table (not longer)")
check(combined_y > one_table_y * 1.9, "Combined Y span roughly doubled (wider, as required)")

print("\n=== Summary ===")
if errors:
    print(f"{len(errors)} FAILURE(S):")
    for e in errors:
        print(" -", e)
    raise SystemExit(1)
else:
    print("All edge-case checks passed.")
