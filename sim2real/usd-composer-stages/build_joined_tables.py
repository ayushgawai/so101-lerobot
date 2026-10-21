"""Duplicates /World/Table in room-and-table.usd as /World/Table_02 and
translates the whole assembly so the two tabletops sit LONG-EDGE-TO-LONG-EDGE,
forming one wider work surface (two classroom/lab tables pushed together
side-by-side along their long sides).

Inspection findings (see usd-composer-stages/inspect_table.py output):
  - Stage: Z-up, metersPerUnit = 1.0 (stage units are meters).
  - /World/Table is a plain Xform authored directly in room-and-table.usd's
    root layer (no references/payload of its own). Children are
    /World/Table/TableTop (Cube) and Leg1..Leg4 (Mesh), each a plain prim
    spec with its own translate/orient/scale xformOps. No material bindings
    or displayColor authored anywhere in the subtree.
  - Table's local transform is identity, and its computed world transform
    is also identity, so local-space and world-space translations coincide.
  - TableTop world bbox: X in [-0.6, 0.6] (1.2 m), Y in [-0.35, 0.35] (0.7 m),
    Z in [0.70, 0.75] (0.05 m thick), center (0, 0, 0.725).
  - Horizontal axes (up axis is Z) are X and Y. X = 1.2 m is the LARGER
    horizontal dimension -> long axis. Y = 0.7 m is the smaller -> short
    axis. The tabletop's long edges (the two edges of length 1.2 m) run
    parallel to X, located at y = -0.35 and y = +0.35.
  - To join LONG EDGE to LONG EDGE, Table_02 must be translated
    PERPENDICULAR to the long edges, i.e. along Y (the short horizontal
    dimension) -- NOT along X. Translating along X would instead push the
    short edges (0.7 m edges) together end-to-end, lengthening the
    assembly instead of widening it, which is explicitly the wrong result.
  - Room floor spans X,Y in [-2, 2] and the table is centered at the
    origin, so there is equal clearance on either side of Y; +Y was chosen
    arbitrarily as the free side.
  - Because Sdf.CopySpec is a plain-spec copy (no references/payload/
    instancing on this subtree), duplicating the whole /World/Table prim
    spec tree in place is a safe, exact replication of the original.

Run with the same throwaway usd-core venv as build_room_and_table.py:
    usdenv\\Scripts\\python.exe usd-composer-stages\\build_joined_tables.py
"""
import os
import shutil

from pxr import Usd, UsdGeom, Sdf, Gf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
ASSETS_USD_DIR = os.path.join(REPO_ROOT, "source", "sim_to_real_so101", "assets", "usd")

SRC_PATH = os.path.join(ASSETS_USD_DIR, "room-and-table.usd")
OUT_PATH = os.path.join(ASSETS_USD_DIR, "room-and-table_joined.usd")

SEAM_METERS_TARGET = 0.003  # 3mm, within the requested 2-5mm range

# --- 1: inspect stage / tabletop bbox ---------------------------------------
src_stage = Usd.Stage.Open(SRC_PATH)
up_axis = UsdGeom.GetStageUpAxis(src_stage)
meters_per_unit = UsdGeom.GetStageMetersPerUnit(src_stage)
assert up_axis == UsdGeom.Tokens.z, f"Unexpected upAxis: {up_axis}"

bbcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)
top_prim = src_stage.GetPrimAtPath("/World/Table/TableTop")
top_range = bbcache.ComputeWorldBound(top_prim).ComputeAlignedRange()
top_min, top_max = top_range.GetMin(), top_range.GetMax()
top_dims = top_max - top_min
top_center = (top_min + top_max) * 0.5

table_prim = src_stage.GetPrimAtPath("/World/Table")
xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
table_world_mat = xform_cache.GetLocalToWorldTransform(table_prim)

# --- 2: determine long axis / join axis from the up axis + bbox ------------
# Up axis is Z (index 2) -> horizontal axes are X (0) and Y (1).
HORIZONTAL_AXES = [0, 1]
horiz_dims = {i: top_dims[i] for i in HORIZONTAL_AXES}
LONG_AXIS_INDEX = max(horiz_dims, key=horiz_dims.get)
JOIN_AXIS_INDEX = min(horiz_dims, key=horiz_dims.get)
AXIS_NAMES = {0: "X", 1: "Y", 2: "Z"}
LONG_AXIS_NAME = AXIS_NAMES[LONG_AXIS_INDEX]
JOIN_AXIS_NAME = AXIS_NAMES[JOIN_AXIS_INDEX]

seam_stage_units = SEAM_METERS_TARGET / meters_per_unit
join_dimension = top_dims[JOIN_AXIS_INDEX]  # short horizontal dimension
translation_magnitude = join_dimension + seam_stage_units

table_xform = UsdGeom.Xformable(table_prim)
translate_op = None
orient_op = None
scale_op = None
for op in table_xform.GetOrderedXformOps():
    if op.GetOpName() == "xformOp:translate":
        translate_op = op
    elif op.GetOpName() == "xformOp:orient":
        orient_op = op
    elif op.GetOpName() == "xformOp:scale":
        scale_op = op
assert translate_op is not None, "/World/Table has no xformOp:translate"
orig_translate = translate_op.Get()

new_translate = Gf.Vec3d(orig_translate)
new_translate[JOIN_AXIS_INDEX] += translation_magnitude

print("=== Inspection report ===")
print(f"Table prim: /World/Table")
print(f"Top prim: /World/Table/TableTop")
print(f"Stage up-axis: {up_axis}")
print(f"metersPerUnit: {meters_per_unit}")
print(f"Top world-space bounds: min={tuple(top_min)} max={tuple(top_max)}")
print(f"Top horizontal dimensions: X={top_dims[0]} Y={top_dims[1]}")
print(f"Detected long axis: {LONG_AXIS_NAME} ({top_dims[LONG_AXIS_INDEX]} m)")
print(f"Detected short/join axis: {JOIN_AXIS_NAME} ({top_dims[JOIN_AXIS_INDEX]} m)")
print(f"Long edge direction: parallel to {LONG_AXIS_NAME}")
print(f"Translation axis: {JOIN_AXIS_NAME}")
print(f"Physical seam: {SEAM_METERS_TARGET} m")
print(f"Seam in stage units: {seam_stage_units}")
print(f"Calculated translation distance: {translation_magnitude}")
print(f"Calculated Table_02 translation: {tuple(new_translate)} (orig was {tuple(orig_translate)})")
print(f"Table local->world matrix (should be identity): {table_world_mat}")
print()
print("Joining LONG EDGE to LONG EDGE.")
print("Translation is perpendicular to the long edge.")
assert JOIN_AXIS_INDEX != LONG_AXIS_INDEX, "Would join short edge to short edge -- aborting"

# --- 3: duplicate, translate, save as new file ------------------------------
shutil.copyfile(SRC_PATH, OUT_PATH)
print(f"\nCopied {SRC_PATH} -> {OUT_PATH}")

out_stage = Usd.Stage.Open(OUT_PATH)
root_layer = out_stage.GetRootLayer()

ok = Sdf.CopySpec(root_layer, Sdf.Path("/World/Table"), root_layer, Sdf.Path("/World/Table_02"))
if not ok:
    raise RuntimeError("Sdf.CopySpec of /World/Table -> /World/Table_02 failed")
print("Duplicated /World/Table -> /World/Table_02 (Sdf.CopySpec, same layer)")

table2_prim = out_stage.GetPrimAtPath("/World/Table_02")
table2_xform = UsdGeom.Xformable(table2_prim)
t2_translate_op = None
for op in table2_xform.GetOrderedXformOps():
    if op.GetOpName() == "xformOp:translate":
        t2_translate_op = op
        break
assert t2_translate_op is not None, "/World/Table_02 has no xformOp:translate after copy"
t2_translate_op.Set(new_translate)
print(f"Set /World/Table_02 xformOp:translate = {tuple(new_translate)}")

out_stage.GetRootLayer().Save()
print(f"Saved {OUT_PATH}")
