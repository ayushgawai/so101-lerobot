"""Validates room-and-table_joined.usd after the realism pass
(build_realistic_tables.py): existence, seam/coplanarity/alignment
preservation, realistic proportions, floor contact, no unintended
intersections, material bindings, and no unrelated prims touched.
"""
import os

from pxr import Usd, UsdGeom, UsdShade

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
ASSETS_USD_DIR = os.path.join(REPO_ROOT, "source", "sim_to_real_so101", "assets", "usd")
STAGE_PATH = os.path.join(ASSETS_USD_DIR, "room-and-table_joined.usd")
SRC_PATH = os.path.join(ASSETS_USD_DIR, "room-and-table.usd")

errors = []


def check(cond, msg):
    status = "OK" if cond else "FAIL"
    print(f"[{status}] {msg}")
    if not cond:
        errors.append(msg)


stage = Usd.Stage.Open(STAGE_PATH)
src = Usd.Stage.Open(SRC_PATH)
bbcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)
xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())


def rng(path, s=stage):
    return bbcache.ComputeWorldBound(s.GetPrimAtPath(path)).ComputeAlignedRange()


# --- 1-3: existence -----------------------------------------------------
check(stage.GetPrimAtPath("/World/Table").IsValid(), "/World/Table exists")
check(stage.GetPrimAtPath("/World/Table_02").IsValid(), "/World/Table_02 exists")
check(stage.GetPrimAtPath("/World/Table/TableTop").IsValid(), "/World/Table/TableTop exists")
check(stage.GetPrimAtPath("/World/Table_02/TableTop").IsValid(), "/World/Table_02/TableTop exists")

top1 = rng("/World/Table/TableTop")
top2 = rng("/World/Table_02/TableTop")

# --- 4-5: long-edge join + seam ------------------------------------------
seam = top2.GetMin()[1] - top1.GetMax()[1]
print(f"Measured seam: {seam*1000:.3f} mm")
check(1.5e-3 <= seam <= 6e-3, "Seam within 2-5mm target")
check(abs(top1.GetMin()[0] - top2.GetMin()[0]) < 1e-9 and abs(top1.GetMax()[0] - top2.GetMax()[0]) < 1e-9,
      "Long (X) edges fully aligned end to end")

# --- 6-8: coplanar, level, rotation match --------------------------------
check(top1.GetMin()[2] == top2.GetMin()[2] and top1.GetMax()[2] == top2.GetMax()[2], "Tops coplanar")
t1_mat = xform_cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/Table"))
t2_mat = xform_cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/Table_02"))
rot1 = [[t1_mat[i][j] for j in range(3)] for i in range(3)]
rot2 = [[t2_mat[i][j] for j in range(3)] for i in range(3)]
check(rot1 == rot2, "Table rotations match")

# --- 9-11: realistic tabletop shape ---------------------------------------
top_dims = top1.GetMax() - top1.GetMin()
thickness = top_dims[2]
print(f"Tabletop dims: {tuple(top_dims)}")
check(abs(top_dims[0] - 1.2) < 1e-6 and abs(top_dims[1] - 0.7) < 1e-6, "Tabletop footprint unchanged (1.2 x 0.7)")
check(0.02 <= thickness <= 0.045, f"Tabletop thickness realistic ({thickness*1000:.1f} mm, target 25-40mm)")

top_mesh = UsdGeom.Mesh(stage.GetPrimAtPath("/World/Table/TableTop"))
pts = top_mesh.GetPointsAttr().Get()
xs = sorted(set(round(p[0], 6) for p in pts))
ys = sorted(set(round(p[1], 6) for p in pts))
# a sharp-corner box would only have 2 unique X and 2 unique Y values; rounding adds more.
check(len(xs) > 4 and len(ys) > 4, f"Tabletop corners are rounded (found {len(xs)} unique X, {len(ys)} unique Y coords, not just 2)")
n_top_rings = len(set(round(p[2], 6) for p in pts))
check(n_top_rings >= 3, f"Tabletop has a beveled edge profile ({n_top_rings} distinct Z levels, expect >=3)")

# --- 12-13: support structure + casters ------------------------------------
for table in ["Table", "Table_02"]:
    base = stage.GetPrimAtPath(f"/World/{table}/Base")
    check(base.IsValid(), f"/World/{table}/Base exists")
    supports = [c.GetName() for c in base.GetChildren()]
    check(set(supports) == {"SupportLeft", "SupportRight"}, f"/World/{table}/Base has 2 support assemblies: {supports}")
    caster_count = 0
    for support in ["SupportLeft", "SupportRight"]:
        sp = f"/World/{table}/Base/{support}"
        check(stage.GetPrimAtPath(f"{sp}/Column").IsValid(), f"{sp}/Column exists")
        for foot in ["FootA", "FootB"]:
            check(stage.GetPrimAtPath(f"{sp}/{foot}").IsValid(), f"{sp}/{foot} exists")
        for caster in ["CasterA", "CasterB"]:
            wheel = stage.GetPrimAtPath(f"{sp}/{caster}/Wheel")
            stem = stage.GetPrimAtPath(f"{sp}/{caster}/Stem")
            check(wheel.IsValid() and stem.IsValid(), f"{sp}/{caster} has Stem+Wheel")
            if wheel.IsValid():
                caster_count += 1
    check(caster_count == 4, f"/World/{table} has 4 casters (found {caster_count})")

# --- 14: casters contact floor, none penetrate -----------------------------
floor_contacts = []
min_wheel_z = 1e9
max_wheel_bottom = -1e9
for table in ["Table", "Table_02"]:
    for support in ["SupportLeft", "SupportRight"]:
        for caster in ["A", "B"]:
            wp = f"/World/{table}/Base/{support}/Caster{caster}/Wheel"
            r = rng(wp)
            min_wheel_z = min(min_wheel_z, r.GetMin()[2])
            max_wheel_bottom = max(max_wheel_bottom, r.GetMin()[2])
            floor_contacts.append(r.GetMin()[2])
print(f"Wheel bottom Z range: min={min_wheel_z} max={max_wheel_bottom}")
check(abs(min_wheel_z) < 1e-6, "At least one wheel exactly touches floor (z=0)")
check(all(abs(z) < 1e-6 for z in floor_contacts), "All wheels touch floor at the same elevation (no floating/penetrating wheels)")

# --- 15: no unintended intersections between the two tables' support parts --
def collect_ranges(table):
    d = {}
    for support in ["SupportLeft", "SupportRight"]:
        for name in ["Column", "MountPlate", "FootA", "FootB"]:
            d[f"{support}/{name}"] = rng(f"/World/{table}/Base/{support}/{name}")
        for caster in ["A", "B"]:
            for part in ["Stem", "Wheel"]:
                d[f"{support}/Caster{caster}/{part}"] = rng(f"/World/{table}/Base/{support}/Caster{caster}/{part}")
    return d

ranges1 = collect_ranges("Table")
ranges2 = collect_ranges("Table_02")

def overlap(r1, r2):
    for i in range(3):
        if r1.GetMax()[i] <= r2.GetMin()[i] or r2.GetMax()[i] <= r1.GetMin()[i]:
            return False
    return True

collisions = []
for n1, r1 in ranges1.items():
    for n2, r2 in ranges2.items():
        if overlap(r1, r2):
            collisions.append((n1, n2))
check(len(collisions) == 0, f"No cross-table support/caster intersections ({len(ranges1)*len(ranges2)} pairs checked): {collisions}")

# also check tabletops don't overlap the OTHER table's support structure
for n2, r2 in ranges2.items():
    if overlap(top1, r2):
        collisions.append((f"Table/TableTop", f"Table_02/{n2}"))
for n1, r1 in ranges1.items():
    if overlap(top2, r1):
        collisions.append((f"Table_02/TableTop", f"Table/{n1}"))
check(len(collisions) == 0, "No table's support structure intersects the other table's tabletop")

# --- 16-17: materials ------------------------------------------------------
def bound_color(path):
    prim = stage.GetPrimAtPath(path)
    bapi = UsdShade.MaterialBindingAPI(prim)
    mat, _ = bapi.ComputeBoundMaterial()
    if not mat or not mat.GetPrim().IsValid():
        return None
    shader = UsdShade.Shader(stage.GetPrimAtPath(str(mat.GetPath()) + "/Shader"))
    color_input = shader.GetInput("diffuseColor")
    return color_input.Get() if color_input else None

top_color = bound_color("/World/Table/TableTop")
col_color = bound_color("/World/Table/Base/SupportLeft/Column")
wheel_color = bound_color("/World/Table/Base/SupportLeft/CasterA/Wheel")
print(f"Tabletop bound color: {top_color}")
print(f"Column bound color: {col_color}")
print(f"Wheel bound color: {wheel_color}")
check(top_color is not None and top_color[0] > 0.6 and top_color[0] < 0.95, "Tabletop material is light gray laminate")
check(col_color is not None and col_color[0] < 0.35, "Metal material is dark gray")
check(wheel_color is not None and wheel_color[0] < 0.15, "Caster wheel material is near-black")

top2_color = bound_color("/World/Table_02/TableTop")
check(top_color == top2_color, "Both tables use the same tabletop material")

# --- 18: no unrelated prims removed -----------------------------------------
src_world_children = {c.GetName() for c in src.GetPrimAtPath("/World").GetChildren()}
out_world_children = {c.GetName() for c in stage.GetPrimAtPath("/World").GetChildren()}
check(src_world_children.issubset(out_world_children), f"All original /World children still present (missing: {src_world_children - out_world_children})")

for name in ["Floor", "Walls", "RobotMount", "SkyLight", "KeyLight"]:
    r_src = rng(f"/World/{name}", src)
    r_out = rng(f"/World/{name}")
    check(r_src.GetMin() == r_out.GetMin() and r_src.GetMax() == r_out.GetMax(), f"/World/{name} unchanged")

# original Looks materials untouched
for mat_name in ["Floor", "Wall", "Mount"]:
    check(stage.GetPrimAtPath(f"/World/Looks/{mat_name}").IsValid(), f"/World/Looks/{mat_name} still exists")

# --- 19: reopens without errors ---------------------------------------------
check(stage.GetPseudoRoot().IsValid(), "Stage reopens without error")
check(stage.GetDefaultPrim().GetPath() == "/World", "defaultPrim still /World")

# --- overall table height / floor level sanity ------------------------------
table1_bbox = rng("/World/Table")
table2_bbox = rng("/World/Table_02")
check(abs(table1_bbox.GetMin()[2]) < 1e-6 and abs(table2_bbox.GetMin()[2]) < 1e-6, "Both tables sit at floor level (z=0)")
check(abs(top1.GetMax()[2] - 0.75) < 1e-6, f"Table height preserved at 0.75 m (within 0.72-0.78 target)")

print("\n=== Summary ===")
if errors:
    print(f"{len(errors)} FAILURE(S):")
    for e in errors:
        print(" -", e)
    raise SystemExit(1)
else:
    print("All validation checks passed.")
