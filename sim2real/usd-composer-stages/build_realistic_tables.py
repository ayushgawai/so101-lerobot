"""Realism pass on room-and-table_joined.usd: replaces the simple box
tabletop + 4 stick legs under /World/Table with a realistic mobile
classroom/lab table (rounded-corner beveled laminate top, two trestle-style
support assemblies with vertical columns, V/Y feet, and caster wheels), then
rebuilds /World/Table_02 as an exact duplicate (Sdf.CopySpec) offset by the
same Y translation used to join the two tables, so the long-edge seam is
byte-for-byte preserved.

Modifies room-and-table_joined.usd IN PLACE. Does not touch room-and-table.usd.

Conventions matched from the existing file (see inspection below):
  - Meshes are authored directly (no references/instancing), unit-style
    geometry with xformOp:translate/orient(quatd)/scale directly on the
    Mesh prim (see /World/Table/Leg1 in the pre-realism file).
  - subdivisionScheme "none", orientation "rightHanded", faceVarying normals,
    PhysicsCollisionAPI applied, extent authored.
  - Materials: UsdShade.Material > child "Shader" (UsdPreviewSurface) with
    inputs:diffuseColor/metallic/roughness, bound via MaterialBindingAPI
    (see /World/Looks/Floor, /World/Looks/Wall, /World/Looks/Mount).

Run with:
    usdenv\\Scripts\\python.exe usd-composer-stages\\build_realistic_tables.py
"""
import math
import os

from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Sdf, Gf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
ASSETS_USD_DIR = os.path.join(REPO_ROOT, "source", "sim_to_real_so101", "assets", "usd")
STAGE_PATH = os.path.join(ASSETS_USD_DIR, "room-and-table_joined.usd")

# ============================================================================
# Geometry helpers
# ============================================================================

def rounded_rect_loop(size_x, size_y, radius, segs):
    """CCW (viewed from +Z looking down) loop of a rounded rectangle centered
    at the XY origin. segs = arc segments per corner."""
    hx, hy = size_x / 2.0, size_y / 2.0
    r = max(0.0, min(radius, hx, hy))
    corners = [
        (hx - r, hy - r, 0.0),
        (-hx + r, hy - r, 90.0),
        (-hx + r, -hy + r, 180.0),
        (hx - r, -hy + r, 270.0),
    ]
    pts = []
    for cx, cy, start_deg in corners:
        n = segs if r > 1e-9 else 1
        for i in range(n):
            ang = math.radians(start_deg + 90.0 * i / n)
            pts.append((cx + r * math.cos(ang), cy + r * math.sin(ang)))
    return pts


def build_rounded_box(size_x, size_y, size_z, radius, segs, top_bevel=0.0, bottom_bevel=0.0):
    """Rounded-corner box, local origin at bottom center, extending from
    z=0 (bottom) to z=size_z (top). Optional top/bottom chamfer bevels."""
    outer = rounded_rect_loop(size_x, size_y, radius, segs)
    n = len(outer)

    inner_top = None
    if top_bevel > 0:
        inner_top = rounded_rect_loop(size_x - 2 * top_bevel, size_y - 2 * top_bevel,
                                       max(0.0, radius - top_bevel), segs)
    inner_bottom = None
    if bottom_bevel > 0:
        inner_bottom = rounded_rect_loop(size_x - 2 * bottom_bevel, size_y - 2 * bottom_bevel,
                                          max(0.0, radius - bottom_bevel), segs)

    points = []

    def add_ring(z, loop):
        base = len(points)
        for (x, y) in loop:
            points.append((x, y, z))
        return base

    z_bot_outer = 0.0
    z_bot_inner = bottom_bevel
    z_top_outer = size_z - top_bevel
    z_top_inner = size_z

    bot_outer_base = add_ring(z_bot_outer, outer)
    bot_inner_base = add_ring(z_bot_inner, inner_bottom) if inner_bottom else None
    top_outer_base = add_ring(z_top_outer, outer)
    top_inner_base = add_ring(z_top_inner, inner_top) if inner_top else None

    counts, idxs = [], []

    def quad(a, b, c, d):
        counts.append(4)
        idxs.extend([a, b, c, d])

    # straight side walls
    for i in range(n):
        j = (i + 1) % n
        quad(bot_outer_base + i, top_outer_base + i, top_outer_base + j, bot_outer_base + j)

    # top
    if inner_top:
        for i in range(n):
            j = (i + 1) % n
            quad(top_outer_base + i, top_outer_base + j, top_inner_base + j, top_inner_base + i)
        counts.append(n)
        idxs.extend([top_inner_base + i for i in range(n)])
    else:
        counts.append(n)
        idxs.extend([top_outer_base + i for i in range(n)])

    # bottom (reversed winding -> downward normal)
    if inner_bottom:
        for i in range(n):
            j = (i + 1) % n
            quad(bot_outer_base + j, bot_outer_base + i, bot_inner_base + i, bot_inner_base + j)
        counts.append(n)
        idxs.extend(list(reversed([bot_inner_base + i for i in range(n)])))
    else:
        counts.append(n)
        idxs.extend(list(reversed([bot_outer_base + i for i in range(n)])))

    return points, counts, idxs


def build_cylinder(radius, width, segs):
    """Cylinder with axis along local Y (wheel-style), centered at origin."""
    pts = []
    for i in range(segs):
        ang = 2 * math.pi * i / segs
        pts.append((radius * math.cos(ang), -width / 2.0, radius * math.sin(ang)))
    neg_base = 0
    for i in range(segs):
        ang = 2 * math.pi * i / segs
        pts.append((radius * math.cos(ang), width / 2.0, radius * math.sin(ang)))
    pos_base = segs

    counts, idxs = [], []
    for i in range(segs):
        j = (i + 1) % segs
        counts.append(4)
        idxs.extend([neg_base + i, pos_base + i, pos_base + j, neg_base + j])
    counts.append(segs)
    idxs.extend(list(reversed([neg_base + i for i in range(segs)])))
    counts.append(segs)
    idxs.extend([pos_base + i for i in range(segs)])
    return pts, counts, idxs


def face_normals(points, counts, idxs):
    normals = []
    p = 0
    for c in counts:
        face_idx = idxs[p:p + c]
        pts3 = [points[k] for k in face_idx]
        nx = ny = nz = 0.0
        for i in range(len(pts3)):
            x1, y1, z1 = pts3[i]
            x2, y2, z2 = pts3[(i + 1) % len(pts3)]
            nx += (y1 - y2) * (z1 + z2)
            ny += (z1 - z2) * (x1 + x2)
            nz += (x1 - x2) * (y1 + y2)
        length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        n = (nx / length, ny / length, nz / length)
        normals.extend([n] * c)
        p += c
    return normals


def extent_of(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return [Gf.Vec3f(min(xs), min(ys), min(zs)), Gf.Vec3f(max(xs), max(ys), max(zs))]


def author_mesh(stage, path, points, counts, idxs, translate=(0, 0, 0), quat=None):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(idxs)
    normals = face_normals(points, counts, idxs)
    mesh.CreateNormalsAttr([Gf.Vec3f(*n) for n in normals])
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateOrientationAttr(UsdGeom.Tokens.rightHanded)
    mesh.CreatePurposeAttr(UsdGeom.Tokens.default_)
    mesh.CreateExtentAttr(extent_of(points))

    prim = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)

    xformable = UsdGeom.Xformable(prim)
    xformable.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat if quat is not None else Gf.Quatd(1, 0, 0, 0))
    xformable.AddScaleOp().Set(Gf.Vec3d(1, 1, 1))
    return mesh


def quat_between(v_from, v_to):
    v_from = Gf.Vec3d(*v_from).GetNormalized()
    v_to = Gf.Vec3d(*v_to)
    length = v_to.GetLength()
    if length < 1e-9:
        return Gf.Quatd(1, 0, 0, 0), 0.0
    v_to_n = v_to / length
    rot = Gf.Rotation(v_from, v_to_n)
    return rot.GetQuat(), length


def quat_rotate_z(deg):
    rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), deg)
    return rot.GetQuat()


# ============================================================================
# Materials
# ============================================================================

def create_preview_material(stage, path, diffuse, metallic, roughness):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def bind_material(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim)
    UsdShade.MaterialBindingAPI(prim).Bind(material)


# ============================================================================
# Design parameters (meters; stage metersPerUnit == 1.0, confirmed below)
# ============================================================================

TOP_SIZE_X = 1.2          # long axis footprint -- UNCHANGED from source (preserves seam)
TOP_SIZE_Y = 0.7           # short/join axis footprint -- UNCHANGED from source
TOP_SURFACE_Z = 0.75       # UNCHANGED from source (table height stays 0.75 m, within 0.72-0.78 target)
TOP_THICKNESS = 0.035      # 35 mm, within 25-40 mm target (source was 50 mm)
TOP_BOTTOM_Z = TOP_SURFACE_Z - TOP_THICKNESS
TOP_CORNER_RADIUS = 0.035  # 35 mm, within 20-50 mm target
TOP_EDGE_BEVEL = 0.004     # 4 mm, within 2-6 mm target
TOP_CORNER_SEGS = 6

SUPPORT_X = 0.40           # inset from tabletop ends (+-0.6) -- inward of corners
COLUMN_SIZE = 0.055        # 55 mm square column, within 40-80 mm target
COLUMN_RADIUS = 0.004
COLUMN_SEGS = 2
COLUMN_BOTTOM_Z = 0.16

MOUNT_PLATE_SIZE = (0.14, 0.10, 0.010)
MOUNT_PLATE_RADIUS = 0.006

FOOT_SPREAD_Y = 0.25       # foot ends at +-0.25 (tabletop half-depth is 0.35 -> stays well inboard)
FOOT_SIZE = 0.045
FOOT_RADIUS = 0.004
CASTER_TOP_Z = 0.11        # height where foot end meets caster stem

WHEEL_RADIUS = 0.045       # 90 mm diameter, within 60-100 mm target
WHEEL_WIDTH = 0.032        # 32 mm, within 20-40 mm target
WHEEL_SEGS = 16
WHEEL_CENTER_Z = WHEEL_RADIUS
STEM_SIZE = 0.030
STEM_RADIUS = 0.004
STEM_BOTTOM_Z = WHEEL_CENTER_Z + WHEEL_RADIUS * 0.30
STEM_TOP_Z = CASTER_TOP_Z

CASTER_SWIVEL_DEG = {"A": 8.0, "B": -12.0}  # per-foot-side variation
CASTER_SWIVEL_JITTER = {"SupportLeft": 0.0, "SupportRight": 6.0}  # per-support extra variation

LAMINATE_COLOR = (0.80, 0.79, 0.76)
LAMINATE_METALLIC = 0.0
LAMINATE_ROUGHNESS = 0.55

METAL_COLOR = (0.16, 0.17, 0.19)
METAL_METALLIC = 0.6
METAL_ROUGHNESS = 0.45

WHEEL_COLOR = (0.03, 0.03, 0.03)
WHEEL_METALLIC = 0.1
WHEEL_ROUGHNESS = 0.6


# ============================================================================
# Support assembly builder
# ============================================================================

def build_support_assembly(stage, table_path, side_name, support_x, metal_mat, wheel_mat):
    base = f"{table_path}/Base/{side_name}"

    # Column: vertical, from COLUMN_BOTTOM_Z up to underside of the mount plate.
    col_top_z = TOP_BOTTOM_Z - MOUNT_PLATE_SIZE[2]
    col_pts, col_c, col_i = build_rounded_box(COLUMN_SIZE, COLUMN_SIZE, col_top_z - COLUMN_BOTTOM_Z,
                                               COLUMN_RADIUS, COLUMN_SEGS)
    m = author_mesh(stage, f"{base}/Column", col_pts, col_c, col_i,
                     translate=(support_x, 0.0, COLUMN_BOTTOM_Z))
    bind_material(m.GetPrim(), metal_mat)

    # Mount plate: thin plate directly under the tabletop, connecting column to top.
    plate_pts, plate_c, plate_i = build_rounded_box(*MOUNT_PLATE_SIZE, MOUNT_PLATE_RADIUS, 2)
    m = author_mesh(stage, f"{base}/MountPlate", plate_pts, plate_c, plate_i,
                     translate=(support_x, 0.0, col_top_z))
    bind_material(m.GetPrim(), metal_mat)

    # Two feet (V shape), extending toward +Y and -Y from the column base.
    foot_top = Gf.Vec3d(support_x, 0.0, COLUMN_BOTTOM_Z)
    for label, sign in (("A", 1.0), ("B", -1.0)):
        foot_end = Gf.Vec3d(support_x, sign * FOOT_SPREAD_Y, CASTER_TOP_Z)
        quat, length = quat_between((0, 0, 1), tuple(foot_end - foot_top))
        f_pts, f_c, f_i = build_rounded_box(FOOT_SIZE, FOOT_SIZE, length, FOOT_RADIUS, COLUMN_SEGS)
        m = author_mesh(stage, f"{base}/Foot{label}", f_pts, f_c, f_i,
                         translate=tuple(foot_top), quat=quat)
        bind_material(m.GetPrim(), metal_mat)

        # Caster at the foot end: short stem + wheel, with a small swivel variation.
        swivel = CASTER_SWIVEL_DEG[label] + CASTER_SWIVEL_JITTER[side_name]
        stem_pts, stem_c, stem_i = build_rounded_box(STEM_SIZE, STEM_SIZE, STEM_TOP_Z - STEM_BOTTOM_Z,
                                                       STEM_RADIUS, COLUMN_SEGS)
        m = author_mesh(stage, f"{base}/Caster{label}/Stem", stem_pts, stem_c, stem_i,
                         translate=(foot_end[0], foot_end[1], STEM_BOTTOM_Z))
        bind_material(m.GetPrim(), metal_mat)

        wheel_pts, wheel_c, wheel_i = build_cylinder(WHEEL_RADIUS, WHEEL_WIDTH, WHEEL_SEGS)
        wheel_quat = quat_rotate_z(swivel)
        m = author_mesh(stage, f"{base}/Caster{label}/Wheel", wheel_pts, wheel_c, wheel_i,
                         translate=(foot_end[0], foot_end[1], WHEEL_CENTER_Z), quat=wheel_quat)
        bind_material(m.GetPrim(), wheel_mat)


# ============================================================================
# Main
# ============================================================================

def main():
    stage = Usd.Stage.Open(STAGE_PATH)
    up_axis = UsdGeom.GetStageUpAxis(stage)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    assert up_axis == UsdGeom.Tokens.z
    assert abs(mpu - 1.0) < 1e-9

    bbcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)

    def rng(path):
        return bbcache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()

    # --- BEFORE report ---------------------------------------------------
    old_top = rng("/World/Table/TableTop")
    old_table = rng("/World/Table")
    old_top2 = rng("/World/Table_02/TableTop")
    seam_before = old_top2.GetMin()[1] - old_top.GetMax()[1]
    print("=== BEFORE ===")
    print(f"upAxis={up_axis} metersPerUnit={mpu}")
    print(f"Table/TableTop world bbox: min={tuple(old_top.GetMin())} max={tuple(old_top.GetMax())}")
    print(f"Tabletop dims: {tuple(old_top.GetMax() - old_top.GetMin())}")
    print(f"Table height (floor->top surface): {old_top.GetMax()[2] - old_table.GetMin()[2]}")
    print(f"Tabletop thickness: {old_top.GetMax()[2] - old_top.GetMin()[2]}")
    print("Existing leg structure: 4x Mesh box legs (Leg1..Leg4), 0.06x0.06 cross-section, full height 0.7")
    print("Existing caster structure: none")
    print(f"Floor elevation (Table bbox min Z): {old_table.GetMin()[2]}")
    print(f"Current center seam (Y): {seam_before}")

    # --- materials ---------------------------------------------------------
    looks = "/World/Looks"
    laminate_mat = create_preview_material(stage, f"{looks}/Laminate", LAMINATE_COLOR,
                                            LAMINATE_METALLIC, LAMINATE_ROUGHNESS)
    metal_mat = create_preview_material(stage, f"{looks}/Metal", METAL_COLOR,
                                         METAL_METALLIC, METAL_ROUGHNESS)
    wheel_mat = create_preview_material(stage, f"{looks}/CasterWheel", WHEEL_COLOR,
                                         WHEEL_METALLIC, WHEEL_ROUGHNESS)

    # --- rebuild /World/Table -----------------------------------------------
    for name in ["TableTop", "Leg1", "Leg2", "Leg3", "Leg4"]:
        p = stage.GetPrimAtPath(f"/World/Table/{name}")
        if p.IsValid():
            stage.RemovePrim(p.GetPath())
    # Table_02 will be rebuilt as a fresh copy after Table is finished.
    t2 = stage.GetPrimAtPath("/World/Table_02")
    if t2.IsValid():
        stage.RemovePrim(t2.GetPath())

    top_pts, top_c, top_i = build_rounded_box(TOP_SIZE_X, TOP_SIZE_Y, TOP_THICKNESS,
                                               TOP_CORNER_RADIUS, TOP_CORNER_SEGS,
                                               top_bevel=TOP_EDGE_BEVEL)
    top_mesh = author_mesh(stage, "/World/Table/TableTop", top_pts, top_c, top_i,
                            translate=(0.0, 0.0, TOP_BOTTOM_Z))
    bind_material(top_mesh.GetPrim(), laminate_mat)

    UsdGeom.Xform.Define(stage, "/World/Table/Base")
    build_support_assembly(stage, "/World/Table", "SupportLeft", -SUPPORT_X, metal_mat, wheel_mat)
    build_support_assembly(stage, "/World/Table", "SupportRight", SUPPORT_X, metal_mat, wheel_mat)

    # --- rebuild /World/Table_02 as an exact duplicate, re-apply the join offset ---
    root_layer = stage.GetRootLayer()
    ok = Sdf.CopySpec(root_layer, Sdf.Path("/World/Table"), root_layer, Sdf.Path("/World/Table_02"))
    if not ok:
        raise RuntimeError("Sdf.CopySpec of /World/Table -> /World/Table_02 failed")

    table2 = stage.GetPrimAtPath("/World/Table_02")
    t2_xform = UsdGeom.Xformable(table2)
    t2_translate_op = None
    for op in t2_xform.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            t2_translate_op = op
            break
    assert t2_translate_op is not None
    # Preserve the exact join offset from before this pass: short_dim (0.7) + 3mm seam.
    t2_translate_op.Set(Gf.Vec3d(0.0, 0.703, 0.0))

    stage.GetRootLayer().Save()
    print(f"\nSaved {STAGE_PATH}")


if __name__ == "__main__":
    main()
