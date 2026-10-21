"""Adds a small clear plastic bowl with a paper label to real-to-sim.usd
(in place -- same file, no copy), following the placement/collision style of
build_aws_builder_cube.py but a simpler shell (no atlas, one label texture).

Shape: a shallow rectangular container with rounded corners and slightly
tapered sides (bottom footprint smaller than the top rim, like a small deli
container). Modeled as a single-wall shell -- rounded-rect bottom cap +
tapered side walls, open top -- with doubleSided=True so the interior is
visible through the clear plastic. Inside, flat on the floor, sits a small
white paper card with a large handwritten black "B" and a small red dot near
its center (generated procedurally with PIL, same approach as the AWS cube's
face textures).

Placement: centered on Table_02 (the second, otherwise-empty tabletop), well
clear of the AWS Builder cube and the SO-101 arm which both live on Table.

Run with Isaac Sim's own Python (needs pxr + PIL, both bundled):
    C:\\Isaac-Sim\\python.bat usd-composer-stages\\add_paper_bowl_real_to_sim.py
"""
import math
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Sdf, Gf
from PIL import Image, ImageDraw, ImageFont

# =============================================================================
# User-configurable values (meters; stage is metersPerUnit = 1.0)
# =============================================================================
BOWL_BOTTOM_HALF_X = 0.045   # bottom footprint: 9.0cm x 6.5cm
BOWL_BOTTOM_HALF_Y = 0.0325
BOWL_TOP_HALF_X = 0.050      # top rim footprint: 10.0cm x 7.5cm (tapered outward)
BOWL_TOP_HALF_Y = 0.0375
BOWL_BOTTOM_CORNER_RADIUS = 0.010
BOWL_TOP_CORNER_RADIUS = 0.012
BOWL_HEIGHT = 0.032          # shallow
CORNER_SEGMENTS = 8          # points per 90-degree corner arc

PAPER_SIZE_X = 0.050
PAPER_SIZE_Y = 0.036
PAPER_Z_OFFSET = 0.0005      # lift off the bowl floor to avoid z-fighting

BOWL_POSITION_X = 0.0        # meters, world space; centered on Table_02
BOWL_POSITION_Y = 0.70       # meters, world space; mid-length of Table_02
BOWL_YAW_DEGREES = 0.0

# =============================================================================
# Paths
# =============================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
DEMO_DIR = os.path.join(REPO_ROOT, "source", "sim_to_real_so101", "demo")

STAGE_PATH = os.path.join(DEMO_DIR, "real-to-sim.usd")

TEXTURE_SUBDIR = os.path.join("tex", "paper_bowl")
TEXTURE_DIR = os.path.join(DEMO_DIR, TEXTURE_SUBDIR)
LABEL_FILENAME = "paper_label.png"
LABEL_PATH = os.path.join(TEXTURE_DIR, LABEL_FILENAME)
LABEL_REL_PATH = "./" + TEXTURE_SUBDIR.replace(os.sep, "/") + "/" + LABEL_FILENAME

FONT_HANDWRITING = r"C:\Windows\Fonts\LHANDW.TTF"

BOWL_ROOT_NAME = "PaperBowl"
LABEL_PX = 1024


# =============================================================================
# Paper label texture: white card, large handwritten black "B", small red dot
# near its center.
# =============================================================================
def build_paper_label():
    os.makedirs(TEXTURE_DIR, exist_ok=True)
    canvas = Image.new("RGB", (LABEL_PX, LABEL_PX), (250, 250, 247))
    draw = ImageDraw.Draw(canvas)

    letter = "B"
    size = int(LABEL_PX * 0.75)
    font = ImageFont.truetype(FONT_HANDWRITING, size)
    bbox = draw.textbbox((0, 0), letter, font=font)
    w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
    while (w > LABEL_PX * 0.8 or h > LABEL_PX * 0.8) and size > 8:
        size -= 4
        font = ImageFont.truetype(FONT_HANDWRITING, size)
        bbox = draw.textbbox((0, 0), letter, font=font)
        w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]

    cx, cy = LABEL_PX / 2, LABEL_PX / 2
    tx = cx - w / 2 - bbox[0]
    ty = cy - h / 2 - bbox[1]
    draw.text((tx, ty), letter, font=font, fill=(18, 18, 18))

    # small red dot near the center of the glyph
    dot_r = LABEL_PX * 0.018
    dot_cx, dot_cy = cx, cy + h * 0.05
    draw.ellipse(
        [dot_cx - dot_r, dot_cy - dot_r, dot_cx + dot_r, dot_cy + dot_r],
        fill=(196, 24, 24),
    )

    canvas.save(LABEL_PATH)
    return LABEL_PATH


# =============================================================================
# Rounded-rectangle profile helper
# =============================================================================
def rounded_rect_points(half_x, half_y, radius, segments):
    """Returns a CCW (viewed from +Z looking down) loop of 2D points tracing
    a rounded rectangle, `segments` points per 90-degree corner arc."""
    radius = min(radius, half_x, half_y)
    corners = [
        (half_x - radius, half_y - radius, 0.0),      # top-right arc start
        (-half_x + radius, half_y - radius, 90.0),    # top-left
        (-half_x + radius, -half_y + radius, 180.0),  # bottom-left
        (half_x - radius, -half_y + radius, 270.0),   # bottom-right
    ]
    pts = []
    for ccx, ccy, start_deg in corners:
        for i in range(segments):
            ang = math.radians(start_deg + 90.0 * i / segments)
            pts.append((ccx + radius * math.cos(ang), ccy + radius * math.sin(ang)))
    return pts


# =============================================================================
# Bowl mesh: rounded-rect bottom cap (n-gon) + tapered side walls (quads),
# open top. Single-wall shell, doubleSided so the clear-plastic interior
# reads correctly from above.
# =============================================================================
def build_bowl_mesh(stage, mesh_path):
    bottom_pts_2d = rounded_rect_points(
        BOWL_BOTTOM_HALF_X, BOWL_BOTTOM_HALF_Y, BOWL_BOTTOM_CORNER_RADIUS, CORNER_SEGMENTS
    )
    top_pts_2d = rounded_rect_points(
        BOWL_TOP_HALF_X, BOWL_TOP_HALF_Y, BOWL_TOP_CORNER_RADIUS, CORNER_SEGMENTS
    )
    n = len(bottom_pts_2d)
    assert n == len(top_pts_2d)

    points = []
    bottom_idx = []
    for x, y in bottom_pts_2d:
        bottom_idx.append(len(points))
        points.append(Gf.Vec3f(x, y, 0.0))
    top_idx = []
    for x, y in top_pts_2d:
        top_idx.append(len(points))
        points.append(Gf.Vec3f(x, y, BOWL_HEIGHT))

    face_vertex_counts = []
    face_vertex_indices = []

    # bottom cap: single n-gon, reversed order so it winds CW from above
    # (outward/downward-facing normal) -- doubleSided covers any mistake here.
    face_vertex_counts.append(n)
    face_vertex_indices.extend(reversed(bottom_idx))

    # side walls: quads between corresponding bottom/top loop points
    for i in range(n):
        j = (i + 1) % n
        face_vertex_counts.append(4)
        face_vertex_indices.extend([bottom_idx[i], bottom_idx[j], top_idx[j], top_idx[i]])

    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
    mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
    mesh.CreateExtentAttr(mesh.ComputeExtent(points))
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)

    # sanity check: first side-wall quad should have an outward-pointing
    # normal (away from the bowl's central axis)
    p0, p1, p3 = points[bottom_idx[0]], points[bottom_idx[1]], points[top_idx[0]]
    wall_normal = Gf.Cross(Gf.Vec3f(p1) - Gf.Vec3f(p0), Gf.Vec3f(p3) - Gf.Vec3f(p0))
    outward_dir = Gf.Vec3f(p0[0], p0[1], 0.0)
    assert Gf.Dot(wall_normal, outward_dir) > 0, "Side wall winding produces an inward-facing normal"

    return mesh


def build_paper_mesh(stage, mesh_path, uv_rect=(0.0, 1.0, 0.0, 1.0)):
    hx, hy = PAPER_SIZE_X / 2.0, PAPER_SIZE_Y / 2.0
    z = PAPER_Z_OFFSET
    points = [
        Gf.Vec3f(-hx, -hy, z),
        Gf.Vec3f(hx, -hy, z),
        Gf.Vec3f(hx, hy, z),
        Gf.Vec3f(-hx, hy, z),
    ]
    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateExtentAttr(mesh.ComputeExtent(points))
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)

    u0, u1, v0, v1 = uv_rect
    st_primvar = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    st_primvar.Set([Gf.Vec2f(u0, v0), Gf.Vec2f(u1, v0), Gf.Vec2f(u1, v1), Gf.Vec2f(u0, v1)])
    return mesh


# =============================================================================
# Materials
# =============================================================================
def build_bowl_material(stage, looks_path):
    material = UsdShade.Material.Define(stage, looks_path.AppendChild("ClearPlastic_Material"))
    pbr = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("PBRShader"))
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.92, 0.95, 0.97))
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.05)
    pbr.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    pbr.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.45)
    pbr.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.22)
    pbr.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")
    return material


def build_paper_material(stage, looks_path):
    material = UsdShade.Material.Define(stage, looks_path.AppendChild("PaperLabel_Material"))
    pbr = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("PBRShader"))
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    pbr.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    pbr.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
    diffuse_input = pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
    material.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")

    st_reader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("stReader"))
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    diffuse_tex = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("DiffuseTexture"))
    diffuse_tex.CreateIdAttr("UsdUVTexture")
    diffuse_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(LABEL_REL_PATH)
    diffuse_tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    diffuse_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader.ConnectableAPI(), "result")
    diffuse_tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    diffuse_tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    diffuse_tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    diffuse_input.ConnectToSource(diffuse_tex.ConnectableAPI(), "rgb")
    return material


# =============================================================================
# Main
# =============================================================================
def main():
    print("=== Adding paper bowl to real-to-sim.usd ===")
    stage = Usd.Stage.Open(STAGE_PATH)
    assert stage, f"Failed to open {STAGE_PATH}"

    up_axis = UsdGeom.GetStageUpAxis(stage)
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    assert up_axis == UsdGeom.Tokens.z, f"Unexpected up axis {up_axis}"
    assert abs(meters_per_unit - 1.0) < 1e-9, f"Unexpected metersPerUnit {meters_per_unit}"

    world = stage.GetPrimAtPath("/World")
    assert world.IsValid(), "Stage has no /World prim"

    bowl_root_path = world.GetPath().AppendChild(BOWL_ROOT_NAME)
    assert not stage.GetPrimAtPath(bowl_root_path).IsValid(), (
        f"{bowl_root_path} already exists -- refusing to overwrite"
    )

    table02_top = stage.GetPrimAtPath("/World/Table_02/TableTop")
    assert table02_top.IsValid(), "/World/Table_02/TableTop not found"
    bbcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)
    table_rng = bbcache.ComputeWorldBound(table02_top).ComputeAlignedRange()
    table_top_z = table_rng.GetMax()[2]
    print(f"Table_02 top z = {table_top_z}")

    # --- texture ---
    build_paper_label()
    print(f"Wrote paper label texture: {LABEL_PATH}")

    # --- geometry ---
    bowl_xform = UsdGeom.Xform.Define(stage, bowl_root_path)
    bowl_prim = bowl_xform.GetPrim()

    geometry_scope = UsdGeom.Scope.Define(stage, bowl_root_path.AppendChild("Geometry"))
    looks_scope = UsdGeom.Scope.Define(stage, bowl_root_path.AppendChild("Looks"))

    bowl_geo_path = geometry_scope.GetPath().AppendChild("Bowl_Geo")
    bowl_mesh = build_bowl_mesh(stage, bowl_geo_path)

    paper_geo_path = geometry_scope.GetPath().AppendChild("Paper_Geo")
    paper_mesh = build_paper_mesh(stage, paper_geo_path)

    bowl_material = build_bowl_material(stage, looks_scope.GetPath())
    UsdShade.MaterialBindingAPI.Apply(bowl_mesh.GetPrim()).Bind(bowl_material)

    paper_material = build_paper_material(stage, looks_scope.GetPath())
    UsdShade.MaterialBindingAPI.Apply(paper_mesh.GetPrim()).Bind(paper_material)

    # --- placement ---
    translate_op = bowl_xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(BOWL_POSITION_X, BOWL_POSITION_Y, table_top_z))
    if abs(BOWL_YAW_DEGREES) > 1e-9:
        bowl_xform.AddRotateZOp().Set(BOWL_YAW_DEGREES)

    # --- physics: static concave collider (thin shell, so use the raw mesh
    # rather than a convex hull, which would fill in the open cavity) ---
    UsdPhysics.CollisionAPI.Apply(bowl_mesh.GetPrim())
    bowl_collision = UsdPhysics.MeshCollisionAPI.Apply(bowl_mesh.GetPrim())
    bowl_collision.CreateApproximationAttr().Set(UsdPhysics.Tokens.none)

    stage.Save()

    # --- diagnostics ---
    check = Usd.Stage.Open(STAGE_PATH)
    bowl_rng = bbcache.ComputeWorldBound(check.GetPrimAtPath(bowl_root_path)).ComputeAlignedRange()
    bmin, bmax = bowl_rng.GetMin(), bowl_rng.GetMax()
    print()
    print("=== Diagnostics ===")
    print(f"Bowl world bbox: min={tuple(bmin)} max={tuple(bmax)}")
    print(f"Bowl bottom z = {bmin[2]:.6f}, table top z = {table_top_z:.6f}, "
          f"gap = {bmin[2] - table_top_z:.2e} (should be ~0)")
    print(f"Bowl prim hierarchy: {bowl_root_path}, {geometry_scope.GetPath()}, "
          f"{bowl_geo_path}, {paper_geo_path}, {looks_scope.GetPath()}")
    print(f"Saved {STAGE_PATH}")


if __name__ == "__main__":
    main()
    simulation_app.close()
