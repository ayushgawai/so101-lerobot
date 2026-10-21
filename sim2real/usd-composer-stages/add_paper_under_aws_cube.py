"""Adds a small white paper card under /World/AWSBuilderCube in real-to-sim.usd
(in place -- same file, no copy), following the same paper-label technique as
add_paper_bowl_real_to_sim.py: a flat card with a large handwritten black
letter (here "A") and a small red dot near its center, generated procedurally
with PIL and UV-mapped onto a simple quad.

Unlike the bowl's paper (which sits inside an open, elevated container), the
AWS cube rests directly on the tabletop with no gap, so "on the bottom of
AWSBuilderCube" is modeled literally: the paper is laid flat on the table at
the cube's (x, y), and the cube is lifted by the paper's thickness so it
rests on top of the card instead of clipping through it. The card's footprint
is larger than the cube's so it peeks out visibly from under all four sides.

Run with Isaac Sim's own Python (needs pxr + PIL, both bundled):
    C:\\Isaac-Sim\\python.bat usd-composer-stages\\add_paper_under_aws_cube.py
"""
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
from PIL import Image, ImageDraw, ImageFont

# =============================================================================
# User-configurable values (meters; stage is metersPerUnit = 1.0)
# =============================================================================
PAPER_HALF_SIZE = 0.033      # square card, 6.6cm x 6.6cm -- bigger than the
                               # 5cm cube so it peeks out on every side
PAPER_THICKNESS_LIFT = 0.0004  # how much to raise the cube so it sits on
                               # top of the (visually flat) card

LETTER = "A"

# =============================================================================
# Paths
# =============================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
DEMO_DIR = os.path.join(REPO_ROOT, "source", "sim_to_real_so101", "demo")

STAGE_PATH = os.path.join(DEMO_DIR, "real-to-sim.usd")

TEXTURE_SUBDIR = os.path.join("tex", "aws_builder_cube")
TEXTURE_DIR = os.path.join(DEMO_DIR, TEXTURE_SUBDIR)
LABEL_FILENAME = "paper_label_a.png"
LABEL_PATH = os.path.join(TEXTURE_DIR, LABEL_FILENAME)
LABEL_REL_PATH = "./" + TEXTURE_SUBDIR.replace(os.sep, "/") + "/" + LABEL_FILENAME

FONT_HANDWRITING = r"C:\Windows\Fonts\LHANDW.TTF"
LABEL_PX = 1024

PAPER_ROOT_NAME = "AWSCubePaper"
CUBE_PATH = "/World/AWSBuilderCube"


# =============================================================================
# Paper label texture: white card, large handwritten black letter, small red
# dot near its center. Same technique as add_paper_bowl_real_to_sim.py.
# =============================================================================
def build_paper_label():
    os.makedirs(TEXTURE_DIR, exist_ok=True)
    canvas = Image.new("RGB", (LABEL_PX, LABEL_PX), (250, 250, 247))
    draw = ImageDraw.Draw(canvas)

    size = int(LABEL_PX * 0.75)
    font = ImageFont.truetype(FONT_HANDWRITING, size)
    bbox = draw.textbbox((0, 0), LETTER, font=font)
    w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
    while (w > LABEL_PX * 0.8 or h > LABEL_PX * 0.8) and size > 8:
        size -= 4
        font = ImageFont.truetype(FONT_HANDWRITING, size)
        bbox = draw.textbbox((0, 0), LETTER, font=font)
        w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]

    cx, cy = LABEL_PX / 2, LABEL_PX / 2
    tx = cx - w / 2 - bbox[0]
    ty = cy - h / 2 - bbox[1]
    draw.text((tx, ty), LETTER, font=font, fill=(18, 18, 18))

    dot_r = LABEL_PX * 0.018
    dot_cx, dot_cy = cx, cy + h * 0.05
    draw.ellipse(
        [dot_cx - dot_r, dot_cy - dot_r, dot_cx + dot_r, dot_cy + dot_r],
        fill=(196, 24, 24),
    )

    canvas.save(LABEL_PATH)
    return LABEL_PATH


def build_paper_mesh(stage, mesh_path, half_size):
    z = 0.0
    points = [
        Gf.Vec3f(-half_size, -half_size, z),
        Gf.Vec3f(half_size, -half_size, z),
        Gf.Vec3f(half_size, half_size, z),
        Gf.Vec3f(-half_size, half_size, z),
    ]
    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateExtentAttr(mesh.ComputeExtent(points))
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)

    st_primvar = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    st_primvar.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    return mesh


def build_paper_material(stage, looks_path):
    material = UsdShade.Material.Define(stage, looks_path.AppendChild("AWSCubePaperLabel_Material"))
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


def main():
    print("=== Adding paper under AWSBuilderCube in real-to-sim.usd ===")
    stage = Usd.Stage.Open(STAGE_PATH)
    assert stage, f"Failed to open {STAGE_PATH}"

    up_axis = UsdGeom.GetStageUpAxis(stage)
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    assert up_axis == UsdGeom.Tokens.z
    assert abs(meters_per_unit - 1.0) < 1e-9

    world = stage.GetPrimAtPath("/World")
    paper_root_path = world.GetPath().AppendChild(PAPER_ROOT_NAME)
    assert not stage.GetPrimAtPath(paper_root_path).IsValid(), (
        f"{paper_root_path} already exists -- refusing to overwrite"
    )

    cube_prim = stage.GetPrimAtPath(CUBE_PATH)
    assert cube_prim.IsValid(), f"{CUBE_PATH} not found"
    cube_xformable = UsdGeom.Xformable(cube_prim)
    translate_op = None
    for op in cube_xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    assert translate_op is not None, f"{CUBE_PATH} has no translate op"
    cube_x, cube_y, cube_z = translate_op.Get()
    print(f"Cube current translate: ({cube_x}, {cube_y}, {cube_z})")

    bbcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)
    cube_rng = bbcache.ComputeWorldBound(cube_prim).ComputeAlignedRange()
    cube_bottom_z_before = cube_rng.GetMin()[2]
    print(f"Cube bottom z (before lift): {cube_bottom_z_before}")

    # --- texture ---
    build_paper_label()
    print(f"Wrote paper label texture: {LABEL_PATH}")

    # --- geometry ---
    paper_xform = UsdGeom.Xform.Define(stage, paper_root_path)
    geometry_scope = UsdGeom.Scope.Define(stage, paper_root_path.AppendChild("Geometry"))
    looks_scope = UsdGeom.Scope.Define(stage, paper_root_path.AppendChild("Looks"))

    paper_geo_path = geometry_scope.GetPath().AppendChild("Paper_Geo")
    paper_mesh = build_paper_mesh(stage, paper_geo_path, PAPER_HALF_SIZE)

    paper_material = build_paper_material(stage, looks_scope.GetPath())
    UsdShade.MaterialBindingAPI.Apply(paper_mesh.GetPrim()).Bind(paper_material)

    # paper sits flat on the table, lifted a hair above the tabletop surface
    # itself so it doesn't z-fight with it where it peeks out past the cube
    paper_z = cube_bottom_z_before + PAPER_THICKNESS_LIFT
    paper_xform.AddTranslateOp().Set(Gf.Vec3d(cube_x, cube_y, paper_z))

    # --- lift the cube so it rests on top of the card instead of clipping ---
    translate_op.Set(Gf.Vec3d(cube_x, cube_y, cube_z + PAPER_THICKNESS_LIFT))

    stage.Save()

    # --- diagnostics ---
    check = Usd.Stage.Open(STAGE_PATH)
    paper_rng = bbcache.ComputeWorldBound(check.GetPrimAtPath(paper_root_path)).ComputeAlignedRange()
    cube_rng_after = bbcache.ComputeWorldBound(check.GetPrimAtPath(CUBE_PATH)).ComputeAlignedRange()
    print()
    print("=== Diagnostics ===")
    print(f"Paper world bbox: min={tuple(paper_rng.GetMin())} max={tuple(paper_rng.GetMax())}")
    print(f"Cube world bbox (after lift): min={tuple(cube_rng_after.GetMin())} max={tuple(cube_rng_after.GetMax())}")
    print(f"Cube bottom z = {cube_rng_after.GetMin()[2]:.6f}, paper top z = {paper_rng.GetMax()[2]:.6f}, "
          f"gap = {cube_rng_after.GetMin()[2] - paper_rng.GetMax()[2]:.2e} (should be ~0)")
    print(f"Saved {STAGE_PATH}")


if __name__ == "__main__":
    main()
    simulation_app.close()
