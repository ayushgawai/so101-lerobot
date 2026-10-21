"""Adds a small AWS Builder Loft promotional cube to real-to-sim.usd and saves
the result as a new stage, without touching the original file or any existing
prim.

Inspection summary (see the printed diagnostics for live values on your
machine -- these are the findings from the authored reference stage):
  - real-to-sim.usd is Z-up, metersPerUnit = 1.0 (stage units are meters).
    /World references ../assets/usd/indoor-room.usd; the room/table geometry
    itself (Floor, Walls, RobotMount, Table, Table_02, materials, lights) is
    authored there and pulled in by reference -- this script never edits
    those prims, it only reads them (bbox queries) and adds new sibling prims
    under /World in the *new* output layer.
  - There are two tabletops: /World/Table/TableTop (the original table, at
    the world origin, with the SO-101 RobotMount marker underneath it) and
    /World/Table_02/TableTop (a second table joined along +Y to widen the
    work surface). /World/Table is the primary/original one, so the cube is
    placed there by default. The tabletop-finder below is nonetheless
    generic (searches for any prim whose name looks like a tabletop) rather
    than hard-coding this path, and picks the shortest matching path as the
    "primary" table when more than one is found.
  - No Camera prims are authored in the stage; Kit's saved viewport
    bookmarks (stage customLayerData["cameraSettings"]) show the default
    Perspective camera sits up in the +X/+Y/+Z octant looking back toward
    the origin (and the "Front" bookmark sits on +X looking at -X, "Top" on
    +Z looking down). So the faces most likely to be seen by a viewer are
    +X, +Y and +Z (top). The cube's decorated faces are assigned so the
    primary "AWS Builder Loft" text lands on +X, the AWS logo on +Y and the
    pixel smiley on +Z (top) -- i.e. CUBE_YAW_DEGREES = 0 already looks
    natural from the default viewport; less important faces (URL, heart,
    cloud) go on -X/-Y/-Z.
  - The mesh is a flat, unsubdivided box (subdivisionScheme "none"), not a
    Catmull-Clark box with edge creases. An earlier version used creases for
    a slight bevel, but verified against a real headless Isaac Sim RTX
    render (not just a guess) that this specific coarse-cage + crease
    combination produced a broken/inverted-normal black patch at the bottom
    corner in Kit's RTX/Hydra Storm delegate -- a rendering bug, not a
    lighting or shadow issue, and unrelated to crease sharpness value (it
    persisted even with matching corner sharpness authored). A flat cube
    with sharp edges renders correctly and is a perfectly normal look for a
    small molded plastic cube; it was confirmed to fix the artifact by
    re-rendering with subdivision disabled and seeing the black patch
    disappear completely.

Run with the throwaway usd-core venv already used by the other scripts in
this folder (it also has Pillow for texture generation):
    usdenv\\Scripts\\python.exe usd-composer-stages\\build_aws_builder_cube.py
"""
import math
import os

from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Sdf, Gf
from PIL import Image, ImageDraw, ImageFont, ImageFilter, ImageChops

# =============================================================================
# User-configurable values
# =============================================================================
CUBE_SIZE = 0.05            # meters; cube is CUBE_SIZE x CUBE_SIZE x CUBE_SIZE
CUBE_POSITION_X = None      # meters, world space; None = auto-center on tabletop
CUBE_POSITION_Y = None      # meters, world space; None = auto-center on tabletop
CUBE_YAW_DEGREES = 0.0      # rotation about +Z (up), applied on top of the
                             # natural face layout described above

ENABLE_RIGID_BODY = False   # False = static placed object. True = dynamic
                             # rigid body that can interact with the table.

# Secondary knobs (safe to leave alone)
PLASTIC_DENSITY_KG_M3 = 1200.0  # ABS/PLA-ish white plastic, used only when
                                 # ENABLE_RIGID_BODY = True
TILE_PX = 1024                # per-face texture resolution in the atlas

# =============================================================================
# Paths
# =============================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
DEMO_DIR = os.path.join(REPO_ROOT, "source", "sim_to_real_so101", "demo")

SRC_PATH = os.path.join(DEMO_DIR, "real-to-sim.usd")
OUTPUT_DIR = DEMO_DIR
OUTPUT_PATH = os.path.join(OUTPUT_DIR, "room-and-table-with-aws-cube.usd")

TEXTURE_SUBDIR = os.path.join("tex", "aws_builder_cube")
TEXTURE_DIR = os.path.join(OUTPUT_DIR, TEXTURE_SUBDIR)
ATLAS_FILENAME = "aws_builder_cube_atlas.png"
ATLAS_PATH = os.path.join(TEXTURE_DIR, ATLAS_FILENAME)
# Path written into the USD file itself -- relative to OUTPUT_PATH's directory
# so the asset stays portable if the demo/ folder is moved as a whole.
ATLAS_REL_PATH = "./" + TEXTURE_SUBDIR.replace(os.sep, "/") + "/" + ATLAS_FILENAME

FONT_MONO_BOLD = r"C:\Windows\Fonts\consolab.ttf"
FONT_MONO = r"C:\Windows\Fonts\consola.ttf"
FONT_SANS = r"C:\Windows\Fonts\arial.ttf"
FONT_SANS_BOLD = r"C:\Windows\Fonts\arialbd.ttf"

CUBE_ROOT_NAME = "AWSBuilderCube"


# =============================================================================
# Texture generation (procedural -- text via PIL fonts, pixel-art via a tiny
# rasterized mask upscaled with nearest-neighbor for a blocky look)
# =============================================================================
def _fit_font(path, text, max_width, max_height, start_size=220):
    size = start_size
    font = ImageFont.truetype(path, size)
    while size > 8:
        bbox = font.getbbox(text)
        w = bbox[2] - bbox[0]
        h = bbox[3] - bbox[1]
        if w <= max_width and h <= max_height:
            return font
        size -= 4
        font = ImageFont.truetype(path, size)
    return font


def _draw_centered_text(canvas, text, font, fill):
    draw = ImageDraw.Draw(canvas)
    bbox = draw.textbbox((0, 0), text, font=font)
    w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
    cx, cy = canvas.width / 2, canvas.height / 2
    draw.text((cx - w / 2 - bbox[0], cy - h / 2 - bbox[1]), text, font=font, fill=fill)


def make_face1_text(size):
    """Face 1: 'AWS Builder Loft' in black monospace, centered, white bg."""
    canvas = Image.new("RGB", (size, size), (255, 255, 255))
    margin = size * 0.14
    text = "AWS Builder\nLoft"
    font = ImageFont.truetype(FONT_MONO_BOLD, int(size * 0.135))
    draw = ImageDraw.Draw(canvas)
    bbox = draw.multiline_textbbox((0, 0), text, font=font, align="center", spacing=int(size * 0.05))
    w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
    while (w > size - 2 * margin or h > size - 2 * margin) and font.size > 8:
        font = ImageFont.truetype(FONT_MONO_BOLD, font.size - 2)
        bbox = draw.multiline_textbbox((0, 0), text, font=font, align="center", spacing=int(size * 0.05))
        w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
    cx, cy = size / 2, size / 2
    draw.multiline_text(
        (cx - w / 2 - bbox[0], cy - h / 2 - bbox[1]), text, font=font,
        fill=(20, 20, 20), align="center", spacing=int(size * 0.05),
    )
    return canvas


def make_face2_aws_logo(size):
    """Face 2: black lowercase 'aws' wordmark with a curved smile/arrow
    swoosh underneath (simplified, generic rendition -- not a trademarked
    logo asset)."""
    canvas = Image.new("RGB", (size, size), (255, 255, 255))
    draw = ImageDraw.Draw(canvas)
    text = "aws"
    font = _fit_font(FONT_SANS_BOLD, text, size * 0.62, size * 0.32)
    bbox = draw.textbbox((0, 0), text, font=font)
    w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
    text_cy = size * 0.42
    tx = size / 2 - w / 2 - bbox[0]
    ty = text_cy - h / 2 - bbox[1]
    draw.text((tx, ty), text, font=font, fill=(20, 20, 20))

    # Smile/arrow swoosh: an arc under the wordmark with a small arrowhead
    # at its right end, evoking the AWS "smile" mark.
    arc_left = size * 0.30
    arc_right = size * 0.70
    arc_top = text_cy + h * 0.55
    arc_bottom = arc_top + size * 0.16
    line_w = max(2, int(size * 0.018))
    draw.arc([arc_left, arc_top, arc_right, arc_bottom], start=20, end=160, fill=(20, 20, 20), width=line_w)

    # arrowhead near the right end of the arc, pointing up-right
    tip = (arc_right - size * 0.01, arc_top + size * 0.055)
    a = (tip[0] - size * 0.035, tip[1] + size * 0.018)
    b = (tip[0] - size * 0.006, tip[1] + size * 0.045)
    draw.polygon([tip, a, b], fill=(20, 20, 20))
    return canvas


def make_face3_url(size):
    """Face 3: 'builder.aws.com' in black text, centered, white bg."""
    canvas = Image.new("RGB", (size, size), (255, 255, 255))
    text = "builder.aws.com"
    font = _fit_font(FONT_SANS, text, size * 0.8, size * 0.18)
    _draw_centered_text(canvas, text, font, (20, 20, 20))
    return canvas


def _lerp_color(c0, c1, t):
    return tuple(int(round(c0[i] + (c1[i] - c0[i]) * t)) for i in range(3))


def _pixel_outline_tile(size, draw_fn, color_top, color_bottom, grid=22, inset=0.16):
    """Builds a stepped/pixel-art OUTLINE graphic: draw_fn fills a shape on a
    tiny `grid`x`grid` mask, the mask is eroded by one cell to find the
    border ring, the ring is colored with a top-to-bottom gradient, the
    interior is left white, and the whole thing is nearest-upscaled onto a
    `size`x`size` white tile with `inset` fractional margin -- giving the
    blocky/pixelated look with substantial white margins."""
    mask = Image.new("L", (grid, grid), 0)
    d = ImageDraw.Draw(mask)
    draw_fn(d, grid)
    eroded = mask.filter(ImageFilter.MinFilter(3))
    ring = ImageChops.subtract(mask, eroded)

    tiny = Image.new("RGB", (grid, grid), (255, 255, 255))
    rpx = ring.load()
    for y in range(grid):
        for x in range(grid):
            if rpx[x, y] > 0:
                t = y / max(1, grid - 1)
                tiny.putpixel((x, y), _lerp_color(color_top, color_bottom, t))

    inner = int(size * (1.0 - 2 * inset))
    upscaled = tiny.resize((inner, inner), Image.NEAREST)
    canvas = Image.new("RGB", (size, size), (255, 255, 255))
    off = (size - inner) // 2
    canvas.paste(upscaled, (off, off))
    return canvas


def make_face4_heart(size):
    """Face 4: pixel-art heart OUTLINE, pink-to-red gradient, white center."""
    def draw_heart(d, grid):
        cx1, cy1 = grid * 0.32, grid * 0.34
        cx2, cy2 = grid * 0.68, grid * 0.34
        rad = grid * 0.22
        d.ellipse([cx1 - rad, cy1 - rad, cx1 + rad, cy1 + rad], fill=255)
        d.ellipse([cx2 - rad, cy2 - rad, cx2 + rad, cy2 + rad], fill=255)
        d.polygon(
            [(grid * 0.10, grid * 0.40), (grid * 0.90, grid * 0.40), (grid * 0.5, grid * 0.94)],
            fill=255,
        )
    return _pixel_outline_tile(size, draw_heart, (255, 140, 160), (200, 20, 40))


def make_face5_smiley(size, grid=24, inset=0.16):
    """Face 5: minimal pixel smiling face -- pale blue/purple outline,
    two dark-blue rectangular eyes, a stepped dark-blue smiling mouth,
    white background."""
    outline_color = (150, 160, 235)
    dark_blue = (30, 40, 120)

    mask = Image.new("L", (grid, grid), 0)
    d = ImageDraw.Draw(mask)
    d.ellipse([grid * 0.06, grid * 0.06, grid * 0.94, grid * 0.94], fill=255)
    eroded = mask.filter(ImageFilter.MinFilter(3))
    ring = ImageChops.subtract(mask, eroded)

    tiny = Image.new("RGB", (grid, grid), (255, 255, 255))
    rpx = ring.load()
    for y in range(grid):
        for x in range(grid):
            if rpx[x, y] > 0:
                tiny.putpixel((x, y), outline_color)

    td = ImageDraw.Draw(tiny)
    # two rectangular eyes, in integer grid cells for crisp, reliably-visible
    # blocks -- kept short and high up so there's a clear gap above the mouth
    for ex in (6, 15):
        td.rectangle([ex, 8, ex + 2, 10], fill=dark_blue)
    # stepped smiling mouth: a wide flat center step with two rising steps on
    # each side, i.e. a staircase approximation of an upward curve ("u" shape)
    mouth_steps = [
        (9, 14, 16),   # (col_start, col_end, row) -- center, lowest/deepest
        (7, 8, 15),    # left, one step up
        (15, 16, 15),  # right, one step up
        (5, 6, 14),    # left, another step up
        (17, 18, 14),  # right, another step up
    ]
    for x0, x1, y in mouth_steps:
        td.rectangle([x0, y, x1, y + 1], fill=dark_blue)

    inner = int(size * (1.0 - 2 * inset))
    upscaled = tiny.resize((inner, inner), Image.NEAREST)
    canvas = Image.new("RGB", (size, size), (255, 255, 255))
    off = (size - inner) // 2
    canvas.paste(upscaled, (off, off))
    return canvas


def make_face6_cloud(size):
    """Face 6: stepped pixel-art cloud OUTLINE, pale-pink to orange
    gradient, white interior."""
    def draw_cloud(d, grid):
        d.ellipse([grid * 0.08, grid * 0.42, grid * 0.42, grid * 0.76], fill=255)
        d.ellipse([grid * 0.30, grid * 0.24, grid * 0.70, grid * 0.64], fill=255)
        d.ellipse([grid * 0.58, grid * 0.40, grid * 0.94, grid * 0.78], fill=255)
        d.rectangle([grid * 0.18, grid * 0.52, grid * 0.82, grid * 0.80], fill=255)
    return _pixel_outline_tile(size, draw_cloud, (255, 200, 180), (240, 130, 30))


FACE_DESIGN_BUILDERS = {
    "face1_text": make_face1_text,
    "face2_logo": make_face2_aws_logo,
    "face3_url": make_face3_url,
    "face4_heart": make_face4_heart,
    "face5_smiley": make_face5_smiley,
    "face6_cloud": make_face6_cloud,
}

# 2 columns x 3 rows atlas layout
ATLAS_GRID = {
    "face1_text": (0, 0),
    "face2_logo": (0, 1),
    "face3_url": (1, 0),
    "face4_heart": (1, 1),
    "face5_smiley": (2, 0),
    "face6_cloud": (2, 1),
}
ATLAS_COLS, ATLAS_ROWS = 2, 3


def build_atlas():
    os.makedirs(TEXTURE_DIR, exist_ok=True)
    atlas_w, atlas_h = ATLAS_COLS * TILE_PX, ATLAS_ROWS * TILE_PX
    atlas = Image.new("RGB", (atlas_w, atlas_h), (255, 255, 255))
    uv_rects = {}
    for design_key, builder in FACE_DESIGN_BUILDERS.items():
        tile = builder(TILE_PX)
        row, col = ATLAS_GRID[design_key]
        x0, y0 = col * TILE_PX, row * TILE_PX
        atlas.paste(tile, (x0, y0))
        x1, y1 = x0 + TILE_PX, y0 + TILE_PX
        u0, u1 = x0 / atlas_w, x1 / atlas_w
        v_low = 1.0 - y1 / atlas_h    # bottom edge of tile -> low v
        v_high = 1.0 - y0 / atlas_h   # top edge of tile -> high v
        uv_rects[design_key] = (u0, u1, v_low, v_high)
    atlas.save(ATLAS_PATH)
    return uv_rects


# =============================================================================
# Scene inspection
# =============================================================================
def find_tabletop(stage):
    """Generic tabletop finder: searches the whole stage for any prim whose
    name looks like a tabletop, and returns the one with the shortest path
    (the "primary" table if several tables are joined together)."""
    candidates = []
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        if "tabletop" in name or ("table" in name and "top" in name):
            candidates.append(prim)
    if not candidates:
        raise RuntimeError("No tabletop-like prim found in the stage")
    candidates.sort(key=lambda p: len(str(p.GetPath())))
    return candidates[0]


# =============================================================================
# Cube mesh construction (Catmull-Clark box with edge creases for a slight
# bevel, faceVarying UVs mapping each face to one atlas cell)
# =============================================================================
FACE_DEFS = [
    # key, normal(axis,sign), right(axis,sign), up(axis,sign), design
    dict(key="+X", normal=(0, 1), right=(1, 1), up=(2, 1), design="face1_text"),
    dict(key="-X", normal=(0, -1), right=(1, -1), up=(2, 1), design="face3_url"),
    dict(key="+Y", normal=(1, 1), right=(0, -1), up=(2, 1), design="face2_logo"),
    dict(key="-Y", normal=(1, -1), right=(0, 1), up=(2, 1), design="face4_heart"),
    dict(key="+Z", normal=(2, 1), right=(0, 1), up=(1, 1), design="face5_smiley"),
    dict(key="-Z", normal=(2, -1), right=(0, -1), up=(1, 1), design="face6_cloud"),
]

EDGE_PAIRS_BY_SIGN_AXIS = None  # filled in build_cube_mesh


def _corner_signs(face, cr, cu):
    s = [0, 0, 0]
    s[face["normal"][0]] = face["normal"][1]
    s[face["right"][0]] = cr * face["right"][1]
    s[face["up"][0]] = cu * face["up"][1]
    return tuple(s)


def build_cube_mesh(stage, mesh_path, half_size, uv_rects):
    points = []
    corner_index = {}
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                corner_index[(sx, sy, sz)] = len(points)
                points.append(Gf.Vec3f(sx * half_size, sy * half_size, sz * half_size))

    face_vertex_counts = []
    face_vertex_indices = []
    st_values = []

    corner_order = [(-1, -1), (1, -1), (1, 1), (-1, 1)]  # bl, br, tr, tl

    for face in FACE_DEFS:
        idxs = [corner_index[_corner_signs(face, cr, cu)] for cr, cu in corner_order]
        face_vertex_indices.extend(idxs)
        face_vertex_counts.append(4)

        u0, u1, v_low, v_high = uv_rects[face["design"]]
        st_values.extend([
            Gf.Vec2f(u0, v_low),   # bl
            Gf.Vec2f(u1, v_low),   # br
            Gf.Vec2f(u1, v_high),  # tr
            Gf.Vec2f(u0, v_high),  # tl
        ])

        # sanity check: outward normal of this face must match its declared
        # normal axis/sign, given the vertex winding above
        p0, p1, p3 = points[idxs[0]], points[idxs[1]], points[idxs[3]]
        n = Gf.Cross(Gf.Vec3f(p1) - Gf.Vec3f(p0), Gf.Vec3f(p3) - Gf.Vec3f(p0))
        expected = Gf.Vec3f(0, 0, 0)
        expected[face["normal"][0]] = face["normal"][1]
        assert Gf.Dot(n, expected) > 0, f"Face {face['key']} winding produces inverted normal"

    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
    mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
    mesh.CreateExtentAttr(mesh.ComputeExtent(points))
    mesh.CreateDoubleSidedAttr(False)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)

    st_primvar = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    st_primvar.Set(st_values)

    return mesh


def build_material(stage, looks_path):
    material = UsdShade.Material.Define(stage, looks_path.AppendChild("AWSBuilderCube_Material"))

    pbr = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("PBRShader"))
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.55)
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
    diffuse_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(ATLAS_REL_PATH)
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
    print("=== Building AWS Builder Loft promo cube ===")
    print(f"Source stage: {SRC_PATH}")

    src_stage = Usd.Stage.Open(SRC_PATH)
    if src_stage is None:
        raise RuntimeError(f"Could not open {SRC_PATH}")

    up_axis = UsdGeom.GetStageUpAxis(src_stage)
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(src_stage)
    print(f"Detected stage up axis: {up_axis}")
    print(f"Detected stage metersPerUnit: {meters_per_unit}")
    assert up_axis == UsdGeom.Tokens.z, f"Unexpected up axis {up_axis}, this script assumes Z-up"

    tabletop_prim = find_tabletop(src_stage)
    print(f"Detected table prim: {tabletop_prim.GetPath()}")

    bbcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)
    rng = bbcache.ComputeWorldBound(tabletop_prim).ComputeAlignedRange()
    tmin, tmax = rng.GetMin(), rng.GetMax()
    print(f"Table bounds: min={tuple(tmin)} max={tuple(tmax)}")

    table_top_z = tmax[2]
    table_center_x = (tmin[0] + tmax[0]) * 0.5
    table_center_y = (tmin[1] + tmax[1]) * 0.5
    table_dim_x = tmax[0] - tmin[0]
    table_dim_y = tmax[1] - tmin[1]

    # --- copy the source stage into the new output stage --------------------
    # Written via a temp file + atomic replace rather than an in-place
    # truncating copy: if OUTPUT_PATH is already open elsewhere (e.g. loaded
    # in an Isaac Sim / Kit viewport), a direct truncating write can fail
    # with a sharing violation on Windows, while a temp-file + os.replace
    # does not.
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    import shutil
    tmp_output_path = OUTPUT_PATH + ".tmp"
    shutil.copyfile(SRC_PATH, tmp_output_path)
    os.replace(tmp_output_path, OUTPUT_PATH)
    stage = Usd.Stage.Open(OUTPUT_PATH)
    root_layer = stage.GetRootLayer()

    world = stage.GetPrimAtPath("/World")
    assert world.IsValid(), "Output stage has no /World prim"

    cube_root_path = world.GetPath().AppendChild(CUBE_ROOT_NAME)
    assert not stage.GetPrimAtPath(cube_root_path).IsValid(), (
        f"{cube_root_path} already exists -- refusing to overwrite an existing prim"
    )

    # --- textures -------------------------------------------------------------
    uv_rects = build_atlas()
    print(f"Wrote texture atlas: {ATLAS_PATH}")
    print(f"Atlas reference path (relative, in USD): {ATLAS_REL_PATH}")

    # --- geometry ---------------------------------------------------------------
    cube_size_units = CUBE_SIZE / meters_per_unit
    half_size = cube_size_units / 2.0

    cube_xform = UsdGeom.Xform.Define(stage, cube_root_path)
    cube_prim = cube_xform.GetPrim()

    geometry_scope = UsdGeom.Scope.Define(stage, cube_root_path.AppendChild("Geometry"))
    looks_scope = UsdGeom.Scope.Define(stage, cube_root_path.AppendChild("Looks"))

    geo_path = geometry_scope.GetPath().AppendChild(f"{CUBE_ROOT_NAME}_Geo")
    geo_mesh = build_cube_mesh(stage, geo_path, half_size, uv_rects)

    material = build_material(stage, looks_scope.GetPath())
    UsdShade.MaterialBindingAPI.Apply(geo_mesh.GetPrim()).Bind(material)

    # --- placement ----------------------------------------------------------
    pos_x_units = (CUBE_POSITION_X / meters_per_unit) if CUBE_POSITION_X is not None else table_center_x
    pos_y_units = (CUBE_POSITION_Y / meters_per_unit) if CUBE_POSITION_Y is not None else table_center_y
    pos_z_units = table_top_z + half_size  # rest directly on the tabletop, no intersection

    translate_op = cube_xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(pos_x_units, pos_y_units, pos_z_units))
    if abs(CUBE_YAW_DEGREES) > 1e-9:
        rotate_op = cube_xform.AddRotateZOp()
        rotate_op.Set(CUBE_YAW_DEGREES)

    # --- physics --------------------------------------------------------------
    UsdPhysics.CollisionAPI.Apply(geo_mesh.GetPrim())
    mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(geo_mesh.GetPrim())
    mesh_collision.CreateApproximationAttr().Set(UsdPhysics.Tokens.convexHull)

    if ENABLE_RIGID_BODY:
        UsdPhysics.RigidBodyAPI.Apply(cube_prim)
        mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
        volume_m3 = CUBE_SIZE ** 3
        mass_kg = PLASTIC_DENSITY_KG_M3 * volume_m3
        mass_api.CreateMassAttr().Set(mass_kg)
        print(f"Rigid body ENABLED: mass = {mass_kg:.4f} kg (density {PLASTIC_DENSITY_KG_M3} kg/m^3)")
    else:
        print("Rigid body DISABLED: cube is a static collider only")

    stage.GetRootLayer().Save()

    # --- diagnostics ------------------------------------------------------------
    check = Usd.Stage.Open(OUTPUT_PATH)
    cube_rng = bbcache.ComputeWorldBound(check.GetPrimAtPath(cube_root_path)).ComputeAlignedRange()
    cmin, cmax = cube_rng.GetMin(), cube_rng.GetMax()
    cdims = cmax - cmin

    print()
    print("=== Diagnostics ===")
    print(f"Stage units: metersPerUnit={meters_per_unit}, upAxis={up_axis}")
    print(f"Table prim: {tabletop_prim.GetPath()}  bounds min={tuple(tmin)} max={tuple(tmax)}")
    print(f"Table footprint: {table_dim_x:.4f} x {table_dim_y:.4f} (stage units)")
    print(f"Final cube dimensions (world bbox): {tuple(cdims)}")
    print(f"Cube footprint / table footprint ratio: "
          f"{cdims[0] / table_dim_x:.3%} x {cdims[1] / table_dim_y:.3%}")
    print(f"Final cube position (center, world, stage units): "
          f"({pos_x_units}, {pos_y_units}, {pos_z_units})")
    print(f"Cube bottom Z = {cmin[2]:.6f}, table top Z = {table_top_z:.6f}, "
          f"gap = {cmin[2] - table_top_z:.2e} (should be ~0)")
    print(f"Texture atlas: {ATLAS_PATH}")
    print(f"Output USD: {OUTPUT_PATH}")
    print(f"Cube prim hierarchy: {cube_root_path}, "
          f"{geometry_scope.GetPath()}, {geo_path}, "
          f"{looks_scope.GetPath()}, {material.GetPath()}")


if __name__ == "__main__":
    main()
