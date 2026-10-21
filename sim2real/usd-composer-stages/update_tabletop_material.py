"""Material-only pass on room-and-table_joined.usd: retints the shared
/World/Looks/Laminate material (bound to both /World/Table/TableTop and
/World/Table_02/TableTop) to a light warm-gray commercial laminate look.

Touches ONLY shader input values on /World/Looks/Laminate/Shader. No prims
are added/removed/moved; no geometry, transforms, or other materials are
touched.

Target: sRGB #BEBEB7 (190, 190, 183), converted to linear for
UsdPreviewSurface's diffuseColor (which is authored in linear color space).
"""
import os

from pxr import Usd, UsdShade, Sdf, Gf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
ASSETS_USD_DIR = os.path.join(REPO_ROOT, "source", "sim_to_real_so101", "assets", "usd")
STAGE_PATH = os.path.join(ASSETS_USD_DIR, "room-and-table_joined.usd")


def srgb_to_linear(c8):
    c = c8 / 255.0
    if c <= 0.04045:
        return c / 12.92
    return ((c + 0.055) / 1.055) ** 2.4


TARGET_SRGB_8BIT = (190, 190, 183)  # #BEBEB7
TARGET_LINEAR = Gf.Vec3f(*[srgb_to_linear(c) for c in TARGET_SRGB_8BIT])
NEW_ROUGHNESS = 0.62
NEW_METALLIC = 0.0
NEW_OPACITY = 1.0

stage = Usd.Stage.Open(STAGE_PATH)
shader = UsdShade.Shader(stage.GetPrimAtPath("/World/Looks/Laminate/Shader"))
assert shader.GetPrim().IsValid(), "/World/Looks/Laminate/Shader not found"

diffuse_input = shader.GetInput("diffuseColor")
old_diffuse = diffuse_input.Get()
old_roughness = shader.GetInput("roughness").Get()
old_metallic = shader.GetInput("metallic").Get()

print("=== BEFORE ===")
print(f"diffuseColor (linear): {old_diffuse}")
print(f"roughness: {old_roughness}")
print(f"metallic: {old_metallic}")

diffuse_input.Set(TARGET_LINEAR)
shader.GetInput("roughness").Set(NEW_ROUGHNESS)
shader.GetInput("metallic").Set(NEW_METALLIC)
shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(NEW_OPACITY)

stage.GetRootLayer().Save()

print("\n=== AFTER ===")
print(f"diffuseColor (linear): {diffuse_input.Get()}  (target sRGB #BEBEB7 = {TARGET_SRGB_8BIT})")
print(f"roughness: {shader.GetInput('roughness').Get()}")
print(f"metallic: {shader.GetInput('metallic').Get()}")
print(f"opacity: {shader.GetInput('opacity').Get()}")
print(f"\nSaved {STAGE_PATH}")
