"""Builds a standalone "room + table" USD asset: the existing indoor-room.usd
(floor + 4 enclosing walls + lighting + mount marker) plus the table
authored in the Composer demo stage, with the robot deliberately left out --
mirrors indoor-room.usd's own "reusable asset, no robot" pattern (see
build_indoor_scene.py) so this can be referenced into Isaac Sim/Lab or
Composer independently of any particular robot placement.

Table geometry is copied via Sdf.CopySpec directly from
digital-twin-demo-pick-place.usd's /World/Table (a plain Cube + 4 Mesh legs,
no materials/textures to worry about -- confirmed by inspection) rather than
re-authored by hand, so it's guaranteed to match the Composer demo exactly.

Produces:
    source/sim_to_real_so101/assets/usd/room-and-table.usd

Run with the same throwaway usd-core venv as build_indoor_scene.py:
    usdenv\\Scripts\\python.exe usd-composer-stages\\build_room_and_table.py
"""
import os

from pxr import Usd, UsdGeom, Sdf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
ASSETS_USD_DIR = os.path.join(REPO_ROOT, "source", "sim_to_real_so101", "assets", "usd")

ROOM_PATH = os.path.join(ASSETS_USD_DIR, "indoor-room.usd")
DEMO_STAGE_PATH = os.path.join(SCRIPT_DIR, "digital-twin-demo-pick-place.usd")
OUT_PATH = os.path.join(ASSETS_USD_DIR, "room-and-table.usd")

stage = Usd.Stage.CreateNew(OUT_PATH)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

room_rel_path = os.path.relpath(ROOM_PATH, ASSETS_USD_DIR).replace(os.sep, "/")
world.GetPrim().GetReferences().AddReference(room_rel_path)

# The room's RobotMount marker was designed for the tabletop lightbox rig
# (see build_indoor_scene.py); a full-size table now occupies the same
# origin, so the marker would just poke out from under the tabletop --
# hide it here rather than deleting the reference's own content.
mount_over = stage.OverridePrim("/World/RobotMount")
UsdGeom.Imageable(mount_over).CreateVisibilityAttr("invisible")

# --- Copy /World/Table verbatim from the Composer demo stage ---------------
demo_layer = Sdf.Layer.FindOrOpen(DEMO_STAGE_PATH)
if demo_layer is None:
    raise RuntimeError(f"Could not open {DEMO_STAGE_PATH}")

dest_layer = stage.GetRootLayer()
Sdf.CreatePrimInLayer(dest_layer, "/World/Table")
ok = Sdf.CopySpec(demo_layer, "/World/Table", dest_layer, "/World/Table")
if not ok:
    raise RuntimeError("Sdf.CopySpec of /World/Table failed")
print("Copied /World/Table from digital-twin-demo-pick-place.usd")

stage.GetRootLayer().Save()
print(f"Wrote {OUT_PATH}")

# --- Verify ------------------------------------------------------------------
check = Usd.Stage.Open(OUT_PATH)
print("UpAxis:", UsdGeom.GetStageUpAxis(check))
print("MetersPerUnit:", UsdGeom.GetStageMetersPerUnit(check))
bbcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)
for path in ["/World/Floor", "/World/Walls/North", "/World/Table", "/World/RobotMount"]:
    prim = check.GetPrimAtPath(path)
    imageable = UsdGeom.Imageable(prim)
    rng = bbcache.ComputeWorldBound(prim).ComputeAlignedRange()
    print(f"{path}: visibility={imageable.ComputeVisibility()} min={rng.GetMin()} max={rng.GetMax()}")
print("All prims:", [str(p.GetPath()) for p in check.Traverse()])
