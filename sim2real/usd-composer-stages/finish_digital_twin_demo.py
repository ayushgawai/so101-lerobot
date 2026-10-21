"""Finishes usd-composer-stages/digital-twin-demo-pick-place.usd in place:

1. Fixes a stray authoring bug on /World/GroundPlane/CollisionMesh -- its
   local mesh points are a correctly-centered 50x50m quad, but a leftover
   xformOp:translate=(2.16, -2.80, -2.05) sank the *visible* floor 2m below
   the actual floor level (z=0, where the table legs and PhysX collision
   plane already sit). Zeroed out so the visible floor matches the physics
   floor and the table/robot sitting on it.
2. Encloses the ground plane in a 4m x 4m room (walls only -- the ground
   plane itself remains the floor, per the request to build "with the
   ground plane"), matching build_indoor_scene.py's room dimensions/style
   so this stays visually consistent with the rest of the workshop's USD
   assets. No ceiling, so the new top camera and the room's own lighting
   both keep a clear view down into the scene.
3. Adds PhysicsCollisionAPI to the table legs (Leg1-4) -- TableTop already
   had it, the legs didn't; a real physical robot could clip through them.
4. Adds /World/TopCamera: a straight top-down camera framing the table +
   robot workspace.
5. Rebinds the *existing* second Composer viewport (RenderProduct
   ViewportTexture_0, currently pointed at the generic /OmniverseKit_Persp
   camera) to the new TopCamera. ViewportTexture_1 already points at the
   WristCamera that's mounted on the gripper -- so after this, reopening
   the file in Composer's saved 2-viewport layout shows Top camera in one
   pane and the wrist camera in the other, with no manual viewport setup
   needed beyond what was already saved in the file.

Run with the same throwaway usd-core venv as build_indoor_scene.py:
    usdenv\\Scripts\\python.exe usd-composer-stages\\finish_digital_twin_demo.py
"""
import os

from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Sdf, Gf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STAGE_PATH = os.path.join(SCRIPT_DIR, "digital-twin-demo-pick-place.usd")

ROOM_SIZE = 4.0        # meters, matches build_indoor_scene.py
WALL_HEIGHT = 2.5       # meters
WALL_THICKNESS = 0.1    # meters

TABLE_TOP_Z = 0.75       # from /World/Table's authored bbox (see inspection)
ROBOT_TOP_Z = 1.02       # from /World/so101_new_calib's authored bbox

stage = Usd.Stage.Open(STAGE_PATH)
if stage is None:
    raise RuntimeError(f"Could not open {STAGE_PATH}")

# --- 1. Fix the stray GroundPlane visual-mesh offset -----------------------
ground_mesh = UsdGeom.Xformable(stage.GetPrimAtPath("/World/GroundPlane/CollisionMesh"))
for op in ground_mesh.GetOrderedXformOps():
    if op.GetOpName() == "xformOp:translate":
        before = op.Get()
        op.Set(Gf.Vec3d(0, 0, 0))
        print(f"Fixed GroundPlane/CollisionMesh translate: {before} -> (0, 0, 0)")

# --- 2. Enclose in walls -----------------------------------------------------
def make_material(name, color):
    mat = UsdShade.Material.Define(stage, f"/World/Room/Looks/{name}")
    shader = UsdShade.Shader.Define(stage, f"/World/Room/Looks/{name}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def make_wall(path, center, size, material):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(size[0], size[1], size[2]))
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim())
    UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


wall_mat = make_material("Wall", (0.85, 0.84, 0.80))
half = ROOM_SIZE / 2.0
wall_z = WALL_HEIGHT / 2.0
make_wall("/World/Room/Walls/North", (0, half, wall_z), (ROOM_SIZE, WALL_THICKNESS, WALL_HEIGHT), wall_mat)
make_wall("/World/Room/Walls/South", (0, -half, wall_z), (ROOM_SIZE, WALL_THICKNESS, WALL_HEIGHT), wall_mat)
make_wall("/World/Room/Walls/East", (half, 0, wall_z), (WALL_THICKNESS, ROOM_SIZE, WALL_HEIGHT), wall_mat)
make_wall("/World/Room/Walls/West", (-half, 0, wall_z), (WALL_THICKNESS, ROOM_SIZE, WALL_HEIGHT), wall_mat)
print(f"Added 4 walls enclosing a {ROOM_SIZE}m x {ROOM_SIZE}m room, {WALL_HEIGHT}m tall.")

# --- 3. Collision on the table legs -----------------------------------------
for i in range(1, 5):
    leg = stage.GetPrimAtPath(f"/World/Table/Leg{i}")
    if leg.IsValid() and not leg.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(leg)
        print(f"Added PhysicsCollisionAPI to /World/Table/Leg{i}")

# --- 4. Top-down camera -------------------------------------------------------
# Framed on the table center, high enough to see the whole table + robot
# workspace; "up" = world +Y so the image reads like a floor plan.
eye = Gf.Vec3d(0.0, 0.0, 2.3)
target = Gf.Vec3d(0.0, 0.0, TABLE_TOP_Z)
up = Gf.Vec3d(0.0, 1.0, 0.0)
view_matrix = Gf.Matrix4d().SetLookAt(eye, target, up)
cam_to_world = view_matrix.GetInverse()

top_camera = UsdGeom.Camera.Define(stage, "/World/TopCamera")
top_camera.CreateFocalLengthAttr(18.0)
top_camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
xf = UsdGeom.Xformable(top_camera)
xf.ClearXformOpOrder()
transform_op = xf.AddTransformOp()
transform_op.Set(Gf.Matrix4d(cam_to_world))
print(f"Created /World/TopCamera at {cam_to_world.ExtractTranslation()}, looking down at table center.")

# --- 5. Rebind the existing second viewport to the new TopCamera -----------
render_product_path = "/Render/OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_0"
render_product = stage.GetPrimAtPath(render_product_path)
if render_product.IsValid():
    camera_rel = render_product.GetRelationship("camera")
    old_targets = camera_rel.GetTargets()
    camera_rel.SetTargets([Sdf.Path("/World/TopCamera")])
    print(f"Rebound {render_product_path} camera: {old_targets} -> ['/World/TopCamera']")
else:
    print(f"WARNING: {render_product_path} not found -- viewport not rebound")

stage.GetRootLayer().Save()
print(f"Saved {STAGE_PATH}")

# --- Verify ------------------------------------------------------------------
check = Usd.Stage.Open(STAGE_PATH)
bbcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)
for path in ["/World/GroundPlane", "/World/Room/Walls/North", "/World/Table", "/World/so101_new_calib"]:
    prim = check.GetPrimAtPath(path)
    rng = bbcache.ComputeWorldBound(prim).ComputeAlignedRange()
    print(f"{path}: min={rng.GetMin()} max={rng.GetMax()}")

for viewport_path in [
    "/Render/OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_0",
    "/Render/OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_1",
]:
    rel = check.GetPrimAtPath(viewport_path).GetRelationship("camera")
    print(f"{viewport_path} -> {rel.GetTargets()}")
