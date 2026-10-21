"""Add a wrist-camera Camera prim to real-to-sim.usd, seated at the physical
camera_mount bracket already modeled under
/World/SO_ARM101_USD/gripper/visuals/camera_mount (from the referenced
SO-ARM101-USD.usd asset -- lens/camera_cover/mount/focus/socket/pcb/bolts
meshes, the webcam housing on the real hardware).

That bracket's sub-meshes carry no xformOps of their own; their placement is
baked directly into mesh points, and camera_mount's own local frame turns
out not to matter for this either -- what's derived here is the mount's
real position/orientation via each sub-part's true LOCAL-frame AABB
(transforming every mesh point through the full prim-to-world-to-gripper
chain; taking a world-aligned box and rotating it after the fact is wrong,
since gripper's frame is rotated relative to world).

Boresight: the vector from "mount" (the base plate bolted to the gripper
body) to "lens" (the front optical element) -- verified against the
authored camera's world-forward axis afterwards (dot ~1.0).

Standoff: the eye is stood off from the FULL housing cluster's frontmost
point along that boresight (not just the lens sub-mesh's own extent) by a
fixed margin, so the eye clears the socket clip / pcb / bolts clutter
immediately around the lens. An earlier pass that only cleared the lens
sub-mesh by half its own depth left the eye embedded in that clutter --
caught by an actual RTX render (through Isaac Sim, not the pxr-only
usd-core venv) showing a macro shot of the socket bracket instead of the
workspace. Confirmed fixed by rendering the corrected pose and also a wide
external view with a debug marker at the eye position, both with the
timeline stopped -- this stage's root_joint has a disjointed-body-transform
warning on load, and running physics snaps the whole articulation off its
rest pose during the frames a render would accumulate.
"""
import os

from pxr import Usd, UsdGeom, Gf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
STAGE_PATH = os.path.join(
    REPO_ROOT, "source", "sim_to_real_so101", "demo", "real-to-sim.usd"
)

GRIPPER_PATH = "/World/SO_ARM101_USD/gripper"
CAMERA_MOUNT_PATH = f"{GRIPPER_PATH}/visuals/camera_mount"
WRIST_CAMERA_PATH = f"{GRIPPER_PATH}/WristCamera"
RENDER_PRODUCT_PATH = "/Render/OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_0"

STANDOFF_MARGIN = 0.015  # 15mm clear of the housing cluster's frontmost point

stage = Usd.Stage.Open(STAGE_PATH)
if stage is None:
    raise RuntimeError(f"Could not open {STAGE_PATH}")

xf_cache = UsdGeom.XformCache()

gripper_prim = stage.GetPrimAtPath(GRIPPER_PATH)
if not gripper_prim.IsValid():
    raise RuntimeError(f"{GRIPPER_PATH} not found")
gripper_world = xf_cache.GetLocalToWorldTransform(gripper_prim)
gripper_world_inv = gripper_world.GetInverse()


def local_points(root_path):
    """All mesh points in root's subtree, transformed into gripper-local
    space point-by-point (not a rotated world-aligned box)."""
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        raise RuntimeError(f"{root_path} not found")
    pts_out = []
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if not pts:
            continue
        mesh_to_gripper = xf_cache.GetLocalToWorldTransform(prim) * gripper_world_inv
        pts_out.extend(mesh_to_gripper.Transform(Gf.Vec3d(*p)) for p in pts)
    return pts_out


def bbox_center(pts):
    mn = Gf.Vec3d(*[min(p[i] for p in pts) for i in range(3)])
    mx = Gf.Vec3d(*[max(p[i] for p in pts) for i in range(3)])
    return (mn + mx) * 0.5


mount_center = bbox_center(local_points(f"{CAMERA_MOUNT_PATH}/mount"))
lens_center = bbox_center(local_points(f"{CAMERA_MOUNT_PATH}/lens"))

boresight = lens_center - mount_center
boresight_dir = boresight / boresight.GetLength()

housing_pts = local_points(CAMERA_MOUNT_PATH)
frontmost_proj = max(Gf.Dot(p, boresight_dir) for p in housing_pts)
lens_center_proj = Gf.Dot(lens_center, boresight_dir)
eye = lens_center + boresight_dir * ((frontmost_proj + STANDOFF_MARGIN) - lens_center_proj)

target = eye + boresight_dir
up = Gf.Vec3d(0.0, 1.0, 0.0)
view_matrix = Gf.Matrix4d().SetLookAt(eye, target, up)
cam_to_gripper = view_matrix.GetInverse()

camera_prim = UsdGeom.Camera.Define(stage, WRIST_CAMERA_PATH)
# Match the wrist-cam optics already authored for the same hardware module
# in digital-twin-demo-pick-place.usd's WristCamera, for consistency across
# the two demo stages.
camera_prim.CreateFocalLengthAttr(18.14756202697754)
camera_prim.CreateHorizontalApertureAttr(20.954999923706055)
camera_prim.CreateVerticalApertureAttr(15.290800094604492)
camera_prim.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10000000))

xf = UsdGeom.Xformable(camera_prim)
xf.ClearXformOpOrder()
xf.AddTransformOp().Set(Gf.Matrix4d(cam_to_gripper))

# Rebind the Composer viewport to the new wrist camera so reopening the file
# shows the wrist-cam view instead of the generic perspective camera.
render_product = stage.GetPrimAtPath(RENDER_PRODUCT_PATH)
if not render_product.IsValid():
    raise RuntimeError(f"{RENDER_PRODUCT_PATH} not found")
camera_rel = render_product.GetRelationship("camera")
old_targets = camera_rel.GetTargets()
camera_rel.SetTargets([camera_prim.GetPath()])

stage.GetRootLayer().Save()
print(f"Saved {STAGE_PATH}")
print(f"WristCamera eye (gripper-local): {eye}")
print(f"WristCamera boresight (gripper-local): {boresight_dir}")
print(f"Rebound {RENDER_PRODUCT_PATH} camera: {old_targets} -> ['{WRIST_CAMERA_PATH}']")

# --- Verify: world-space forward should point along the mount->lens direction ---
check = Usd.Stage.Open(STAGE_PATH)
check_xf_cache = UsdGeom.XformCache()
cam = check.GetPrimAtPath(WRIST_CAMERA_PATH)
cam_world = check_xf_cache.GetLocalToWorldTransform(cam)
cam_pos = cam_world.ExtractTranslation()
cam_forward = -cam_world.ExtractRotationMatrix().GetRow(2)
world_boresight = gripper_world.TransformDir(boresight_dir)
world_boresight = world_boresight / world_boresight.GetLength()
print(f"\nWristCamera world pos: {cam_pos}")
print(f"WristCamera world forward: {cam_forward}")
print(f"Expected world boresight (from mount->lens): {world_boresight}")
print(f"Dot product (should be ~1.0): {Gf.Dot(cam_forward, world_boresight):.6f}")

rp_check = check.GetPrimAtPath(RENDER_PRODUCT_PATH)
print(f"Verify: {RENDER_PRODUCT_PATH} -> {rp_check.GetRelationship('camera').GetTargets()}")
