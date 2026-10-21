"""Re-anchors /World/.../gripper_link/WristCameraMount/WristCamera in
digital-twin-demo-pick-place.usd using the real hardware geometry from
C:\\Users\\OMNI-User\\Desktop\\SO-ARM100\\Simulation\\SO101\\so101_new_calib.urdf
as the reference, per user request.

That URDF has no camera link/mesh at all (confirmed by listing the whole
SO101 sim folder) -- there's no bracket geometry to copy a mount offset
from. What it *does* define precisely is gripper_frame_link, the real
gripper/TCP frame: a fixed joint off gripper_link at
xyz=(-0.0079, -0.000218121, -0.0981274) rpy=(0, pi, 0). This exactly
matches gripper_link/gripper_frame_link already baked into this stage's
robot geometry (confirmed by direct comparison), confirming the demo
file's robot mesh is built from this exact URDF.

The *previous* WristCamera authoring placed it 0.22m away from
gripper_link along an axis unrelated to that TCP direction -- far outside
the robot's own body, not grounded in anything the URDF describes.

Fix: keep WristCameraMount at gripper_link's own origin (already correct,
identity transform) and re-point WristCamera itself to sit at that origin,
aimed straight down the gripper's real approach axis at the URDF-defined
TCP point (gripper_frame_link's offset) -- the standard eye-in-hand
convention, derived only from real URDF numbers, no invented bracket
offset.
"""
import os

from pxr import Usd, UsdGeom, Gf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STAGE_PATH = os.path.join(SCRIPT_DIR, "digital-twin-demo-pick-place.usd")

GRIPPER_LINK_PATH = (
    "/World/so101_new_calib/Geometry/base_link/shoulder_link/upper_arm_link/"
    "lower_arm_link/wrist_link/gripper_link"
)
WRIST_CAMERA_PATH = f"{GRIPPER_LINK_PATH}/WristCameraMount/WristCamera"

# From the URDF's gripper_frame_joint (fixed, gripper_link -> gripper_frame_link).
TCP_LOCAL_OFFSET = Gf.Vec3d(-0.0079, -0.000218121, -0.0981274)

stage = Usd.Stage.Open(STAGE_PATH)
if stage is None:
    raise RuntimeError(f"Could not open {STAGE_PATH}")

camera_prim = stage.GetPrimAtPath(WRIST_CAMERA_PATH)
if not camera_prim.IsValid():
    raise RuntimeError(f"{WRIST_CAMERA_PATH} not found")

xf = UsdGeom.Xformable(camera_prim)
old_ops = [(op.GetOpName(), op.Get()) for op in xf.GetOrderedXformOps()]
print("Old WristCamera local xform:", old_ops)

# Look-at, all in gripper_link's local space. Boresight points from
# gripper_link's origin at the real TCP point defined by the URDF; the eye
# is pulled back 3cm along that same line (no invented lateral offset --
# the URDF has no camera bracket to derive one from) so the lens clears the
# gripper body's own mesh instead of sitting embedded inside it (confirmed
# by bbox check: gripper_link's origin sits inside its own subtree's bbox).
target = TCP_LOCAL_OFFSET
boresight = target - Gf.Vec3d(0.0, 0.0, 0.0)
boresight_dir = boresight / boresight.GetLength()
STANDOFF = 0.03
eye = -boresight_dir * STANDOFF
up = Gf.Vec3d(0.0, 1.0, 0.0)
view_matrix = Gf.Matrix4d().SetLookAt(eye, target, up)
cam_to_gripper = view_matrix.GetInverse()

xf.ClearXformOpOrder()
xf.AddTransformOp().Set(Gf.Matrix4d(cam_to_gripper))

stage.GetRootLayer().Save()
print(f"Saved {STAGE_PATH}")

# --- Verify: camera's world-space forward should point at the world-space TCP ---
check = Usd.Stage.Open(STAGE_PATH)
xf_cache = UsdGeom.XformCache()
cam = check.GetPrimAtPath(WRIST_CAMERA_PATH)
cam_world = xf_cache.GetLocalToWorldTransform(cam)
cam_pos = cam_world.ExtractTranslation()
cam_rot = cam_world.ExtractRotationMatrix()
cam_forward = -cam_rot.GetRow(2)

tcp_prim = check.GetPrimAtPath(f"{GRIPPER_LINK_PATH}/gripper_frame_link")
tcp_world = xf_cache.GetLocalToWorldTransform(tcp_prim).ExtractTranslation()

to_tcp = (tcp_world - cam_pos)
to_tcp_normalized = to_tcp / to_tcp.GetLength()
print(f"WristCamera world pos: {cam_pos}")
print(f"WristCamera world forward: {cam_forward}")
print(f"Direction to URDF TCP point: {to_tcp_normalized}")
print(f"Dot product (should be ~1.0 if aimed correctly): {Gf.Dot(cam_forward, to_tcp_normalized):.6f}")
