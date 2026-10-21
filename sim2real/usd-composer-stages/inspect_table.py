"""Inspect /World/Table in room-and-table.usd: hierarchy, transforms, bbox,
references/payloads, instancing, material bindings, stage upAxis/metersPerUnit.

Read-only -- does not modify the source file.
"""
import os

from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
ASSETS_USD_DIR = os.path.join(REPO_ROOT, "source", "sim_to_real_so101", "assets", "usd")
SRC_PATH = os.path.join(ASSETS_USD_DIR, "room-and-table.usd")

stage = Usd.Stage.Open(SRC_PATH)
print("=== Stage metadata ===")
print("upAxis:", UsdGeom.GetStageUpAxis(stage))
print("metersPerUnit:", UsdGeom.GetStageMetersPerUnit(stage))
print("defaultPrim:", stage.GetDefaultPrim().GetPath())

print("\n=== /World children ===")
world = stage.GetPrimAtPath("/World")
for child in world.GetChildren():
    print(" ", child.GetPath(), child.GetTypeName())

print("\n=== /World/Table subtree ===")
table = stage.GetPrimAtPath("/World/Table")
print("Table prim exists:", table.IsValid())
print("Table typeName:", table.GetTypeName())
print("Table specifier:", table.GetSpecifier())
print("Table IsInstance:", table.IsInstance())
print("Table instanceable:", table.IsInstanceable())

refs = table.GetMetadata("references")
print("Table references metadata:", refs)
payloads = table.GetMetadata("payload")
print("Table payload metadata:", payloads)

# Local transform ops on Table itself
xform = UsdGeom.Xformable(table)
ops = xform.GetOrderedXformOps()
print("Table local xformOps:")
for op in ops:
    print("   ", op.GetOpName(), op.Get())

def dump_prim(prim, indent=0):
    pad = "  " * indent
    print(f"{pad}{prim.GetPath()}  type={prim.GetTypeName()}  specifier={prim.GetSpecifier()}")
    xf = UsdGeom.Xformable(prim)
    if xf:
        for op in xf.GetOrderedXformOps():
            print(f"{pad}   xformOp: {op.GetOpName()} = {op.Get()}")
    # material binding
    binding_api = UsdShade.MaterialBindingAPI(prim)
    rel = prim.GetRelationship("material:binding")
    if rel and rel.IsValid():
        targets = rel.GetTargets()
        if targets:
            print(f"{pad}   material:binding -> {targets}")
    for child in prim.GetChildren():
        dump_prim(child, indent + 1)

print("\n=== Full /World/Table hierarchy dump ===")
dump_prim(table)

print("\n=== World-space bbox of /World/Table/Top ===")
bbcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=True)
top = stage.GetPrimAtPath("/World/Table/TableTop")
print("Top prim exists:", top.IsValid(), "type:", top.GetTypeName())
rng = bbcache.ComputeWorldBound(top).ComputeAlignedRange()
mn = rng.GetMin()
mx = rng.GetMax()
print("Top world bbox min:", mn)
print("Top world bbox max:", mx)
print("Top dimensions (x,y,z):", mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2])
center = (mn + mx) * 0.5
print("Top world bbox center:", center)

print("\n=== World-space bbox of /World/Table (whole assembly) ===")
rng2 = bbcache.ComputeWorldBound(table).ComputeAlignedRange()
print("Table world bbox min:", rng2.GetMin(), "max:", rng2.GetMax())

print("\n=== Table world transform matrix ===")
xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
world_mat = xform_cache.GetLocalToWorldTransform(table)
print(world_mat)

print("\n=== Legs bboxes ===")
for leg_name in ["Leg1", "Leg2", "Leg3", "Leg4"]:
    leg = stage.GetPrimAtPath(f"/World/Table/{leg_name}")
    if leg.IsValid():
        r = bbcache.ComputeWorldBound(leg).ComputeAlignedRange()
        print(leg_name, "min:", r.GetMin(), "max:", r.GetMax())
    else:
        print(leg_name, "NOT FOUND")

print("\n=== Layer stack / composition ===")
for layer in stage.GetLayerStack():
    print(" layer:", layer.identifier)

print("\n=== Root layer prim specs at /World/Table (to check where it's authored) ===")
root_layer = stage.GetRootLayer()
table_spec = root_layer.GetPrimAtPath("/World/Table")
print("Table spec in root layer exists:", table_spec is not None)
if table_spec:
    print("Table spec typeName:", table_spec.typeName)
    print("Table spec children:", table_spec.nameChildren.keys())
