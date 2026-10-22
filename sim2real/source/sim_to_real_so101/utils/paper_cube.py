"""Author a paper-wrapped soft PVC cube using Isaac Sim 6 volume deformables.

Uses schema names/attributes so the scene builder also works with usd-core.
PhysX cooks tetrahedra and binds the visible skin when Isaac Sim opens the USD.
Material values describe an effective homogeneous object, not a measured PVC/paper laminate.
"""
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


def attribute(prim, name, dtype, value):
    prim.CreateAttribute(name, dtype, custom=False).Set(value)


def schemas(prim, *names):
    for name in names:
        prim.AddAppliedSchema(name)


def is_soft_cube(prim):
    applied = prim.GetMetadata("apiSchemas")
    return bool(applied and "OmniPhysicsDeformableBodyAPI" in applied.GetAddedOrExplicitItems())


def is_rubber_cube(prim):
    return prim.GetCustomDataByKey("contactModel") == "rigid_compliant"


def configure_rubber_cube(stage, cfg):
    """Hard-rubber cube: rigid body whose paper surface gives ~1 mm under the jaws.

    A FEM cube in PhysX only grips once the pads sink deep enough to squash it
    visibly (gel-like), so firm rubber is modelled with compliant contact instead.
    """
    root = stage.GetPrimAtPath("/World/Cube")
    root.SetCustomDataByKey("contactModel", "rigid_compliant")
    root.SetCustomDataByKey("physicalModel", "Rigid hard-rubber PVC core, white paper surface, compliant contact")
    root.SetCustomDataByKey("materialParametersMeasured", False)
    # A dynamic body cannot use a triangle-mesh collider.
    UsdPhysics.MeshCollisionAPI.Apply(stage.GetPrimAtPath("/World/Cube/Collision")).CreateApproximationAttr("convexHull")
    mat = stage.GetPrimAtPath("/World/Looks/WhitePolymer")
    mat.SetDisplayName("White paper over hard-rubber PVC")
    schemas(mat, "PhysxMaterialAPI")
    for name, dtype, value in (
        ("compliantContactStiffness", Sdf.ValueTypeNames.Float, cfg["contact_stiffness_n_per_m"]),
        ("compliantContactDamping", Sdf.ValueTypeNames.Float, cfg["contact_damping_ns_per_m"]),
        ("compliantContactAccelerationSpring", Sdf.ValueTypeNames.Bool, False),
        ("frictionCombineMode", Sdf.ValueTypeNames.Token, "max"),
    ):
        attribute(mat, "physxMaterial:"+name, dtype, value)


def configure_soft_cube(stage, cfg):
    root = stage.GetPrimAtPath("/World/Cube")
    root.RemoveAPI(UsdPhysics.RigidBodyAPI)
    root.RemoveAPI(UsdPhysics.MassAPI)  # Deformable mass comes from rest volume and material density.
    root.RemoveProperty("physics:mass")
    root.SetCustomDataByKey("assumedMassKg", cfg["mass_kg"])
    root.SetCustomDataByKey("physicalModel", "Volume FEM: effective soft PVC core with white paper contact surface")
    root.SetCustomDataByKey("materialParametersMeasured", False)
    schemas(root, "OmniPhysicsDeformableBodyAPI", "PhysxBaseDeformableBodyAPI",
            "PhysxAutoDeformableBodyAPI", "PhysxAutoDeformableHexahedralMeshAPI",
            "PhysxAutoDeformableMeshSimplificationAPI")
    for name, dtype, value in (
        ("physxDeformableBody:resolution", Sdf.ValueTypeNames.UInt, cfg["simulation_resolution"]),
        ("physxDeformableBody:solverPositionIterationCount", Sdf.ValueTypeNames.UInt, 64),
        ("physxDeformableBody:selfCollision", Sdf.ValueTypeNames.Bool, False),
        ("physxDeformableBody:linearDamping", Sdf.ValueTypeNames.Float, .5),
        ("physxDeformableBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float, .5),
        ("physxDeformableBody:remeshingResolution", Sdf.ValueTypeNames.UInt, 12),
        ("physxDeformableBody:targetTriangleCount", Sdf.ValueTypeNames.UInt, 600),
    ):
        attribute(root, name, dtype, value)
    root.CreateRelationship("physxDeformableBody:cookingSourceMesh", custom=False).SetTargets(["/World/Cube/Collision"])
    source = stage.GetPrimAtPath("/World/Cube/Collision")
    source.RemoveAPI(UsdPhysics.CollisionAPI)
    source.RemoveAPI(UsdPhysics.MeshCollisionAPI)
    stage.GetPrimAtPath("/World/Cube/Visual").RemoveProperty("normals")

    for name in ("SimulationMesh", "CollisionMesh"):
        tet = UsdGeom.TetMesh.Define(stage, "/World/Cube/"+name)
        tet.CreatePurposeAttr(UsdGeom.Tokens.guide)
        if name == "SimulationMesh":
            schemas(tet.GetPrim(), "OmniPhysicsVolumeDeformableSimAPI")
        else:
            UsdPhysics.CollisionAPI.Apply(tet.GetPrim())
            schemas(tet.GetPrim(), "PhysxCollisionAPI")
            attribute(tet.GetPrim(), "physxCollision:contactOffset", Sdf.ValueTypeNames.Float, .001)
            attribute(tet.GetPrim(), "physxCollision:restOffset", Sdf.ValueTypeNames.Float, 0.)

    # Bind every skin (including the paper creases and tape) to the FEM body.
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.PointBased):
            schemas(prim, "OmniPhysicsDeformablePoseAPI:default")
            attribute(prim, "deformablePose:default:omniphysics:purposes", Sdf.ValueTypeNames.TokenArray, ["bindPose"])
            points = UsdGeom.PointBased(prim).GetPointsAttr().Get()
            if points is not None:
                attribute(prim, "deformablePose:default:omniphysics:points", Sdf.ValueTypeNames.Point3fArray, points)

    mat = UsdShade.Material(stage.GetPrimAtPath("/World/Looks/WhitePolymer"))
    mat.GetPrim().SetDisplayName("White paper over soft PVC")
    mat.GetPrim().RemoveAPI(UsdPhysics.MaterialAPI)
    for name in ("physics:staticFriction", "physics:dynamicFriction", "physics:restitution"):
        mat.GetPrim().RemoveProperty(name)
    schemas(mat.GetPrim(), "OmniPhysicsDeformableMaterialAPI", "PhysxDeformableMaterialAPI")
    for name, value in (
        ("density", cfg["mass_kg"]/cfg["side"]**3),
        ("staticFriction", cfg["static_friction"]),
        ("dynamicFriction", cfg["dynamic_friction"]),
        ("youngsModulus", cfg["youngs_modulus_pa"]),
        ("poissonsRatio", cfg["poissons_ratio"]),
    ):
        attribute(mat.GetPrim(), "omniphysics:"+name, Sdf.ValueTypeNames.Float, value)
    attribute(mat.GetPrim(), "physxDeformableMaterial:elasticityDamping", Sdf.ValueTypeNames.Float, .05)
    UsdShade.MaterialBindingAPI.Apply(root).Bind(mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants, materialPurpose="physics")


def configure_paper_gripper(stage, cfg):
    mat = UsdShade.Material.Define(stage, "/World/Looks/GripperPaperContact")
    physics = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    physics.CreateStaticFrictionAttr(cfg["static_friction"])
    physics.CreateDynamicFrictionAttr(cfg["dynamic_friction"])
    physics.CreateRestitutionAttr(0.)
    schemas(mat.GetPrim(), "PhysxMaterialAPI")
    attribute(mat.GetPrim(), "physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token, "max")
    for link in ("gripper", "jaw"):
        root = stage.GetPrimAtPath(f"/World/SO_ARM101_USD/{link}/collisions")
        # The source annotates an Xform collision group. Put actual shape APIs
        # on its geometry, and make only these collision instances editable.
        while True:
            instances = [prim for prim in Usd.PrimRange(root) if prim.IsInstance()]
            if not instances:
                break
            for prim in instances:
                prim.SetInstanceable(False)
        root.RemoveAPI(UsdPhysics.CollisionAPI)
        root.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        UsdShade.MaterialBindingAPI.Apply(root).Bind(mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants, materialPurpose="physics")
        for prim in Usd.PrimRange(root):
            if prim.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(prim)
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexDecomposition")
                schemas(prim, "PhysxConvexDecompositionCollisionAPI")
                attribute(prim, "physxConvexDecompositionCollision:maxConvexHulls", Sdf.ValueTypeNames.Int, 8)
                attribute(prim, "physxConvexDecompositionCollision:hullVertexLimit", Sdf.ValueTypeNames.Int, 64)
                schemas(prim, "PhysxCollisionAPI")
                attribute(prim, "physxCollision:contactOffset", Sdf.ValueTypeNames.Float, .001)
                attribute(prim, "physxCollision:restOffset", Sdf.ValueTypeNames.Float, 0.)
    joint = stage.GetPrimAtPath("/World/SO_ARM101_USD/joints/Jaw")
    for name, value in (("stiffness", cfg["stiffness"]), ("damping", cfg["damping"]), ("maxForce", cfg["effort_limit_nm"])):
        joint.GetAttribute("drive:angular:physics:"+name).Set(value)


def configure_soft_scene(stage):
    for path in ("/World/Table/TableTop", "/World/Table_02/TableTop"):
        # Solid convex tabletops prevent FEM nodes leaking through individual
        # triangles of the thin original shell under compression.
        UsdPhysics.MeshCollisionAPI.Apply(stage.GetPrimAtPath(path)).CreateApproximationAttr("convexHull")
    scene = stage.GetPrimAtPath("/World/PhysicsScene")
    schemas(scene, "PhysxSceneAPI")
    for name, dtype, value in (
        ("enableGPUDynamics", Sdf.ValueTypeNames.Bool, True),
        ("broadphaseType", Sdf.ValueTypeNames.Token, "GPU"),
        ("solverType", Sdf.ValueTypeNames.Token, "TGS"),
        ("timeStepsPerSecond", Sdf.ValueTypeNames.UInt, 120),
        ("maxPositionIterationCount", Sdf.ValueTypeNames.UInt, 64),
    ):
        attribute(scene, "physxScene:"+name, dtype, value)


def soft_cube_world_bounds(stage):
    # Compute from live skin points, not an undeformed authored extent.
    skin = UsdGeom.Mesh(stage.GetPrimAtPath("/World/Cube/Visual"))
    matrix = UsdGeom.Xformable(skin).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    result = Gf.Range3d()
    for point in skin.GetPointsAttr().Get():
        result.UnionWith(matrix.Transform(Gf.Vec3d(point)))
    return result


def restore_soft_cube_points(stage):
    """Restore cooked bind-pose geometry while the timeline is stopped."""
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Cube")):
        if prim.IsA(UsdGeom.PointBased):
            bind = prim.GetAttribute("deformablePose:default:omniphysics:points")
            if bind and bind.Get() is not None:
                UsdGeom.PointBased(prim).GetPointsAttr().Set(bind.Get())
