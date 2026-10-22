"""Build the photographed tabletop demo from its Markdown specification.

Requires usd-core and numpy. Run from any directory; source assets stay intact.
"""

import argparse
import json
import math
import os
from pathlib import Path
import re
import sys

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, UsdUtils, Vt


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "source"))
from sim_to_real_so101.utils.paper_cube import (configure_rubber_cube, configure_paper_gripper, configure_soft_scene, is_rubber_cube)
DEFAULT_SPEC = ROOT / "docs/screenshot-scene-technical-specification.md"
BOX_FACES = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4],
             [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]


def transform(prim, position, scale=None):
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*position))
    if scale is not None:
        x.AddScaleOp().Set(Gf.Vec3f(*scale))


def mesh(stage, path, points, faces, material=None):
    shape = UsdGeom.Mesh.Define(stage, path)
    points = np.asarray(points, dtype=np.float32)
    shape.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
    shape.CreateFaceVertexCountsAttr([len(f) for f in faces])
    shape.CreateFaceVertexIndicesAttr([int(i) for f in faces for i in f])
    shape.CreateExtentAttr([Gf.Vec3f(*points.min(axis=0).tolist()), Gf.Vec3f(*points.max(axis=0).tolist())])
    shape.CreateSubdivisionSchemeAttr("none")
    shape.CreateDoubleSidedAttr(True)
    if material:
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material)
    return shape


def box(stage, path, center, size, material=None, collision=True):
    points = np.array([[-.5, -.5, -.5], [.5, -.5, -.5], [.5, .5, -.5], [-.5, .5, -.5],
                       [-.5, -.5, .5], [.5, -.5, .5], [.5, .5, .5], [-.5, .5, .5]])
    shape = mesh(stage, path, points, BOX_FACES, material)
    transform(shape, center, size)
    uv = UsdGeom.PrimvarsAPI(shape).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, "faceVarying")
    uv.Set([(0, 0), (1, 0), (1, 1), (0, 1)] * 6)
    if collision:
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    return shape


def material(stage, name, color, roughness=.6, metallic=0, opacity=1, ior=1.5):
    mat = UsdShade.Material.Define(stage, "/World/Looks/" + name)
    shader = UsdShade.Shader.Define(stage, str(mat.GetPath()) + "/Preview")
    shader.CreateIdAttr("UsdPreviewSurface")
    for key, val, dtype in [
        ("diffuseColor", Gf.Vec3f(*color), Sdf.ValueTypeNames.Color3f),
        ("roughness", roughness, Sdf.ValueTypeNames.Float),
        ("metallic", metallic, Sdf.ValueTypeNames.Float),
        ("opacity", opacity, Sdf.ValueTypeNames.Float),
        ("ior", ior, Sdf.ValueTypeNames.Float),
    ]:
        shader.CreateInput(key, dtype).Set(val)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def rounded_profile(width, depth, radius, segments=12):
    points = []
    for cx, cy, start in [(width/2-radius, depth/2-radius, 0),
                           (-width/2+radius, depth/2-radius, 90),
                           (-width/2+radius, -depth/2+radius, 180),
                           (width/2-radius, -depth/2+radius, 270)]:
        for i in range(segments):
            a = math.radians(start + 90*i/(segments-1))
            points.append((cx + radius*math.cos(a), cy + radius*math.sin(a)))
    return np.array(points)


def wood_surface(stage, table_path, cfg, surface_z):
    """Code-native vertex color grain; no photograph or generated bitmap needed."""
    nx, ny = cfg["grid"]
    points, colors, faces = [], [], []
    for i in range(nx+1):
        x = -.6 + 1.2*i/nx
        ymax = .35 if abs(x) <= .565 else .315 + math.sqrt(max(0, .035**2-(abs(x)-.565)**2))
        for j in range(ny+1):
            y = -ymax + 2*ymax*j/ny
            phase = y*760 + 3.5*math.sin(x*5+y*7) + .6*math.sin(x*21-y*10)
            grain = .047*math.sin(phase) + .023*math.sin(phase*2.07+x*3)
            grain += .032*math.sin(y*91 + .7*math.sin(x*7))
            grain += .011*math.sin(x*390+y*1470)
            points.append((x, y, surface_z + 0.0000001))
            colors.append(np.clip(np.array(cfg["color"])+grain, 0, 1))
    for i in range(nx):
        for j in range(ny):
            k = i*(ny+1)+j
            faces.append([k, k+ny+1, k+ny+2, k+1])
    mat = material(stage, "WoodGrain", cfg["color"], cfg["roughness"])
    shader = UsdShade.Shader(stage.GetPrimAtPath(str(mat.GetPath()) + "/Preview"))
    reader = UsdShade.Shader.Define(stage, str(mat.GetPath()) + "/GrainColor")
    reader.CreateIdAttr("UsdPrimvarReader_float3")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("woodColor")
    reader.CreateInput("fallback", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(*cfg["color"]))
    shader.GetInput("diffuseColor").ConnectToSource(reader.ConnectableAPI(), "result")
    top = mesh(stage, table_path + "/WoodGrainSurface", points, faces, mat)
    UsdGeom.PrimvarsAPI(top).CreatePrimvar("woodColor", Sdf.ValueTypeNames.Color3fArray, "vertex").Set(
        Vt.Vec3fArray.FromNumpy(np.asarray(colors, dtype=np.float32)))


def cube(stage, cfg):
    root = UsdGeom.Xform.Define(stage, "/World/Cube")
    transform(root, cfg["center"])
    root.GetPrim().SetCustomDataByKey("physicalModel", "Rigid approximation; real object is squishy PVC or PP")
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(cfg["mass_kg"])
    mat = material(stage, "WhitePolymer", cfg["color"], cfg["roughness"])
    physmat = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    physmat.CreateStaticFrictionAttr(cfg["static_friction"])
    physmat.CreateDynamicFrictionAttr(cfg["dynamic_friction"])
    physmat.CreateRestitutionAttr(cfg["restitution"])
    points, faces = [], []
    half, radius, n = cfg["side"]/2, cfg["corner_radius"], 32
    # Right-handed face bases; every face shares the same bounded rounded box.
    bases = [((1, 0, 0), (0, 1, 0), (0, 0, 1)),
             ((-1, 0, 0), (0, -1, 0), (0, 0, 1)),
             ((0, 1, 0), (-1, 0, 0), (0, 0, 1)),
             ((0, -1, 0), (1, 0, 0), (0, 0, 1)),
             ((0, 0, 1), (1, 0, 0), (0, 1, 0)),
             ((0, 0, -1), (-1, 0, 0), (0, 1, 0))]
    for index, (normal, u, v) in enumerate(bases):
        start = len(points)
        normal, u, v = map(np.array, (normal, u, v))
        for i in range(n+1):
            for j in range(n+1):
                a, b = -1+2*i/n, -1+2*j/n
                p = half*(normal + a*u + b*v)
                q = np.clip(p, -half+radius, half-radius)
                outward = p-q
                outward = outward/np.linalg.norm(outward)
                p = q + radius*outward
                fade = (1-a*a)*(1-b*b)
                crease = math.sin(a*21+b*13+index)*math.sin(b*18-a*3)
                p += normal*cfg["wrinkle_amplitude"]*fade*crease
                points.append(p)
        for i in range(n):
            for j in range(n):
                k = start+i*(n+1)+j
                faces.append([k, k+n+1, k+n+2, k+1])
    points = np.array(points)
    # The supplied 55 mm dimension is an actual bound, including subtle creases.
    points = (points-points.min(axis=0))/np.ptp(points, axis=0)*cfg["side"]-half
    shape = mesh(stage, "/World/Cube/Visual", points, faces, mat)
    # Derive the shading normals from the creased surface, not an ideal box.
    quads = np.array(faces)
    face_normals = np.cross(points[quads[:, 1]]-points[quads[:, 0]], points[quads[:, 3]]-points[quads[:, 0]])
    normals = np.zeros_like(points)
    for column in range(4):
        np.add.at(normals, quads[:, column], face_normals)
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-12)
    shape.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(normals, dtype=np.float32)))
    shape.SetNormalsInterpolation("vertex")
    collider = box(stage, "/World/Cube/Collision", (0, 0, 0), [cfg["side"]]*3, mat)
    UsdGeom.Imageable(collider).CreateVisibilityAttr("invisible")
    UsdGeom.Imageable(collider).CreatePurposeAttr("guide")
    UsdShade.MaterialBindingAPI.Apply(collider.GetPrim()).Bind(mat, materialPurpose="physics")
    tape = material(stage, "WhiteTape", (.91, .91, .88), .30)
    # Thin raised strips indicate the folded/taped top visible in the close-up.
    for name, center, size in [
        ("TapeTopA", (0, -.010, half-.00015), (.048, .011, .00020)),
        ("TapeTopB", (0, .013, half-.00013), (.048, .009, .00020)),
    ]:
        box(stage, "/World/Cube/"+name, center, size, tape, collision=False)


def container(stage, cfg):
    root = UsdGeom.Xform.Define(stage, "/World/Container")
    transform(root, cfg["origin"])
    root.GetPrim().SetCustomDataByKey("dimensionsAreEstimates", True)
    mat = material(stage, "FrostedPlastic", cfg["color"], cfg["roughness"],
                   opacity=cfg["opacity"], ior=cfg["ior"])
    # Kit uses the MDL transmission surface; other USD viewers use Preview.
    # OmniGlass is a renderer template here, not a claim that the object is glass.
    transmission = UsdShade.Shader.Define(stage, str(mat.GetPath())+"/Transmission")
    transmission.SetSourceAsset(Sdf.AssetPath("OmniGlass.mdl"), "mdl")
    transmission.SetSourceAssetSubIdentifier("OmniGlass", "mdl")
    transmission.CreateInput("glass_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*cfg["color"]))
    transmission.CreateInput("glass_ior", Sdf.ValueTypeNames.Float).Set(cfg["ior"])
    transmission.CreateInput("frosting_roughness", Sdf.ValueTypeNames.Float).Set(cfg["roughness"])
    transmission.CreateInput("thin_walled", Sdf.ValueTypeNames.Bool).Set(True)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(transmission.ConnectableAPI(), "out")
    bottom, top = cfg["bottom_size"], cfg["top_size"]
    radius, t, h, r = cfg["corner_radius"], cfg["wall_thickness"], cfg["height"], cfg["rim_radius"]
    outer_bottom = rounded_profile(*bottom, radius)
    outer_top = rounded_profile(*top, radius)
    inner_bottom = rounded_profile(bottom[0]-2*t, bottom[1]-2*t, radius-t)
    inner_top = rounded_profile(top[0]-2*t, top[1]-2*t, radius-t)
    count = len(outer_top)
    # Outward normals of the rounded perimeter, including straight sections.
    tangent = np.roll(outer_top, -1, axis=0)-np.roll(outer_top, 1, axis=0)
    normals = np.column_stack((tangent[:, 1], -tangent[:, 0]))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    rings = [np.column_stack((outer_bottom, np.zeros(count))),
             np.column_stack((outer_top, np.full(count, h-r)))]
    for angle in np.linspace(0, math.pi, 13)[1:]:
        xy = outer_top - r*(1-math.cos(angle))*normals
        rings.append(np.column_stack((xy, np.full(count, h-r+r*math.sin(angle)))))
    rings.extend([np.column_stack((inner_top, np.full(count, h-2*r))),
                  np.column_stack((inner_bottom, np.full(count, cfg["bottom_thickness"])))])
    faces = [list(range(count-1, -1, -1))]
    for ring in range(len(rings)-1):
        for i in range(count):
            j = (i+1) % count
            faces.append([ring*count+i, ring*count+j, (ring+1)*count+j, (ring+1)*count+i])
    faces.append(list(range((len(rings)-1)*count, len(rings)*count)))
    shell = mesh(stage, "/World/Container/Shell", np.concatenate(rings), faces, mat)
    # The thicker molded lip scatters more light and looks milky white in the photo.
    rim_mat = material(stage, "PlasticRim", (.87, .89, .88), .28, opacity=.65)
    rim_faces = [1+ring*count+i for ring in range(1, 13) for i in range(count)]
    subset = UsdGeom.Subset.CreateGeomSubset(shell, "RolledLip", UsdGeom.Tokens.face,
                                            rim_faces, "materialBind")
    UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(rim_mat)
    # Segment-by-segment convex wedges retain the rounded rectangular cavity.
    # These remain static: there is no rigid-body API on the container root.
    UsdGeom.Scope.Define(stage, "/World/Container/Collisions")
    base_ring = np.column_stack((outer_bottom, np.zeros(count)))
    base_top = np.column_stack((outer_bottom, np.full(count, cfg["bottom_thickness"])))
    base_faces = [list(range(count-1, -1, -1)), list(range(count, 2*count))]
    base_faces += [[i, (i+1) % count, (i+1) % count+count, i+count] for i in range(count)]
    colliders = [mesh(stage, "/World/Container/Collisions/Bottom", np.concatenate([base_ring, base_top]), base_faces)]
    for i in range(count):
        j = (i+1) % count
        points = [(outer_bottom[i, 0], outer_bottom[i, 1], cfg["bottom_thickness"]),
                  (outer_bottom[j, 0], outer_bottom[j, 1], cfg["bottom_thickness"]),
                  (inner_bottom[j, 0], inner_bottom[j, 1], cfg["bottom_thickness"]),
                  (inner_bottom[i, 0], inner_bottom[i, 1], cfg["bottom_thickness"]),
                  (outer_top[i, 0], outer_top[i, 1], h),
                  (outer_top[j, 0], outer_top[j, 1], h),
                  (inner_top[j, 0], inner_top[j, 1], h),
                  (inner_top[i, 0], inner_top[i, 1], h)]
        colliders.append(mesh(stage, f"/World/Container/Collisions/Wall_{i:02d}", points, BOX_FACES))
    for shape in colliders:
        UsdGeom.Imageable(shape).CreateVisibilityAttr("invisible")
        UsdGeom.Imageable(shape).CreatePurposeAttr("guide")
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr("convexHull")
    label(stage, cfg)


def label(stage, cfg):
    paper = material(stage, "LabelPaper", (.91, .89, .94), .85)
    ink = material(stage, "BlackInk", (.012, .014, .013), .9)
    red = material(stage, "RedInk", (.46, .018, .012), .8)
    z, side = cfg["bottom_thickness"]+.00012, cfg["label_side"]
    box(stage, "/World/Container/Label/Paper", (0, 0, z), (side, side, .00015), paper, collision=False)
    z += .0001
    # Vector B lying in the XY plane, upright from the front (-Y) camera.
    points = [(-.015, -.023, z), (-.010, -.023, z), (-.010, .023, z), (-.015, .023, z)]
    mesh(stage, "/World/Container/Label/BStem", points, [[0, 1, 2, 3]], ink)
    for name, cy in [("BUpper", .0115), ("BLower", -.0115)]:
        points, faces = [], []
        for angle in np.linspace(-math.pi/2, math.pi/2, 40):
            for rx, ry in [(.023, .0115), (.0185, .007)]:
                points.append((-.011+rx*math.cos(angle), cy+ry*math.sin(angle), z))
        for i in range(39):
            faces.append([2*i, 2*i+2, 2*i+3, 2*i+1])
        mesh(stage, "/World/Container/Label/"+name, points, faces, ink)
    points = [(0+.003*math.cos(a), .003*math.sin(a), z+.00002) for a in np.linspace(0, 2*math.pi, 32, endpoint=False)]
    mesh(stage, "/World/Container/Label/RedDot", points, [list(range(32))], red)


def relative_assets(layer, directory):
    def visit(path):
        if not path.IsPropertyPath():
            return
        attr = layer.GetAttributeAtPath(path)
        if attr and attr.typeName == Sdf.ValueTypeNames.Asset and isinstance(attr.default, Sdf.AssetPath):
            value = attr.default.path
            if os.path.isabs(value):
                attr.default = Sdf.AssetPath(os.path.relpath(value, directory).replace(os.sep, "/"))
    layer.Traverse(Sdf.Path.absoluteRootPath, visit)


def add_robot(stage, cfg, spec, output):
    path = "/World/SO_ARM101_USD"
    robot = UsdGeom.Xform.Define(stage, path)
    asset = ROOT / spec["robot_asset"]
    robot.GetPrim().GetReferences().AddReference(os.path.relpath(asset, output.parent).replace(os.sep, "/"))
    transform(robot, cfg["origin"])
    # Author identity explicitly so the asset's native root orientation is not used.
    UsdGeom.Xformable(robot).AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1))
    source = Usd.Stage.Open(str(ROOT/spec["reference_stage"]))
    clamps = path+"/base/DeskClamps"
    Sdf.CreatePrimInLayer(stage.GetRootLayer(), path+"/base")
    if not Sdf.CopySpec(source.GetRootLayer(), clamps, stage.GetRootLayer(), clamps):
        raise RuntimeError("Could not copy source desk clamps")
    # Universal Preview surfaces accompany the native MDL materials for portability.
    for name, color, rough, metal in [
        ("material_a_3d_printed", cfg["printed_color"], .48, 0),
        ("material_sts3215", (.035, .035, .035), .45, 0),
        ("Aluminum_Anodized_Black", (.025, .027, .03), .42, .65),
        ("Silver", (.63, .64, .66), .32, 1),
    ]:
        matpath = path+"/Looks/"+name
        mat = UsdShade.Material(stage.GetPrimAtPath(matpath))
        preview = material(stage, "Robot_"+name, color, rough, metal)
        mat.CreateSurfaceOutput().ConnectToSource(preview.GetSurfaceOutput().GetConnectedSource()[0], "surface")
    printed_shader = stage.GetPrimAtPath(path+"/Looks/material_a_3d_printed/Shader")
    printed_shader.GetAttribute("inputs:diffuse_color_constant").Set(Gf.Vec3f(*cfg["printed_color"]))
    # Do not require a robot wrist-camera render product for this scene.
    for p in list(Usd.PrimRange(robot.GetPrim())):
        if p.IsA(UsdGeom.Camera):
            stage.OverridePrim(p.GetPath()).SetActive(False)
    # Match the existing task's camera_ego mount without requiring Isaac Lab.
    # Parent to the moving rigid link, not the static scene-camera scope.
    camera_cfg = spec["wrist_camera"]
    camera = UsdGeom.Camera.Define(stage, path + "/gripper/WristCamera")
    camera.GetPrim().SetActive(True)
    camera.GetPrim().SetDisplayName("Wrist Camera")
    camera.GetPrim().SetDocumentation("Gripper-mounted viewpoint; approximate optics, not calibrated real-camera intrinsics.")
    transform(camera, camera_cfg["position"])
    camera.AddRotateXYZOp().Set(Gf.Vec3f(*camera_cfg["rotation_xyz_deg"]))
    camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)
    camera.CreateFocalLengthAttr(camera_cfg["focal_length"])
    camera.CreateHorizontalApertureAttr(camera_cfg["horizontal_aperture"])
    camera.CreateVerticalApertureAttr(camera_cfg["vertical_aperture"])
    camera.CreateClippingRangeAttr(Gf.Vec2f(*camera_cfg["clipping_range"]))
    camera.CreateFocusDistanceAttr(.2)
    camera.CreateFStopAttr(0)  # Keep the manipulation view sharp at all distances.


def add_cameras_lights(stage, spec):
    lights = spec["lights"]
    dome = UsdLux.DomeLight.Define(stage, "/World/SkyLight")
    dome.CreateIntensityAttr(lights["dome_intensity"])
    dome.CreateEnableColorTemperatureAttr(True)
    dome.CreateColorTemperatureAttr(lights["temperature_kelvin"])
    key = UsdLux.RectLight.Define(stage, "/World/KeyLight")
    key.CreateIntensityAttr(lights["key_intensity"])
    key.CreateWidthAttr(lights["key_size"][0])
    key.CreateHeightAttr(lights["key_size"][1])
    transform(key, lights["key_position"])
    for name, cfg in spec["cameras"].items():
        cam = UsdGeom.Camera.Define(stage, "/World/Cameras/"+name)
        matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*cfg["eye"]), Gf.Vec3d(*cfg["target"]), Gf.Vec3d(0, 0, 1)).GetInverse()
        UsdGeom.Xformable(cam).AddTransformOp().Set(matrix)
        cam.CreateFocalLengthAttr(cfg["focal_length"])
        cam.CreateHorizontalApertureAttr(36)
        cam.CreateVerticalApertureAttr(20.25)
        cam.CreateClippingRangeAttr(Gf.Vec2f(.01, 100))
        cam.CreateFocusDistanceAttr((Gf.Vec3d(*cfg["eye"])-Gf.Vec3d(*cfg["target"])).GetLength())
    stage.GetRootLayer().customLayerData = {
        "cameraSettings": {"boundCamera": "/World/Cameras/DemoCamera"},
        "sourceSpecification": "../../../docs/screenshot-scene-technical-specification.md",
    }


def validate(output, spec):
    stage = Usd.Stage.Open(str(output))
    checks = {}
    def check(name, value):
        checks[name] = bool(value)
        if not value:
            raise RuntimeError("Validation failed: "+name)
    check("meters_z_up_world", UsdGeom.GetStageMetersPerUnit(stage) == 1 and
          UsdGeom.GetStageUpAxis(stage) == "Z" and str(stage.GetDefaultPrim().GetPath()) == "/World")
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    def bound(path):
        r = bounds.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
        return np.array(r.GetMin()), np.array(r.GetMax())
    floor_min, floor_max = bound("/World/Floor")
    check("ground_surface_z_zero", abs(floor_max[2]) < 1e-6)
    check("four_colliding_walls", all(stage.GetPrimAtPath("/World/Walls/"+n).HasAPI(UsdPhysics.CollisionAPI) for n in spec["walls"]))
    for index, offset in enumerate(spec["tables"]):
        name = "/World/Table" if index == 0 else "/World/Table_02"
        lo, hi = bound(name)
        check("table_%d_bounds" % index, np.allclose(lo, np.array([-.6, -.35, 0])+offset, atol=1e-6) and
              np.allclose(hi, np.array([.6, .35, .75])+offset, atol=1e-6))
    lo, hi = bound("/World/Cube/Visual")
    check("cube_55mm", np.allclose(hi-lo, spec["cube"]["side"], atol=1e-6))
    check("cube_rests_on_table", abs(lo[2]-spec["table_surface_z"]) < 1e-6)
    cube_root, cube_mat = stage.GetPrimAtPath("/World/Cube"), stage.GetPrimAtPath("/World/Looks/WhitePolymer")
    check("cube_hard_rubber_rigid", is_rubber_cube(cube_root) and cube_root.HasAPI(UsdPhysics.RigidBodyAPI) and
          np.isclose(UsdPhysics.MassAPI(cube_root).GetMassAttr().Get(), spec["cube"]["mass_kg"]))
    check("cube_convex_collider", UsdPhysics.MeshCollisionAPI(stage.GetPrimAtPath("/World/Cube/Collision")).GetApproximationAttr().Get() == "convexHull")
    for attr, key in (("compliantContactStiffness", "contact_stiffness_n_per_m"), ("compliantContactDamping", "contact_damping_ns_per_m")):
        check("cube_rubber_"+key, np.isclose(cube_mat.GetAttribute("physxMaterial:"+attr).Get(), spec["cube"][key]))
    for prop, key in (("staticFriction", "static_friction"), ("dynamicFriction", "dynamic_friction")):
        check("cube_paper_"+key, np.isclose(cube_mat.GetAttribute("physics:"+prop).Get(), spec["cube"][key]))
    for table in ("Table", "Table_02"):
        check(table+"_solid_top_collider", UsdPhysics.MeshCollisionAPI(stage.GetPrimAtPath("/World/"+table+"/TableTop")).GetApproximationAttr().Get() == "convexHull")
    lo, hi = bound("/World/Container/Shell")
    check("container_rests_on_table", abs(lo[2]-spec["table_surface_z"]) < 1e-6)
    check("container_dimensions", np.allclose(hi-lo, [*spec["container"]["top_size"], spec["container"]["height"]], atol=1e-6))
    if "photo_measurements" in spec:
        measured = spec["photo_measurements"]
        cube_lo, cube_hi = bound("/World/Cube/Visual")
        bowl_lo, bowl_hi = bound("/World/Container/Shell")
        base_lo, _ = bound("/World/SO_ARM101_USD/base/visuals/base_so101_v2/mesh")
        check("photo_base_edge_to_cube_190mm", abs(base_lo[1]-cube_hi[1]-measured["base_edge_to_cube_edge_m"]) < 1e-6)
        check("photo_cube_to_bowl_edge_110mm", abs(cube_lo[0]-bowl_hi[0]-measured["cube_edge_to_bowl_rim_m"]) < 1e-6)
        check("bowl_left_of_cube", bowl_hi[0] < cube_lo[0])
        check("cube_bowl_center_spacing_210mm", abs(np.linalg.norm(
            (bowl_lo[:2]+bowl_hi[:2]-cube_lo[:2]-cube_hi[:2])/2)-.210) < 1e-6)
        check("photo_cube_bowl_forward_alignment", abs((cube_lo[1]+cube_hi[1]-bowl_lo[1]-bowl_hi[1])/2) < 1e-6)
    shell = UsdGeom.Mesh(stage.GetPrimAtPath("/World/Container/Shell"))
    pts = np.array(shell.GetPointsAttr().Get())
    ids, counts = shell.GetFaceVertexIndicesAttr().Get(), shell.GetFaceVertexCountsAttr().Get()
    cursor, sealed_top, edges = 0, False, {}
    for n in counts:
        face_ids = list(ids[cursor:cursor+n])
        face = pts[face_ids]
        if np.ptp(face[:, 2]) < 1e-7 and face[:, 2].min() > spec["container"]["height"]-.001:
            sealed_top |= bool(np.ptp(face[:, 0]) > .1 and np.ptp(face[:, 1]) > .1)
        for a, b in zip(face_ids, face_ids[1:]+face_ids[:1]):
            edge = tuple(sorted((a, b)))
            edges[edge] = edges.get(edge, 0)+1
        cursor += n
    check("container_open_top", not sealed_top)
    check("container_shell_manifold", all(n == 2 for n in edges.values()))
    colliders = stage.GetPrimAtPath("/World/Container/Collisions").GetChildren()
    check("container_compound_colliders", len(colliders) == 49 and all(p.HasAPI(UsdPhysics.CollisionAPI) for p in colliders))
    check("container_collider_cavity", all("Wall_" not in p.GetName() or
          np.linalg.norm(np.array(UsdGeom.Mesh(p).GetPointsAttr().Get())[:, :2].mean(axis=0)) > .06 for p in colliders))
    robot = stage.GetPrimAtPath("/World/SO_ARM101_USD")
    robot_prims = list(Usd.PrimRange(robot, Usd.TraverseInstanceProxies()))
    native = Usd.Stage.Open(str(ROOT/spec["robot_asset"]))
    native_root = native.GetDefaultPrim()
    native_meshes = {str(p.GetPath().MakeRelativePath(native_root.GetPath())) for p in
                     Usd.PrimRange(native_root, Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh)}
    composed_meshes = {str(p.GetPath().MakeRelativePath(robot.GetPath())) for p in robot_prims if p.IsA(UsdGeom.Mesh)}
    check("native_robot_meshes", native_meshes == composed_meshes and len(native_meshes) > 0)
    check("six_robot_revolute_joints", sum(p.GetTypeName() == "PhysicsRevoluteJoint" for p in robot_prims) == 6)
    for link in ("gripper", "jaw"):
        shapes = [p for p in Usd.PrimRange(stage.GetPrimAtPath(f"/World/SO_ARM101_USD/{link}/collisions")) if p.IsA(UsdGeom.Mesh)]
        check(link+"_actual_mesh_colliders", bool(shapes) and all(p.HasAPI(UsdPhysics.CollisionAPI) for p in shapes))
        check(link+"_paper_contact_binding", all(str(UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial("physics")[0].GetPath()) == "/World/Looks/GripperPaperContact" for p in shapes))
    check("jaw_squeeze_effort_cap", np.isclose(stage.GetPrimAtPath("/World/SO_ARM101_USD/joints/Jaw").GetAttribute("drive:angular:physics:maxForce").Get(), spec["gripper_contact"]["effort_limit_nm"]))
    wrist_camera = UsdGeom.Camera(stage.GetPrimAtPath("/World/SO_ARM101_USD/gripper/WristCamera"))
    check("wrist_camera_active", bool(wrist_camera) and wrist_camera.GetPrim().IsActive())
    camera_xform = UsdGeom.Xformable(wrist_camera)
    check("wrist_camera_follows_gripper", not camera_xform.GetResetXformStack() and
          wrist_camera.GetPrim().GetParent().HasAPI(UsdPhysics.RigidBodyAPI))
    check("wrist_camera_mount", np.allclose(camera_xform.GetLocalTransformation().ExtractTranslation(),
          spec["wrist_camera"]["position"], atol=1e-8))
    mesh_count = 0
    for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if p.IsA(UsdGeom.Mesh):
            mesh_count += 1
            shape = UsdGeom.Mesh(p)
            valid, reason = UsdGeom.Mesh.ValidateTopology(shape.GetFaceVertexIndicesAttr().Get(),
                shape.GetFaceVertexCountsAttr().Get(), len(shape.GetPointsAttr().Get()))
            check("mesh_topology:"+str(p.GetPath()), valid)
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(output))
    # Omniverse's built-in MDL library is resolved by Kit rather than core USD.
    missing_local = [str(p) for p in unresolved if not str(p).lower().endswith(".mdl")]
    check("local_assets_resolve", not missing_local)
    check("no_composition_errors", not stage.GetCompositionErrors())
    report = {"status": "passed", "stage": output.name, "mesh_count": mesh_count,
              "checks_passed": len(checks), "checks": checks,
              "unresolved_renderer_modules": [str(p) for p in unresolved],
              "dependency_layers": [os.path.relpath(l.realPath, output.parent).replace(os.sep, "/") for l in layers],
              "limitations": ["This report validates authored structure; dynamic grasp test is separate", "Effective homogeneous soft PVC/paper material parameters are estimates, not measured laminate properties",
                              "Photo ruler readings are approximate; base-edge to asset correspondence is assumed",
                              "Container width is photo-estimated; height, bottom footprint and material parameters are demo estimates"]}
    report_path = output.with_suffix(".validation.json")
    report_path.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(f"PASS: {len(checks)} checks; {mesh_count} meshes. Report: {report_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    args = parser.parse_args()
    text = args.spec.read_text(encoding="utf-8")
    spec = json.loads(re.search(r"```json\s*\n(.*?)\n```", text, re.S).group(1))
    output = ROOT/spec["output"]
    # Build the environment in memory before composing the external robot asset.
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, spec["meters_per_unit"])
    UsdGeom.SetStageUpAxis(stage, spec["up_axis"])
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    stage.SetStartTimeCode(-1)
    stage.SetEndTimeCode(0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1)
    source = Usd.Stage.Open(str(ROOT/spec["table_source"]))
    # This existing environment-only layer is already flattened. Copying it
    # directly also preserves Kit-specific metadata that core USD cannot flatten.
    flat = source.GetRootLayer()
    for path in ["/World/Looks", "/World/Table", "/World/Table_02"]:
        if not Sdf.CopySpec(flat, path, stage.GetRootLayer(), path):
            raise RuntimeError("Could not copy "+path)
    for i, position in enumerate(spec["tables"]):
        path = "/World/Table" if i == 0 else "/World/Table_02"
        transform(stage.GetPrimAtPath(path), position)
        wood_surface(stage, path, spec["wood"], spec["table_surface_z"])
    looks = lambda name: UsdShade.Material(stage.GetPrimAtPath("/World/Looks/"+name))
    box(stage, "/World/Floor", spec["floor"]["center"], spec["floor"]["size"], looks("Floor"))
    for name, wall in spec["walls"].items():
        box(stage, "/World/Walls/"+name, wall["center"], wall["size"], looks("Wall"))
    mount = UsdGeom.Cylinder.Define(stage, "/World/RobotMount")
    mount.CreateRadiusAttr(.6)
    mount.CreateHeightAttr(.02)
    transform(mount, (0, 0, .01))
    mount.CreateVisibilityAttr("invisible")
    # Reference locator only: a hidden floor disc must not create an unseen obstacle.
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    cube(stage, spec["cube"])
    configure_rubber_cube(stage, spec["cube"])
    configure_soft_scene(stage)
    container(stage, spec["container"])
    # Reference paths resolve relative to the future output layer.
    stage.GetRootLayer().Export(str(output), args={"format": "usda"})
    stage = Usd.Stage.Open(str(output))
    add_robot(stage, spec["robot"], spec, output)
    configure_paper_gripper(stage, spec["gripper_contact"])
    add_cameras_lights(stage, spec)
    relative_assets(stage.GetRootLayer(), output.parent)
    stage.GetRootLayer().documentation = "Screenshot-derived lab demo. Parameters: sim2real/docs/screenshot-scene-technical-specification.md."
    stage.GetRootLayer().Save()
    print("Wrote", output)
    validate(output, spec)


if __name__ == "__main__":
    main()
