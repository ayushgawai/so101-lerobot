"""Renders shaded 3D preview images of the realistic joined tables directly
from the authored USD mesh geometry (actual points/faces/materials), so the
result can be visually confirmed against the requested design description.
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from pxr import Usd, UsdGeom, UsdShade, Gf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
ASSETS_USD_DIR = os.path.join(REPO_ROOT, "source", "sim_to_real_so101", "assets", "usd")
STAGE_PATH = os.path.join(ASSETS_USD_DIR, "room-and-table_joined.usd")

stage = Usd.Stage.Open(STAGE_PATH)
xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())


def bound_color(prim):
    bapi = UsdShade.MaterialBindingAPI(prim)
    mat, _ = bapi.ComputeBoundMaterial()
    if not mat or not mat.GetPrim().IsValid():
        return (0.7, 0.7, 0.7)
    shader = UsdShade.Shader(stage.GetPrimAtPath(str(mat.GetPath()) + "/Shader"))
    color_input = shader.GetInput("diffuseColor")
    c = color_input.Get() if color_input else None
    return tuple(c) if c else (0.7, 0.7, 0.7)


def collect_faces(root_path, light_dir=np.array([0.4, -0.5, 0.8])):
    light_dir = light_dir / np.linalg.norm(light_dir)
    faces = []  # list of (verts(list of xyz), shaded_color)
    root = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idxs = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts:
            continue
        world = xform_cache.GetLocalToWorldTransform(prim)
        wpts = [world.Transform(Gf.Vec3d(*p)) for p in pts]
        base_color = np.array(bound_color(prim))
        p = 0
        for c in counts:
            face_idx = idxs[p:p + c]
            p += c
            verts = [np.array([wpts[k][0], wpts[k][1], wpts[k][2]]) for k in face_idx]
            if len(verts) < 3:
                continue
            v1 = verts[1] - verts[0]
            v2 = verts[2] - verts[0]
            n = np.cross(v1, v2)
            norm = np.linalg.norm(n)
            if norm < 1e-12:
                continue
            n = n / norm
            intensity = 0.35 + 0.65 * max(0.0, float(np.dot(n, light_dir)))
            color = tuple(np.clip(base_color * intensity, 0, 1))
            faces.append((verts, color))
    return faces


def render(faces, out_path, elev, azim, title, xlim=None, ylim=None, zlim=None):
    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")
    polys = [f[0] for f in faces]
    colors = [f[1] for f in faces]
    coll = Poly3DCollection(polys, facecolor=colors, edgecolor=(0, 0, 0, 0.15), linewidths=0.3)
    ax.add_collection3d(coll)

    all_pts = np.vstack([np.array(v) for f in faces for v in [f[0]]])
    all_pts = np.vstack([p for f in faces for p in f[0]])
    if xlim is None:
        cx, cy, cz = all_pts.mean(axis=0)
        span = (all_pts.max(axis=0) - all_pts.min(axis=0)).max() / 2 * 1.1
        xlim = (cx - span, cx + span)
        ylim = (cy - span, cy + span)
        zlim = (max(0, cz - span), cz + span)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_zlim(*zlim)
    try:
        ax.set_box_aspect((xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]))
    except Exception:
        pass
    ax.set_xlabel("X (long axis)")
    ax.set_ylabel("Y (join/short axis)")
    ax.set_zlabel("Z (up)")
    ax.view_init(elev=elev, azim=azim)
    ax.set_title(title)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {out_path}")
    return xlim, ylim, zlim


faces_all = collect_faces("/World/Table") + collect_faces("/World/Table_02")
xlim, ylim, zlim = render(faces_all, os.path.join(SCRIPT_DIR, "joined_tables_iso.png"),
                           elev=22, azim=-50,
                           title="room-and-table_joined.usd -- isometric view (both tables)")

render(faces_all, os.path.join(SCRIPT_DIR, "joined_tables_front.png"),
       elev=8, azim=-90,
       title="Front view -- long edges joined, seam running the full long dimension",
       xlim=xlim, ylim=ylim, zlim=zlim)

render(faces_all, os.path.join(SCRIPT_DIR, "joined_tables_underside.png"),
       elev=-25, azim=-60,
       title="Underside view -- support columns, V-feet, and casters",
       xlim=xlim, ylim=ylim, zlim=zlim)

# close-up on the seam / inner support area to check for collisions visually
seam_faces = collect_faces("/World/Table/Base/SupportRight") + collect_faces("/World/Table_02/Base/SupportLeft")
render(seam_faces, os.path.join(SCRIPT_DIR, "joined_tables_seam_closeup.png"),
       elev=15, azim=-60,
       title="Close-up: inner support assemblies nearest the seam (no collision)")
