"""Generates a procedural diffuse/roughness/normal texture set for the demo
tabletops (/World/Table/TableTop and /World/Table_02/TableTop share this one
material), matching this art direction:

  Color: mostly warm gray, taupe, and light beige, with muted charcoal-gray
  areas. Overall low saturation.
  Pattern: broad horizontal bands and streaks. Irregular and cloud-like
  rather than geometric, with darker and lighter layers running across the
  surface.
  Texture: visually soft, brushed, and slightly mottled -- washed concrete /
  suede / lightly weathered stone. Blur makes the texture read smooth rather
  than sharply detailed.
  Gradient: subtle tonal transitions from pale gray-beige to deeper gray.
  Upper and central areas contain alternating light/dark bands; the lower
  portion gradually becomes darker and more uniform.
  Edges: mostly soft, feathered transitions; a couple of horizontal lines
  near the bottom are slightly more defined but stay blurred/diffuse.

Mesh-UV convention (matches apply_table_material.py): st.t (V) = (y-minY)/
(maxY-minY), so V=1 sits at the tabletop's +Y edge and V=0 at its -Y edge.
Per the existing wall/floor scripts on this machine, UsdUVTexture samples
image row 0 (top of PNG) at V=1 and row RES-1 (bottom of PNG) at V=0 -- so
"row 0 / top of this image" = physical +Y edge = the "upper" region in the
art direction, and "row RES-1 / bottom of this image" = physical -Y edge =
the "lower portion".

Run with plain Isaac Sim python (PIL/numpy only, no kit bootstrap needed):
    C:\\Isaac-Sim\\python.bat usd-composer-stages\\gen_table_textures.py
"""
import os

import numpy as np
from PIL import Image, ImageFilter

rng = np.random.default_rng(20260814100)

RES = 2048
OUT_DIR = (
    r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop\source"
    r"\sim_to_real_so101\demo\tex\table"
)
os.makedirs(OUT_DIR, exist_ok=True)


def fbm_grid(res, freq_v, freq_u, octaves, seed):
    """Isotropic-ish fractal noise (fine mottle / roughness / bump grain
    only -- NOT used for the band pattern, since interpolating a genuinely
    2D random grid always produces round blobs no matter how few columns it
    has)."""
    local_rng = np.random.default_rng(seed)
    out = np.zeros((res, res), dtype=np.float32)
    amp = 1.0
    total_amp = 0.0
    fv, fu = freq_v, freq_u
    for _o in range(octaves):
        grid = local_rng.random((max(fv, 1), max(fu, 1))).astype(np.float32)
        img = Image.fromarray((grid * 255).astype(np.uint8), mode="L")
        img = img.resize((res, res), Image.BICUBIC)
        out += (np.asarray(img, dtype=np.float32) / 255.0) * amp
        total_amp += amp
        amp *= 0.5
        fv = int(fv * 1.8) + 1
        fu = int(fu * 1.8) + 1
    return out / total_amp


def band_noise_layer(res, num_bands, h_control_points, seed, row_smooth=2):
    """One horizontal-band layer: num_bands 1D noise profiles (rows), each
    built from h_control_points random values upsampled across the FULL
    width -- so every band gets its own irregular, wavy horizontal
    silhouette instead of a straight edge. Adjacent rows are then smoothed
    together (row_smooth passes of a 3-tap vertical blur on the small grid,
    before upsampling) so band boundaries stay continuous curves rather than
    pinching to zero at points where two independently-random rows disagree
    -- that pinching is what turns "bands" into round blobs. The rows are
    finally blended only *vertically* (source width already equals res, so
    resizing to (res, res) leaves horizontal detail untouched and just
    interpolates between adjacent bands) -- this is what produces elongated
    horizontal streaks rather than blobs."""
    local_rng = np.random.default_rng(seed)
    rows = local_rng.random((num_bands, h_control_points)).astype(np.float32)
    for _ in range(row_smooth):
        padded = np.pad(rows, ((1, 1), (0, 0)), mode="edge")
        rows = (padded[:-2] + 2 * padded[1:-1] + padded[2:]) / 4.0
    row_img = Image.fromarray((rows * 255).astype(np.uint8), mode="L")
    row_img = row_img.resize((res, num_bands), Image.BICUBIC)  # upsample horizontally only
    full_img = row_img.resize((res, res), Image.BICUBIC)  # blend bands vertically only
    return np.asarray(full_img, dtype=np.float32) / 255.0


def fbm_bands(res, octaves, base_bands, base_hpoints, seed):
    """Multi-octave stack of band_noise_layer, coarse bands dominating with
    progressively thinner (more-numerous) bands layered on top. Band count
    grows fast between octaves but horizontal control points grow only
    slightly, so even the finest octave stays wide/streaky (many thin rows,
    each with a broad, smooth horizontal wave) instead of degenerating into
    square, blob-like cells."""
    out = np.zeros((res, res), dtype=np.float32)
    amp = 1.0
    total_amp = 0.0
    nb, hp = base_bands, base_hpoints
    for o in range(octaves):
        out += band_noise_layer(res, nb, hp, seed + o * 97) * amp
        total_amp += amp
        amp *= 0.55
        nb = int(nb * 2.1) + 1
        hp = min(int(hp * 1.1) + 1, 7)
    return out / total_amp


def smoothstep(edge0, edge1, x):
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


u = np.linspace(0.0, 1.0, RES, dtype=np.float32)
uu, vv = np.meshgrid(u, u)  # uu = horizontal (s); vv = row-based (0 = top of image)

# ---------------------------------------------------------------------------
# DIFFUSE
# ---------------------------------------------------------------------------
# Broad horizontal bands: a handful of wide, wavy-edged bands stacked and
# blended vertically -- reads as distinct darker/lighter layers running
# across the surface.
bands = fbm_bands(RES, octaves=3, base_bands=5, base_hpoints=3, seed=1)

# Cloud-like streaks: more numerous, thinner bands layered on top, each
# still broad/smooth horizontally -- irregular streaky variation within/
# between the broad bands, not a repeat of the same shapes.
streaks = fbm_bands(RES, octaves=3, base_bands=16, base_hpoints=4, seed=2)

# Fine mottling for the "brushed / suede / weathered stone" micro-variation.
mottle = fbm_grid(RES, freq_v=40, freq_u=40, octaves=5, seed=3)

# Charcoal-patch mask: sparse, soft-edged darker horizontal bands layered
# over the base.
charcoal_mask_raw = fbm_bands(RES, octaves=2, base_bands=7, base_hpoints=3, seed=4)
charcoal_mask = smoothstep(0.60, 0.82, charcoal_mask_raw)  # only the higher patches carry charcoal

# Banding amplitude fades out toward the bottom (lower portion -> darker, uniform).
band_amplitude = 1.0 - smoothstep(0.62, 0.95, vv)

# Overall vertical darkening: pale gray-beige near the top, deeper gray by the bottom.
darken = smoothstep(0.0, 1.0, vv)

LIGHT = np.array([203, 196, 183], dtype=np.float32)     # pale gray-beige
WARM_MID = np.array([160, 150, 135], dtype=np.float32)  # warm gray/taupe
DEEP_GRAY = np.array([104, 101, 97], dtype=np.float32)  # deeper gray (bottom base)
CHARCOAL = np.array([80, 78, 76], dtype=np.float32)     # muted charcoal-gray patches

# base tone: LIGHT at top fading to DEEP_GRAY at bottom
color = LIGHT[None, None, :] * (1 - darken[..., None]) + DEEP_GRAY[None, None, :] * darken[..., None]

# banding: alternating light/dark horizontal layers, pushing tone toward
# WARM_MID (or away from it) -- strongest in the upper/central area, fading
# out toward the bottom as the surface becomes darker and more uniform
band_signed = (bands - 0.5) * 2.0  # [-1, 1]
band_shift = band_signed[..., None] * (WARM_MID[None, None, :] - color) * 1.4 * band_amplitude[..., None]
color = color + band_shift

# streaks: finer irregular cloud-like tonal drift within/between the bands
streak_signed = (streaks - 0.5) * 2.0
color = color + streak_signed[..., None] * 16.0 * (0.35 + 0.65 * band_amplitude[..., None])

# charcoal patches, muted (not full-strength) and low-saturation
color = color * (1 - charcoal_mask[..., None] * 0.75) + CHARCOAL[None, None, :] * (charcoal_mask[..., None] * 0.75)

# fine brushed/mottled micro-variation
color += (mottle[..., None] - 0.5) * 9.0

# a couple of horizontal lines near the bottom, slightly more defined but
# still soft/feathered (not sharp)
for line_v, strength in [(0.90, 16.0), (0.965, 12.0)]:
    dist = np.abs(vv - line_v)
    line_mask = np.exp(-(dist ** 2) / (2 * 0.006 ** 2))
    color -= line_mask[..., None] * strength

color = np.clip(color, 0, 255).astype(np.uint8)
diffuse_img = Image.fromarray(color, mode="RGB")
diffuse_img = diffuse_img.filter(ImageFilter.GaussianBlur(radius=3.0))  # soft, brushed, smooth-not-sharp
diffuse_path = f"{OUT_DIR}\\table_diffuse.png"
diffuse_img.save(diffuse_path)
print("Saved", diffuse_path)

# ---------------------------------------------------------------------------
# ROUGHNESS  (matte-satin brushed/stone finish; charcoal patches a touch rougher)
# ---------------------------------------------------------------------------
rough_noise = fbm_grid(RES, freq_v=20, freq_u=20, octaves=5, seed=5)
roughness = 0.58 + (rough_noise - 0.5) * 0.14  # ~0.51-0.65
roughness += charcoal_mask * 0.08
roughness = np.clip(roughness, 0.0, 1.0)
rough_img = Image.fromarray((roughness * 255).astype(np.uint8), mode="L")
rough_img = rough_img.filter(ImageFilter.GaussianBlur(radius=1.5))
rough_path = f"{OUT_DIR}\\table_roughness.png"
rough_img.save(rough_path)
print("Saved", rough_path)

# ---------------------------------------------------------------------------
# NORMAL  (very shallow brushed micro-bump; blur keeps it smooth, not sharp)
# ---------------------------------------------------------------------------
bump = fbm_grid(RES, freq_v=48, freq_u=48, octaves=6, seed=6)
bump_img = Image.fromarray((bump * 255).astype(np.uint8), mode="L")
bump_img = bump_img.filter(ImageFilter.GaussianBlur(radius=2.2))  # heavier blur -> reads smooth
bump_arr = np.asarray(bump_img, dtype=np.float32) / 255.0

gy, gx = np.gradient(bump_arr)
strength = 1.1  # shallow, subtle -- washed stone/suede, not gravel
nx = -gx * strength
ny = -gy * strength
nz = np.ones_like(bump_arr)
norm = np.sqrt(nx * nx + ny * ny + nz * nz)
nx, ny, nz = nx / norm, ny / norm, nz / norm

normal_rgb = np.stack(
    [(nx * 0.5 + 0.5) * 255, (ny * 0.5 + 0.5) * 255, (nz * 0.5 + 0.5) * 255],
    axis=-1,
).astype(np.uint8)
normal_img = Image.fromarray(normal_rgb, mode="RGB")
normal_path = f"{OUT_DIR}\\table_normal.png"
normal_img.save(normal_path)
print("Saved", normal_path)

print("Done.")
