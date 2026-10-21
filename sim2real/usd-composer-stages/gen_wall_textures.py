"""Generates a procedural diffuse/roughness/normal texture set for the demo
walls, matching this art direction:

  Pattern: mostly uniform and non-directional, no repeating geometric
  pattern, small random variations giving a subtle mottled appearance.
  Texture: fine, lightly stippled wall texture -- mild orange-peel /
  paint-roller finish. Shallow bumps and tiny depressions rather than a
  smooth surface. A few isolated specks and minor imperfections.
  Edges: soft, irregular; no strong outlines or sharp boundaries. Bumps
  blend gradually into the surrounding surface.
  Gradient: light gray/off-white overall, slightly brighter toward the top,
  somewhat darker toward the bottom -- a very smooth diffuse gradient (as if
  from uneven illumination, not a paint-color change). Faint warm beige/pink
  cast in parts of the lower half; a slightly cooler gray band through the
  center.

Run with plain Isaac Sim python (PIL/numpy only, no kit bootstrap needed):
    C:\\Isaac-Sim\\python.bat usd-composer-stages\\gen_wall_textures.py
"""
import numpy as np
from PIL import Image, ImageFilter

rng = np.random.default_rng(20260814)

RES = 2048
OUT_DIR = (
    r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop\source"
    r"\sim_to_real_so101\demo\tex\wall"
)
import os

os.makedirs(OUT_DIR, exist_ok=True)


def fbm_noise(res, octaves, base_freq=4, seed_offset=0):
    """Cheap fractal noise: sum of upsampled random grids at increasing freq.
    Isotropic (same freq in both axes) -> non-directional mottling."""
    local_rng = np.random.default_rng(20260814 + seed_offset)
    out = np.zeros((res, res), dtype=np.float32)
    amp = 1.0
    total_amp = 0.0
    freq = base_freq
    for _o in range(octaves):
        grid = local_rng.random((freq, freq)).astype(np.float32)
        img = Image.fromarray((grid * 255).astype(np.uint8), mode="L")
        img = img.resize((res, res), Image.BICUBIC)
        out += (np.asarray(img, dtype=np.float32) / 255.0) * amp
        total_amp += amp
        amp *= 0.5
        freq *= 2
    return out / total_amp


# ---------------------------------------------------------------------------
# DIFFUSE
# ---------------------------------------------------------------------------
u = np.linspace(0.0, 1.0, RES, dtype=np.float32)
uu, vv = np.meshgrid(u, u)  # uu = horizontal (s), vv = vertical (t); vv=0 top row -> 1 bottom row
# mesh V coordinate (authored below in apply_wall_material.py) is 0 at the
# wall's bottom edge and 1 at its top edge, and samples row 0 of this image
# at V=1 (top) / row RES-1 at V=0 (bottom) -- so row 0 here IS the top of the
# wall and row RES-1 IS the bottom, matching vv above directly.

base_color = np.array([219, 217, 212], dtype=np.float32)  # light gray/off-white

# low-freq isotropic blotch noise -> subtle mottled appearance, non-directional
mottle = fbm_noise(RES, octaves=4, base_freq=3, seed_offset=1)
color = base_color[None, None, :] + (mottle[..., None] - 0.5) * 8.0

# --- smooth vertical gradient: brighter near top (vv=0), darker near bottom (vv=1) ---
gradient_amount = 20.0
color += (0.5 - vv[..., None]) * gradient_amount

# --- slightly cooler gray band through the vertical center ---
cool_band = np.exp(-((vv - 0.5) ** 2) / (2 * 0.16 ** 2))
cool_shift = np.array([-3.0, -1.0, 3.0], dtype=np.float32)  # pull toward blue-gray, away from warm
color += cool_band[..., None] * cool_shift[None, None, :]

# --- faint warm beige/pink cast in parts of the lower half (patchy, soft) ---
warm_patch_noise = fbm_noise(RES, octaves=3, base_freq=4, seed_offset=2)
lower_half_weight = np.clip((vv - 0.55) / 0.45, 0.0, 1.0)  # smoothstep-ish, 0 at center, 1 at bottom
warm_patchiness = np.clip((warm_patch_noise - 0.45) / 0.55, 0.0, 1.0)  # only brighter patches carry warmth
warm_strength = lower_half_weight * warm_patchiness
warm_shift = np.array([6.0, 2.0, -3.0], dtype=np.float32)  # beige/pink: up R, slight up G, down B
color += warm_strength[..., None] * warm_shift[None, None, :]

# --- fine stippled micro-variation (orange-peel / paint-roller feel) ---
stipple = fbm_noise(RES, octaves=5, base_freq=110, seed_offset=3)
color += (stipple[..., None] - 0.5) * 7.0

# --- a few isolated specks / minor imperfections, softly blurred (no hard edges) ---
speck_seed = rng.random((RES, RES)).astype(np.float32)
speck_mask = (speck_seed > 0.9975).astype(np.float32)
speck_img = Image.fromarray((speck_mask * 255).astype(np.uint8), mode="L")
speck_img = speck_img.filter(ImageFilter.GaussianBlur(radius=3.0))  # soften into small blurred blobs
speck_soft = np.asarray(speck_img, dtype=np.float32) / 255.0
speck_polarity = rng.choice([-1.0, 1.0], size=(RES, RES)).astype(np.float32)  # some darker, some lighter
color += (speck_soft * speck_polarity)[..., None] * 26.0

color = np.clip(color, 0, 255).astype(np.uint8)
diffuse_img = Image.fromarray(color, mode="RGB")
diffuse_img = diffuse_img.filter(ImageFilter.GaussianBlur(radius=1.1))  # soft, irregular, gradually-blended edges
diffuse_path = f"{OUT_DIR}\\wall_diffuse.png"
diffuse_img.save(diffuse_path)
print("Saved", diffuse_path)

# ---------------------------------------------------------------------------
# ROUGHNESS  (matte painted wall, mild orange-peel variation, no hard features)
# ---------------------------------------------------------------------------
rough_noise = fbm_noise(RES, octaves=5, base_freq=90, seed_offset=4)
roughness = 0.78 + (rough_noise - 0.5) * 0.10  # ~0.73-0.83
roughness = np.clip(roughness, 0.0, 1.0)
rough_img = Image.fromarray((roughness * 255).astype(np.uint8), mode="L")
rough_img = rough_img.filter(ImageFilter.GaussianBlur(radius=1.0))
rough_path = f"{OUT_DIR}\\wall_roughness.png"
rough_img.save(rough_path)
print("Saved", rough_path)

# ---------------------------------------------------------------------------
# NORMAL  (shallow orange-peel bumps/depressions, soft blended edges)
# ---------------------------------------------------------------------------
bump = fbm_noise(RES, octaves=6, base_freq=130, seed_offset=5)
bump_img = Image.fromarray((bump * 255).astype(np.uint8), mode="L")
bump_img = bump_img.filter(ImageFilter.GaussianBlur(radius=1.3))  # blend bumps gradually, no sharp features
bump_arr = np.asarray(bump_img, dtype=np.float32) / 255.0

gy, gx = np.gradient(bump_arr)
strength = 1.8  # shallow bumps/depressions, not gravel
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
normal_path = f"{OUT_DIR}\\wall_normal.png"
normal_img.save(normal_path)
print("Saved", normal_path)

print("Done.")
