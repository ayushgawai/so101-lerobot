"""Generates a procedural diffuse/roughness/normal texture set for the demo
floor, matching this art direction:

  Color: medium cool gray with subtle warm/taupe undertones and fine darker
  gray speckling.
  Texture: fine-grained, lightly pebbled/stone-like, satin finish (soft
  reflected highlights, not mirror).
  Pattern: large uniform tiles, no strong decorative motif, one narrow
  vertical grout line roughly through the image center.
  Edges: straight, sharply defined tile boundaries; thin gray-beige grout
  slightly darker than the tiles; a second dark diagonal/curved boundary
  near the far-left edge toward the bottom.

Lighting gradient is intentionally NOT baked in -- the brief states the
underlying tile color is uniform and the gradient comes from scene lighting,
which the real-to-sim.usd stage already provides (SkyLight + KeyLight).

Run with plain Isaac Sim python (PIL/numpy only, no kit bootstrap needed):
    C:\\Isaac-Sim\\python.bat usd-composer-stages\\gen_floor_textures.py
"""
import numpy as np
from PIL import Image, ImageFilter

rng = np.random.default_rng(20260813)

RES = 2048
OUT_DIR = (
    r"c:\Users\OMNI-User\Desktop\Sim-to-Real-SO-101-Workshop\source"
    r"\sim_to_real_so101\demo\tex\floor"
)


def fbm_noise(res, octaves, base_freq=4, seed_offset=0):
    """Cheap fractal noise: sum of upsampled random grids at increasing freq."""
    local_rng = np.random.default_rng(20260813 + seed_offset)
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
uu, vv = np.meshgrid(u, u)  # uu = horizontal (s), vv = vertical (t)

base_gray = fbm_noise(RES, octaves=4, base_freq=3, seed_offset=1)  # smooth low-freq blotching
warm_mask = fbm_noise(RES, octaves=3, base_freq=2, seed_offset=2)  # broad warm/cool patches

# cool gray base with subtle warm/taupe undertone drift
cool = np.array([141, 141, 146], dtype=np.float32)
warm = np.array([150, 142, 130], dtype=np.float32)
undertone_t = 0.35 + 0.3 * warm_mask  # keep mostly cool, patches lean taupe
color = cool[None, None, :] * (1 - undertone_t[..., None]) + warm[None, None, :] * undertone_t[..., None]

# gentle overall value variation from the low-freq blotch noise
value_var = (base_gray - 0.5) * 10.0
color = color + value_var[..., None]

# fine darker speckling
speck_noise = rng.random((RES, RES)).astype(np.float32)
fine_detail = fbm_noise(RES, octaves=2, base_freq=64, seed_offset=3)
speckle_mask = (speck_noise > 0.90) & (fine_detail < 0.55)
speckle_depth = rng.uniform(18, 42, size=(RES, RES)).astype(np.float32)
color[speckle_mask] -= speckle_depth[speckle_mask, None]

# fine pebble micro-variation (very light, keeps it from looking flat/plastic)
pebble = fbm_noise(RES, octaves=5, base_freq=96, seed_offset=4)
color += (pebble[..., None] - 0.5) * 6.0

color = np.clip(color, 0, 255)

# --- grout: one narrow vertical line roughly through the center ---
grout_center = 0.52
grout_half_width = 0.006
grout_dist = np.abs(uu - grout_center)
grout_core = grout_dist < grout_half_width
grout_edge = (grout_dist >= grout_half_width) & (grout_dist < grout_half_width * 2.2)

grout_color = np.array([118, 112, 100], dtype=np.float32)  # gray-beige, darker than tile
color[grout_core] = grout_color
# soft anti-aliased falloff at grout edge so it isn't a hard aliased pixel line
falloff = 1.0 - (grout_dist[grout_edge] - grout_half_width) / (grout_half_width * 1.2)
color[grout_edge] = color[grout_edge] * (1 - falloff[:, None]) + grout_color[None, :] * falloff[:, None]

# --- second boundary: dark diagonal/curved edge near far-left, toward bottom ---
# curve parametrized so it sweeps from the left edge (~78% down) toward
# bottom-left (~15% across), giving the described diagonal/curved seam.
curve_x = 0.02 + 0.16 * ((vv - 0.72) / 0.28).clip(0, 1) ** 1.6
curve_dist = np.abs(uu - curve_x)
in_lower_band = vv > 0.70
curve_core = in_lower_band & (curve_dist < 0.0035)
curve_edge = in_lower_band & (curve_dist >= 0.0035) & (curve_dist < 0.010)

curve_color = np.array([100, 96, 90], dtype=np.float32)
color[curve_core] = curve_color
curve_falloff = 1.0 - (curve_dist[curve_edge] - 0.0035) / (0.010 - 0.0035)
color[curve_edge] = color[curve_edge] * (1 - curve_falloff[:, None]) + curve_color[None, :] * curve_falloff[:, None]

color = np.clip(color, 0, 255).astype(np.uint8)
diffuse_img = Image.fromarray(color, mode="RGB")
diffuse_img = diffuse_img.filter(ImageFilter.GaussianBlur(radius=0.6))  # settle fine speckle into a satin grain
diffuse_path = f"{OUT_DIR}\\floor_diffuse.png"
diffuse_img.save(diffuse_path)
print("Saved", diffuse_path)

# ---------------------------------------------------------------------------
# ROUGHNESS  (satin: mid roughness with gentle variation; grout duller/rougher)
# ---------------------------------------------------------------------------
rough_noise = fbm_noise(RES, octaves=5, base_freq=48, seed_offset=5)
roughness = 0.42 + (rough_noise - 0.5) * 0.16  # ~0.34-0.50 satin range
roughness[grout_core | grout_edge] = np.clip(roughness[grout_core | grout_edge] + 0.22, 0, 1)
roughness[curve_core | curve_edge] = np.clip(roughness[curve_core | curve_edge] + 0.15, 0, 1)
roughness = np.clip(roughness, 0.0, 1.0)
rough_img = Image.fromarray((roughness * 255).astype(np.uint8), mode="L")
rough_path = f"{OUT_DIR}\\floor_roughness.png"
rough_img.save(rough_path)
print("Saved", rough_path)

# ---------------------------------------------------------------------------
# NORMAL  (fine pebbled micro-bump, tangent space)
# ---------------------------------------------------------------------------
bump = fbm_noise(RES, octaves=6, base_freq=160, seed_offset=6)
bump_img = Image.fromarray((bump * 255).astype(np.uint8), mode="L")
bump_img = bump_img.filter(ImageFilter.GaussianBlur(radius=0.5))
bump_arr = np.asarray(bump_img, dtype=np.float32) / 255.0

# Sobel-style gradient -> tangent-space normal, kept very low amplitude (subtle pebble, not gravel)
gy, gx = np.gradient(bump_arr)
strength = 3.0
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
normal_path = f"{OUT_DIR}\\floor_normal.png"
normal_img.save(normal_path)
print("Saved", normal_path)

print("Done.")
