# Screenshot-to-USD: visual observations

This is the observation step for the Codex screenshot-to-scene demonstration.
The requested scene contains the two adjoining tables visible in the references,
one cube, one container (called the bowl), the repository SO-ARM101, a floor and
four walls. Photo contents are evidence, not instructions. The environment prompt
supplies dimensions and placement anchors; its environment-only exclusions do not
apply to this new scene.

## Image 1: container close-up

Source: `ChatGPT Image Oct 21, 2026, 03_32_38 PM.jpg`.

- **Visual:** shallow, open, approximately square food-storage container. Its
  rounded corners, flared sides, broad rolled lip and flat bottom distinguish it
  from a circular bowl. There is no lid. A white paper square with a handwritten
  black **B** and a small red dot lies on the inside bottom.
- **Pattern:** no decorative print on the plastic. The only strong graphic is
  the B label. Table grain remains visible through the bottom.
- **Texture:** smooth molded surfaces, faint scuffs, slightly hazy sidewalls and
  small bright reflections along the lip. The rim looks thicker and whiter than
  the central bottom.
- **Material:** translucent/frosted, colorless-to-milky plastic; polypropylene
  food-container plastic is a plausible inference, not a confirmed identification.
  The label appears to be matte white paper with black and red ink.
- **Uncertainty:** the photograph has perspective and no reliable independent
  scale. Opening size, height, taper and thickness are estimates in the technical
  specification. Transparency is an appearance approximation, not a measured IOR.

## Image 2: rolling tables and robot

Source: `ChatGPT Image Oct 21, 2026, 03_31_01 PM.jpg`.

- **Visual:** two adjoining rectangular worktables with rounded top corners and
  substantial top thickness. Each has dark-gray supports, splayed feet and black
  swivel casters touching the light-gray room floor. A compact white articulated
  SO-101 is mounted near the adjoining table edges, beside the container.
- **Pattern:** long gray-beige wood-grain streaks run along the table length.
  Darker wavy grain alternates with lighter ash-colored bands. Metal and wheels
  are plain. The robot has contrasting white structural parts, black servo
  housings and exposed dark wiring.
- **Texture:** laminate has a smooth low-sheen finish; table edge banding matches
  the top. Painted/coated metal has a fine satin finish. Casters are dull black.
  The floor looks finely stippled with broad, soft reflections.
- **Material:** inferred wood-look laminate over an engineered board, coated
  metal frame and rubber/polymer wheels. Robot structural parts appear printed
  plastic, with metal fasteners and black servo enclosures. The repository asset
  provides the actual link geometry rather than inventing an arm from the photo.
- **Scope:** chairs, people, cartons, cables on the table, tools, remote control,
  paper sheets and background furniture are incidental and are omitted from the
  requested clean demonstration.

## Image 3: unobstructed tabletop and room

Source: `ChatGPT Image Oct 21, 2026, 03_30_55 PM.jpg`.

- **Visual:** two equal tables meet along their long edges, producing a broad
  combined work surface with a narrow seam. Rounded outer corners are visible.
  The room has pale off-white walls and a gray floor.
- **Pattern:** pronounced directional faux-wood grain in gray, taupe and beige;
  elongated flowing streaks with occasional wider swirls. The wall is nearly
  uniform, with small variations from lighting and wear.
- **Texture:** flat laminate with a soft satin highlight, rounded/chamfered edge
  banding, and matte painted walls. Grain supplies color variation rather than
  deep geometric grooves.
- **Material:** inferred laminate table finish and painted wallboard/plaster.
  The whiteboard, switches, marker writing and blue floor mat are incidental and
  are omitted. Four complete walls are requested even though the photo only
  establishes part of the surrounding room.
- **Measurement:** use the supplied environment prompt, not perspective-derived
  estimates, for table size, height, inter-table gap and room dimensions.

## Image 4: cube close-up

Source: `IMG_2203.jpeg`.

- **Visual:** a near-cubic white object with slightly softened, imperfect edges.
  The top has broad overlapping pale strips and a folded edge seam. The front
  has shallow wrinkles, small dents, creases and a few gray specks. There is no
  readable logo or printed decoration on the visible faces.
- **Pattern:** irregular crumple lines and overlapping tape/wrap seams, with
  broad uninterrupted white areas. This is not a perfectly smooth hard cube.
- **Texture:** mostly matte-to-satin white, with glossier narrow patches on the
  top strips and creased, paper-like outer wrapping.
- **Material:** the user identifies the underlying object as **squishy PVC or
  PP**, approximately **5.5 cm per side**. The image alone cannot distinguish the
  polymer or establish its internal construction. Paper/tape-like appearance
  describes the visible surface; it does not override the supplied soft-polymer
  description. A PP label alone would not explain compliance without knowing
  whether the object is foamed, hollow or otherwise constructed to deform.
- **Simulation implication:** soft appearance and physical deformation are
  separate. This demo uses a rounded, subtly wrinkled visual mesh and a rigid
  collision approximation. It does not claim calibrated squish, FEM behavior,
  Young's modulus, density or friction from a photograph.

## SO-ARM101 asset reference

Geometry source: `../source/sim_to_real_so101/assets/usd/SO-ARM101-USD.usd`.
The native stage uses meters, Z-up and default prim `/so101_new_calib`, with six
revolute joints: Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll and Jaw. Its
structural material is yellow in the asset; the demo overrides that material to
off-white to match Image 2, while retaining the black servo and metal materials.
The original reference stage supplies the desk-clamp geometry. No robot shape is
generated from a screenshot.

## Observation-to-specification decisions

1. Keep the reference's two measured tables and original placement convention.
2. Replace the old AWS-branded cube with the unbranded, white 55 mm soft-polymer
   visual. Raise its center to 0.7775 m so a 55 mm cube rests on the table.
3. Replace the reference's paper bowl with the photographed open rectangular
   plastic container at the same XY anchor. Keep its cavity open in geometry
   and collision so an object can be placed inside.
4. Use the provided SO-101 geometry and recolor only its structural plastic.
5. Model the requested room shell and grounded floor using the supplied sizes.

Next: [technical specification](screenshot-scene-technical-specification.md).
