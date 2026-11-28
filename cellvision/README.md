# cellvision: the S1 overhead cameras, in numpy (task 3.2)

Two fixed 2D cameras at Station 1 (GP7 cell placing), chosen 26 Nov 2026:

| Camera | Looks at | Job | Lens | Pixel on the cell tops |
|---|---|---|---|---|
| infeed | the cell infeed pick point | locate each loose cell before the GP7 picks it | 16 mm (A) | 0.130 mm |
| stack | the pallet nest | measure each cell after it is placed: the per-cell gauge for 3.4's loop (k = cell index) | 12 mm (A) | 0.174 mm |

Both are SICK midiCam2 5 MP mono: 2448 × 2048 px, 3.45 µm pixels, Sony IMX264, global shutter,
12 bit (S: SICK product overview and the I2D605C datasheet, the colour sibling with the same
sensor; read 26 Nov 2026). Both hang 1.6 m above the floor, above the GP7's swept envelope.
No ROS here: a reviewer can run it with numpy and scipy.

```bash
python -m pytest -p no:launch_testing -p no:launch_ros -q cellvision/tests     # ~20 s
python -m cellvision.study [--n 200]          # ~15 min -> data/cellvision/study.json
```

## How a cell is measured (maths in the module docstrings)

1. **Camera** (`camera.py`): pinhole model; rendering ray-casts box scenes (can tops, terminals,
   pads, end plate, cooling plate) with jittered supersampling, a Gaussian blur, shot and read
   noise, 12-bit quantisation.
2. **Calibration** (`calib.py`): a dot-grid target lying on the measurement plane gives a
   homography (normalised DLT, Hartley & Zisserman Alg. 4.2). Checked on held-out dots.
3. **Locating** (`locate.py`): coarse pose from blob moments; edge points from gradient peaks
   along each side's normal (sampled in plane millimetres through the homography); then a
   **pixel-integrated edge-model fit** per side, mapped to the plane as H^T l; a robust
   5-parameter rectangle fit (centre, yaw, both sizes).

## What building it showed (26 Nov 2026)

- **Bilinear interpolation cannot find a subpixel edge.** Its derivative is constant across
  each pixel, so the gradient peak is a plateau and the estimate snaps to the sample grid
  (seen as identical points and a ~44 µm bias). Cubic B-splines fixed the snapping.
- **The gradient peak is biased by the edge's phase on the pixel grid ("pixel locking").** On
  exactly formed pixels it is off by up to 0.18 / 0.10 / 0.04 px at a blur of 0.3 / 0.6 / 1.0 px.
  An edge at a random angle averages this out along its length; an edge nearly parallel to the
  pixel rows (a placed cell, squared to the line) does not, and showed ~20 µm on the stack
  camera. Fitting the image formation itself (a blurred step integrated over the pixel) is
  unbiased to 1e-5 px on the same pixels. `study.py` reports both locators on every image.
- **A fixed sub-sample grid in the renderer quantises edges** (1/(2·ss) px): replaced by
  jittered stratified sampling, which estimates a pixel's area integral without bias.

## Results (27 Nov 2026, `data/cellvision/study.json`)

| | x σ | y σ | yaw σ | max |e| | failures |
|---|---|---|---|---|---|
| infeed, 200 poses, edge-model fit | 0.9 µm | 1.3 µm | 0.4 mdeg | 4.4 µm | 0 |
| infeed, same images, gradient peak | 1.9 µm | 3.4 µm | 1.8 mdeg | 18 µm | 0 |
| stack, 100 stacks, edge-model fit | **26.5 µm** | 2.2 µm | 6.6 mdeg | 72 µm | 3 |

The stack camera's x error is parallax, not the locator: every cell's top was varied ±0.3 mm (A)
and a non-telecentric lens moves a cell r·dz/(Z − dz) along the line (up to ±76 µm for the end
cells; the parallax study matches that formula to ~1 µm at all 12 positions). Against a ±0.20 mm
position tolerance (A) that is %GRR 40 % in x (AIAG: not acceptable) and 3 % in y. A per-cell
height reading at S1 (a point laser) would let the correction be applied exactly; until then the
stack camera's x should not drive the correction loop (3.4).

**Open:** 3 of the 100 stacks are not measured. On one long side the edge-model fit drifted
toward the neighbouring cell's edge (it is now bounded and refuses rather than wanders); the
gradient-peak points on those same sides were ~0.1 mm off on 2 of the 3, so no fallback is used.
Something about those stacks (tight neighbours at different heights, yawed) defeats both edge
methods; not yet found.

## Limits (the numbers are an ideal-world floor, as dataset 1 is)

Not modelled: lens-distortion residual, uneven lighting, can-edge rounding and lid print, dust,
vibration and thermal drift of the camera mounts. Parallax is modelled: a non-telecentric lens
sees a cell top dz above the calibrated plane shifted outward by r·dz/(Z − dz). The renderer
blurs the pixelated image rather than the continuous one, a model mismatch of up to ~0.03 px
(~5 µm) for the edge-model fit.
