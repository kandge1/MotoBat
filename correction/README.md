# correction: task 1.12, the cell-placement loop in numpy

Plan §4B.7 and §9.9. No ROS, no arm. Kept out of `defectgen/src` on purpose: the known-cases
fingerprint covers the detector, and the loop is not part of it.

```bash
python -m correction.run [--setup fixed|arm]          # ~15 s, < 200 MB -> data/correction/<setup>/
python -m pytest -p no:launch_testing -p no:launch_ros -q correction/tests
```

Inputs: tolerances from `defectgen/.../configs/detect.yaml`, gauge noise and bias from the newest
1.9 %GRR result (`data/dataset/validation/grr.<setup>.json`, else the v1 one), the plant from
`loop.yaml` (all assumed, marked A). Rerun after v3's `validate grr` to refresh.

## The model, per axis (x mm, y mm, yaw deg), k = cell index across a shift

```
x_k     = g·u_k + d_k + w_k          where cell k lands. d: offset + wear ramp + a gripper change
e_k     = x_k + b_m + v_k            what the station measures. b_m: gauge bias, v: gauge noise (1.9)
u_{k+1} = −Kp·e_k − Ki·Σ_{j≤k} e_j   the offset added to the next cell
```

## What it shows, and why (derivations in `loop.py`)

1. **The plan's band 0 < K < 2/g belongs to the I form,** u_{k+1} = u_k − K·e_k. There
   e_{k+1} = (1 − gK)·e_k: one pole at 1 − gK. Below 1 it settles smoothly, at 1 in one cell
   (deadbeat), between 1 and 2 it alternates and settles, past 2 it diverges. The P form written
   literally (u_{k+1} = −K·e_k, recomputed from scratch) is stable only for gK < 1 and leaves
   d/(1 + gK) of any constant offset. The general PI loop has poles z² + (a_p + a_i − 1)z − a_p = 0
   (Jury: |a_p| < 1, a_i > 0, a_i + 2a_p < 2).
2. **Deadbeat is the wrong gain on a noisy line.** With white placement noise σ_w and gauge noise
   σ_v, the I loop's steady spread is σ² = σ_w² + K(σ_w² + σ_v²)/(2 − K). At K = 1 that is
   2σ_w² + σ_v²: correcting every cell fully on one reading adds the noise back in (Deming's
   funnel). The best K trades drift tracking (mean lag r/K for a ramp r per cell) against this.
3. **A gauge bias goes straight into the product.** Integral action zeroes the *measured* error, so
   placement settles at −b_m. 1.9's −25 µm bias in cell X becomes +25 µm on every cell. Fix the
   gauge (calibrate it), not the loop.
4. **Terminal height is a decision, not a correction.** Moving the next cell cannot change this
   cell's height. Rework before the carrier (so before the weld) when the measured adjacent
   difference is past tol − guard band; the guard band trades escapes to the weld for false rework.

Each claim is a test in `tests/test_loop.py`, including closed form vs Monte Carlo for the spread.

## Limits of this first pass

- The plant (σ_w, offset, ramp, gripper change) is assumed. The open-loop rate depends on it
  directly; the shape of the K-sweep and the deadbeat penalty do not.
- 1.9 measures cell X only; y is assumed to read like x, and yaw is derived from two terminals
  115 mm apart.
- 1.9's cell-X AV is the line scan's sampling phase per pallet load, so in reality it is
  common to the cells of one module, not white per cell. Modelled as white here.
- 1.9 is measured at station 6, after all twelve cells. The loop needs a reading after each
  cell (module_spec step 3); the same sensor class is assumed.
