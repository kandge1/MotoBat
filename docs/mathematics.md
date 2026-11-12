# MotoBat mathematics: a study guide

**What this is.** Every method behind the MotoBat resume bullets, derived from first principles,
with the project's own numbers as worked examples. One section per day, for a week. Each
section ends with questions to answer on a blank page, closed-book. When you can answer all of
them, you can defend that bullet in an interview.

**What it is not.** It doesn't cover frames, forward kinematics or IK; `project_plan.md`
§9.1–9.3 already does. It doesn't replace the code: each section names the function that
implements it, so you can read the two side by side.

Written 11 Nov 2026. Every formula here was checked against the code it names. Every number
comes from the final fixed-setup run (`data/validate_pr_fixed_final.txt`,
`data/validate_grr_fixed_final.txt`), unless the text says otherwise.

| Day | Topic | Resume bullet it backs |
|---|---|---|
| 1 | Sensor models: triangulation, noise, reflectance, dropout | 2 (ray-cast sensor models) |
| 2 | Residuals, robust noise estimates, thresholds, the D/2 and D/3 rule | 3 (golden-scan comparison) |
| 3 | Registration: ICP, why whole-module ICP failed, the housing datum | 3 (ICP claim) |
| 4 | Detection statistics: precision, recall, Wilson, Poisson bounds | 3 (775 modules, Wilson CIs) |
| 5 | Gage R&R: the ANOVA method, %GRR, ndc | 4 (50 % GRR on cell position) |
| 6 | The correction loop: stability, steady state, noise | 4 (0 < K < 2/g) |
| 7 | Review: every self-test question, closed-book | all |

Notation: σ is a standard deviation, a hat (p̂) is an estimate, E[·] is an expected value.
Units are mm unless stated otherwise.

---

## Day 1 — Sensor models

Code: `defectgen/src/defectgen/scan.py`. Parameters: `configs/sensors.yaml`, `configs/materials.yaml`.

### 1.1 Laser triangulation: how a height becomes a measurement

A laser projects a line straight down onto the part. A camera looks at that line from the side,
at the **triangulation angle** θ measured from the laser plane: 22° for the LJ-X8200, 30° for the
Ranger3.

If the surface rises by Δz, the lit spot moves Δz up the laser line. The camera sees that
movement foreshortened by its viewing angle. Seen from the camera, the spot shifts by

```
  Δs = Δz · sin θ
```

across the camera image. The sensor measures Δs (where the line falls on its pixels) and
inverts it:

```
  Δz = Δs / sin θ         so     σ_z = σ_s / sin θ
```

**Worked example: the Ranger3's σ_z** (`sensors.yaml`, tagged D). One pixel covers about 70 µm
of the part. A good peak finder locates the laser line to about 1/16 of a pixel, so
σ_s ≈ 70/16 = 4.4 µm. With θ = 30°, sin θ = 0.5:

```
  σ_z ≈ 4.4 / 0.5 = 8.75 µm  →  rounded to 10 µm in the config
```

The trade-off: a larger θ means a smaller σ_z, but the camera looks more steeply sideways, so
more surfaces hide behind walls (occlusion, §1.3).

### 1.2 Why noise grows with distance squared

The same geometry can be written with a baseline b (laser to camera) and focal length f. The
image position x of the spot relates to range z by similar triangles:

```
  z = b·f / x
```

Differentiate:

```
  dz/dx = −b·f / x² = −z² / (b·f)
```

So a fixed pixel error σ_x becomes a range error

```
  σ_z = z² / (b·f) · σ_x   ∝  z²
```

That's why every sensor's noise is scaled by `(distance / reference_distance) ** 2` in
`_line_profiles` and `snapshot` (`noise_distance_exponent: 2.0`, tagged D). Twice the
standoff means four times the noise.

### 1.3 Two kinds of occlusion

A point is measured only if **both** of these hold:
1. The laser (or projector) reaches it: no wall between the light and the point.
2. The camera sees it: no wall between the point and the camera.

The camera sits off to one side, so a tall feature can hide a lit point from the camera even
when the laser hits it. `scan.py` casts both rays for every point. This is the geometric reason
a fixed top-down sensor misses housing walls, and part of why the arm exists (day 5's coverage
numbers).

### 1.4 Reflectance: why bare copper drops out

`signal()` computes how much light returns toward the camera, relative to a white matte surface
seen square-on (= 1). It has two parts.

**Diffuse (Lambert).** A matte surface scatters light equally in all directions. The light it
receives falls off with the cosine of the incidence angle:

```
  diffuse = k_d · cos θ_l
```

Here k_d is the fraction of light reflected and θ_l is the angle between the surface normal
and the direction to the light.

**Specular (Beckmann lobe).** A metal mirrors most light around the reflection direction.
Define the **half-vector** h, the unit vector halfway between "to light" and "to camera". A
perfect mirror sends light to the camera only when h lines up with the surface normal. A rough
metal spreads that over a lobe whose width is the roughness α. The Beckmann distribution gives
the fraction of surface micro-facets that face along h:

```
  D(θ_h) = exp(−tan²θ_h / α²) / (π α² cos⁴θ_h)          θ_h = angle between normal and h
```

The specular term in the code is:

```
  specular = r₀ · D(θ_h) / (4 cos θ_l cos θ_v)
```

Here r₀ is the reflectance at normal incidence, per wavelength band. The returned signal is

```
  s = π (k_d/π + specular) cos θ_l  =  k_d cos θ_l + π · specular · cos θ_l
```

**What this predicts.** Bare Cu and Al have small k_d and small α (shiny):
- Away from the mirror angle, D(θ_h) is tiny and k_d is tiny, so s < `min_signal` (0.02). The
  point is lost: **dark dropout**.
- At the mirror angle, D(θ_h) is huge, so s > `saturation` (4.0). The camera saturates and the
  point is lost again: **glare dropout**.

Plastic (large k_d) returns a steady signal from most angles. That's the whole specular-dropout
story in two inequalities.

### 1.5 Noise from weak light, and quantisation

Fewer photons give a noisier peak. Below `noise_ref_signal` the code scales σ by
√(s_ref / s), capped at ×10. This matches shot noise, where relative noise goes as
1/√(photon count).

The output is then **quantised**: rounded to steps of `quantum_z` (0.5 µm on the LJ-X8200,
2 µm on the Ranger3). Rounding error is uniform over one step, so its σ is step/√12 (derived on
day 5, §5.6).

### 1.6 Why the parameters were frozen and hashed

`params_hash()` fingerprints both YAML files. The hash (`779e2255dc4c`) is pinned in
`tests/test_scan.py` and written into every scan. If you tune a sensor model after seeing
detection results, you can make any detector look good. Freezing it first rules that out, and
the hash proves which model produced each number.

### Day 1 self-test

1. Derive Δz = Δs / sin θ from a sketch of the laser, the camera and a step in the surface.
2. Why does noise grow as distance squared? Derive it from z = b·f/x.
3. A sensor has a 50 µm pixel footprint, 1/10-pixel peak finding and θ = 25°. What is σ_z?
   (Answer: 5 / sin 25° ≈ 11.8 µm.)
4. What is the half-vector, and why does a shiny metal return light only when it lines up with
   the normal?
5. Name the two ways a bare copper busbar loses points. Which config numbers decide each one?
6. Why was the sensor model frozen before any detection result existed?

---

## Day 2 — Residuals, robust statistics and thresholds

Code: `detect.residuals`, `coverage.counts`, `detect.capable`. Parameters: `configs/detect.yaml`
(`residual:`, `capability:`).

### 2.1 The golden capture

A scan of a real module is compared, ray by ray, with a **golden capture**: the same rays, from
the same sensor model and the same pose, cast at the clean CAD module. Ray *i* of the scan and
ray *i* of the golden capture look at the same spot. For each ray the residual is

```
  r_i = (g_i − s_i) · a_i
```

Here g_i is the golden hit point, s_i is the scanned hit point and a_i is the sensor's
measuring direction (unit vector). r > 0 means the surface came back nearer the sensor than
designed: something raised or added. r < 0 means something sunk or missing.

Comparing ray by ray means the sensor's own blind spots, shadows and dropouts cancel out. A
point the sensor can never see is missing from the golden capture too.

### 2.2 Estimating noise robustly: the MAD and 1.4826

You need σ, the noise of r, to set a threshold. A plain standard deviation is ruined by the very
defects you're hunting: one weld bead 0.3 mm high pulls it up. Instead use the **median absolute
deviation**:

```
  MAD = median( |r_i − median(r)| )
```

The median ignores up to half the data being wild, so a few defects don't move it.

To turn the MAD into a σ, ask what MAD a pure Gaussian would give. For r ~ N(0, σ²), half the
values satisfy |r| < MAD. So

```
  P(|r| < MAD) = 0.5   ⇔   P(r < MAD) = 0.75   ⇔   MAD = σ · Φ⁻¹(0.75) = 0.6745 σ
```

Here Φ is the standard normal cumulative distribution. Therefore

```
  σ̂ = MAD / 0.6745 = 1.4826 · MAD
```

That's the constant in `residuals()` and `_strip_offset()`. The code estimates σ **per
material**, so shiny copper (noisier) and matte plastic each get their own threshold.

### 2.3 The threshold, and why it's 5σ rather than 3σ

A ray is flagged when |r| > τ, with

```
  τ = max(k · σ̂_material, floor)        k = 5, floor = 5 µm
```

How often does pure noise cross the threshold? For a Gaussian:

```
  P(|r| > 3σ) = 2.7 × 10⁻³
  P(|r| > 5σ) = 5.7 × 10⁻⁷
```

A scan has millions of rays. At 3σ, a clean module would light up thousands of false rays. At
5σ, about one. On top of that, a finding needs a **connected region**: at least `min_points` (6)
neighbouring rays and `min_area` (0.02 mm²). Six independent noise exceedances landing next to
each other is effectively impossible, so clean modules give zero findings. That's what the
dataset shows: 0 findings on all 30 clean modules.

### 2.4 The rule of thumb: spacing ≤ D/2, noise ≤ D/3

`project_plan.md` §3.7.3 states it. To measure a defect of size D:

- **Sample spacing ≤ D/2.** An edge between two samples is known only to within one spacing.
  You need at least two samples across a feature to see it at all, a spatial version of the
  Nyquist idea. This gates **positions** found from outlines: `detect.capable()` refuses to
  judge a cell position against tolerance T unless the spacing is ≤ T/2.
- **Noise ≤ D/3.** Then D is at least 3σ above the noise, so a defect stands clear of it.

Both are rules of thumb (tagged A in spirit), not derived optima. Say so if asked. Coverage
(`coverage.counts`) applies both halves per ~1.5 mm patch. A patch is covered when some capture
puts points on it that pass the spacing rule and whose model σ passes the noise rule.

**Worked example.** The cell position tolerance is 0.20 mm, so the spacing must be ≤ 0.10 mm.
The Ranger3's profile pitch is exactly 0.10 mm: it just qualifies. Day 5 shows the consequence.

### Day 2 self-test

1. Why compare ray by ray against a golden capture instead of against the CAD surface?
2. Derive 1.4826 from the Gaussian. Where does 0.75 come from?
3. Why does a single weld bead ruin a plain standard deviation but not the MAD?
4. Compute P(|r| > 4σ). How many false rays would 2 million noise-only rays give?
   (Answer: 6.3 × 10⁻⁵, about 127.)
5. Why does the detector need a connected region and not just one ray over threshold?
6. State the D/2 and D/3 rule and the reason for each half. Which half gates the cell position?

---

## Day 3 — Registration

Code: `detect.icp`, `detect.housing_fit`, `detect.datum_strips`, `detect._strip_offset`,
`detect.solve_datum`, `detect.joint_datum`. Background: `project_plan.md` §4B.4 "As built",
§4B.6 "A strip that moves with the defect".

Before comparing rays, the scan must be put in the golden capture's frame: the pallet never
places the module exactly where it should be. Finding that correction is **registration**.

### 3.1 Point-to-plane ICP, derived

ICP (iterative closest point) repeats two steps:
1. For each scan point p_i, find the nearest golden point q_i, with the surface normal n_i there.
2. Find the rigid motion (R, t) that best moves the p_i onto those surfaces.

**Point-to-plane** measures each error along the normal, so a point may slide along a flat
surface without penalty:

```
  r_i = n_iᵀ (R p_i + t − q_i)
  minimise  Σ r_i²
```

R makes this nonlinear. Linearise for a small rotation. Any small rotation is R ≈ I + [ω]×,
where ω = (ω_x, ω_y, ω_z) holds the small angles in radians and [ω]× p = ω × p. Then

```
  n · (ω × p) = ω · (p × n)          (scalar triple product: cyclic)

  r_i ≈ n_iᵀ(p_i − q_i)  +  (p_i × n_i)ᵀ ω  +  n_iᵀ t
```

That is linear in the six unknowns x = (ω, t). Stack one row per point:

```
  a_i = [ (p_i × n_i)ᵀ , n_iᵀ ]      b_i = −n_iᵀ (p_i − q_i)

  minimise ‖A x − b‖²   →   (AᵀA) x = Aᵀ b        (the normal equations)
```

Solve, apply the motion, re-find the nearest points and repeat until it stops moving. Open3D's
`TransformationEstimationPointToPlane` does exactly this.

**Tukey weighting (second pass).** Plain least squares lets a defect pull the fit. The second
pass re-weights each point at every iteration (iteratively reweighted least squares) with the
Tukey biweight:

```
  w(r) = (1 − (r/k)²)²   for |r| < k,     0 otherwise        k = 0.05 mm
```

A point more than 50 µm off gets zero weight. Defects become outliers, not signal.

### 3.2 Why whole-module ICP failed: two reasons, both from the equations

**Reason 1: the dominant surface drags the fit.** Take a toy 1-D case. N_a points sit where they
should (offset 0). N_b points sit on a part lifted by δ. Least squares picks the shift t that
minimises

```
  N_a · t²  +  N_b · (t − δ)²
  d/dt = 0   →   t = N_b · δ / (N_a + N_b)
```

The fit moves by the lifted part's **share of the points** times its lift. On this module the
carrier and busbars are a big share of what a top-down sensor sees. A carrier seated 0.3 mm
high dragged the whole capture by 0.3 mm (§4B.4), which hid the defect and made every terminal
read 0.3 mm low. Tukey doesn't save you: a robust estimator follows the **majority**, and here
the moved part was the majority.

**Reason 2: from above, X, Y and yaw are barely constrained.** Look at the rows a_i when the
sensor looks straight down. Most points lie on flat tops, so n_i ≈ ẑ = (0, 0, 1):

```
  n_iᵀ t = t_z                 → only t_z appears; t_x, t_y have zero coefficient
  p_i × ẑ = (p_y, −p_x, 0)     → only ω_x, ω_y (roll, pitch) appear; ω_z (yaw) has none
```

So in AᵀA the t_x, t_y and ω_z entries come only from the few points on sloped or vertical
faces (edges). Those directions have **small eigenvalues**. Least squares amplifies noise along
a direction in proportion to 1/eigenvalue, so those three unknowns come out badly wrong: a
0.15 mm fixture shift came back 0.10 mm off. This is the "ill-conditioned normal equations" idea
from linear algebra (Nirali M1 Unit VI: eigenvalues).

**The fix, as built (`housing_fit`).** Split the six unknowns by what constrains them:
- **Z, roll and pitch** from point-to-plane ICP on the **housing only**: the end and side plates
  don't move when a cell or busbar is out of place. The fitted yaw is thrown away (`_in_plane`).
- **X, Y and yaw** from the **outlines of the housing strips** (§3.3), which are edges, exactly
  what the flat tops lack.

### 3.3 X, Y and yaw from the housing strips

Each strip (the top of an end plate or a side plate) is seen as a band of points. Its centreline
offset between scan and golden gives one equation. A small rotation θ about z followed by a
translation (t_x, t_y) moves a point (x, y) to

```
  x' ≈ x − θ y + t_x
  y' ≈ y + θ x + t_y          (cos θ ≈ 1, sin θ ≈ θ)
```

So:

```
  end strip at (x, y):    measured offset_x = t_x − θ y       row [1, 0, −y]
  side strip at (x, y):   measured offset_y = t_y + θ x       row [0, 1,  x]
  long edge, length L:    measured slope dyaw = θ             row [0, 0,  L],  rhs L·dyaw
```

The last row is multiplied by L so all rows are in mm, comparable in the least-squares sum.
`solve_datum` stacks the rows and solves for (t_x, t_y, θ) by least squares. For a whole pallet
load this is done **once** (`joint_datum`), from the best view of each strip. Arm views that see
no whole strip inherit it.

### 3.4 The robust line through each strip, and the fault it fixed

`_strip_offset` cuts each strip into 2 mm bins along its length. In each bin the centreline is
halfway between the two edges seen. Then it fits a straight line to (golden − scan) centreline
offsets d(u) = a + b·u, where u is the position along the strip:
- a is the offset at the strip's reference point; b is its slope (the yaw).
- Bins more than 3σ̂ from the line (σ̂ = 1.4826 · MAD of the residuals, day 2) are dropped and
  the line is refitted, up to three times.

**Why it's needed: the datum fault (9 Nov).** The side plates are 1.5 mm sheet. A cell shifted
toward a side plate bows it outward locally (`settle.py`). The old method drew one rectangle
round the whole strip, so the bow moved its centreline, and the datum "corrected" away part of
the cell's own shift. The cell then read short: the reading-error slope went from +0.018 to
−0.081, meaning ~8–11 % of every shift was lost, and 33 % through a part view. With bins and a
robust line, the bow affects a minority of bins, which get rejected, and the datum holds. A
failing test caught it: 0.20 mm read as 0.14 mm, and now reads 0.20 mm.

**Interview version:** "A datum must not move with the defect. Mine did, because the sheet I
registered to bends when a cell pushes on it. I found it from a reading-error slope, and fixed
it with a robust per-bin fit that ignores a local bow."

### Day 3 self-test

1. Write the point-to-plane residual and linearise it for a small rotation. Where does p × n
   come from?
2. Derive t = N_b δ / (N_a + N_b). Use it to explain the 0.3 mm carrier result.
3. Why can't Tukey weighting fix reason 1?
4. Show from the rows a_i that a top-down view constrains t_z, roll and pitch but not t_x,
   t_y or yaw. What does that do to AᵀA?
5. Write the small-angle equations for an end strip and a side strip. Why is the yaw row
   multiplied by L?
6. Explain the datum fault and its fix in three sentences.

---

## Day 4 — Detection statistics

Code: `validate.wilson`, `validate.poisson_upper`, `validate.pr`. Output:
`data/validate_pr_fixed_final.txt`.

### 4.1 Precision and recall

For one defect class:

```
  TP = defects found      FN = defects missed      FP = findings with no defect behind them

  Recall    = TP / (TP + FN)     "of the real defects, how many did I catch?"
  Precision = TP / (TP + FP)     "of my alarms, how many were real?"
```

On a production line a missed defect ships a bad module (recall matters). A false alarm scraps
or reworks a good one (precision, and the false-reject rate, matter).

MotoBat-specific rules (`validate.py` docstring):
- Recall counts only defects **in view**, where the setup looks by design. A defect where the
  setup never looks is `not_covered`, counted separately (MIL-HDBK-1823A practice).
- A label **below tolerance** is not a defect. Flagging it is a false reject.
- `partial` (some targets of a case found) is not a hit, but the module was still rejected (NOK).

### 4.2 Why an interval at all

The detector found 24 of 25 out-of-spec cell shifts in one cell of the table. Is the recall
96 %? That's the estimate p̂. With only 25 tries the true rate could easily be 85 % with a
lucky run. An interval says what range of true rates is consistent with what you saw.

### 4.3 Why not the textbook interval

The usual (Wald) interval is

```
  p̂ ± z √( p̂(1 − p̂) / n )          z = 1.96 for 95 %
```

At 25 of 25, p̂ = 1, so p̂(1 − p̂) = 0 and the interval is [100 %, 100 %]: perfect certainty from
25 trials, which is nonsense. Near 0 % or 100 % it also spills outside [0, 1]. Most cells in
this dataset sit at or near 100 %, exactly where Wald fails.

### 4.4 The Wilson interval, derived

Wald plugs p̂ into the standard error. Wilson instead asks: **for which true p would the
observed p̂ not be surprising?** It keeps the true p in the standard error:

```
  |p̂ − p| ≤ z √( p(1 − p) / n )
```

Square both sides:

```
  (p̂ − p)² ≤ (z²/n) · p(1 − p)
```

Expand, and collect powers of p:

```
  p² − 2p̂p + p̂²  ≤  (z²/n) p − (z²/n) p²

  p² (1 + z²/n)  −  p (2p̂ + z²/n)  +  p̂²  ≤  0
```

That's a quadratic in p, opening upward, so it's ≤ 0 between its two roots. The quadratic
formula gives the roots:

```
        p̂ + z²/(2n)  ±  z √( p̂(1 − p̂)/n + z²/(4n²) )
  p  =  ───────────────────────────────────────────────
                       1 + z²/n
```

Those two roots are the interval. Compare with `validate.wilson`: `c` is the centre,
`h` the half-width, `d` the denominator. Note:
- The centre is pulled from p̂ toward 1/2. With few trials, extreme estimates are shrunk.
- The z²/(4n²) term keeps the width above zero at p̂ = 0 or 1.
- Both roots always lie in [0, 1].

**Worked example: 24 of 25** (p̂ = 0.96, n = 25, z² = 3.8416):

```
  z²/n = 0.15366        denominator d = 1.15366
  centre = (0.96 + 0.07683) / 1.15366 = 0.8987
  p̂(1 − p̂)/n = 0.001536      z²/(4n²) = 0.001537      sum = 0.003073
  half-width = 1.96 × √0.003073 / 1.15366 = 1.96 × 0.05543 / 1.15366 = 0.0942

  interval = [0.805, 0.993]   →  the table's "96% [80–99]"
```

**25 of 25:** [0.867, 1.000], the table's "100% [87–100]". With 25 trials and no misses,
you can only claim the true recall is above about 87 %.

**Exercise:** the resume's 94 % is 363 rejected of 387 out-of-spec defects in view. Compute its
Wilson interval. (Answer: about [0.909, 0.958].)

### 4.5 False alarms: a Poisson bound, and the rule of three

False alarms are rare events counted over many modules, so they're modelled as Poisson with rate
λ per module. Seeing **zero** in n modules has probability

```
  P(0 | λ) = e^{−λn}
```

The 95 % upper bound is the λ at which seeing zero becomes a 5 % event:

```
  e^{−λn} = 0.05   →   λ = −ln(0.05) / n = 2.996 / n  ≈  3/n
```

That's the **rule of three**. **Worked example:** 0 false alarms in 775 modules gives
λ ≤ 3/775 = 0.0039 per module, the table's "rate ≤ 0.0039". So "0 false alarms" means "at most
about 4 per 1000 modules, at 95 % confidence", not "never".

For k > 0 events the exact bound uses the chi-square distribution:

```
  λ_upper = χ²_{0.95}(2k + 2) / (2n)
```

The arm's partial run had 8 in 252: χ²_{0.95}(18) = 28.87, so λ ≤ 28.87 / 504 = 0.057 per
module. With k = 0, χ²_{0.95}(2) = 5.99 = 2 × 2.996, which gives the rule of three back.

### Day 4 self-test

1. Define precision and recall. Which one does a missed defect hurt, and which does a false
   alarm hurt?
2. Why does recall here count only defects in view?
3. Show the Wald interval failing at 25 of 25.
4. Derive the Wilson interval from |p̂ − p| ≤ z√(p(1−p)/n). Do the algebra on paper.
5. Compute Wilson for 24 of 25 by hand.
6. Derive the rule of three. What does "0 in 775" let you claim?

---

## Day 5 — Gage R&R (measurement system analysis)

Code: `validate.anova_grr`, `validate.bias_linearity`, `validate.grr`. Output:
`data/validate_grr_fixed_final.txt`. Standard: AIAG *Measurement Systems Analysis*, 4th ed.

### 5.1 The question it answers

A measurement has noise. If the noise is large **compared with the tolerance**, the gauge can't
tell a good part from a bad one near the limit, whatever its resolution. Gage R&R splits the
measurement spread into:
- **Repeatability (EV, "equipment variation"):** same part, same setup, measured again.
- **Reproducibility (AV, "appraiser variation"):** same part, different setup. Classically a
  different operator. Here, a different **pallet load**: the module is lifted out and placed
  again, so the fixture error and the sensor's sampling phase change.

### 5.2 The study design

A **crossed** study: p = 10 parts × o = 3 loads × r = 2 trials. Every part is measured in every
load, twice. The parts were deliberately spread over ±1.5 × tolerance. The reading of part i,
load j, trial k is modelled as

```
  x_ijk = μ + P_i + O_j + (PO)_ij + ε_ijk
```

All terms are random, independent, zero mean, with variances σ²_P (true part-to-part),
σ²_O (load), σ²_PO (part × load interaction: a load that treats some parts differently from
others) and σ² (repeatability).

### 5.3 Sums of squares

Write x̄_i for part i's mean over all its readings, x̄_j for load j's mean, x̄_ij for the mean of
the r trials of part i in load j, and m for the grand mean. Then

```
  SS_P  = o r Σ_i (x̄_i − m)²                     df = p − 1
  SS_O  = p r Σ_j (x̄_j − m)²                     df = o − 1
  SS_PO = r Σ_ij (x̄_ij − x̄_i − x̄_j + m)²         df = (p − 1)(o − 1)
  SS_E  = Σ_ijk (x_ijk − x̄_ij)²                  df = p o (r − 1)

  mean square  MS = SS / df
```

These four add up to the total sum of squares; that's the ANOVA identity.

### 5.4 Expected mean squares, derived for one term

You need what each MS estimates. Take MS_O. The load mean is

```
  x̄_j = μ + P̄ + O_j + (PO)̄_j + ε̄_j
```

P̄ (the mean over the same 10 parts) is the same for every load, because the study is crossed,
so it cancels in x̄_j − m. What's left varies from load to load with variance

```
  Var = σ²_O  +  σ²_PO / p  +  σ² / (p r)
```

(a mean of p interaction terms has variance σ²_PO/p; a mean of p·r errors has variance
σ²/(pr)). For any o values with variance v, E[Σ (y_j − ȳ)²] = (o − 1) v. So

```
  E[SS_O] = p r (o − 1) [ σ²_O + σ²_PO/p + σ²/(pr) ]
  E[MS_O] = σ² + r σ²_PO + p r σ²_O
```

The same reasoning gives the full table:

```
  E[MS_E]  = σ²
  E[MS_PO] = σ² + r σ²_PO
  E[MS_O]  = σ² + r σ²_PO + p r σ²_O
  E[MS_P]  = σ² + r σ²_PO + o r σ²_P
```

Solve from the top down. These are the formulas in `anova_grr`:

```
  σ²     = MS_E
  σ²_PO  = (MS_PO − MS_E) / r
  σ²_O   = (MS_O − MS_PO) / (p r)
  σ²_P   = (MS_P − MS_PO) / (o r)          (each clamped at 0)
```

**Pooling.** If the interaction isn't significant (F = MS_PO / MS_E has p-value > 0.05), it's
merged into the error: MS_E' = (SS_PO + SS_E) / (df_PO + df_E), and MS_E' replaces both MS_PO
and MS_E above.

### 5.5 From variance components to %GRR

```
  EV  = σ                     repeatability
  AV  = √(σ²_O + σ²_PO)       reproducibility
  GRR = √(EV² + AV²)          the measurement system
  PV  = σ_P                   part variation
  TV  = √(GRR² + PV²)         total

  %GRR (tolerance) = 6 · GRR / (USL − LSL)
  ndc              = floor(1.41 · PV / GRR)
```

**Why 6σ.** ±3σ holds 99.73 % of a normal spread, so 6·GRR is the width the measurement noise
occupies. %GRR to tolerance is the share of the tolerance band that noise eats. AIAG guide:
< 10 % acceptable, 10–30 % conditional, > 30 % not acceptable.

**ndc** (number of distinct categories): how many groups of parts the gauge can tell apart
reliably. 1.41 ≈ √2. AIAG wants ≥ 5. It describes this study's deliberately spread parts, so
judge the gauge by %GRR to tolerance (the `validate.py` docstring explains why).

**Worked example: cell position, fixed line-scan.** Tolerance ±0.20 mm, so USL − LSL = 400 µm.
From the study: EV = 6.5 µm, AV = 33.0 µm.

```
  GRR = √(6.5² + 33.0²) = √(42.25 + 1089) = 33.6 µm
  %GRR = 6 × 33.6 / 400 = 50.4 %          → not acceptable
```

Compare terminal height: GRR = 0.5 µm, tolerance width 200 µm, so 6 × 0.5 / 200 = 1.5 %
(the table's 1.6 % uses unrounded σ). Excellent. The difference is the finding.

### 5.6 Why cell position fails: quantisation, Δ/√12

Cell position along the conveyor (X) comes from where the line-scan sees the cell's edges. The
Ranger3 takes one profile every Δ = 0.1 mm of travel. An edge between two profiles is reported
at one of them. Each pallet load lands the module at a different **phase** relative to the
profiles, so the error is anywhere in [−Δ/2, +Δ/2] with equal probability: a uniform
distribution.

Variance of a uniform distribution on [−Δ/2, Δ/2]:

```
  σ² = (1/Δ) ∫_{−Δ/2}^{Δ/2} x² dx = (1/Δ) · [x³/3] = (1/Δ) · (Δ³/12) = Δ²/12

  σ = Δ / √12 = 0.1 / 3.464 = 28.9 µm
```

Two trials within one load reuse the same phase, so this shows up as **AV** (between loads), not
EV (within a load). The measured AV is 33 µm, close to 28.9 µm plus the fixture's own scatter.
Treat that as a plausibility check, not a proof. Separating the two contributions properly is a
good exercise: rerun the study with the fixture error set to zero.

**Interview version:** "Terminal height measures at 1.6 % of tolerance. Cell position measures at
50 %, because the line-scan's 0.1 mm profile pitch quantises the edge, and the phase changes with
every pallet load. That's a sensor-placement result, not a detector bug, and it's one of the
arguments for the arm."

### 5.7 Bias and linearity

Separately from spread: is the gauge **centred**? For each part, bias = mean reading − true
value. Fit bias = a + b · true value. a is the average offset. b ≠ 0 means big and small parts
read differently (non-linearity). Cell position shows −25 µm mean bias and a 0.018 slope.

### Day 5 self-test

1. What's the difference between repeatability and reproducibility? What plays the "appraiser"
   in MotoBat, and why?
2. Write the random-effects model and the four sums of squares.
3. Derive E[MS_O]. Why does P̄ cancel?
4. Solve the expected mean squares for the four variance components.
5. Why 6σ? Compute %GRR for cell position from EV and AV.
6. Derive Δ²/12. Why does quantisation appear in AV and not EV?

---

## Day 6 — The correction loop

Code: not built yet (task 1.12, `project_plan.md` §4B.7, §9.9). This day ties straight into
your controls study: it's Ogata's z-plane material on a one-state system.

### 6.1 The model, stated carefully

Cell k is placed with error e_k (measured position − nominal). The GP7 places each cell with a
**running offset** c_k that the loop adjusts. Write g for the plant gain (commanded offset →
achieved offset; 1 ideally), d for a constant disturbance (gripper offset) and w_k for random
placement scatter:

```
  e_k     = d + g·c_k + w_k                 (plant)
  c_{k+1} = c_k − K·e_k                     (controller: nudge the offset by −K × the error)
```

**Watch the wording.** §9.9 writes the controller as `u_{k+1} = −K·e_k` and calls it
proportional. The bound 0 < K < 2/g holds when u is the **increment** to a running offset, as
above. That is, the offset is the integral of past errors. If instead the commanded offset were
just −K·e_k with no memory, the recursion would be e_{k+1} = d − gK·e_k: stable only for
gK < 1, and a constant d would leave a permanent error d / (1 + gK). Fix the wording in §9.9
when 1.12 is built, and know which one you mean in an interview.

### 6.2 Stability: 0 < K < 2/g

Ignore noise for now (w = 0). Subtract e_k from e_{k+1}:

```
  e_{k+1} − e_k = g (c_{k+1} − c_k) = −gK·e_k

  e_{k+1} = (1 − gK) · e_k      →     e_k = (1 − gK)^k · e_0
```

The error shrinks to zero only if |1 − gK| < 1:

```
  −1 < 1 − gK < 1   ⇔   0 < gK < 2   ⇔   0 < K < 2/g     (for g > 0)
```

What the regions look like, with a = 1 − gK:

| gK | a | Behaviour |
|---|---|---|
| 0 < gK < 1 | 0 < a < 1 | Smooth, one-sided decay |
| gK = 1 | 0 | **Deadbeat**: corrected in one cell (Ogata week 13) |
| 1 < gK < 2 | −1 < a < 0 | Over-corrects: sign alternates each cell, decaying |
| gK > 2 | a < −1 | Alternates and grows: unstable |

That table is what the planned K-sweep should show.

### 6.3 The same thing with the z-transform

z is the one-cell shift: Z{e_{k+1}} = z·E(z) (zero initial conditions). Transform both
equations, with D(z) the disturbance:

```
  (z − 1) C(z) = −K E(z)            →   C(z) = −K E(z) / (z − 1)
  E(z) = D(z) + g C(z)

  E(z) = D(z) − gK E(z)/(z − 1)

  E(z) / D(z) = (z − 1) / (z − 1 + gK)
```

- **Pole** at z = 1 − gK. Stability in discrete time means every pole inside the unit circle,
  |z| < 1, which is the same condition as §6.2.
- **Zero** at z = 1: the controller's integrator.

### 6.4 Steady state: constant offset and drift (final value theorem)

Discrete final value theorem: lim e_k = lim_{z→1} (z − 1) E(z), when the limit exists.

**Constant disturbance**, d_k = d, so D(z) = d·z/(z − 1):

```
  lim e_k = lim_{z→1} (z − 1) · (z − 1)/(z − 1 + gK) · d z/(z − 1) = 0
```

A constant gripper offset is removed completely, thanks to the zero at z = 1.

**Drift** (fixture wear), d_k = ρ·k, so D(z) = ρ z/(z − 1)²:

```
  lim e_k = lim_{z→1} (z − 1) · (z − 1)/(z − 1 + gK) · ρ z/(z − 1)² = ρ / (gK)
```

A ramp leaves a constant lag of ρ/(gK). A second integrator would remove it, which is why §4B.7
mentions integral action for systematic drift. (Checked by simulation: ρ = 0.002, gK = 0.5
settles at 0.004.)

### 6.5 Noise: why you can't just turn K up

Add placement scatter w_k (variance σ²_w, new each cell) and measurement noise n_k (variance
σ²_n, the sensor's). The controller sees e_k + n_k. Redo the recursion:

```
  e_{k+1} = (1 − gK) e_k  +  w_{k+1} − w_k  −  gK n_k
```

With a = 1 − gK, in z-transform form:

```
  E(z) = (z − 1)/(z − a) · W(z)  −  gK/(z − a) · N(z)
```

For white noise through a filter with impulse response h_k, the output variance is the input
variance times Σ h_k². Expand each filter in powers of 1/z:

```
  gK/(z − a)       :  h_k = gK·a^{k−1}, k ≥ 1      Σ h_k² = (gK)²/(1 − a²)
  (z − 1)/(z − a)  :  h_0 = 1, h_k = (a − 1)a^{k−1}  Σ h_k² = 1 + (1 − a)²/(1 − a²) = 2/(1 + a)
```

With 1 − a = gK and 1 + a = 2 − gK, so 1 − a² = gK(2 − gK):

```
  σ²_e = [ 2σ²_w + gK·σ²_n ] / (2 − gK)
```

(Checked by simulation for gK = 0.2, 0.5, 1.0, 1.5, 1.9: agreement within 0.5 %.)

What it says:
- At gK → 0 (no correction), σ²_e → σ²_w, but the constant offset d is never removed.
- σ²_e rises steadily with gK, and goes to infinity as gK → 2.
- So **bias and drift want a large K** (fast removal, small ρ/(gK)), while **noise wants a small
  K** (each correction re-injects measurement noise and chases last cell's scatter).

The K-sweep is the experiment that shows this trade-off. Choosing K is choosing a point on it.
That's the result to present.

### Day 6 self-test

1. Write the plant and controller equations. Explain why the controller is really integral
   action.
2. Derive e_{k+1} = (1 − gK) e_k and the bound 0 < K < 2/g.
3. What happens at gK = 1, and what is it called in Ogata?
4. Derive E(z)/D(z). Where is the pole, and where is the zero?
5. Use the final value theorem for a step and a ramp disturbance.
6. Derive σ²_e. Explain the trade-off it creates in one sentence.

---

## Day 7 — Review

Closed-book, on blank paper:
1. Answer every self-test question from days 1–6. Mark each as solid, shaky or wrong.
2. For each resume bullet, give a 2-minute spoken explanation, then answer "why?" three times
   in a row about it.
3. Re-derive the four results you'll most likely be asked about: Wilson, %GRR (with the
   cell-position numbers), the point-to-plane linearisation, and 0 < K < 2/g.
4. Rework anything marked shaky, then repeat it the next day from memory.

**Things to say honestly if asked:**
- Many tolerances are assumed (`detect.yaml`, every A), so recall and false-reject results are
  conditional on them (`status_update.md`, deferred item D1).
- The dataset is simulated. The sensor models were frozen before any results, but they're still
  models: no multipath, speckle or motion blur (`project_plan.md` §4B.3).
- The arm results assume exact robot poses; robot pose error isn't modelled yet.
- The D/2, D/3 rule is a rule of thumb, not a derived optimum.
