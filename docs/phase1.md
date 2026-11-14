# Phase 1 reference: what the dataset is, what the numbers mean

*Written 12 Nov 2026. This is the reference for Phase 1's vocabulary, case counts and statistics: what
each word means, why each number is what it is, and where the code and results live. Derivations are in
[`mathematics.md`](mathematics.md) (a study guide, by day). The decisions and results in date order are in
[`project_plan.md`](project_plan.md) §4B. What's done is in [`status_update.md`](status_update.md).
Problems found and fixed are in [`fixlist.md`](fixlist.md).*

---

## 1. The question Phase 1 answers

> On a battery-module assembly line, does an inspection sensor on a robot arm (several views per
> module) find assembly defects better than a fixed line-scan sensor the module passes under once?
> And are its measurements good enough to drive a correction loop?

Phase 1 answers it in simulation, with no robot yet:
1. Build a module from the real CAD (1.1), put known defects in it (1.2), scan it with modelled
   sensors (1.3), and detect the defects (1.4).
2. Choose the arm's views (1.5).
3. Repeat this for 775 modules per setup, each with a known answer (1.7). Then count how often each
   setup is right (1.8), how repeatable its measurements are (1.9), and compare the setups (1.10).
   Find the smallest defect each setup can find (1.11). Then use the measurements to correct the
   process (1.12).

Everything is computed. Because every defect is injected, the true answer is always known. A real
line can't offer that.

**This first dataset is the ideal world, on purpose.** The motion is perfect (no robot pose or path
error, no conveyor vibration), and every module is the CAD model exactly apart from its one defect.
The sensor rays are infinitely thin. So its numbers are an upper bound and the baseline. Later
datasets add one of these back at a time and measure what each costs. The full list of what is left
out is in `project_plan.md`, the scan-model limitations under §4B.3. One visible effect: the arm's
GRR shows ~0 µm repeatability, which is what perfect motion gives, not what a real HC10DT would.

---

## 2. Vocabulary

### The line and the station

| Term | Meaning |
|---|---|
| **Module** | One battery module: 12 cells in a housing (end plates, side plates, cooling plate), terminals on top, then busbars welded to the terminals. |
| **Stage** | Where on the line the inspection happens. `pre_busbar`: cells placed, no busbars yet. `pre_weld`: busbars placed, not welded. `post_weld`: welded. Each defect class is inspected at the stage where it can occur and still be fixed. |
| **Setup** | The sensor arrangement being tested. `fixed`: a Ranger3-class line-scan the module passes under once, looking straight down. `arm`: an HC10DT arm carrying a Zivid-class snapshot camera and an LJ-X8200-class profilometer, flying the view plan from 1.5. |
| **Capture** | One scan: a single snapshot, or a single sweep of a line sensor. The fixed setup makes one capture per module. The arm makes 13–15 (§6). |
| **View plan** | The arm's list of captures for one stage, chosen by 1.5's greedy algorithm (`data/coverage/<stage>.plan.json`). |
| **Pallet load / fixture error** | Each time a module is placed on the pallet, it sits slightly off its nominal position. Each scan draws a fresh random error: σ 0.03 mm in X/Y, 0.01 mm in Z, 0.01° yaw, 0.005° roll/pitch (`dataset.yaml`, assumed). The detector must register it out. |

### Defects

| Term | Meaning |
|---|---|
| **Class** | What the detector reports: `cell_placement`, `fod`, `weld_arc`, … |
| **Family** | What the dataset injects. Usually one per class, but `cell_placement` has two families (`cell_shift`, `cell_yaw`), because position and yaw have separate tolerances. There are 20 families (§3). |
| **Kind** | How a family is judged. **Tolerance**: a measured quantity with a limit (a cell shifted 0.3 mm against a 0.2 mm tolerance). **Presence**: something that shouldn't be there at all, judged by size (FOD = foreign object debris, spatter, dents). **Binary**: either present or not (a missing cell or weld, reversed polarity, a missing pad). |
| **Level / size** | How big the injected defect is. Tolerance families use multiples of their tolerance; presence families use absolute sizes (§3.1). |
| **D** | For a presence class, the smallest size that matters (FOD 0.25 mm radius, dents 0.1 mm deep, spatter 0.15 mm radius). In coverage (1.5) D is also the feature size a capture must resolve (§5). |

### What the dataset records

| Term | Meaning |
|---|---|
| **Case** | One module, at one stage, with one injected defect (or none) of one size, plus a **seed**. Each case is scanned by both setups. |
| **Seed** | One number that fixes everything random: where the defect goes (which cell, which direction, which surface point), each scan's fixture error and each scan's sensor noise. The same seed rebuilds the case bit for bit, so only the recipe and results are stored, not the ~150 MB point clouds. |
| **Tier** | A group of cases built for one purpose: `pod`, `clean` or `grr` (§3). |
| **Truth** | What the case *should* produce, decided from the injected parameters before any scan (`dataset.judge`): `must_find` (out of spec, or a binary defect, or presence ≥ D), `in_spec` (a tolerance defect below its tolerance, so a good part), `dont_care` (presence below D: too small to matter, kept only for the detection curve), `clean` (nothing injected). |
| **Label** | The ground-truth record of an injected defect: class, location and bounding box, magnitude. The scorer matches findings to labels. |
| **Finding** | Something the detector reports: class, location, magnitude, which capture. |
| **Report** | All findings for one case and setup, plus measurements, the OK/NOK decision and timings (`<setup>.report.json`). |

### Outcomes (per case, per setup: `dataset.score`)

| Outcome | When | Counts as |
|---|---|---|
| `hit` | must-find defect, a finding of its class on it | correct detection (TP) |
| `miss` | must-find defect, nothing of its class on it, **where the setup looks by design** | missed defect (FN) |
| `partial` | a defect with several targets (e.g. 5 spatter drops, 2 seams of one weld), some found | not a hit for recall, but the module was still rejected |
| `not_covered` | must-find defect missed **where the setup never looks** (1.5's coverage map says no capture sees it) | reported separately from recall, as a station property |
| `pass` | in-spec part, nothing of its class reported | correct |
| `false_reject` | in-spec part reported as defective (or a clean module with any finding) | false positive: a good part scrapped or reworked |
| `clean_ok` | clean module, no findings at all | correct |
| `found` / `not_found` | `dont_care` sizes | used only for the detection curve (1.11) |

Two counts sit beside the outcome:
- **false alarms (`fa`)**: findings on *no* label at all. These are the detector imagining something.
- **other**: a finding of a *different* class on the injected defect. These are knock-on effects or
  cross-talk. Example: a carrier sitting 0.3 mm high really does lift all five busbars, so
  `busbar_liftoff` ×5 on a `carrier_seating` case is "other", not a false alarm.

---

## 3. The three tiers, and where every number comes from

Each setup runs **775 cases**: 565 pod + 30 clean + 180 grr.

### 3.1 `pod`: 565 cases (POD = probability of detection)

**Purpose:** for each family and size, does the setup find it, and does it leave good parts alone?
This feeds recall, false rejects and precision (1.8), the fixed-vs-arm comparison (1.10) and the
detection curves (1.11).

**Sizes per family** (`dataset.yaml`):
- **Tolerance families (13):** 7 levels at 0.5, 0.8, 1.1, 1.3, 1.6, 2.0, 3.0 × tolerance.
  - 0.5 and 0.8 are *in spec*: they test whether a good part is wrongly rejected.
  - 1.1 and 1.3 sit just over the line, where detection is hardest and the curve changes fastest.
  - 1.6, 2.0 and 3.0 are clear defects.
- **Presence families (3):** 6 absolute sizes each, spanning D. FOD radius 0.05–0.6 mm (D 0.25);
  dent depth 0.02–0.8 mm (D 0.1); spatter radius 0.05–0.3 mm (D 0.15). Sizes below D are `dont_care`.
- **Binary families (4):** 1 level each (present).

**Seeds:** 5 per (family, level). Each seed puts the defect somewhere else, with a different fixture
error and noise.

**The count:**
```
  13 tolerance families × 7 levels = 91
   3 presence families  × 6 levels = 18
   4 binary families    × 1 level  =  4
                                    ----
                                    113 (family, level) cells × 5 seeds = 565
```

**Truths among the 565:**
- 395 must_find: 13×5 tolerance levels above spec ×5 seeds = 325, plus 10 presence levels ≥ D ×5 = 50, plus 4 binary ×5 = 20.
- 130 in_spec: 13 × 2 levels × 5.
- 40 dont_care: 8 presence levels below D × 5.

**Why 5 seeds:** each family gets 5 out-of-spec levels × 5 seeds = 25 must-find cases. With 25
cases, a perfect score proves recall ≥ 87% at 95% confidence (Wilson, §4.2). More seeds would
tighten that, at proportional compute cost. Five is the assumed compromise (tagged **A**). The held-out
set (1.13) adds fresh seeds later.

**Tolerances used** (`configs/detect.yaml`, tagged S/D/A as in §7):

| Family | Tolerance | Tag |
|---|---|---|
| cell shift / busbar shift | 0.20 mm | A |
| cell yaw / busbar yaw | 0.10° | A |
| terminal height | 0.10 mm | S (spec §4; confirm, fixlist D1) |
| terminal tilt | 0.31° | D (tilt that raises one pad edge 0.1 mm) |
| busbar gap (lift-off) | 0.15 mm | A |
| carrier seating | 0.20 mm | A |
| short weld | 10% of length | A |
| seam offset | 0.25 mm | A |
| weld arc (bow) / path drift | 0.10 mm | A |
| bead width | 20% | A |
| reinforcement (crown height) | 0.10 mm | A |

Most are assumed. So every in-spec/out-of-spec result is *conditional on these tolerances*, and the
write-up says so (fixlist D1).

### 3.2 `clean`: 30 cases

**Purpose:** modules with nothing injected. Any finding is a false alarm. This measures how often a
good module is rejected for nothing, the line's scrap-for-no-reason rate.

**The count:** 10 modules × 3 stages = 30. Ten per stage is assumed. These 30 aren't the only
false-alarm evidence: findings on no label are counted over **every** scanned module in all three
tiers (775 per setup), because each one is an independent fixture and noise draw.

### 3.3 `grr`: 180 cases (Gage R&R)

**Purpose:** is the setup a good enough *gauge* for the three measurements a correction loop would
use? Terminal height, cell position (along X) and busbar gap. "Found it or not" (recall) doesn't
answer that. The question is whether repeated measurements of the same part agree closely enough,
compared with the tolerance.

**The design**, the standard one from AIAG *Measurement Systems Analysis* (4th ed.):
- **10 parts** per characteristic, with true values spread across ±1.5 × tolerance (the gap from
  0.1 to 3 × tolerance, since a gap can't be negative). AIAG's typical study uses 10 parts.
- **3 "appraisers"**: in a factory that means 3 operators. Here it means **3 pallet loads**: the
  module is lifted out and placed again, so the fixture error changes.
- **2 trials** per load: the same placement scanned again, with only the sensor noise new. AIAG uses
  2–3.

**The count:** 3 characteristics × 10 parts × 3 loads × 2 trials = **180**.

A part keeps its defect (location and true value) across all 6 of its scans. Only placement and noise
change.

---

## 4. The statistics, in plain terms

Code: [`defectgen/src/defectgen/validate.py`](../defectgen/src/defectgen/validate.py). Derivations:
[`mathematics.md`](mathematics.md) Day 4 (detection statistics) and Day 5 (Gage R&R).

### 4.1 Recall, false-reject rate, precision, false alarms

```
  Recall            = hits / (hits + misses + partials)     of out-of-spec defects in view, how many found
  Rejected          = modules decided NOK / must-find cases  the line's decision (a partial still rejects)
  False-reject rate = false_rejects / in-spec cases          good parts thrown out
  Precision         = correct findings / all findings of that class
                      (correct + on in-spec parts + on nothing)
  False alarms      = findings on no label, per module scanned
```

Recall is **conditional on coverage**: `not_covered` cases are listed but left out. A sensor can't be
blamed for a place it never looks, but the station can, so coverage is reported beside recall, not
hidden. This is the convention of MIL-HDBK-1823A, the US DoD handbook on POD for NDT.

**Reading error** per family (1.8): reading minus true size over every pod case, as mean, sd and
slope against size. A slope off zero is a *proportional* bias. Recall can look fine while the
readings are biased: this table is what exposed the datum fault on 9 Nov.

### 4.2 Wilson intervals: "how sure are we?"

25 hits out of 25 doesn't mean recall *is* 100%. With only 25 tries, a detector whose true recall is
90% would still score 25/25 about 7% of the time. A **confidence interval** gives the range of true
recalls consistent with what was seen.

The textbook (Wald) interval, `p̂ ± 1.96 √(p̂(1−p̂)/n)`, breaks exactly where these results sit. At
25/25, p̂ = 1, so the width is zero: "certainty" from 25 samples. The **Wilson interval** asks the
question the other way round. Which true rates p would make the observed k/n *unsurprising*, within
1.96 standard errors **computed at p itself**?

```
  |k/n − p| ≤ 1.96 √( p(1 − p) / n )
```

Squaring this gives a quadratic in p. Its two roots are the interval:

```
  centre = (p̂ + z²/2n) / (1 + z²/n)
  half   = z √( p̂(1−p̂)/n + z²/4n² ) / (1 + z²/n)        z = 1.96 for 95 %
```

| Seen | Wilson 95% |
|---|---|
| 25/25 | 86.7 – 100% |
| 24/25 | 80.5 – 99.3% |
| 10/10 | 72.2 – 100% |
| 0/30 (clean flagged) | 0 – 11.4% |

It stays inside [0, 1], is never zero-width, and is accurate at small n. Full derivation:
`mathematics.md` §4.4. Code: `validate.wilson`.

### 4.3 False alarms: a Poisson bound

False alarms are rare events counted over many modules, so they're treated as a **rate** (alarms per
module), not a proportion. Given k alarms over N modules, the 95% upper bound on the true rate comes
from the Poisson distribution (exact, via chi-square):

```
  rate_upper = χ²(0.95; 2k + 2) / (2N)
```

With **k = 0** this becomes `−ln(0.05)/N ≈ 3/N`, the **rule of three**. Zero alarms in 775 modules
means "fewer than 0.39% per module, with 95% confidence". Example: the first arm run's 8 alarms in
595 modules gave an upper bound of 2.4% per module. Code: `validate.poisson_upper`.

### 4.4 Gage R&R: "is the gauge good enough?"

Every reading of the grr study is modelled as

```
  reading = true part value + load effect + (part × load) + repeat noise
```

An ANOVA, which compares the spread of averages with the spread within groups, separates these into
standard deviations:

| Symbol | Name | Meaning here |
|---|---|---|
| **EV** | repeatability ("equipment variation") | rescanning the same placement: sensor noise |
| **AV** | reproducibility ("appraiser variation") | reloading the pallet: fixture error and sampling phase |
| **GRR** | √(EV² + AV²) | the measurement system's total spread |
| **PV** | part variation | how much the parts really differ |

The verdict, against the tolerance:

```
  %GRR to tolerance = 6 · σ_GRR / (USL − LSL)
```

Six standard deviations cover 99.7% of readings. %GRR is the share of the tolerance band the gauge's
own scatter takes up. AIAG's guide: **< 10% acceptable, 10–30% conditional, > 30% not acceptable.**

- **%GRR to study variation** (σ_GRR / σ_total) and **ndc** (number of distinct categories,
  1.41 × σ_part / σ_GRR) are also printed. But the parts were *spread by design*, not drawn from a
  real process, so these two describe the study, not the line. Judge by %GRR to tolerance, as AIAG
  advises for this case.
- Tolerance width: terminal height ±0.10 → 0.20 mm; cell position ±0.20 → 0.40 mm; the busbar gap is
  one-sided, 0 to 0.15 → 0.15 mm.
- **Bias** (mean reading − true value) and **linearity** (does the bias grow with size?) are reported
  beside it.
- Interaction pooling: if the part × load term isn't significant (F-test p > 0.05), it's folded into
  repeatability (standard AIAG practice).

First result, fixed setup: terminal height 1.5%, busbar gap 0.5% (excellent); **cell position along X
56% (fails)**. Most of that is AV, load to load. The line-scan's profiles are 0.1 mm apart, so an
edge's reading jumps in steps depending on where the samples happen to fall. Each reload shifts that
phase. This is quantisation, Δ/√12 (`mathematics.md` §5.6), not noise, and it's a key argument for
the arm's finer profilometer.

### 4.5 POD curves and the minimum detectable defect (1.11, planned)

For each presence class (and tolerance class), plot hit/miss against size and fit a **logistic
curve**, P(found) = 1 / (1 + e^−(a + b·size)). From it read **a50** and **a90**: the sizes found 50%
and 90% of the time. a90 is "the smallest defect this setup reliably finds", the usual NDT figure,
again after MIL-HDBK-1823A. The `dont_care` sizes exist so the curve has data below D.

---

## 5. Coverage (1.5): "is the setup even looking?"

Before asking whether a defect was found, ask whether any capture *could* have found it.

1. The module's surfaces are cut into ~1.5 mm **patches**. Each defect class lists the patches it
   needs (e.g. dents: housing walls and tops).
2. A capture **covers** a patch for a class of feature size D only if both halves of the rule hold
   (§3.7.3, `mathematics.md` §2.4):
   - **spacing ≤ D/2**: at least two samples across the smallest feature (Nyquist-style);
   - **noise ≤ D/3**: the point scatter is small enough to see a D-sized change.

   Since 12 Nov: for a sweep, spacing includes the **profile pitch** (speed ÷ rate), so a fast sweep
   can't count for fine classes. A class may also set a separate `noise_D` (dents: 0.1 mm deep but D
   2 mm wide).
3. Coverage is judged **per capture**, never summed. A measurement happens inside one capture.
4. The **greedy plan** repeatedly adds the capture with the most coverage gained per second, until
   nothing adds ≥ 0.1%.

Result (final plan, 12 Nov): arm 15 / 13 / 15 captures, 51 / 46 / 63 s per stage. Housing-wall
coverage is 89–92% for the arm against 9–25% for the fixed pass. Captures are both sweeps (slow
0.05 mm pitch for 0.2 mm-class features, fast 200 mm/s for walls) and snapshots.

---

## 6. One case, start to finish

```
recipe (tier, family, size, seed)
  │
  ├─ judge → truth: must_find / in_spec / dont_care / clean
  ├─ build the module from CAD, inject the defect          → labels.json
  ├─ for each setup (fixed, arm):
  │     for each capture: new fixture error + new noise
  │        → ray-cast the sensor model (scan.py)
  │     detect (detect.py):
  │        register to the housing (X/Y/yaw from the housing strips; Z/roll/pitch per capture)
  │        → measure features (cells, terminals, busbars, seams) with plane/edge fits
  │        → compare every ray with the golden (defect-free) module as measured → residual regions
  │        → classify regions; decide OK / NOK                → <setup>.report.json
  │     score against labels → outcome, fa, other            → <setup>.score.json
  └─ one line in <tier>/index.jsonl
```

Typical time per case (12 Nov): fixed ~15 s; arm ~2–3 min (13–15 captures, each recast for the golden
reference). That's why the arm column takes ~1.5 days and the fixed one ~3.5 hours.

---

## 7. The S / D / A tags

Every parameter in the configs carries a tag:
- **S**, sourced: from a datasheet, standard or paper (cited beside it).
- **D**, derived: computed from sourced values (the formula is beside it).
- **A**, assumed: an engineering guess. The result is conditional on it, and it's on the list to
  source (`project_plan.md` §3.7.9, fixlist D1).

The sensor and material parameters were **frozen and hashed** before any detection result was seen
(`test_sensor_parameters_are_frozen`). That way the sensors can't be tuned after the fact to make the
detector look good.

---

## 8. Phase 1 tasks at a glance

| Task | What | Code | Output |
|---|---|---|---|
| 1.1 | Module generator from CAD, 3 stages | `module.py`, `parts.py` | meshes + labels |
| 1.2 | Defect injection | `defects.py`, `weld.py` | labelled defects |
| 1.3 | Sensor models (line-scan, snapshot, profilometer) | `scan.py`, `configs/sensors.yaml` | point clouds |
| 1.4 | Detection | `detect.py`, `fit.py`, `evaluate.py` | reports |
| 1.5 | Coverage + arm view plan | `coverage.py` | `data/coverage/` |
| 1.6 | ROS interfaces (ROS lane) | `motobat_interfaces` | — |
| 1.7 | Labelled dataset (this document) | `dataset.py`, `configs/dataset.yaml` | `data/dataset/` |
| 1.8 | Precision/recall with Wilson intervals | `validate.py pr` | `data/dataset/validation/` |
| 1.9 | %GRR | `validate.py grr` | same |
| 1.10 | Fixed vs arm | (from 1.5 + 1.8) | — |
| 1.11 | Minimum detectable defect, sensitivity | planned | — |
| 1.12 | Correction loop (numpy) | planned (`mathematics.md` Day 6) | — |
| 1.13 | Held-out validation set | deferred | — |

---

## 9. Where things are

| What | Where |
|---|---|
| Dataset design, counts, levels, seeds | `defectgen/src/defectgen/configs/dataset.yaml`; `project_plan.md` §4B.6 "1.7 dataset — Design" |
| Tolerances and detector thresholds | `defectgen/src/defectgen/configs/detect.yaml` |
| Coverage classes, D, noise_D | `defectgen/src/defectgen/configs/coverage.yaml` |
| Outcome rules | `dataset.py`: `judge`, `score` |
| Statistics | `validate.py`: `wilson`, `poisson_upper`, `anova_grr`, `bias_linearity`, `pr`, `grr` |
| Derivations (Wilson, Poisson, ANOVA, ICP, MAD, control loop) | [`mathematics.md`](mathematics.md) |
| Run results, current | `data/dataset/` (v2 rerun, 12 Nov) |
| Run results, first run | `data/dataset_plan_v1/`, `data/validate_pr_{fixed,arm}_final.txt` |
| What went wrong and was fixed | [`fixlist.md`](fixlist.md) §F (F6: the arm plan) |
| Regression checks | `python -m defectgen.regress <group>` (see its docstring) |
| Run progress | `python3 ~/motobat/progress.py -w`, `data/PROGRESS.md` |

Commands:
```
python -m defectgen.dataset --tier pod|clean|grr [--setups fixed arm]   # run (resumes; from the repo root)
python -m defectgen.dataset --tier pod --summary                        # outcome table so far
python -m defectgen.validate pr  --setup fixed|arm                      # 1.8
python -m defectgen.validate grr --setup fixed|arm                      # 1.9
```
