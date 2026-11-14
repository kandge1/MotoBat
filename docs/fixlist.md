# MotoBat — Fix List

**One question this file answers: what is known to be wrong right now, and where does each thing
belong?**

This is a working checklist, not a reference. When an item is done, strike it through with the
date (`~~item~~ — **done 5 Nov 2026**`) rather than deleting it, per the house style in
`module_spec.md` §5. When an item turns out to be a decision rather than a fix, move it to
`project_plan.md` and leave a pointer here.

Section references like §4A.4 point into `project_plan.md`; §7 and §12 point into
`module_spec.md`.

*Last updated: 13 Nov 2026. **§F is new**: eight findings from the 1.7 dataset (F5: run speed, first pass done 12 Nov; F6: the arm plan and detector faults the arm column exposed, fixed 12 Nov; F7: the v2 rerun's arm false alarms, fixed 13 Nov; F8: a spatter drop F7's threshold lost, fixed 13 Nov). F1 and F2 are
geometry against tolerance contradictions, F3 and F4 detector limitations. D1 now matters more: the
dataset's in-spec / out-of-spec labels rest on the tolerances. A, B, C and E are closed (1 Nov and 4 Nov).*

---

## 0. Resolved on 1 Nov 2026 — recorded so they are not re-asked

| Question | Answer |
|---|---|
| What is `AS_108261` in the mount STEP? | **Keyence LJ-X8200.** The mount carries Zivid 2+ M60, LJ-X8200 and `Mount v3`. No midiCam2, which matches the 28 Oct decision. |
| LJ-X8200 head mass | **1200 g** (S — user). This was the missing figure blocking the payload sum. |
| midiCam2 on the wrist | Future version only, if the Zivid's 2D proves insufficient. Gated on task 1.11. |
| Flange pattern | **DT, not DTP** — 63 mm PCD, measured and independently verified by the user, 1 Nov 2026. See C2. |
| Is the Zivid FOV volume in the CAD? | **No.** The product name `Zivid 2 Plus M60 with FOV v1` is just Zivid's download filename. The cone will be generated in code for TF2 / RViz. |
| `Mount v3` material | **AL-5052.** |

---

## A. CAD re-exports — ✅ closed 1 Nov 2026

- [x] ~~**A1 — Export `terminal_pos` and `terminal_neg` as separate STLs.**~~ — **done
      1 Nov 2026.** Exported as `cell_can`, `terminal_pos`, `terminal_neg` in both STEP and STL,
      one solid each, correctly named products. This completes §7's 12-body list and unblocks the
      terminal-height and terminal-tilt defect classes in task 1.2.
      **Standing rule that came out of it:** the generator loads `cell_can` + the two terminals,
      **never** `cell_assembled` — the assembled cell is a single merged mesh with no terminal to
      transform, and exists only for renders.

- [x] ~~**A2 — Decimate `inspector_flange.stl`.**~~ — **deferred by decision, 1 Nov 2026.** The
      detail is wanted and the file is under the 100 MB limit. Note that §9 only requires
      *collision* geometry to be simple, not visuals, so the detailed mesh and a cheap collision
      proxy can coexist. **Revisit on performance, not on the limit:** if RViz or Gazebo turns
      sluggish, or `defectgen` ray-casting is slow, this file is the first suspect, since the
      sensor models intersect against every triangle in it.

- [x] ~~**A3 — Exclude the Zivid FOV volume from collision and visual.**~~ — **not applicable,
      1 Nov 2026.** There is no FOV geometry in the file. Generating it in code is the better
      approach regardless: it can follow the datasheet's 300–1100 mm working range instead of
      being frozen at one distance. The warning stays recorded in §4A.4 in case an FOV solid is
      ever imported.

- [x] ~~**A4 — Confirm `Mount v3`'s material.**~~ — **done 1 Nov 2026, AL-5052.**
      **Correction carried into §4A.4:** do **not** set the camera bodies to a material and read a
      total assembly mass. They are placeholder shells; a hollow plastic Zivid weighs nothing like
      a real one. Read `Mount v3`'s mass **in isolation** and add the datasheet figures
      (Zivid 1.0 kg, Keyence 1.2 kg). The LJ-X8000A controller is not on the arm and must not be
      added — though its cable routes through the wrist, which is an open §4A.4 design question.

---

## B. Repository structure — ✅ closed 1 Nov 2026

- [x] ~~**B1 — `src/battery_description/` is not a package and breaks the prefix rule.**~~ —
      **done.** The directory is gone; its contents folded into `motobat_description`. This avoids
      creating a second package that would hold nothing but geometry, which §3.7 warns against.
- [x] ~~**B2 — Rename `models/` to `meshes/`.**~~ — **done.**
- [x] ~~**B3 — Decide whether STEP ships inside the ROS package.**~~ — **done: it does not.**
      STEP lives in `cad/` at the repo root, so the package ships only what Gazebo and MoveIt
      actually read.

Also done in the same pass: **every file renamed to its §7 body name** (`Bus bar long.stl` →
`busbar_series.stl`, and so on). Spaces in paths are awkward in URDF, SDF and shell, and matching
§7 means a filename, a Fusion User Parameter prefix and a generator config key all read the same.
All moves were made with rename detection intact, so file history follows.

Resulting layout — see `project_structure.md` §2 for the reasoning:

```
cad/
  module/            15 × .step   (12 bodies + cell_assembled + module_assembled)
  inspector/         inspector_flange.step
src/
  motobat_description/
    meshes/
      module/        15 × .stl
      inspector/     inspector_flange.stl
```

---

## C. Documentation corrections — ✅ closed 1 Nov 2026

- [x] ~~C1 — §4A.4: `AS_108261` → Keyence LJ-X8200~~
- [x] ~~C2 — §4A.4: flange is **DT**, 63 mm PCD, ISO 9409-1-63-4-M6; adapter clearance holes on
      PCD 63~~ — **done.** How it was settled, recorded for the write-up: opposite hole centres
      *are* the pitch circle diameter, and they measure 63 mm against the DTP's 50. The 5 mm holes
      (M6 tapping drill) and 6 mm keying hole (Ø6 H7 dowel) are common to both variants and do not
      discriminate — only the PCD does. This agrees with the URDF being `hc10dt`, so the proposal
      naming the HC10DTP was the error. **One item moved to D: the ø31.5 centring diameter.**
- [x] ~~C3 — payload sum with working~~ — **done**, ≈ 2.5–3.0 kg of 10.
- [x] ~~C4 — remove "Keyence head mass TBD"~~ — **done.**

---

## D. Still open

> **None of these block the current task.** Task 1.1 needs the CAD, and the CAD is done. Each
> item below gates a *specific later* task, listed in the table. Carry on with 1.1; come back to
> each one when the task it gates comes up.

Ordered by what each can cost, worst first.

| # | Item | Blocks | Can it change a conclusion? |
|---|---|---|---|
| D1 | MDPI 0.1 mm tolerance | results 1.8–1.12 | **Yes** |
| D2 | Module-line cycle time | task 2.7 | **Yes** — leaves the central comparison half-answered |
| D3 | CMC component height | next CAD pass on the CMC | No — a design decision |
| D4 | `Mount v3` mass | task 2.3 write-up | No |
| D5 | ø31.5 centring diameter | nothing, unless physically built | No |

---

### D1 — Confirm the 0.1 mm adjacent-terminal tolerance from MDPI *Batteries* 6(3):47

**What it is.** The claim that adjacent terminals must match within ±0.1 mm. §4 attributes it to
that paper but says it came "via search" — from a search result, not from reading the paper.

**What it costs.** This number is load-bearing for the whole project. It sets the defect size,
which drives the entire §3.7.3 sensor analysis (σ ≲ D/3, sample spacing ≲ D/2) — which is *why*
the Zivid screens and the profilometer adjudicates. It also sets the defect severity ranges in
task 1.2 and is the justification that this inspection is worth doing at all.

If the paper says 0.2 mm, or says 0.1 mm about a different quantity, or measures it differently,
the sensor argument shifts underneath the results. **This is the one item in D that can
invalidate a conclusion rather than just leave a gap.** §3.7.9's "source before claiming results"
exists for exactly this.

**To close.** Read the paper itself.

### D2 — A module-line cycle time

**What it is.** Seconds per module on a real prismatic module assembly line.

**What it costs.** The headline question (§3.7.6) is whether an arm finds what a fixed line-scan
misses **and what it costs in cycle time**. Task 2.7 will produce "the arm takes N seconds per
module." Without an industrial figure to compare against, there is no way to say whether N is
acceptable or disqualifying — you get the coverage and recall half of the result and no verdict
on feasibility.

A 1.5–2.5 s/unit figure appears in proposal §3.2, but it is unsourced and may be per *cell
placement* rather than per module. Those are very different numbers, and using the wrong one
would make the arm look either trivially fine or hopelessly slow.

**To close.** Find a real module-line takt, or state the assumption explicitly in the README's
Limitations section rather than quietly adopting the proposal's number.

### D3 — CMC component height

**What it is.** The cover underside sits at z = 105.5 and the bare CMC board tops out at 104.1,
leaving **1.4 mm** of headroom for anything mounted on that board (§5 height check).

**What it costs.** 1.4 mm is almost certainly not enough for a realistically populated board: an
electrolytic capacitor is 5–10 mm, a 0.1" header ≈ 8 mm, and even a modest SMD connector is
2–4 mm. Model a populated CMC and it will clip straight through the cover. Raising the walls is
not available — that breaks the 108 mm envelope.

Three ways out, none expensive:

1. Model the CMC as a **bare board**, no components. Nothing measures it, so this is likely fine.
2. **Recess the cover** locally over the CMC.
3. Accept the interpenetration — the cover is opaque and nothing inspects underneath it.

**Why it is listed at all.** Not because it is hard, but because deciding late means discovering
the clash at an assembly fit-check after the board is modelled. It is a decision, not a research
gap.

### D4 — `Mount v3` mass

**What it is.** The adapter plate's mass, now that the material is set to AL-5052.

**What it costs.** Very little. §4A.4's payload reads "≈ 2.5–3.0 kg" because plate + cabling is
a guessed range; with the real plate figure it becomes computed rather than estimated. You are
at roughly 3 kg of a 10 kg payload either way, so no design decision turns on it — but it is a
number that goes in the write-up and it is currently tagged **D** with a missing input.

**To close.** Read `Mount v3`'s mass **in isolation** and add the datasheet figures (Zivid
1.0 kg, Keyence 1.2 kg). Do not read a whole-assembly mass — the camera bodies are placeholder
shells and whatever material they carry is meaningless (see A4).

### D5 — The ø31.5 centring diameter

**What it is.** On a tool flange the bolts only clamp; a spigot — a boss on the robot seating
into a recess in the plate — is what centres the plate concentrically. ø31.5 is that diameter.
It came from SMC's catalogue for the **DTP**, and the flange is a **DT**, so the figure now has
no source.

**What it costs.** Nothing in this project. §4A.4 already states the plate is a fixed joint to
`s3_tool0` in simulation, with bolts and pins not simulated. No result depends on it.

It would matter if the mount were ever physically built: a wrong centring diameter means the
plate either will not seat or seats loose, and loose means its position depends on how the bolts
were tightened — which would break the CAD-to-URDF correspondence that §4A.4 calls ground truth.

**Why it is listed at all.** It was tagged **S** when it was really **A**. Leaving a wrong
confidence tag in place is precisely what the S/D/A/M system exists to prevent, so it gets
demoted and recorded rather than quietly kept.

**To close.** Yaskawa's HC10DT manual drawing.

---

**Watch item, not a task.** `inspector_flange.stl` at 99 MB — revisit only if performance
suffers (A2).

---

## E. Found by building the generator — 1 Nov 2026

Loading every mesh and comparing it against `module_spec.md` turned up things no amount of
reading would have. Both are recorded as strict `xfail`s in `defectgen/tests/`, so the suite is
green today and will fail loudly the moment each is fixed — telling you to delete the marker.

- [x] ~~**E1 — Re-export `terminal_neg` without the fill plug.**~~ It exports 38.5 wide against
      `terminal_pos`'s 22, so the plug is merged in. *Downgraded 3 Nov 2026.* The 153.5-wide
      module this was blamed for was a generator bug: the part was placed by its bounding-box
      centre instead of its pad centre, which pushed the plug outboard. Measured against the
      Fusion `cell_assembled`, the plug runs inboard. Fixed in `parts.py` (`pad_datum`), together
      with `+`/`−` on swapped sides and flipped cells whose terminals were not turned
      (`module.cell_transform`). The module is now 151.0 wide. What remains: a height or tilt
      defect on a `−` terminal also moves the plug, which is part of the can. *Blocks: nothing.*
      Tests: `test_parts.py::test_terminals_have_matching_footprints` (strict xfail),
      `test_module.py::test_terminals_sit_where_the_fusion_cell_has_them`.
      *4 Nov 2026:* still open — `cell_can` and `terminal_neg` were not part of the 6fb58f8 re-export.
      *Closed 4 Nov 2026 — by design.* The fill plug on the negative terminal is a feature: the
      terminals' asymmetric lengths key into the carrier, which is pocketed for the plug, so a cell
      put in backwards cannot take the carrier. A height or tilt defect on a `−` terminal moving the
      plug with it is therefore correct. The strict xfail is replaced by
      `test_parts.py::test_terminals_are_asymmetric_so_the_carrier_can_key_them`.

- [x] **E2 — The CMC overruns the cover by 0.2 mm.** *Not real (3 Nov 2026).* The arithmetic used the
      carrier's top (102.0) as the CMC seat. In `module_assembled` the CMC sits flush on the carrier
      deck at 98.0 → 101.2, between the pegs, leaving 3.8 to the wall top. Fixed with
      `carrier_deck_h: 3.0`; `test_budgets.py::test_cmc_clears_the_cover` now passes.

- [x] **E3 — `busbar_carrier.stl` is 7.0 tall; the carrier inside `module_assembled` is 8.0.** The
      assembly's carrier has CMC mounting pegs to z = 103.0 (223.6 mm² of top faces) that the
      standalone export lacks. Re-export the carrier from the current Fusion design and set
      `carrier_h` to match. The standalone export also has wide bosses at z +5.6 that reach under
      solid CMC board (48 of 264 sampled points), so on it the CMC would ride 2.6 high on the
      bosses. Until the re-export, `settle.py` leaves the CMC at the assembly's deck height
      instead of settling it.
      *Resolved 4 Nov 2026:* the user re-exported the carrier (commit 6fb58f8). Both it and the assembly's carrier are 7.0 tall; the 8.0 seen earlier came from the previous assembly. One mismatch is left, see E6.

- [x] **E4 — Six cells in `module_assembled` face the wrong way for 3P4S.** Cells 3, 6, 7, 9, 10
      and 11 are rotated against the rest of their parallel group. By fill-plug side the assembly
      is three blocks of four — cells 0–3 one way, 4–7 the other, 8–11
      the first — which is a 4P3S layout, while its busbars are 3P4S. 3P4S needs
      [0–2][3–5][6–8][9–11] alternating. The generator follows spec §2.
      *Resolved 4 Nov 2026:* module_assembled is now 3P4S. `test_module.py::test_module_matches_the_fusion_assembly` checks every generated part against a body of it to 0.05 mm. It runs the chain from group 0's positive end, so the generator now flips the even groups (`module.is_flipped`).

- [x] **E5 — The carrier sits on the fill plugs.** The plug merged into `terminal_neg` stands 5.0
      above the cap; the carrier's underside is on the cap with no pocket over the plug, so every
      negative terminal overlaps the carrier by ≈ 1426 mm³ — in `module_assembled` too. Either
      pocket the carrier over the plugs or model the plug at its real (much lower) height as a
      can feature. Every plug clashes regardless of which way its cell faces, so the carrier has
      no keying pockets at present (the user recalls designing keying that may have been lost
      while simplifying the carrier). Until fixed, `settle.py` rests the carrier on the terminal
      pads only (`_pad_only`).
      *Resolved 4 Nov 2026:* the re-exported carrier clears every fill plug, and `settle.py` no longer rests the carrier on the pads alone. A cell put in backwards does not fit: its plug lifts the carrier 2.9 mm (realistic demo) to 5.0 mm, which is the keying working (`test_a_reversed_cell_keeps_the_carrier_from_seating`).

- [x] ~~**E3 — `cmc` is non-manifold**~~ — **closed as not-a-defect.** It has no holes, so a ray
      cannot leak through it. The generator checks for holes rather than strict two-manifoldness,
      because CAD parts whose sub-bodies sit exactly face-to-face always go non-manifold once
      duplicate vertices merge, and nothing downstream cares.

**Six dimension corrections** also came out of this and are already written into
`module_spec.md` §5 — walls 101.0, carrier 7.0, busbar width 12, busbar lengths 64.5 / 148.5,
CMC 250 × 50 × 3.2, overhang rule `n × 28 + 8.5`. Z now closes at 107.0, not 107.5.

- [x] **E6 — The standalone `cmc.stl` and `busbar_carrier.stl` disagree with `module_assembled`.**
      `cmc.stl` was last exported at 2de6caa; its twelve 6 mm peg holes sit about 0.5 mm off the
      carrier's pegs along X, so half of every peg is under solid board. `busbar_carrier.stl` also
      has 12 pegs where the carrier in `module_assembled` has 8 (none at x ≈ ±22.5). The assembly
      itself is consistent. Re-export both standalone files from the assembly. Until then the CMC
      is left at its deck height and not settled (`settle.ORDER`).
      *Closed 4 Nov 2026:* the user re-exported both from the assembly. The 0.5 mm hole offset was never in the CAD: the CMC can be laid in at 90 or 270 deg with identical extents, and the loader picked the wrong one. Its hole pattern is 0.5 mm off-centre, so it showed as offset. `parts.py` now turns it 270 deg, and `test_cmc_holes_sit_on_the_carrier_pegs` checks all 12 holes against the pegs to 0.05 mm. The CMC is held by its pegs and rides with the carrier (`settle._carry_cmc`) instead of being settled.

- [x] **E7 — Busbars sit in carrier slots with zero side clearance.** A busbar moved 0.2 mm / 0.1°
      can no longer drop into its slot, so settling rests it on top of the carrier, exactly 1.000 mm
      high. That is what a zero-clearance part does, but real slots have a sliding fit for exactly
      this reason, and it makes realistic busbar errors read as 1 mm lift-off. Zero clearance is
      right for *mating* faces (spec §7: the gap is the measurement); it is wrong for *locating*
      fits. Give the busbar slots (and any other locating feature: terminal windows, CMC pegs) a
      design clearance in Fusion. A typical starting point is 0.2–0.3 mm per side; the user decides.
      *Closed 4 Nov 2026:* the user added 1.5 mm relief per side on the busbar slots. A busbar moved 0.2 mm / 0.1 deg, or 1.2 mm sideways, now drops into its slot.

---

## F. Found by the 1.7 dataset — 9 Nov 2026

The dataset scores every reading against the tolerances in `defectgen/configs/detect.yaml`. Most of
those are tagged **A** (assumed; §3.7.9 item 2). Where the generated geometry and two tolerances
cannot both hold, an in-spec part is rejected *correctly*, so the result is about the spec or the
CAD, not the detector. Write-up: `project_plan.md` §4B.6, "A strip that moves with the defect".

| # | Item | Kind | Blocks |
|---|---|---|---|
| F1 | Carrier 0.16 high lifts busbars past the 0.15 gap tolerance | tolerance contradiction | in-spec false-reject rate of `carrier_seating` (1.8) |
| F2 | Busbar 0 rests 0.5 mm high after an in-spec 0.1 mm Y shift | CAD check | in-spec false-reject rate of `busbar_shift` (1.8) |
| F3 | Narrow beads over-read in width | detector limitation | in-spec width false rejects (1.8) |
| F4 | Weld bow measure has ~no margin to its tolerance | detector limitation | false-alarm rate (1.8) |
| ~~F5~~ | ~~Arm cases run on one core: ~270 s per welded module~~ — **first pass done 12 Nov 2026**: 1.6–2× faster, results unchanged | speed | cheap reruns: 1.13 held-out set, any detector change |
| F6 | The arm column measured a flawed arm plan: FOD 60 %, in-spec welds rejected, wall dents 12/20 — **fixed 12 Nov 2026**, arm column to be rerun | coverage + detector | a fair fixed-vs-arm verdict (1.10) |
| F7 | The v2 arm rerun raised 1,126 false-alarm regions in 9 of its first 128 modules, plus 7 single ones — **fixed 13 Nov 2026**, rerun v2 stopped | detector | a false-alarm rate that means something; the v2 run was spending 30 h on it |
| F8 | F7's honest dim-ray threshold lost two fixed spatter drops: a region one ray short at 5σ — **fixed 13 Nov 2026** (region growing), gates running | detector | spatter detection rate on both setups (1.8) |

### F1 — Carrier seating vs busbar gap

**What it is.** A carrier sitting 0.16 mm high is in spec (carrier tolerance, assumed). It really
does lift the busbars it carries by 0.16 mm, which is past the 0.15 mm busbar-to-pad gap tolerance.
The detector measures both correctly. 5 of 10 in-spec `carrier_seating` cases are rejected for the
gap.

**To close.** Source both tolerances (D1, §3.7.9). If both hold, the carrier tolerance must be
tighter than the gap's; if not, one of them changes. It is a spec decision, not a fix.

### F2 — Busbar 0 lifts on an in-spec shift

**What it is.** `pod/busbar_shift/L0_0.1_s0`: busbar 0 shifted dy −0.10 mm (pose tolerance ±0.2)
comes to rest at z +0.506 mm, tilted 4.8°, and at dy −0.163. It is resting on something along its
−Y edge (≈ y −63.6). E7's 1.5 mm slot relief should prevent this, so something else is in the way.
The detector reads `busbar_liftoff` 0.50, which is right for the generated part.

**To close.** In Fusion, check what is within 0.1–0.2 mm of busbar 0's −Y edge (carrier rib,
terminal window, wall). Then either add clearance or record the real limit.

### F3 — Narrow beads read about 1.4× in width change

**What it is.** The sensor model's flank sampling makes a narrower bead read narrower still, so
in-spec ×0.84 beads can be rejected (2 of 10 in-spec `width_deviation`). An area/crown estimate
was tried (`width_change_area`, kept in the reports): it reads narrow beads better, but reads
+0.03 to +0.05 on clean beads and a −0.1 mm crown as +0.07 width. Kept the edge measure.

**To close.** A decision: accept and state it, or revisit with the welding follow-up
(`welding_followup.md`), which judges beads against their expected shape.

### F4 — Weld bow noise reaches its tolerance

**What it is.** On untouched seams the bow (sagitta) reading has p99.9 ≈ 0.098–0.106 mm against
`arc_tol` 0.10. The single false alarm in the 775-case fixed run was one of those (0.1009).

**To close.** A decision: accept about 1 per 1000 modules, smooth the bow measure, or source a
real seam limit (ISO 13919-2, §3.7.9).

### F5 — The arm column runs on one core

**What it is.** An arm case takes ~115 s before welding and ~260–290 s after: the bead geometry
roughly doubles every step. From the per-case `timings_s` in `arm.report.json` (welded module):
golden render ~100 s, residual ~70 s, measure ~35 s, align ~20 s, scan ~45 s. About 85% of the
wall time is the main thread (11 Nov: 356 CPU-min against ~38 per helper thread). Only the ray
casting uses the 32 threads, and the GPU (RTX 4060, 8 GB; torch+CUDA in the venv) sits idle. A full
arm column is ~1.5 days, so every detector change costs that again.

**What will not work.** Two arm jobs at once: one peaks at 8.4 GB RSS (10.7 GB with page cache) on a
16 GB machine. They would also share `_spill/` if they ever took the same case: that SIGBUS'd on 11 Nov.

**To do, after the 1.7 run (never mid-run: half a column from changed code is a mixed dataset).**
1. Profile one welded case (cProfile or `py-spy`; py-spy needs sudo on a live PID, ptrace_scope 1).
2. Run the 12 captures in parallel. They are independent through golden, align, measure and
   residual; a thread pool keeps one copy of the scene if the hot loops release the GIL.
3. Vectorise what the profile shows is Python-level looping in `detect.py`.
4. Optionally put the nearest-point and residual work on the GPU. It is float32, so it is not
   bit-identical: that is a full rerun of both columns, not a patch.

**Done when.** A sample of finished cases (both setups, every stage) reruns to identical
`score.json` and `report.json` apart from `timings_s` and `seconds`, and the welded arm case time
is measured again. For step 4: the 1.8/1.9 numbers are recomputed and unchanged within their intervals.

**First pass, done 12 Nov 2026.**
- **What changed.** `scan.line_scan` runs its profile chunks on a thread pool (`DEFECTGEN_THREADS`, default 8;
  the gain levels off at about 4). Each chunk keeps its own drop counts, merged in chunk order. The Open3D
  scene sits behind a lock, and Open3D threads it itself. The default chunk went from 400 to 100 profiles, so
  memory stays bounded; results don't depend on chunking (`test_scans_are_reproducible...`).
  `detect.Reference._cast` gathers two pose columns per point instead of a 4×4 copy (1.5 GB per 12 M-point
  sweep). Profiling showed that 166 of 293 s in a welded arm case was the line-sensor model's numpy, run
  for the scan and for every golden recast. Ray tracing itself was 4.5 s.
- **Speed.** Welded arm case 281–306 → 141–175 s. Pre-weld arm 110–118 → 69–91 s. Fixed 21–26 → 13–18 s.
  Peak RSS 8.4 → 8.0 GB.
- **Proof it changes no data.** Threaded vs serial sweeps on the welded module and every post-weld arm
  sweep plus the fixed pass: bit-identical, including drop counts and their key order. Eleven finished cases
  (pod at all three stages, clean post-weld, one GRR trial; both setups) rerun to identical `score.json`, and
  every report number within 1e-9. The largest difference was 3e-14, the same as rerunning the unchanged
  code, which isn't bit-reproducible: threaded reductions in Open3D/BLAS. New test:
  `test_threaded_sweep_is_identical_to_serial`. 179 tests pass.
- **Left for later.** The detector's own numpy (measure, fits, residual regions: ~95 s of a welded arm case)
  still runs on one core. Running captures in parallel would hold several 12 M-point captures at once. Not
  worth it at 16 GB for now. GPU: step 4 above, unchanged.

### F6 — The arm column measured a flawed arm plan

**What it was.** The first finished arm column (12 Nov 2026) beat the fixed station on cell and busbar pose
and weld arcs, but debris recall was 60 %, 10/10 in-spec reinforcement and 9/10 in-spec width welds were
rejected, and wall dents were found 12/20. None of these is a property of an arm. Each is a fault in the
plan or the detector, and the first run is what found them. It is archived as the "before" (`data/coverage_v1/`,
the first `data/dataset/`).

**Causes and fixes.**
1. *Coverage promised too much* (plan §4B.6). The FOD class used D = 1.0 mm for 0.4–0.6 mm particles, so a
   centre strip no sweep resolved counted as covered. Handling damage was judged with snapshot points too
   noisy for 0.1 mm dents. Fix: FOD D = 0.5; a class may set `noise_D` apart from its spacing D
   (handling damage: 0.1), `coverage.yaml`.
2. *Fast wall sweeps.* With the noise rule fixed, the plan grew to 92–105 s per stage. Each sweep now
   also has a 200 mm/s variant at the same 1 kHz (0.2 mm pitch). A sweep counts for a class only if its
   pitch ≤ D/2, so fast sweeps serve the walls (D = 2) and never cell or busbar pose (D = 0.2). Final
   plan: 15 / 13 / 15 captures, 51 / 46 / 63 s.
3. *Misregistration on steep surfaces.* The residual threshold assumed only noise, but X/Y/yaw come from
   the housing datum, good to half a sample. A surface moved sideways by d reads d·|n_h|/|n·a| along the
   ray. That is zero on a level top, d on a wall seen side-on (3,592 false regions from the side-on
   sweeps), and ~3 d on a cap's recess wall seen edge-on by a close-up snapshot (2 false regions).
   `tau ≥ misreg × that gain`, with the normal taken from the golden grid, steepest face at a crease
   (`detect.sideways_gain`). Tests: `test_sideways_gain_...`, `test_a_steep_face_moved_...`.
4. *In-spec welds read as defects.* The as-measured reference moves beads but does not reshape them, so
   an in-spec crown or width change stayed as residual. Residuals inside a judged seam's footprint now
   count as explained. Cost: spatter landing on a bead goes unreported. Reshaping the reference bead
   is in status_update's "Deferred to refine".

**Proof.** Clean modules through the final plan, every stage, placed exactly and with fixture error
(`python -m defectgen.regress arm`): 0 findings in 6/6. Before fix 3's second form: 2 false findings.
Every-class sensitivity through the arm: see the 1.7 rerun entry in status_update.

**Done when.** Both columns are rerun on this detector (fixed too: it shares the detector) and the
1.8/1.9/1.10 numbers are recomputed.

### F7 — The v2 rerun's arm false alarms: a bulge it did not expect, and dim rays

**What it was.** Paused at arm pod 127/565 (13 Nov 2026, 11:56). The fixed column had finished and
matched v1 case for case. The arm column had 1,126 false-alarm *regions*. They came from 9 modules with
a bulge (1,119 regions) and 4 separate noise events (7 regions, over clean, GRR and pod). Per module that is about 4 %, against ≤ 0.39 % for the fixed station. A per-region count overstates
it, since one bulge broke into 489 regions; the per-module rate is what a line feels.

**Causes and fixes.**
1. *A cell pushes the side plate out; the reference did not.* The generator bulges a 1.5 mm side plate
   outward wherever a shifted cell stands proud of it (`settle._bulge_side_plates`), which is physical. The
   detector's "module as measured" moved the measured cell but did not re-apply the bulge. An in-spec
   cell 0.09 mm toward a plate left a ~0.1 mm bulge as residual, about one threshold deep, so noise cut it
   into hundreds of 6–9-ray "fod" regions. Only the new side-on fast sweep (`sweep_x_y∓75.5_r±60_fast`) looks
   straight at the plates. v1's arm saw them only obliquely; 2 of its 8 false alarms sat on the side plate beside a shifted cell. Out of
   spec, the cell's own finding absorbed the regions, which is why only in-spec and near-tolerance
   shifts showed it. Fix: `Reference.as_measured_scene` re-applies the bulge for measured cell motion.
   Test: `test_the_module_as_measured_bulges_the_side_plate_a_measured_cell_pushes`. Worst case
   (`cell_shift L0_0.1_s0`): 489 → 0, measurement unchanged (0.1025).
2. *One noise figure per material, but dim rays are noisier.* Sensor noise scales with returned signal
   (shot noise). Polished aluminium seen away from its mirror angle returns little. In one snapshot, cap
   rays ran from σ 0.028 mm (bright) to 0.073 mm (signal < 0.1), against the material-wide 0.033 the
   threshold used. 5σ was about 2σ on the dim patches, so noise alone made 6-ray regions. Fix: per
   capture and material, fit σ² = a² + b²/signal to signal-binned robust σ (`detect.noise_vs_signal`,
   `residual.noise_bin_rays` = 2000) and threshold each ray at k·σ(signal). This only ever raises the
   threshold; the material σ stays the lower bound. Real sensors return the signal (Zivid SNR, Keyence
   intensity), so this needs nothing a station lacks. Test: `test_noise_follows_the_returned_signal_...`.
   The three cases that reproduced it: 1 → 0 each.

**Why the gates missed it.** The pre-launch arm gate ran `single` (one defect above tolerance) and
clean modules, and `subtol` only on the fixed setup. Its only cell shift was 0.36 mm, out of spec, so
the cell's finding absorbed the bulge. The v3 gates add `subtol` and `fixture` on the arm, and the L0–L2
(in-spec to just-out) seed-0 case of every tolerance family, every level of FOD, damage and spatter, and
the binary classes, on both setups: `data/gates_v3.sh`, logs `data/regress_logs/*_v3.log`.

**Done when.** The gates show no lost detections against v2 and no false alarms. Then both columns are
rerun on this detector (v2 archived to `data/dataset_plan_v2`).

### F8 — F7's dim-ray threshold lost two spatter drops: a region one ray short

**What it is.** The F7 gate on the fixed setup found 59 of 61 targeted cases unchanged. The other two,
`spatter L3_0.15_s0` and `L4_0.2_s0`, went from *partial* (1 of 5 drops found, as in v1 and v2) to *miss*.
The lost drop sat on copper, and its rays came back dim (signal 0.02–0.03, against ~0.96 around it).
F7's noise fit raised the threshold there from 0.050 to 0.10–0.12 mm, and the drop kept 5 rays over it,
one short of `min_points` (6). With the fit off, both cases return to *partial*.

**Which threshold was right.** The simulator's noise on those rays is ~0.044 mm (σ grows as
√(0.5/signal) below 0.5). So v2's 0.050 was ~1σ there and its find was luck, and F7's threshold is the
honest one, still under the true 5σ. The drop *is* there, though: its six rays are all well above the
noise taken together. A test that judges each ray alone throws that away.

**Fix: grow a region over weaker rays joined to a strong one** (`detect.residuals`, `residual.k_grow` = 3).
After the strict pass, a connected group of rays each over 3σ, *and* over every misregistration
allowance (those are bounds, not noise), that holds at least one ray over the full 5σ threshold and
overlaps no region the strict pass kept, is judged as a region by the same filters (min_points, area,
thinness, aspect). Every region the strict pass keeps is unchanged, so this can only add findings.
In noise alone a 5σ ray (one-sided p ≈ 3×10⁻⁷) with five connected neighbours each over 3σ (p ≈ 1.3×10⁻³,
at most 8 neighbours each) is ~10⁻¹⁰ per capture. The risk is systematic residuals, not noise, and the
gates measure that. `stats["regions_grown"]` counts the added regions per capture. Test:
`test_a_region_one_ray_short_at_5_sigma_is_kept_on_its_3_sigma_neighbour`. Spatter L3/L4/L5 on fixed:
back to *partial*, 0 false alarms.

**Done when.** `data/gates_f8.sh` (the F7 gate groups, plus the 8 other arm modules v2 had false alarms
on) shows no lost detection against v3's gates and no new false alarm. Then v3 launches on this detector.

**Why this keeps happening, and the guard (13 Nov 2026).** Each fix moves the trade-off between
finding defects and raising false alarms. F7 raised a threshold to kill false alarms and cost a
real find, so a fix has to be checked against every past failure, not only its own. The checks were
rebuilt by hand for each fix and compared by eye. They are now a committed list,
`defectgen/known_cases/cases.yaml`: 138 cases, each with its accepted outcome and false-alarm count
and the fix-list item it came from. `run.py` judges them PASS / BETTER / FAIL. A full pass records a
fingerprint of the detector's code and configs. `data/chain_v3.sh` and `cloud/run_remote.sh` refuse to
start without a pass for the code as it is. Tested on old results: v3-without-F8 fails exactly the 2
spatter cases, and v2 fails 12 of the 13 F7 cases.

