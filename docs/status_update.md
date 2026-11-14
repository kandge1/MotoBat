# MotoBat — Status and Task Order

**One question this file answers: what do I do next?**

`project_plan.md` is the reference — the reasoning, the maths, the decisions.
It is long on purpose. This file is the checklist that runs off it. When they
disagree, `project_plan.md` is right about *why* and this file is right about
*when*.

Section references like §4B.1 point into `project_plan.md`.

*Last updated: 12 Nov 2026 — first pass of Phases 1–3 before refining; deferred items listed under
"How work is sequenced". (6 Nov: 1.4 detection v1; Phase 2 opened as a parallel lane.)*

---

> **6 Nov 2026: Phase 2 now runs in parallel, in a second git worktree.**
> `../MotoBat-arm` on branch `feature/hc10dt_gazebo` takes the Phase 2 tasks that need nothing
> from Phase 1: 2.1, 2.2, 2.4 and 2.8, plus 1.6 (`motobat_interfaces`, ROS-only). This folder
> stays on Phase 1. The lanes, which docs each one edits, and the worktree setup are under
> "How work is sequenced" below and in `project_plan.md` §8.

> **8 Nov 2026: tasks 1.4 (`2001f72`) and 1.5 (`0f002a9`) are done; current task is 1.7, the
> labelled dataset.** `python -m defectgen.detect data/<run> --stage <stage>`, findings in RViz
> with `show_detections.py` (WORKFLOW.md, "Detection").

> **Tasks 1.1 and 1.2 are done (3 Nov 2026).** `defectgen/` emits all three stages from the real
> CAD and injects 13 of the 15 in-scope defect classes. 85 tests passing. Current task is now
> **1.3 — the three ray-cast sensor models**. Two CAD re-exports are outstanding in fixlist §E;
> neither blocks 1.3.
>
> **To look at the output:** `python -m defectgen --out data/preview_clean` and
> `python -m defectgen --out data/preview_defective --demo-defects` write three STLs plus a
> labels JSON each. Open them in Fusion (Insert → Insert Mesh) alongside `module_assembled`, or
> double-click for the Windows 3D Viewer. The demo severities are exaggerated on purpose —
> real ones are tenths of a millimetre and invisible by eye.

## ⏭ QUEUED — after fixlist E6 and E7 are closed (4 Nov 2026)

Agreed with the user, in this order. Fixlist first: settling results are only
meaningful once the busbar slots have their clearance (E7) and the standalone
CMC and carrier match `module_assembled` (E6).

1. ✅ **Gap → weld consequence** (4 Nov 2026, `weld.py`). The gap is measured
   under every beam sample after settling, so a busbar tilted on a chip gives a
   different gap at each terminal. The weld follows from it: underfill and toe
   undercut above `weld_gap_ok`, `lack_of_fusion` (internal) above
   `weld_gap_fusion`, `burn_through` above `weld_gap_hole`. Each consequence
   carries `cause`. Causes propagate through settling by contact
   (`plan.influence`), so a weld on a busbar resting on a carrier lifted by a
   backwards cell names `reversed_polarity`. The thresholds are still assumptions
   and must be sourced for **Cu-on-Al**.
2. ◐ **Realistic weld bead — first version done** (4 Nov 2026). Beads are no
   longer boxes. Each is built from a **beam trace**: the commanded stitch plus
   robot drift (`weld_path_drift`), an arc (`weld_arc`) and circular beam
   wobble (`beam_wobble`, a process setting). Energy deposited by a Gaussian
   spot along the trace sets the melt. The spot size is derived so a straight
   pass melts exactly `weld_stitch_w`. Crown, ≈10 µm ripples and an end crater
   follow from it, on a 50 µm heightfield in the busbar's frame. **Still to do:**
   a laser power / speed parameter, so wobble and slow passes change heat input
   (now constant line energy), and the remaining ISO 13919-2 surface classes
   (humping, surface pores, crater cracks). **Next, using the MOTOPAC-RL2D's
   numbers** (`project_plan.md` §4A.7): 320 µm spot, wobble up to 500 Hz, a 25 mm
   field, and **power modulated along the trace**, even within one wobble
   circle. Then test Yaskawa's gap-tolerance claim: does wobble widen the
   `weld.py` gap window?
3. **Heat tint and soot — copper busbars** (decision and sources: `module_spec.md` §5, "Busbar material"; make the material a config parameter). Model heat input → peak temperature
   field (moving heat source) → oxide thickness (oxidation kinetics) → colour
   (thin-film interference; Cu₂O dark red, CuO black). Because tint and
   burn-through share a cause, **too much heat produces both**: an over-heated
   zone shows heavy oxidation colour *and* burn-through/excessive penetration,
   labelled as one cause with two symptoms. Tint is a colour effect (Zivid colour
   channel, RViz vertex colours), not profilometer geometry.
   **Corrected 4 Nov 2026** (`cu_on_al_laser_welding.md` finding 2): on our 4 mm Al
   pad, overheating does not burn through the way it does in foil studies. Its
   second symptom is **excess Al melting and intermetallic growth**, which is
   internal. So: overheat → Cu tint (visible) + `excess_penetration` / IMC
   (internal label). The **hole** stays tied to the *gap* (melt dropping
   through the top sheet), as `weld.py` already models it.
4. ◐ **Materials library and specular dropout — first version in `scan.py`**
   (4 Nov 2026): reflectance per material and band, and dark/glare dropout.
   Micro-roughness (Ra ≈ 0.2–0.5 µm) sits below every sensor's Z noise, so it
   enters only through the lobe width `alpha`, not as geometry. Still open:
   measured BRDFs (RGL-EPFL / MERL) in place of the assumed `kd`/`alpha`/`r0`,
   multipath inside weld beads, and the heat-tint colour layer (item 3).

Parked, not scheduled: injection-moulding defects in the carrier/cover
(short shot, warpage, sink, flash) as a defect source.

Parked for a follow-up project (9 Nov 2026): **process-driven welds** — weld outcomes caused
by scanner speed, power, wobble, seam-position error, part vibration and gap; spatter as
time-ordered events (landing ahead of the beam, on the bead, in a burn-through); heat tint.
Design sketch, research digest (Path Robotics' public footprint, open weld datasets, Cu/Al
battery-welding literature, physics models) and first steps: [`welding_followup.md`](welding_followup.md).
MotoBat keeps its idealised weld defects and the 1.7 baseline dataset.

## ▶ CURRENT TASK

> **Task 1.7 — the labelled dataset (§4B.6).** Modules with one injected defect each, swept
> over size around each class's tolerance, several seeds per (class × size), scanned by the
> fixed Ranger3 pass and by the arm's 1.5 view plan, each scan re-fixtured with a random
> pallet error. Scored with the out-of-spec rule (a label below tolerance is not a defect;
> flagging it is a false reject). Three tiers: `pod` (P/R, POD, fixed vs arm), `clean`
> (false alarms per module) and `grr` (%GRR, AIAG crossed design). Seeded and regenerable;
> outputs in `data/dataset/`, not committed.
>
> Design: `project_plan.md` §4B.6 "1.7 dataset". Lives in `defectgen/dataset.py`,
> `configs/dataset.yaml`. Heavy compute: one heavy `defectgen` job at a time across lanes.
>
> State, 9 Nov: the fixed setup is done for all three tiers (results in §4B.6 "Fixed run").
> Arm hardening is done (§4B.6 "Arm hardening": registration from the housing strips, once
> per load). The fixed re-run on it then exposed a datum fault (a side strip that bows
> with a shifted cell; §4B.6 "A strip that moves with the defect"), now fixed, so the whole
> dataset is re-running on the fixed detector: arm `clean`, fixed column, arm `pod`
> (`data/chain_datum_fix.sh`, progress in `data/PROGRESS.md`). 1.8 (P/R with intervals,
> reading error) and 1.9 (%GRR) read the stored results and are built alongside the runs.

**Scope changed on 15 Sep: from inside a single cell to a battery module
assembly line.** Cell-internal defects are buried under anode and separator,
so no external optical sensor can see them. Even an LJ-X8020 has the
resolution but not the line of sight. Module-level defects are on the surface,
come with real tolerances (a > 0.1 mm height difference between adjacent
terminals can cause a flawed weld), and give all four robots a job. The
question MotoBat now answers is **whether a robot-mounted multi-view sensor
finds what a fixed line-scan misses, and what it costs in cycle time.** Full
reasoning, numbers and sources: **`project_plan.md` §3.7.**

---

## How work is sequenced

**There is no fixed deadline (15 Sep).** Work is ordered by dependency, and
each phase unblocks the next:

1. **Phase 1, numpy:** generator → defects → three sensor models → detection →
   coverage and view selection → dataset → metrics → correction-loop K-sweep
2. **Phase 2, HC10DT in Gazebo:** MoveIt, flange, reachable view plan, real
   motion time, fixed vs arm with cycle time
3. **Phase 3, the line:** GP7 places cells, GP4 places the busbar carrier,
   GP25 + MOTOPAC-RL2D welds, the loop is closed in simulation, microScan3
   safety zones
4. **Phase 4:** optional Isaac Sim fidelity, then the write-up and demo

**Never trade away** the Phase 1 metrics or the fixed-vs-arm comparison.
Everything else is presentation around them.

### Two lanes, from 6 Nov 2026

Phase 1 and Phase 2 are only partly sequential. Several Phase 2 tasks need nothing that
Phase 1 produces. Those tasks are now worked in parallel, in a second
[git worktree](https://git-scm.com/docs/git-worktree): the same repo, a second folder, its own
branch. Each lane has its own VS Code window and its own Claude Code session.

| Lane | Folder | Branch | Tasks |
|---|---|---|---|
| Phase 1, numpy | `~/projects/MotoBat` | `feature/defect_generation_and_detection` | 1.4 onward (`defectgen/`) |
| Phase 2, ROS | `~/projects/MotoBat-arm` | `feature/hc10dt_gazebo` (from `315bf90`) | 2.1, 2.2, 2.4, 2.8, and 1.6 |

**What stays sequential:** 2.5 needs 1.5's viewpoints and 2.1. 2.6 needs a decimated mesh
export from `defectgen`, so that export is Phase 1 lane work. 2.7 needs 2.5 and 1.10.

**Rules that keep the lanes from colliding:**
- **Docs:** the Phase 1 lane owns this file, `project_plan.md` and `WORKFLOW.md`. The ROS
  lane edits only the Phase 2 rows here and §4A of the plan. Anything else it needs to record
  goes in its commit messages or a note that is folded in at merge.
- **Code:** the ROS lane does not touch `defectgen/`. The shared venv's editable install points
  at the Phase 1 folder, so `python -m defectgen` always runs that folder's code.
- **Simulation:** only one lane runs RViz, Gazebo or a heavy `defectgen` job at a time (16 GB
  of RAM, and both lanes use `ROS_DOMAIN_ID=42`).
- **Priority:** the Phase 1 metrics still come first. The ROS lane uses time that Phase 1
  spends waiting on compute (the 1.7 dataset, the 1.11 sweeps).
- **Merging:** merge the Phase 1 branch to `main` first. The ROS branch's PR then shows only
  its own commits.

### First pass, then refine (11 Nov 2026)

**The plan:** take Phases 1, 2 and 3 to a working first pass, end to end. Then come back for
what was skipped. Phase 4 waits: it has no cloud budget, and 4.1 (Isaac Sim) was already
optional. The write-up, demo and preprint (4.2–4.5) cost nothing and follow the refine pass.

A first pass is **done** when the task runs, has a result, and states its limits. It is not
done when the result is polished. Anything cut to get there goes in the table below, with
why, so a skip is a choice on the record and not a gap found later.

**Estimates (11 Nov, evenings and weekends):**
- **Phase 1:** first pass by about Sun 15 Nov. Mon 16 Nov if the arm snapshot false alarms
  need a detector change, which means re-running the arm column.
- **Phase 2:** about 4–5 more days. It shares the 16 GB with the Phase 1 runs, so it ends
  mid-to-late next week.
- **Phase 3:** about 13–17 working sessions, roughly 3–5 weeks.

**Deferred to refine.** When an item is picked up, strike it through with the date, as in
the fixlist.

| Item | Why it can wait | Where |
|---|---|---|
| 1.13 held-out validation set (and the optional learned classifier) | First-pass numbers come from the dev set. This set checks they hold on fresh seeds. ~2 days of compute. | plan §4B.6, "Later" |
| 1.11 full sensitivity sweep (noise, spacing, standoff) | A cut-down sweep on the fixed setup answers "which sensor class". The full sweep is days of compute. | plan §4B.6 |
| Dataset speed, beyond the first pass (F5: detector numpy on more cores, GPU) | First pass done 12 Nov (1.6–2×, results unchanged). The rest needs more RAM or a full rerun. | fixlist F5 |
| Tolerance sourcing (D1, every **A** in `detect.yaml`) | In-spec/out-of-spec labels rest on assumed tolerances. Results are stated as conditional on them. | fixlist D1, plan §3.7.9 |
| Geometry/tolerance contradictions (F1, F2) | Spec or CAD decisions, not detector work. | fixlist F1, F2 |
| Detector limits (F3 narrow-bead width, F4 bow noise) | Measured and stated. Fix only if the result needs it. | fixlist F3, F4 |
| Arm assumes exact robot poses | Robot pose error is not modelled in 1.7. Add it after 2.5 gives real poses. | plan §4B.6 |
| Arm snapshot false alarms (bowed plate, foam), **if not fixed in the first pass** | Only if the fix forces a rerun that would block 1.10. | plan §4B.6 |
| Reshape the golden bead to the measured crown and width | First pass (12 Nov): residuals inside a measured seam's footprint count as explained by the bead. That stopped in-spec reinforcement/width welds being rejected (10/10 and 9/10 on the arm). Reshaping the reference bead is the proper fix. | detect.py `_regions`, plan §4B.6 |
| Spatter on the bead itself | A side effect of the line above: spatter landing inside a seam's footprint is not reported. Spatter beside the bead still is. State it as a limit, or fix it with the reshaped bead. | detect.py `_regions` |

---

## Phase 0 — Setup, repair, restructure ✅ COMPLETE

Branch `feature/restructure`. This phase exists so the rest of the project is not
spent fighting the environment.

| # | Task | Status |
|---|---|---|
| 0.1 | Ubuntu 24.04 + ROS 2 Jazzy, MoveIt2, Gazebo, RViz2 | ✅ verified |
| 0.2 | Diagnose 8 GB ceiling → dead DIMM slot A, not bad RAM (§1.0) | ✅ |
| 0.3 | Dell warranty transfer + onsite motherboard replacement | ✅ 31 Aug |
| 0.4 | Verify 16 GB dual-channel (`free -h` → 15 Gi) | ✅ |
| 0.5 | Repair PyTorch CUDA stack (§1.1) | ✅ torch 2.6.0+cu124, matmul verified |
| 0.6 | Fix `fix_torch.sh` name-guessing bug that hid the CUDA 13 strays | ✅ |
| 0.7 | Restore numpy 1.26.4 pin after torch pulled 2.x (§1.3) | ✅ |
| 0.8 | Pin SB3 2.5.0 / gymnasium <1.1 to match torch 2.6 | ✅ |
| 0.9 | Repo root becomes the colcon workspace; `src/` holds packages | ✅ |
| 0.10 | Install logs → `logs/`, gitignored | ✅ |
| 0.11 | `motobat_env.sh` version-controlled at repo root, self-locating | ✅ |
| 0.12 | Installer stops generating the env file / empty packages | ✅ |
| 0.13 | Package naming + layout rules written (`project_structure.md`) | ✅ |
| 0.14 | `bootstrap_rviz.sh` builds from repo root; xacro verified (§1.2) | ✅ 39 links / 38 joints |
| 0.15 | Fix `robot_description` YAML-parse crash in both launch files (§1.4) | ✅ |
| 0.16 | Headless verify: `/joint_states` 50 Hz, 24 dynamic + 14 static TF frames | ✅ |
| 0.17 | Four arms in RViz, sliders drive all 24 joints independently | ✅ 4 Sep |
| 0.18 | `dmidecode` confirms DIMM A + B, distinct serials — slot A alive | ✅ 4 Sep |

**Phase 0 is complete. Nothing is deferred and nothing is outstanding.**

Closed out: `feature/restructure` merged to `main` (PR #1). Phase 1 runs on
`feature/defect_generation_and_detection`.

---

## Phase 1 — §4B: module generation, detection, coverage, correction ◀ NOW

**Pure numpy / trimesh / Open3D. No ROS, no simulator.** This is the core of
the contribution.

| # | Task | § | Output |
|---|---|---|---|
| 1.0 | Resolution gate + scope decision | 3.7 | ✅ 15 Sep, in `project_plan.md` |
| 1.1 | Mesh-native module generator, three stages, `(mesh, labels)` seam | 4B.1 | ✅ 1 Nov 2026 — `defectgen/`, 65 tests passing. Loads the real CAD, normalising each mesh into the §7 frame on the way in. Terminal placement corrected 3 Nov against the Fusion `cell_assembled` (fixlist E1); the module is 151.0 wide. |
| 1.2 | Defect injection: cell pose, terminal height/tilt, missing cell, reversed polarity, busbar pose/gap, weld seam, adhesive bead, FOD | 4B.2 | ✅ 3 Nov 2026 — `defects.py`, 13 of 15 in-scope classes. Each injector is tagged with the §4B.2 stage it first applies at, so one recipe describes all three stages. **Not done:** class 10 (gap-filler bead — no pre-cell stage exists yet) and class 12 (insulation film — deliberately never modelled as geometry, `module_spec.md` §3). |
| 1.3 | Three ray-cast sensor models: fixed Ranger3 line-scan, wrist Zivid-class snapshot, wrist LJ-X8200-class profilometer. Both occlusions, dropout, noise; parameters **frozen and hashed before any results**. | 4B.3 | ✅ 4 Nov 2026 — `scan.py`. Two models: laser-line triangulation (LJ-X8200 and Ranger3 share it) and structured-light snapshot (Zivid). Each covers both occlusions, the range window, a grazing-angle cut, and per-material reflectance (`configs/materials.yaml`: diffuse plus Beckmann lobe per band), giving dark and glare dropout on bare Cu/Al. Noise grows as the returned light weakens and with distance². Line-sensor exposure falls as the profile rate rises (up to each sensor's max rate). Height is quantised. The profilometer triangulation angle is **22.0°, measured** from the Keyence model's receiver-window tilt in the flange STEP (was an assumed 23°). **Frozen: params hash `779e2255dc4c`**, pinned in `tests/test_scan.py` and written into every scan. Stated limitations (multipath, speckle, motion during capture, sub-surface scattering, thermal drift): `project_plan.md` §4B.3. Also: Zivid 2D quadrant photos (`--sensor photos`, display render, not frozen) and per-run `data/scan_logs/`. |
| 1.4 | Detection: align → ICP → per-feature fits within one capture → residual segmentation → classify | 4B.4 | ✅ 7 Nov 2026 (`2001f72`) — `detect.py`, `fit.py`, `evaluate.py`, `configs/detect.yaml`; maths and measured findings in `project_plan.md` §4B.4 "As built". Every scan is compared ray by ray with a **golden capture** (the same rays cast at the clean module through the same frozen sensor model), and every feature is reported as f(scan) − f(golden). **Alignment registers to the housing, not the whole module.** Whole-module ICP was measured to fail: a 0.3 mm carrier lift dragged the fit 0.3 mm, and XY is barely constrained from above. Every in-scope class except `cover_misalignment` (final stage, not scanned) has a detector, and the clean module gives zero findings at all three stages. `ScanResult` now carries the organised ray index (no numbers changed; hash still `779e2255dc4c`), so **older scans must be rescanned**. Second pass (6 Nov): residuals are now taken against the module **as measured** (every measured part moved to its measured pose and recast), which removed the false alarms parts moving *within* tolerance caused (a 0.1 mm carrier gave 5,321); a capture measures a position only if its sample spacing is ≤ tolerance/2 (§3.7.3), so the Zivid now reports cell/busbar/seam positions as uncovered rather than wrong; scans record what they scanned and stale ones are skipped; reports carry OK/NOK/INCOMPLETE. Open: seam limits from ISO 13919-2 (not sourced); labels below tolerance score as FN (fix in 1.7); weld drift on the line-scan is marginal under a fixture error (one clean stitch 0.144 vs 0.10). 143 tests pass. Not yet: P/R with intervals over a seeded dataset (1.7–1.8). |
| 1.5 | Coverage metric + greedy view selection (no kinematics yet) | 4B.5 | ✅ 7 Nov 2026 (`0f002a9`) — `coverage.py`, `configs/coverage.yaml`, `show_coverage.py` (RViz). Coverage is judged per ~1.5 mm patch against both halves of the §3.7.3 rule (spacing ≤ D/2, noise ≤ D/3), counted from the frozen sensor models. **Fixed line-scan: ~100% of up-facing features, 96% of weld seams, 80% of spatter, but only 9–25% of handling damage (housing walls). Arm plan (8–12 captures): 98–100% on every class but spatter (93%).** The view plan (`data/coverage/<stage>.plan.json`) is ready for 2.5. Found and fixed a tilted-snapshot bug in `scan.py` (no earlier result affected). Results: `project_plan.md` §4B.5. **Corrected 12 Nov 2026** (fixlist F6, after the arm column missed what coverage said it saw): FOD D 1.0 → 0.5 mm, handling damage judged at noise D 0.1 (`noise_D`), and a sweep counts for a class only if its profile pitch ≤ D/2. Every sweep also has a 200 mm/s variant (0.2 mm pitch) for the walls. Final plan: 15 / 13 / 15 captures, 51 / 46 / 63 s; walls 89–92 % (fixed 9–25 %), FOD 99.7–100 %. Old plan in `data/coverage_v1/`. |
| 1.6 | `motobat_interfaces`: `DefectReport`, `ScanResult`, `CorrectionCmd` | §7 | first ROS package. **ROS lane** (6 Nov 2026): it needs no `defectgen` code, and 1.4's output now fixes what a report contains |
| 1.7 | Labelled dataset, **several seeds per (class × severity) cell** | 4B.6 | ◐ 9 Nov 2026 — `dataset.py`, `configs/dataset.yaml`; design, pilot and results in `project_plan.md` §4B.6 "1.7 dataset". **Fixed setup done:** `pod` 565, `clean` 30, `grr` 180 cases (seeded; outputs in `data/dataset/`, not committed). No false alarm in any of the 775. Out-of-spec: 343 hit, 17 partial, 27 miss, 8 not covered, of 395. In-spec: 8 false rejects of 120. Most families are found on every seed from 1.1–1.3 × tol. Weak spots, as measured: line-scan X quantisation (cell/busbar position, cell yaw), crown sag reads short, spatter on and beside beads is not seen (bead band), a flush weld reads as `missing_weld`. Scoring: a miss where the setup does not look by design (1.5's coverage map) is `not_covered`; `--rescore` re-scores stored reports. Fixed on the way: a zero crown divided by zero in `weld._shape`. **Arm hardening done (9 Nov):** the arm's false alarms were registration, not oblique-view detection (placed exactly, every arm view compares clean). X, Y and yaw now come from four housing strips (end- and side-plate tops, each trusted across its width only), solved once per pallet load from each strip's best whole view; this also removed a 0.045 mm bias in the fixed pass's end-plate datum. Clean modules through the arm plan with a fixture error: 0 findings at all three stages (was 6 / 6 / 50); `regress` gained an `arm` group, `probe_arm.py` shows per-capture registration. Assumes exact robot poses. **Fixed column re-run on the strip datum** (old results in `data/dataset_prestrips/`; this run in `data/dataset_strips_v1/`): 0 findings on clean modules, 1 false alarm in 775 (a weld bow 1 % over tol on an untouched seam: the bow measure's noise tail), in-spec short welds rejected 5/10 → 0/10. **Then a datum fault, found by the one failing test:** a cell shifted toward a side plate bows the sheet, the side strip moved with it, and the datum subtracted part of the cell's own shift (11 % in the fixed run, 33 % through a part view). Fixed by a robust per-bin centreline per strip (`detect._strip_offset`); §4B.6 "A strip that moves with the defect". **Re-run on the fixed detector, done 12 Nov 2026**: fixed `clean`/`grr`/`pod` and arm `clean`/`pod`, one detector for both columns (`data/validate_pr_{fixed,arm}_final.txt`). Fixed: 0 false alarms in 775. Arm: 0/30 clean modules with a finding, 8 false alarms in 595, all on snapshots. That arm column measured a flawed arm *plan*, not an arm: FOD 60 % (a centre strip no sweep resolved), 10/10 and 9/10 in-spec reinforcement/width modules rejected, wall dents 12/20. **Fixed 12 Nov 2026** (fixlist F6): coverage sizes and fast wall sweeps (row 1.5); misregistration allowance from the surface slope under each ray (`detect.sideways_gain`); residuals on a judged bead count as explained. Regression through the new plan: clean modules 0 findings at every stage, placed and with fixture error; one defect of every class 23/23 found, 0 false alarms. **Both columns to be rerun** (`data/chain_v2.sh`; first run archived to `data/dataset_plan_v1/`), now with the arm `grr` tier. Arm ~2.5–3 min per case: ~1.5 days. Detector changes found by the pilot (drift window, discontinuity band, memory) re-verified with `python -m defectgen.regress` (now in the repo). 162 tests pass. **v2 stopped 13 Nov 2026 at arm pod 127/565** (fixlist F7): its fixed column was complete and equal to v1 case for case, but the arm column had 1,126 false-alarm regions. Two causes. (1) The module as measured did not bow the side plate that an in-spec cell shift bows, which the new side-on sweeps saw. (2) Snapshot noise grows on dim rays (σ 0.07 vs 0.03 mm). Fixed: `as_measured_scene` re-applies the bulge, and `detect.noise_vs_signal` fits σ² = a² + b²/signal per capture and material. Gates (`data/gates_v3.sh`): 184 tests pass; fixed regress groups identical to v2; arm clean 6/6 with 0 findings, subtol 0 FP, fixture all found with 0 FP, single 23/23 with 0 FP; 61 targeted arm pod cases with 0 false alarms in 60 (1 in a 0.05 mm handling-damage case that v1 had 4 in), and 4 cases now found that v1 missed. **v3** (`data/chain_v3.sh`, arm first, `data/tripwire.py` pauses on a false-alarm spike). Dataset 1 is ideal-world by design (plan §4B.3). Cloud runs: `cloud/README.md` (Vultr, `--shard i/N`). |
| 1.8 | **Precision/recall per class, with confidence intervals** | 4B.6 | ◐ 9 Nov 2026 — `validate.py pr`; method in `project_plan.md` §4B.6 "1.8 and 1.9". Recall (conditional on coverage), module decision, false-reject rate (same class, and module decision), precision per finding class, false alarms per module (Poisson bound), all with Wilson 95 %. First look, on the earlier fixed run, already paid off: in-spec carriers at 0.16 mm are rejected because they really do lift the busbars past the 0.15 mm gap tolerance (the two assumed tolerances contradict each other), and in-spec short welds were rejected because their bare bead ends read as dents (a detector fault, fixed). Now also the reading error per family (|reading| − size: mean, sd, slope over size), which is what exposed the datum fault (cell shift slope +0.018 → −0.081). Also found: an in-spec 0.1 mm busbar Y shift makes the generator rest busbar 0 0.5 mm high (a second geometry/tolerance contradiction, to check against CAD). Numbers after the re-run. |
| 1.9 | **%GRR** for terminal height, cell position, busbar gap | 4B.6 | ◐ 9 Nov 2026, fixed setup — `validate.py grr`, AIAG ANOVA method, 10 parts × 3 pallet loads × 2 trials. %GRR to tolerance: terminal height **1.5 %**, busbar gap **0.5 %**, cell position X **56 %** (AIAG: > 30 % not acceptable). Cell X is the line-scan's 0.1 mm profile pitch, varying with each load's sampling phase, not noise (EV 7 µm, AV 37 µm). Arm column to follow. |
| 1.10 | **Fixed vs arm: coverage % and recall per class** | 4B.6, 3.7.6 | the central comparison |
| 1.11 | Minimum detectable defect per class; sensitivity to noise, point spacing, standoff | 4B.6 | → which sensor class the flange needs |
| 1.12 | **Correction loop in numpy: K-sweep, open- vs closed-loop placement defect rate, reject/rework logic** | 4B.7, 9.9 | the headline control result |
| 1.13 | Held-out validation set (new seeds, detector frozen), then optional learned region classifier (rules vs gradient-boosted trees on held-out data) | 4B.6 | planned 10 Nov 2026: §4B.6 "Later: a held-out validation set" and "Optional, after the held-out set". Report 1.8–1.12 from the held-out set. |

Tag the commit that produces 1.8–1.12: `git tag -a v0.1-detection-baseline`.

**Source before claiming results** (§3.7.9): ~~terminal and busbar
dimensions~~ (**resolved** — vendor drawing 26 Sep, as-built busbars 22 Oct,
`module_spec.md` §4–5), the cell-placement / busbar-gap / weld-seam
tolerances, and a module-line cycle time. Confirm the 0.1 mm terminal figure
from the MDPI paper itself.

**Do not** let this phase acquire a ROS dependency. A reviewer must be able to
`pip install -e defectgen && pytest` without touching ROS 2 (§2 of
`project_structure.md`).

---

## Phase 2 — HC10DT inspection in Gazebo

> **Running in parallel from 6 Nov 2026**, in `../MotoBat-arm` on `feature/hc10dt_gazebo`
> (see "Two lanes" above). Open now: **2.1, 2.2, 2.4, 2.8**. 2.3 is already in progress
> (Fusion). 2.5–2.7 wait on Phase 1.

| # | Task | § | Notes |
|---|---|---|---|
| 2.1 | HC10DT MoveIt config — **KDL, not OPW** | 4A.2 | `joint_5_b` offset 162 mm in Y; wrist axes do not intersect. No OPW file exists. Do not hunt for one. |
| 2.2 | `ros2_control` + `joint_trajectory_controller`, HC10DT in Gazebo | 4A.3 | |
| 2.3 | Flange: snapshot 3D + profilometer (**+ 2D camera only if 1.11 requires it**), poses as variables | 4A.4, 3.7.3 | **◀ IN PROGRESS, ahead of sequence.** `inspector_flange` exists, with HC10DT flange alignment features, carrying the Zivid 2+ M60 and the Keyence LJ-X8200 on an AL-5052 plate. Keep it **parametric** (sensor poses as Fusion User Parameters) so 1.11's result can swap the sensor class without a remodel. Check FOV occlusion and standoff compatibility. **Payload ≈ 2.5–3.0 kg of 10** (Zivid 1.0 + Keyence 1.2 + plate + cabling) — take sensor masses from datasheets, not from the CAD shells. Flange is the **DT** pattern, 63 mm PCD (§4A.4). |
| 2.4 | `inspector_flange.xacro` — flange + sensor frames on `s3_tool0` | 4A.4 | derive origins from CAD **numerically**; a 2° eyeball error corrupts every registration downstream and will look like an algorithm bug |
| 2.5 | Reachable view plan: Phase 1 viewpoints filtered by IK and collision, ordered to minimise motion | 4B.5 | the arm earns its place here. Must account for: **two different standoffs** (Zivid ≈ 600 mm nominal, 300–1100; LJ-X8200 ≈ 245 mm) implying separate poses per sensor; the **≈ 39° wall-occlusion limit** on outer terminal edges (`module_spec.md` §5); and housing-to-housing occlusion. **Needs 2.1 first** — scan poses cannot be checked for reach or collision without the HC10DT MoveIt config. |
| 2.6 | Module meshes in Gazebo; depth camera + narrow-fan `gpu_lidar` as the sensors; fixed line-scan station alongside | 4B.8, 4A.3 | The CAD exists, so this loads real meshes. **Gazebo cannot load STEP** (STL/OBJ/DAE/glTF only), and **STL has no units while Gazebo assumes metres** — every mesh tag needs `<scale>0.001 0.001 0.001</scale>` or the module is 1000× too large. **(4 Nov 2026)** A defectgen module is ≈ 1 M triangles, mostly weld beads (`weld.py`). Gazebo needs a decimated or plain-bead export, with the fine bead detail left to the numpy sensor models. Gazebo also cannot drop points on specular copper, which strengthens §3.7.8 (fidelity lives in Phase 1). |
| 2.7 | **Fixed vs arm with real cycle time** (motion + capture per module) | 3.7.6 | completes the central comparison |
| 2.8 | Verify TF tree: `ros2 run tf2_tools view_frames` | 4A.4 | |

---

## Phase 3 — The line: GP7 cells, GP4 carrier, GP25 weld, closed loop, safety

> **Welder decided 4 Nov 2026: GP25 + MOTOPAC-RL2D300-GP25JP** (`project_plan.md`
> §4A.7). The GP4 can't carry any laser head (4 kg payload; the lightest head
> found is 5.5 kg). The GA50 was rejected as a path-accuracy arm for large parts,
> because here the scanner draws the seam.

| # | Task | § | Notes |
|---|---|---|---|
| 3.1 | GP7 MoveIt + OPW. **Derive the OPW parameters from the URDF** (none ship, verified) and check them against forward kinematics. | 4A.1 | |
| 3.2 | GP7 cell pick-and-place, located by a fixed overhead 2D camera | 3.7.3 | same pattern as the myCobot vision pick-and-place |
| 3.3 | GP4 MoveIt + OPW (vendor file exists); busbar-carrier, CMC and cover placement | 4A.1 | ≈ 0.3 kg loads |
| 3.3b | GP25 MoveIt + OPW **derived from the URDF** (none ships); MOTOPAC-RL2D scanner as the end effector; the weld is a scanner pattern drawn while the arm holds | 4A.7 | payload 18.3 / 25 kg before fibre and dress |
| 3.4 | Close the loop in simulation: `DefectReport` → `CorrectionCmd` → next cell's placement | 9.9 | Labels now carry `cause` (`weld.py`), so a reject at the pre-weld gap check can be credited with the weld defects it prevented |
| 3.0 | `motobat_line.urdf.xacro`: replace the second GP4 with the **GP25** (welder, §4A.7), giving GP7 + GP4 + HC10DT + GP25; rebuild, relaunch headless | 4A.7 | `motoman_gp25_support` is vendored |
| 3.5 | microScan3 safety zones → HC10DT speed and separation monitoring | 3.7.3 | ISO/TS 15066; this is the controls-and-automation showpiece |
| 3.5b | **Laser cell safety.** The GP25 carries an 8 kW infrared fibre laser: an enclosed, interlocked cell (ISO 11553). The collaborative HC10DT cannot share space with people while the beam is live, so S3 (inspection) and S4 (weld) are physically separate. The laser enable is interlocked with the enclosure and the pallet stop | 4A.7 | added 4 Nov 2026 |
| 3.6 | Latency instrumentation: sensor publish → registration → correction, from rosbag2 timestamps | 4A.5 | |

---

## Phase 4 — Fidelity, results, write-up

| # | Task | § |
|---|---|---|
| 4.1 | Optional: Isaac Sim, only if Gazebo's rendering limits a result | 4 |
| 4.2 | README rewritten for the module line, structured as a paper: Problem → Method → Results → **Limitations** → Future work (cell internals with the collaborator; carbon fibre) | 3.6, 3.7.7 |
| 4.3 | Demo video: cells placed → HC10DT multi-view inspection → defect caught → next placement corrected | 3.6 |
| 4.4 | Verify quick-start from a clean clone in a fresh container | §7 of `project_structure.md` |
| 4.5 | arXiv preprint (cs.RO) — needs one endorsement, not necessarily the advisor | 3.6 |

> **The write-up's motivation section comes from §3.7.** Three measured
> findings about sensing justify the scope and the sensors before a single
> result is shown.

---

## Standing rules

These have each already cost time once.

- **Never `set -u` in a script that sources a ROS setup file.** §1.2.
- **Wrap every `robot_description` in `ParameterValue(..., value_type=str)`.**
  Otherwise launch YAML-parses your URDF and dies. Applies to the MoveIt
  configs and Gazebo bringup still to come. §1.4.
- **A clean `colcon build` does not mean the launch files run.** Launch each
  one once, headlessly, before calling a milestone done. §1.4.
- **Command arms only via `FollowJointTrajectory`; read state only from
  `/joint_states`.** Never call Isaac Sim's articulation API from a control
  node — that boundary is the entire sim-to-hardware claim. §6.
- **numpy stays at 1.26.4.** Open3D links the 1.x ABI. A `--force-reinstall`
  of torch will silently pull 2.x. §1.3.
- **Any `pip install` that can pull torch transitively needs the same
  `--index-url`, or `--no-deps`.** §1.1.
- **`pip uninstall` on a name that is not installed *succeeds*.** An uninstall
  list of guessed names looks identical to one that worked. §1.1.
- **Use `mb-venv` for Open3D/PyTorch.** Never pip into system Python — PEP 668
  blocks it and overriding breaks apt.
- **Every ROS package gets the `motobat_` prefix.** Package names are one flat
  global namespace across every overlaid workspace. `project_structure.md` §3.3.
- **Create packages when you write them**, never in advance.
- **If point clouds drop or truncate under load, switch to FastDDS before
  debugging anything else.** §5. MotoROS2 requires FastDDS anyway.
