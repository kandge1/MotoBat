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
| Flange as Fusion User Parameters; LJ-X 0.3 mm into the bracket (2.3) | Frames are already derived from the STEP (2.4); parameters only matter when 1.11 swaps a sensor class. CAD work. | status 2.3 |
| HC10DT joint accelerations (2.7) | Not published by Yaskawa; a = 2·v_max assumed. 2.7 reports ×0.5 and ×2; ask Yaskawa or measure on a real arm. | joint_limits.yaml |
| Dataset v4 arm column (2.9) | Runs on Vultr when funded (user, 26 Nov). Needs fixlist F11 (`fixed+arm` setup, plan folder) first. Cycle times stand without it; detection per class uses v3 until then. | fixlist F11 |
| Gazebo sensor rates (2.6) | Lasers at 50 Hz, not 1 kHz, to keep real time with two lidars: clouds are visual (profiles 2–4 mm apart). Measurement stays in defectgen. | inspector_sensors.gazebo.xacro |
| Arm snapshot false alarms (bowed plate, foam), **if not fixed in the first pass** | Only if the fix forces a rerun that would block 1.10. | plan §4B.6 |
| Dataset 1 (v3) detector findings: FOD 0.4 escapes, foam false FOD, wrong-class calls, spatter partial, over-reads, scorer not counting wrong-class false alarms | Dataset 1 is used for Phases 2/3 as built. Fixed after v3 on the affected families; v4 (realism) starts from the fixed detector and rechecks them per factor. | fixlist F10 |
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
| 1.6 | `motobat_interfaces`: `DefectReport`, `ScanResult`, `CorrectionCmd` | §7 | ✅ first pass 14 Nov 2026, in the arm lane (`../MotoBat-arm/src/motobat_interfaces`, uncommitted). `ScanResult` (organised `PointCloud2`: the ray grid is the cloud's height × width, NaN = no return; signal and sigma fields), `DefectReport` (OK / NOK / INCOMPLETE, `Finding`, `Measurement`, `Uncovered`, `CaptureInfo`), `CorrectionCmd.action`. Geometry in metres (REP 103); measured values keep the detector's unit (mm, deg) with the unit stated, so they compare directly with `detect.yaml`. 7 tests: four real v3 reports (all three stages, OK and NOK) convert and round-trip; an unstated unit fails the test. The converter moves to `motobat_perception` when that exists |
| 1.7 | Labelled dataset, **several seeds per (class × severity) cell** | 4B.6 | ◐ 9 Nov 2026 — `dataset.py`, `configs/dataset.yaml`; design, pilot and results in `project_plan.md` §4B.6 "1.7 dataset". **Fixed setup done:** `pod` 565, `clean` 30, `grr` 180 cases (seeded; outputs in `data/dataset/`, not committed). No false alarm in any of the 775. Out-of-spec: 343 hit, 17 partial, 27 miss, 8 not covered, of 395. In-spec: 8 false rejects of 120. Most families are found on every seed from 1.1–1.3 × tol. Weak spots, as measured: line-scan X quantisation (cell/busbar position, cell yaw), crown sag reads short, spatter on and beside beads is not seen (bead band), a flush weld reads as `missing_weld`. Scoring: a miss where the setup does not look by design (1.5's coverage map) is `not_covered`; `--rescore` re-scores stored reports. Fixed on the way: a zero crown divided by zero in `weld._shape`. **Arm hardening done (9 Nov):** the arm's false alarms were registration, not oblique-view detection (placed exactly, every arm view compares clean). X, Y and yaw now come from four housing strips (end- and side-plate tops, each trusted across its width only), solved once per pallet load from each strip's best whole view; this also removed a 0.045 mm bias in the fixed pass's end-plate datum. Clean modules through the arm plan with a fixture error: 0 findings at all three stages (was 6 / 6 / 50); `regress` gained an `arm` group, `probe_arm.py` shows per-capture registration. Assumes exact robot poses. **Fixed column re-run on the strip datum** (old results in `data/dataset_prestrips/`; this run in `data/dataset_strips_v1/`): 0 findings on clean modules, 1 false alarm in 775 (a weld bow 1 % over tol on an untouched seam: the bow measure's noise tail), in-spec short welds rejected 5/10 → 0/10. **Then a datum fault, found by the one failing test:** a cell shifted toward a side plate bows the sheet, the side strip moved with it, and the datum subtracted part of the cell's own shift (11 % in the fixed run, 33 % through a part view). Fixed by a robust per-bin centreline per strip (`detect._strip_offset`); §4B.6 "A strip that moves with the defect". **Re-run on the fixed detector, done 12 Nov 2026**: fixed `clean`/`grr`/`pod` and arm `clean`/`pod`, one detector for both columns (`data/validate_pr_{fixed,arm}_final.txt`). Fixed: 0 false alarms in 775. Arm: 0/30 clean modules with a finding, 8 false alarms in 595, all on snapshots. That arm column measured a flawed arm *plan*, not an arm: FOD 60 % (a centre strip no sweep resolved), 10/10 and 9/10 in-spec reinforcement/width modules rejected, wall dents 12/20. **Fixed 12 Nov 2026** (fixlist F6): coverage sizes and fast wall sweeps (row 1.5); misregistration allowance from the surface slope under each ray (`detect.sideways_gain`); residuals on a judged bead count as explained. Regression through the new plan: clean modules 0 findings at every stage, placed and with fixture error; one defect of every class 23/23 found, 0 false alarms. **Both columns to be rerun** (`data/chain_v2.sh`; first run archived to `data/dataset_plan_v1/`), now with the arm `grr` tier. Arm ~2.5–3 min per case: ~1.5 days. Detector changes found by the pilot (drift window, discontinuity band, memory) re-verified with `python -m defectgen.regress` (now in the repo). 162 tests pass. **v2 stopped 13 Nov 2026 at arm pod 127/565** (fixlist F7): its fixed column was complete and equal to v1 case for case, but the arm column had 1,126 false-alarm regions. Two causes. (1) The module as measured did not bow the side plate that an in-spec cell shift bows, which the new side-on sweeps saw. (2) Snapshot noise grows on dim rays (σ 0.07 vs 0.03 mm). Fixed: `as_measured_scene` re-applies the bulge, and `detect.noise_vs_signal` fits σ² = a² + b²/signal per capture and material. Gates (`data/gates_v3.sh`): every group unchanged or better except fixed spatter, where F7's honest threshold lost drops that v1/v2 found by luck. **F8 (14 Nov):** a small compact group of rays over 3σ, two over 5σ, off every edge, is a region on its summed evidence; a first form (grow over any weak area) drew one-ray-wide foam-edge lines under the stress fixture error and was withdrawn. **F9 (14 Nov):** arm cells were placed from one terminal, so a 0.08° yaw sample step on a 57.5 mm lever moved unmoved cells 0.1 mm in the module as measured (the arm's long-standing handling-damage false alarm); now both terminals. **Known cases** (`defectgen/known_cases/`): 243 dataset and regress cases with accepted results and their fix-list items; a full pass fingerprints the detector, and `chain_v3.sh` / `cloud/run_remote.sh` refuse to start without one. **Proof run** (`data/overnight_f8.sh`, 14 Nov): pytest, the stress known cases, then the null sweep (every clean, in-spec and below-detectable case, 200 per setup, into v3's folder), then every known case; v3 continues (`data/chain_v3.sh`, arm first, tripwire) only on 0 false alarms. Dataset 1 is ideal-world by design (plan §4B.3). Cloud runs: `cloud/README.md` (Vultr, `--shard i/N`). **✅ Dataset 1 (v3) done 16 Nov 2026 00:22** (`data/dataset/`, the third build: v1 and v2 archived as `dataset_plan_v1/`, `dataset_plan_v2/` for their failures). Both setups, pod 565 + clean 30 + grr 180 each. False alarms (findings on no label): fixed 0 / 775, arm 1 / 775. What it showed beyond 1.8–1.11: fixlist F10 (a–j), left unfixed in dataset 1 on purpose; Phases 2/3 use it as built. Results written up in `project_plan.md` §4B.6 "Dataset 1 — results". **F10 a, c, h fixed 16 Nov**; check run `data/check_f10.sh` → `data/dataset_f10/` (known cases, null sweep, affected families, both setups; fixed first). **Check done 17 Nov:** 249/249 known cases pass; false alarms fixed 0 / 267, arm 1 / 273 (the foam FOD, F10b); arm FOD 15/15, cap dents classed right. F10 a, c confirmed. |
| 1.8 | **Precision/recall per class, with confidence intervals** | 4B.6 | ✅ first pass 16 Nov 2026 on dataset 1 — `validate.py pr`, method in `project_plan.md` §4B.6 "1.8 and 1.9" (`data/dataset/validation/pr.{fixed,arm}.txt`). Recall, module decision, false-reject rate, precision per finding class, false alarms per module (Poisson bound), Wilson 95 %, plus the reading error per family. **Fixed:** 0 false alarms in 775 (≤ 0.0039 / module); precision 95–100 % every class; recall 100 % on 11 of 19 classes, lowest spatter 7 % complete (87 % of modules rejected), reinforcement and weld arc 72 %. **Arm:** 1 false alarm in 775 (≤ 0.0061); recall 100 % on 13 classes, FOD 80 % (F10a), spatter 27 % complete (100 % rejected). In-spec rejects that are spec/CAD, not detector: carrier 0.16 (F1, 5/10 both), busbar 0.1 shift (F2). Caveat: `pr` counts wrong-class findings in `pod` ("other") but not in `grr` (F10h; now listed). After the F10 fixes (check run, 17 Nov): arm FOD 100 % (15/15), false alarms unchanged. |
| 1.9 | **%GRR** for terminal height, cell position, busbar gap | 4B.6 | ✅ first pass 16 Nov 2026, both setups on dataset 1 — `validate.py grr`, AIAG ANOVA, 10 parts × 3 pallet loads × 2 trials (`data/dataset/validation/grr.{fixed,arm}.txt`). %GRR to tolerance, fixed / arm: terminal height **1.6 % / 0.0 %**, busbar gap **0.5 % / 0.2 %**, cell position X **50 % / 27 %** (AIAG: < 10 % acceptable, 10–30 % conditional, > 30 % not). Cell X bias −25 µm fixed, −1.4 µm arm. Fixed cell X is the line-scan's 0.1 mm pitch varying with each load's sampling phase (AV 33 µm, EV 6.5 µm); the arm, placing cells from both terminals (F9), halves it. Arm zeros are ideal-world (exact robot poses): v4 adds pose error. |
| 1.10 | **Fixed vs arm: coverage % and recall per class** | 4B.6, 3.7.6 | ✅ first pass 16 Nov 2026 — `python -m analysis.compare` → `data/analysis/compare.md`: per class 1.5 coverage, 1.8 recall and module decision, 1.11 a90, both setups; per stage captures, scan time (1.5 estimate) and detection time. **The arm's case is coverage and position:** handling damage on walls 25 % → 92 % coverage, spatter 80 → 93 %, FOD 94 → 100 %; cell placement recall 86 → 98 % (no line-scan pitch); cell X %GRR halved. **Its cost:** 13–15 captures vs 1, ~46–63 s scan vs ~4 s, ~130–175 s detection vs 12–17 s on the laptop, and the F10 detector faults its oblique views expose (FOD 0.4 escapes, wrong-class calls). Neither setup alone: the fixed line for speed on up-facing features, the arm where the line cannot see. Cycle time in Phase 2. |
| 1.11 | Minimum detectable defect per class; sensitivity to noise, point spacing, standoff | 4B.6 | ◐ POD first pass 16 Nov 2026 — `python -m analysis.pod` → `data/analysis/pod.{md,json,png}`, 6 tests. Hit/miss logistic in log size (MIL-HDBK-1823A), **Firth-penalised** because most families are completely separated (every miss below every find: the MLE does not exist); a50, a90, a90/95 (one-sided Wald bound, not extrapolated past the largest size), and the n-of-n binomial bound beside it. Tolerance classes sit on their tolerance (e.g. terminal height a50 0.094 / t 0.1, both setups); the fixed line spreads cell shift (a90 0.28 vs arm 0.22) and yaw (0.15 vs 0.12). Not monotone, so read the per-size counts: arm FOD 0.4, arm handling damage 0.4, reinforcement 0.3 (both setups) — F10. 5 seeds per size cap the n-of-n bound at 89 %: demonstrating a90/95 needs ≥ 29 per size (1.13). **Sensitivity sweep, cut-down, done 16 Nov** (`analysis/sensitivity.py`, in `data/check_f10.sh`): fixed line, one factor at a time (noise ×0.5–4, pitch 0.05/0.2, 1280 points, the same camera at 700/1000 mm), 72 cases per setting → `data/sensitivity/summary.md`. **Sensor class: ~10 µm height repeatability (set by the welded stage: 2× noise gives 136 false alarms, all post-weld) and ~0.05 mm along travel (0.05 mm pitch fixes cell pose at 1.1 × tol and 0.15 mm spatter); placement and heights don't care about noise even at 4×.** 1000 mm standoff is blind, not clean (F10k). Plan §4B.6. Screen only (3 seeds, OAT); dataset 2 runs it as a designed experiment (`docs/detector_review.md` G2). |
| 1.12 | **Correction loop in numpy: K-sweep, open- vs closed-loop placement defect rate, reject/rework logic** | 4B.7, 9.9 | ◐ first pass 14 Nov 2026 — `correction/` (outside `defectgen/src`, so the known-cases fingerprint is untouched), `python -m correction.run` → `data/correction/fixed/`. 26 tests. Over an assumed 2400-cell shift: open loop 74 % of cells out of tolerance, I loop at deadbeat (K = 1) 8.3 %, at the best K ≈ 0.07 0.35 %. Findings: the plan's 0 < K < 2/g is the I form's band (u₊ = u − K·e); the literal P form is stable only for K < 1/g and keeps d/(1 + gK); deadbeat doubles placement variance (σ² = σ_w² + K(σ_w² + σ_v²)/(2 − K)); 1.9's −25 µm cell-X gauge bias lands one for one in placement. Terminal height: guard-banded rework before the carrier. Plant assumed (A); rerun on v3's %GRR, both columns **Rerun 16 Nov on dataset 1's %GRR, both gauges** (`data/correction/{fixed,arm}/`): fixed best K 0.07 → 0.35 %, deadbeat 8.3 %; arm σ_v 17.9 µm, bias −1.4 µm: best K 0.11 → 0.26 %, deadbeat 4.7 %. The arm's smaller bias takes the steady x offset from +27 µm to +2 µm. |
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
> (see "Two lanes" above). **18 Nov:** Phase 1's first pass is on `main` (`ac4e259`) and this
> branch is rebased onto it (main + 7 commits, nothing behind). Done: 2.1, 2.2, 2.4, 2.8.
> **25 Nov:** first pass of 2.3, 2.5, 2.6, 2.7 and 2.9's plan done (uncommitted). **26 Nov:** v4's arm
> plan chosen (2.9 `s1`); v4 itself runs on Vultr later, its defectgen changes filed as fixlist F11.
> Phase 2 has nothing else open beyond the Deferred items below.

| # | Task | § | Notes |
|---|---|---|---|
| 2.1 | HC10DT MoveIt config — **KDL, not OPW** ✅ 6 Nov | 4A.2 | `motobat_hc10dt_moveit_config`, group `inspector_arm`, pose `ready`, mock hardware via `FollowJointTrajectory`. **KDL kept by measurement:** benchmarked against TRAC-IK and pick_ik on 3 × 1000 targets. KDL matched the best on inspection-like poses at 10–100× less time. TRAC-IK's MoveIt plugin gives up when its first answer collides (§4A.2 "As built"). Station poses moved to `line_layout.xacro`. **Acceleration limits assumed (A); revisit before 2.7.** |
| 2.2 | `ros2_control` + `joint_trajectory_controller`, HC10DT in Gazebo ✅ 6 Nov | 4A.3 | `motobat_bringup/launch/inspector_gazebo.launch.py`; same MoveIt and controllers as 2.1, only the hardware plugin differs (`sim:=gazebo`). `plan_exec_check.py` passes; the arm holds pose under gravity; real-time factor 1.00. **Inertials added** (the vendor URDF has none): computed from the collision meshes, 48 kg total (S), uniform density (A). **Position gain set to 1.0:** the plugin default 0.1 lagged 2.3° mid-move; 1.0 gives 0.02°, the same as mock hardware. |
| 2.3 | Flange: snapshot 3D + profilometer (**+ 2D camera only if 1.11 requires it**), poses as variables | 4A.4, 3.7.3 | **✅ first pass 25 Nov (uncommitted).** `inspector_flange` carries the Zivid 2+ M60 and the Keyence LJ-X8200 on an AL-5052 plate, DT pattern, 63 mm PCD; frames from the STEP (2.4). **Payload 2.7 kg of 10**; wrist load ≤ 37 % of any limit (2.5). **FOV occlusion:** the robot, bracket and other housing block 0 of 430,434 measured points in any stage (2.5, `occlusion_check.py`). **Standoffs:** each sensor is flown at its own (Zivid 600 mm focus, LJ-X 245 mm reference); 2.5 and 2.9 reach every planned view from base A. **Deferred:** sensor poses as Fusion User Parameters, and the LJ-X sitting 0.3 mm into the bracket (both CAD work, user). |
| 2.4 | `inspector_flange.xacro` — flange + sensor frames on `s3_tool0` ✅ 9 Nov | 4A.4 | All frames derived numerically by `step_frames.py` from the STEP plus vendor drawings, with every drawing dimension checked against the CAD: `zivid_optical` (Zivid offsets from its Ø5 positioning hole; toe-in 2.5°), `ljx_laser` / `ljx_reference` (Keyence drawing: 12 / 27 / 30 / 245 mm), and **`flange_yaw` = −90°** (robot dowel on tool0 +X, measured by hole depth in the vendor mesh). Passes in MoveIt and in Gazebo (holds pose, RTF 0.98, tracking ≤ 0.07°). 2.7 kg payload. Minor: LJ-X profile X sign is assumed (A); LJ-X sits 0.3 mm into the bracket in CAD (user checking). |
| 2.5 | Reachable view plan: Phase 1 viewpoints filtered by IK and collision, ordered to minimise motion | 4B.5 | **✅ first pass 19 Nov (uncommitted).** `view_reach.py` searches robot placements and plans each station's order and moves (OMPL, collision-checked against the module + a 10 cm keep-out); `view_plan.py` shows it in RViz (WORKFLOW.md "Arm view plan"). **No turntable** (user, 14 Nov). Of 222 placements, **a ceiling-hung HC10DT 1.1 m above the pallet top, 0.3 m off the centreline, flies all 21 views of all three stations**. **18 Nov:** moves between views keep the wrist, flange and sensors above the module top (user); the four views lower than that (side and end walls) are entered and left by a straight vertical approach of 14–22 cm. Sweeps rebuilt at 5 mm per point (move_group returns them resampled every 25–40 mm). Verified on base A: 0 of 1,725 move states below the floor off the approaches, 0 of 2,956 states in collision, sweep joint speeds ≤ 82 % of the limits. Cycles 66 / 63 / 80 s (pre-busbar / pre-weld / post-weld; ±a few s, OMPL is random), captures 32 / 30 / 48 s of that. Base B (floor) reaches 16 of 21 under the same rules. **Base A chosen (user, 18 Nov);** the physical layout stays Phase 3. **Mount and wrist load (18 Nov):** the HC10DT data sheet (Yaskawa DS-778-B) lists floor, wall, ceiling and tilt mounting (S). Worst case over all wrist angles, flange 2.7 kg + 0.5 kg cable dress (A): R 9.2 of 27.4 N·m and 0.29 of 0.78 kg·m², B 7.4 / 0.19, T 0.4 / 0.012: at most 37 % of any limit. **A vs B, measured (18 Nov):** from 1.5's cached counts, B loses only handling damage on the walls (A 89–92 %, B 41–53 %); every other class is within a point. `scan_view_plan.py` on a weld-defect module with a 0.4 mm dent on each side wall, post-weld: all 43 weld labels scored the same from both bases, A found both dents and B neither, 0 false alarms from either (WORKFLOW.md, "Simulated scans along the flight"). The best floor placement reaches 17–19 of 21 and never the near-side wall. **Conveyor placeholder (18 Nov):** base A's robot never goes more than 40 mm below the pallet top; beside the pallet only ≥ 313 mm off the centreline, but past the module ends on the centreline itself, where the belt runs. So: a conveyor along the module (3.0 m × pallet width + 2 × 40 mm, 100 mm deep, all A) with a lift-and-locate raising the pallet 50 mm off the belt at the station (A), 30 mm clear of the lowest point. Base A replanned with it: still 21/21, 0 of 3,141 states in collision, transit floor held; cycles 75 / 67 / 86 s. **Smoother moves (planner 2, 18 Nov):** the wiggle the user saw came from each view keeping the first IK branch that worked, so neighbouring views were often elbow-up then elbow-down (12–13 of 14–16 moves swung a joint > 90°). Now each view keeps up to 4 branches and the order and branches are chosen together (exact shortest path over branches for a given order, inside the order search); moves are synchronised joint moves (Yaskawa MOVJ, checked every 0.01 rad) where collision-free, else joint moves through raised waypoints (0.15 / 0.30 / 0.45 m above start, end or midpoint, or via home; planner 3), else OMPL. Same base: moves 29 / 26 / 29 s instead of 43 / 36 / 39 s, joint travel −40 %, 8 big swings per stage; cycles 62 / 57 / 76 s; sweeps ≥ 6° from the B = 0 wrist singularity (was 1°). OMPL is left for 2 of 14–16 moves per stage (wall to end wall, round the module corner; home ↔ overhead snapshot) and for most single-view home trips (viewer only, not in any cycle). A view with one workable branch is retried from 40 more seeds if the fixed seeds miss it (KDL's random restarts once missed one). Verified: 0 of 20,270 states in collision, moves never below the transit floor. **Mount height (18 Nov, same planner):** 0.95 / 1.10 / 1.25 / 1.40 / 1.55 m above the pallet top reach 17 / 21 / 19 / 17 / 15 views. At 1.10 the top-down views are folded tight (wrist 0.21–0.45 m from the shoulder of a 1.2 m reach) while the far side wall is at full stretch (1.20 m); higher relaxes the first and loses the walls, lower loses top-down sweeps. 1.10 kept (`data/reach/heights_A/`). **Robot self-occlusion (18 Nov):** `occlusion_poses.py` (ROS, FK) + `occlusion_check.py` (venv, trimesh): for every capture of every stage, at 16 robot poses per sweep, each measured point's two light paths (laser → point → receiver; camera → point ← projector, defectgen's geometry) cast against the HC10DT's collision meshes, the bracket and the other sensor's housing. Base A: 0 of 430,434 tested points blocked in any stage. Controls: the rays' sensor ends lie on their own housing at every sampled pose (0 mm), and with that housing kept in, 98–100 % of rays are cut, so the test detects blocking. Not modelled: cable dress, the mount plate and conveyor (above the arm and below the pallet). **Line pose (19 Nov):** `line_layout.xacro` now names the floor (world z = 0), the conveyor centreline (`line_y`), the pallet top 0.9 m up and the station-3 pallet stop, and derives the s3 mount from base A's numbers (hung, roll π, at (1.80, −0.30, 2.00) m). URDF forward kinematics in `world` match the planner's poses plus the datum on all 553 occlusion-check poses (0 µm, 0°); `plan_exec_check.py` passes hung, on mock hardware and in Gazebo (tracking ≤ 0.026°). **For `module_spec.md` §11 (Phase 1 lane, not edited here):** S3's HC10DT hangs from an overhead frame, base 1.10 m above the pallet top and 0.30 m to one side of the centreline, level with the module centre; the pallet is lifted 50 mm off the belt at S3; floor 0.9 m below the pallet top (A). The s1/s2/s4 placeholder mounts in `line_layout.xacro` (stacker, welder, sealer at y = 0 on the floor) predate §11's S1–S5 and still need laying out against the conveyor (Phase 3). Original scope: the arm earns its place here. Must account for: **two different standoffs** (Zivid ≈ 600 mm nominal, 300–1100; LJ-X8200 ≈ 245 mm) implying separate poses per sensor; the **≈ 39° wall-occlusion limit** on outer terminal edges (`module_spec.md` §5); and housing-to-housing occlusion. **Needs 2.1 first** — scan poses cannot be checked for reach or collision without the HC10DT MoveIt config. |
| 2.6 | Module meshes in Gazebo; depth camera + narrow-fan `gpu_lidar` as the sensors; fixed line-scan station alongside | 4B.8, 4A.3 | **✅ first pass 25 Nov (uncommitted).** `inspector_gazebo.launch.py world:=line` loads `worlds/inspection_line.sdf`: the conveyor (2.5's placeholder), a **fixed line-scan gate** one pitch upstream (Ranger3 stand-in: `gpu_lidar`, 180 mm fan at 500 mm), and the module on a **pallet driven along the line by a prismatic joint** (`/pallet/x`). `gazebo_module.py` exports any defectgen module per body group, in metres, weld beads and dented walls decimated to 60 k triangles each (2.5 M → 0.19 M); one box collision. Wrist sensors (`inspector_sensors.gazebo.xacro`, Gazebo only): Zivid depth camera (52.8°, 0.3–1.1 m, 612 × 512, 5 Hz) and LJ-X `gpu_lidar` (72 mm fan at 245 mm, 800 rays). Lasers at 50 Hz (A; 1 kHz in defectgen). `line_demo.py`: the pallet runs under the gate at 100 mm/s, indexes to S3, the arm flies a stage's plan by `FollowJointTrajectory`, and every profile and depth frame is placed in the moving `pallet_datum` frame by TF. **Measured (post-weld, 2.9 plan):** fixed pass 4.3 s, index 1.2 s, arm 54.14 s flown = 54.14 s planned, 1,291 captures all placed; both clouds lie on the module (median 15–17 µm, 95 % ≤ 0.13 mm from the mesh). The arm cloud reaches 12 mm above the pallet (walls), the fixed one only 93 mm (tops). **Fixes found:** the robot's start pose (SRDF `ready`) put the hung wrist into the module, and mesh contacts ran the sim at RTF 0.07; `initial_positions.yaml` is now the planner's home → RTF 1.00. Gazebo renders on the NVIDIA GPU when present (PRIME offload; `MOTOBAT_GZ_NVIDIA=0` turns it off). `plan_exec_check.py` re-passes from the new start (mock and Gazebo). Gazebo's sensors are geometric stand-ins: no speckle, no specular dropout (§3.7.8). |
| 2.7 | **Fixed vs arm with real cycle time** (motion + capture per module) | 3.7.6 | **✅ first pass 25 Nov (uncommitted).** Planned cycles at base A (2.5's planner; Gazebo flies them to the 10 ms, 2.6), s per stage, pre-busbar / pre-weld / post-weld: **v3 plan** 61.7 / 56.5 / 76.2; **2.9 arm alone** 56.6 / 47.0 / 65.6 at v3 coverage, 50.7 / 47.3 / 55.8 one point under; **2.9 arm filling the fixed pass's gaps** 46.9 / 35.9 / 64.4 at v3 coverage, **36.6 / 32.5 / 52.1 one point under**. Fixed line-scan: 3.7 s of conveyor travel per module. **Takt (D2), sourced:** 59 s per module on a fully automated module line (PIA Group, reference line; S), 4–7 min per module for a 12-cell prismatic module assembled manually or semi-automatically (batterydesign.net, Peugeot e208; S). An 18 s figure attributed to AMD Machines in search results is not on their page and is not used. **Acceleration (still A, Yaskawa publishes none):** a = 2·v_max; at ×0.5 / ×2 the gap-filling plan gives 43.0 / 39.2 / 61.4 and 38.3 / 32.4 / 48.4 s (v3: 69.7 / 65.1 / 83.1 and 57.1 / 53.2 / 73.2). OMPL legs vary ±2 s run to run. **Verdict:** with the 2.9 gap-filling plan every stage fits a 59 s automated takt at the assumed acceleration (post-weld by 7 s); at half the acceleration post-weld is 2 s over. The v3 plan misses it at two of three stages. This assumes **one inspection cell per stage**; one HC10DT serving all three visits would need 121 s per module. Detection per class stays v3's (1.10) until dataset v4 (2.9). |
| 2.8 | Verify TF tree: `ros2 run tf2_tools view_frames` ✅ 9 Nov | 4A.4 | One tree rooted at `world`, 15 frames, no orphans: arm chain → `s3_tool0` → `s3_flange` → Zivid (`zivid_mount` → `zivid_optical`) and LJ-X (`ljx_mount` → `ljx_laser` → `ljx_reference`). `tf2_echo` from `s3_tool0` matches the CAD-derived offsets to the millimetre. |
| 2.9 | **Minimum-time inspection plan → dataset v4** (added 14 Nov at the user's request) | 4B.5, 3.7.6 | **✅ plan first pass 25 Nov (uncommitted); v4 dataset waits on a decision (below).** `inspection_plan.py`: 1.5's 69 candidate geometries checked from base A by 2.5's checker (52 reachable, `data/reach/cand_A`); each sweep offered at 50 / 100 / 200 mm/s where no joint passes 90 % of its speed limit (counts scaled by profile pitch from 1.5's cache, checked against 1.5's own fast sweeps; no new ray casting); 107 options. **Exact minimum-time cover** by integer program (scipy `milp`, HiGHS; patches with identical covering views merged), seconds per stage, every class at least at v3's coverage (capped at what base A can reach), optionally minus a slack. Two variants: arm alone, and arm with the fixed pass's covered patches counted as done. Results and real cycles: see 2.7. Greedy was not far off at v3's coverage (16 / 13 / 16 views against 15 / 13 / 15); the savings come from faster sweeps, the fixed pass's credit and a point of slack (11 / 8 / 12 views). Move time assumed 2.0 s per view in the objective; 2.5's planner then measured 1.9–2.1 s. **Chosen (user, 26 Nov): `s1`, the gap-filling plan one point under v3, as long as it detects as well.** Local check (`scan_view_plan.py --variant`, the weld-defect module with two wall dents, all three stages, fixed capture included for s0/s1): s1 and s0 give **the same verdict on every label**; pre-busbar 2/2, pre-weld 3/3, post-weld 36/45 for both (v3's arm alone 33/45: s0/s1 also find one spatter drop and two undercuts it missed), 0 false alarms everywhere. One module is a smoke test, not POD: the real acceptance is in v4. **v4 runs on Vultr later (user); nothing in Phases 2–3 waits on it.** The `fixed+arm` setup, a plan-folder option and the acceptance test are filed as **F11** in `docs/fixlist.md` on `fixlist/1_X` (26 Nov). `inspection_plan.py patches` writes each plan's `<stage>.patches.npz` (keys `fixed`, `arm`, `fixed+arm`) for the dataset's coverage scoring. Plans: `data/plan29/{s0,s1}/{arm,arm_fixed}`. |

---

## Phase 3 — The line: GP7 cells, GP4 carrier, GP25 weld, closed loop, safety

> **Welder decided 4 Nov 2026: GP25 + MOTOPAC-RL2D300-GP25JP** (`project_plan.md`
> §4A.7). The GP4 can't carry any laser head (4 kg payload; the lightest head
> found is 5.5 kg). The GA50 was rejected as a path-accuracy arm for large parts,
> because here the scanner draws the seam.

| # | Task | § | Notes |
|---|---|---|---|
| 3.1 | GP7 MoveIt + OPW. **Derive the OPW parameters from the URDF** (none ship, verified) and check them against forward kinematics. | 4A.1 | ◐ **OPW derived and checked 26 Nov** (`motobat_description/scripts/derive_opw.py`, derivation in its docstring; numpy, no ROS). Read from the joint axes at q = 0, not assumed: GP7 a1 0.040, a2 −0.040, b 0, c1 0.330, c2 0.445, c3 0.440, c4 0.080; offsets (0, 0, −90°, 0, 0, 180°), signs (1, 1, −1, −1, −1, −1). **Checks:** the same code reproduces the vendor's `opw_parameters_gp4.yaml` exactly (control); FK vs the URDF chain on 20,000 random joint vectors ≤ 1e-15 m / rad; all 8 IK branches reproduce the pose and contain the source q on 5,000 of 5,000; wrist-centre reach 926.8 mm vs data sheet 927 (S); the HC10DT fails the spherical-wrist test (axes 4 and 6 162 mm apart: negative control). Jmeyer's `opw_kinematics` library (v0.5.5), given these parameters, matches the URDF to 1e-15 and recovers 3,000 of 3,000 source q. The vendor writes tool0's rpy as `3.1415926 -1.570796` (3.3e-7 rad off π, π/2; the GP4 uses `radians(180)`): checks run with those exact, the shipped tilt is reported. **No MoveIt 2 OPW plugin exists** (JeroenDM's `ros2-port` branch is an unfinished 2021 WIP; the ROS index has ROS 1 releases only), so **`motobat_opw_kinematics_plugin` was written (user, 26 Nov):** a MoveIt 2 `KinematicsBase` plugin around `opw_kinematics` (headers copied into `third_party/`, pinned 0.5.5; not on apt for Jazzy). Per query: all 8 branches, each joint moved to the 2π-equivalent nearest the seed inside the URDF limits, nearest first to MoveIt's collision callback. **At start-up it checks OPW FK against the robot model at 200 random states and refuses to load above 1 µm / 1 µrad**; the base and tip offsets are measured from the model, so a TCP other than `tool0` works. gtest on the real GP4 / GP7 / GP25 URDFs (`colcon test`): 2,000 random states each, every branch reproduces its pose ≤ 2e-11, the seeded source comes first; a flipped joint-4 sign or a 1 mm longer upper arm is refused; a callback that rejects the first branch gets the next; out of reach returns NO_IK_SOLUTION. **Median 1 µs per solve** in-process (Release; 87 µs unoptimised). Next: `motobat_gp7_moveit_config` once 3.0 places S1. |
| 3.2 | GP7 cell pick-and-place, located by a fixed overhead 2D camera | 3.7.3 | same pattern as the myCobot vision pick-and-place |
| 3.3 | GP4 MoveIt + OPW (vendor file exists); busbar-carrier, CMC and cover placement | 4A.1 | ≈ 0.3 kg loads. Vendor OPW file reproduced by `derive_opw.py gp4` (26 Nov). |
| 3.3b | GP25 MoveIt + OPW **derived from the URDF** (none ships); MOTOPAC-RL2D scanner as the end effector; the weld is a scanner pattern drawn while the arm holds | 4A.7 | payload 18.3 / 25 kg before fibre and dress. **OPW derived 26 Nov** (`derive_opw.py gp25`, same checks as 3.1): a1 0.150, a2 −0.200, b 0, c1 0.505, c2 0.760, c3 0.795, c4 0.100, same offsets and signs; reach 1,729.8 mm vs 1,730 (S). `motoman_gp25_support` is still `COLCON_IGNORE`d in `_vendor` (bootstrap keep-list). |
| 3.4 | Close the loop in simulation: `DefectReport` → `CorrectionCmd` → next cell's placement | 9.9 | Labels now carry `cause` (`weld.py`), so a reject at the pre-weld gap check can be credited with the weld defects it prevented |
| 3.0 | `motobat_line.urdf.xacro`: replace the second GP4 with the **GP25** (welder, §4A.7), giving GP7 + GP4 + HC10DT + GP25; rebuild, relaunch headless | 4A.7 | `motoman_gp25_support` is vendored (un-ignored in `_vendor` and added to `bootstrap_rviz.sh`'s keep-list, 26 Nov). **Layout decided (user, 26 Nov): three inspection cells, one per stage**, as 2.7's takt verdict assumed: S1 GP7 cells → S2 line-scan gate → S3a HC10DT pre-carrier → S4a GP4 carrier → S3b HC10DT pre-weld → S4b GP25 weld (laser enclosure) → S3c HC10DT post-weld → S5 GP4 CMC + cover. The current xacro (GP4 stacker, GP4 welder, HC10DT, GP7 sealer) predates `module_spec.md` §11 and is replaced. **✅ first pass 26 Nov (uncommitted).** `motobat_line.urdf.xacro` now holds the seven arms (`s1_` GP7, `s3a_/s3b_/s3c_` HC10DT, `s4a_` GP4, `s4b_` GP25, `s5_` GP4), box end effectors with TCP frames (`line_tools.xacro`: GP7 gripper 150 mm, GP4 vacuum 100 mm, RL2D head with `focus` 579.5 mm = 329.5 S + 250 A), and the static equipment as collision boxes (`line_fixtures.xacro`: conveyor, gate, three portal frames for the hung HC10DTs, the S4b laser enclosure with pallet openings, pedestals, five magazines). **Numbers by measurement, not pitch:** `layout_check.py` places each floor robot by a closed-form-IK search (every pick, place, approach and weld hold reachable inside the URDF limits with ≥ 10 % of each joint's range left, base ≥ 30 mm clear of the conveyor, smallest swept footprint) and sweeps its collision meshes, gripper and largest carried part over the task sequence; `line_envelope.py` sweeps the HC10DT over all 10,691 states of 2.5's base-A plan: **−1.17 … +1.07 m along the line** (working sweeps alone ±0.86 m; the end-wall sweeps fly the LJ-X 245 mm past the module and the hung elbow trails further). Pallet stops are spaced so neighbouring envelopes are 0.10 m apart (A), so robots on different pallets never share space and no interference-zone handshake is needed. **Result:** stops at x = 0 / 1.957 / 3.670 / 5.104 / 6.692 / 8.390 / 10.115 m (S1, S3a, S4a, S3b, S4b, S3c, S5), gate 0.659, line 11.2 m; GP7 base 0.30 m off the centreline at 0.60 m, GP4s 0.30 / 0.675, GP25 on the floor 0.50 off; laser enclosure 0.79 m long (x 6.30–7.10). Plot: `layout_check.py --plot data/layout/line_layout.png`. **Checks:** `layout_check.py --verify <expanded URDF>`: no moving link of any robot enters any of 36 fixture boxes (0 collisions; nearest: the HC10DT's wrist passes **6.3 mm** above the conveyor on the low wall views, as 2.5 planned it against the same belt top 0.830 m; everything else ≥ 54 mm). TF in RViz with `demo_line_motion.py` (now replays every robot's real motion) puts `s4b_focus` on the first terminal and `s1_tcp` above the end-plate pick to 0.00 mm. The HC10DT's overhead frame: above its mount plane no moving link comes within 0.33 m of the mount axis and nothing passes 2.32 m, so a 0.2 m drop column to a beam at 2.45 m. **Moved for this:** the gate (0.6 → 0.659; at 0.6 it sat 2 mm from the S3a envelope) and the S3a stop (1.8 → 1.957) in `inspection_line.sdf`, the launch's gate TF and `line_demo.py`; 2.6 re-run headless, post-weld: fixed pass 4.28 s, index 1.21 s, arm 54.18 s flown = 54.14 planned, 1,291 captures all placed, clouds on the mesh as on 25 Nov (arm median 27 µm, 95 % ≤ 0.155 mm; fixed 17 µm, ≤ 0.116). **Limits (first pass):** floor-robot envelopes are of joint moves between task poses, not of planned moves (MoveIt per station next; on a real line these boxes become the controllers' FSU range limits); one OMPL transit sets the HC10DT's −1.17 (constrain it and the cells shorten by ~0.3 m); magazine positions, gripper and head sizes are A; cooling plate, gap filler and side plates (§8 steps 1, 5) are fixture work, not modelled; the HC10DT MoveIt config still plans one cell with prefix `s3_` (= S3a; 2.5's plans are in the base frame, so they serve all three), and the configs need a prefix argument when 3.4 runs the whole line. |
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
