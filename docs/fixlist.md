# MotoBat — Fix List

**One question this file answers: what is known to be wrong right now, and where does each thing
belong?**

This is a working checklist, not a reference. When an item is done, strike it through with the
date (`~~item~~ — **done 5 Nov 2026**`) rather than deleting it, per the house style in
`module_spec.md` §5. When an item turns out to be a decision rather than a fix, move it to
`project_plan.md` and leave a pointer here.

Section references like §4A.4 point into `project_plan.md`; §7 and §12 point into
`module_spec.md`.

*Last updated: 1 Nov 2026 — A, B and C closed. D remains and **none of it blocks the current
task**, each item gating a specific later task (§D). **§E is new**: two CAD issues found by
building the generator, one of which supersedes D3.*

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

- [ ] **E1 — Re-export `terminal_neg` without the fill plug.** It exports 38.5 wide against
      `terminal_pos`'s 22, so the plug is merged in. *Downgraded 3 Nov 2026.* The 153.5-wide
      module this was blamed for was a generator bug: the part was placed by its bounding-box
      centre instead of its pad centre, which pushed the plug outboard. Measured against the
      Fusion `cell_assembled`, the plug runs inboard. Fixed in `parts.py` (`pad_datum`), together
      with `+`/`−` on swapped sides and flipped cells whose terminals were not turned
      (`module.cell_transform`). The module is now 151.0 wide. What remains: a height or tilt
      defect on a `−` terminal also moves the plug, which is part of the can. *Blocks: nothing.*
      Tests: `test_parts.py::test_terminals_have_matching_footprints` (strict xfail),
      `test_module.py::test_terminals_sit_where_the_fusion_cell_has_them`.

- [ ] **E2 — The CMC overruns the cover by 0.2 mm.** This supersedes D3, and shrinks it.
      The 3.2 thickness is **deliberate: 1.6 of PCB plus 1.6 of surface-mount component space**,
      so D3's question — "will the components fit in the headroom?" — was already answered in the
      CAD. What is left is arithmetic: 102.0 carrier top + 3.2 = 105.2 against a 105.0 wall top.
      Drop the CMC 0.2 into the carrier, or take the component allowance to 1.4.
      Test: `test_budgets.py::test_cmc_clears_the_cover`.

- [x] ~~**E3 — `cmc` is non-manifold**~~ — **closed as not-a-defect.** It has no holes, so a ray
      cannot leak through it. The generator checks for holes rather than strict two-manifoldness,
      because CAD parts whose sub-bodies sit exactly face-to-face always go non-manifold once
      duplicate vertices merge, and nothing downstream cares.

**Six dimension corrections** also came out of this and are already written into
`module_spec.md` §5 — walls 101.0, carrier 7.0, busbar width 12, busbar lengths 64.5 / 148.5,
CMC 250 × 50 × 3.2, overhang rule `n × 28 + 8.5`. Z now closes at 107.0, not 107.5.
