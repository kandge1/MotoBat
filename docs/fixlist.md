# MotoBat — Fix List

**One question this file answers: what is known to be wrong right now, and where does each thing
belong?**

This is a working checklist, not a reference. When an item is done, strike it through with the
date (`~~item~~ — **done 5 Nov 2026**`) rather than deleting it, per the house style in
`module_spec.md` §5. When an item turns out to be a decision rather than a fix, move it to
`project_plan.md` and leave a pointer here.

Section references like §4A.4 point into `project_plan.md`; §7 and §12 point into
`module_spec.md`.

*Last updated: 1 Nov 2026 — A, B and C closed in one pass; only D remains.*

---

## 0. Resolved on 1 Nov 2026 — recorded so they are not re-asked

| Question | Answer |
|---|---|
| What is `AS_108261` in the mount STEP? | **Keyence LJ-X8200.** The mount carries Zivid 2+ M60, LJ-X8200 and `Mount v3`. No midiCam2, which matches the 28 Oct decision. |
| LJ-X8200 head mass | **1200 g** (S — user). This was the missing figure blocking the payload sum. |
| midiCam2 on the wrist | Future version only, if the Zivid's 2D proves insufficient. Gated on task 1.11. |
| Flange pattern | **DT, not DTP** — 63 mm PCD, measured. See C2. |
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

Not fixes — these need a source or a measurement before results are claimed (§3.7.9).

- [ ] **`Mount v3` mass.** Read it in isolation now that the material is AL-5052, and close out
      the payload total in §4A.4.
- [ ] **The ø31.5 centring diameter is unverified for the DT.** It came from the SMC **DTP**
      spec and does not carry over. Confirm from Yaskawa's HC10DT manual drawing; tagged **A**
      until then.
- [ ] **CMC component height** must stay under **≈ 1.4 mm**, or the cover needs a local recess
      over the CMC. Raising the walls further breaks the VDA 355 height.
      (`module_spec.md` §5 height check)
- [ ] Confirm the **0.1 mm adjacent-terminal tolerance** from MDPI *Batteries* 6(3):47 directly.
- [ ] A **module-line cycle time** for the fixed-vs-arm comparison. Not found yet.
- [ ] *(Watch, not a task)* `inspector_flange.stl` at 99 MB — revisit if performance suffers (A2).
