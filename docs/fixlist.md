# MotoBat — Fix List

**One question this file answers: what is known to be wrong right now, and where does each thing
belong?**

This is a working checklist, not a reference. When an item is done, strike it through with the
date (`~~item~~ — **done 5 Nov 2026**`) rather than deleting it, per the house style in
`module_spec.md` §5. When an item turns out to be a decision rather than a fix, move it to
`project_plan.md` and leave a pointer here.

Section references like §4A.4 point into `project_plan.md`; §7 and §12 point into
`module_spec.md`.

*Last updated: 1 Nov 2026*

---

## 0. Resolved on 1 Nov 2026 — recorded so they are not re-asked

| Question | Answer |
|---|---|
| What is `AS_108261` in the mount STEP? | **Keyence LJ-X8200.** The mount carries Zivid 2+ M60, LJ-X8200 and `Mount v3`. No midiCam2, which matches the 28 Oct decision. |
| LJ-X8200 head mass | **1200 g** (S — user). This was the missing figure blocking the payload sum. |
| midiCam2 on the wrist | Future version only, if the Zivid's 2D proves insufficient. Gated on task 1.11. |
| Flange pattern | **DT, not DTP** — see §C2 below. |

---

## A. CAD re-exports

- [ ] **A1 — Export `terminal_pos` and `terminal_neg` as separate STLs.** Origin at the centre of
      each terminal's seating face (§4), so a height defect is a pure +Z translation and a tilt is
      a pure rotation about the real pivot.
      **This blocks task 1.2.** The STEP is already correct — `VDA PHEV2 Prismatic Cell.step`
      holds three solids (can + both terminals). But the exported STL is binary and single-mesh,
      so a loader sees one rigid cell with no terminal to move. Terminal-height and terminal-tilt
      defects are injected by transforming the terminal *relative to* the can, so from a merged
      mesh those two classes cannot be generated at all. This is a re-export, not a remodel.

- [ ] **A2 — Decimate `HC10 Flange V0.0.stl`.** It is **99 MB**, against GitHub's 100 MB hard
      limit; the `.step` is 69 MB. Both are already committed raw.
      Prefer decimation over Git LFS: LFS hides the size, decimation removes it. §9 says a sensor
      body only needs a box at the datasheet outline, and `defectgen`'s ray-cast sensor models
      intersect against every triangle in these meshes. Small is correct here, not a compromise.

- [ ] **A3 — Keep the Zivid FOV volume as its own component, excluded from both collision and
      visual meshes.** Otherwise MoveIt treats the view cone as a solid and every scan pose
      "collides". The FOV apex marks the optical origin, which becomes the URDF `<origin>`.

- [ ] **A4 — Confirm `Mount v3`'s material before trusting its mass.** The Fusion document rule is
      **Steel**; if the plate is aluminium the figure will be roughly 3× too high. Blocks the
      payload total in C3.

---

## B. Repository structure

- [ ] **B1 — `src/battery_description/` is not a package and breaks the prefix rule.** It holds
      only `models/`, with no `package.xml`, and `project_structure.md` §3.3 requires the
      `motobat_` prefix on every package. Resolve by folding the meshes into
      `motobat_description` (recommended, see §E) rather than creating a second package that
      would contain nothing but geometry — §3.7 says packages are created when they are written.

- [ ] **B2 — Rename `models/` to `meshes/`.** `models/` means Gazebo SDF model directories;
      `meshes/` is the ROS convention for geometry consumed by URDF.

- [ ] **B3 — Decide whether STEP ships inside the ROS package.** Nothing loads STEP at runtime —
      Gazebo reads STL / OBJ / DAE / glTF only. See §E for the recommendation.

---

## C. Documentation corrections

These follow from the answers in §0 and are not yet written into the docs.

- [ ] **C1 — §4A.4:** replace "`AS_108261` — an unidentified vendor part number" with the Keyence
      LJ-X8200, and drop the instruction to confirm it before the payload sum.

- [ ] **C2 — §4A.4: the flange is DT, and the current text says the question is open.**
      Opposite hole centres *are* the pitch circle diameter, so the measured **63 mm is a 63 mm
      PCD** — the Schmalz HC10DT figure. The DTP is ISO 9409-1-**50**-4-M6, a 50 mm PCD, 13 mm
      smaller. The 5 mm holes (M6 tapping drill) and the 6 mm keying hole (Ø6 H7 dowel) are common
      to both variants and do not discriminate; **only the PCD does.**
      This agrees with the URDF being `hc10dt`, so the proposal naming the HC10DTP was the error.
      Three consequences to write in:
      - Designation is most likely **ISO 9409-1-63-4-M6** (S — measured, 1 Nov 2026).
      - Adapter plate clearance holes go on **PCD 63**, not 50.
      - The **ø31.5 centring diameter is unverified** — it came from the SMC DTP spec and does not
        carry over. Confirm from Yaskawa's HC10DT manual drawing. Tag **A** until then.

- [ ] **C3 — §4A.4 and `status_update.md` task 2.3: write the payload sum with its working.**

      | Item | Mass |
      |---|---|
      | Zivid 2+ M60 | 1.0 kg (S) |
      | LJ-X8200 head | 1.2 kg (S) |
      | `Mount v3` | from Fusion — gated on A4 |
      | Cabling + flange hardware | ≈ 0.3–0.5 kg (A) |
      | **Total** | **≈ 2.5–3.0 kg** against the HC10DT's 10 kg |

      Note that the old "≈ 3 kg" estimate survives for different reasons: dropping the midiCam2
      freed roughly what the Keyence head costs over what was assumed. Two caveats to record: the
      **LJ-X8000A controller is not on the arm** and must not be added; but the head-to-controller
      cable routes through the wrist, which is already an open §4A.4 design question.

- [ ] **C4 —** remove "Keyence head mass TBD" wherever it appears (§4A.4, `status_update.md` 2.3).

---

## D. Still genuinely open

Not fixes — these need a source or a measurement before results are claimed (§3.7.9).

- [ ] CMC components must stay under **≈ 1.4 mm**, or the cover needs a local recess over the CMC.
      Raising the walls further breaks the VDA 355 height. (`module_spec.md` §5 height check)
- [ ] Confirm the **0.1 mm adjacent-terminal tolerance** from MDPI *Batteries* 6(3):47 directly.
- [ ] A **module-line cycle time** for the fixed-vs-arm comparison. Not found yet.
- [ ] `Mount v3` mass — gated on A4.

---

## E. Target layout

Written in the style of `project_structure.md` §3.7.

```
cad/                                 STEP archives — source, committed, not shipped
  module/                            11 × .step  (10 components + PHEV Pack reference)
  inspector/                         HC10 Flange V0.0.step

src/
  motobat_description/
    meshes/
      module/                        12 × .stl  (10 exported + the two new terminals, A1)
      inspector/                     decimated mount .stl (A2)
    models/                          retire — fold into meshes/ (B2)
    urdf/
      inspector_flange.xacro         flange + 2 sensor frames on s3_tool0
```

**Why one `motobat_description` rather than a new `motobat_battery_description`.** The package
already exists, already holds the flange, and already satisfies the prefix rule. A second package
containing nothing but meshes would trip §3.7's "packages are created when they are written, not
in advance". If the module geometry later grows its own URDF and launch files, split it then.

**Why STEP sits outside `src/`.** Nothing loads it at runtime, so keeping it in `cad/` means the
ROS package ships only what Gazebo and MoveIt actually read. STEP stays committed as source, per
the rule now in `project_structure.md` §2 — CAD is hand-authored and cannot be regenerated, which
is the opposite of the `data/` case.

**The one thing to check before adopting this.** `defectgen` lives outside `src/` and must not
import ROS (§2 of `project_structure.md`). Reading meshes from `src/motobat_description/meshes/`
is only a file path, so it creates no ROS dependency — but it does mean `defectgen` reaches into
`src/`, slightly against the grain of why it sits outside. The alternative is `meshes/` at the
repo root, with the package copying or symlinking at build time. The path is the lesser evil.
