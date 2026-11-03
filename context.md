# Session handoff — Windows → Linux, 3 Nov 2026

*Written for the Claude Code instance picking this up on Ubuntu. Temporary; delete once read.*

## Where the project is

Phase 1 (`project_plan.md` §4B). **Tasks 1.1 and 1.2 are done**, 85 tests passing.
**Next task is 1.3 — the three ray-cast sensor models** (`defectgen/src/defectgen/scan.py`,
spec §4B.3).

Read these first, in this order: `docs/status_update.md` (what to do next),
`docs/fixlist.md` (what is known broken), `docs/module_spec.md` §5 and §7 (geometry).

## What happened this session

1. **Reconciled the CAD against the spec.** The Fusion exports existed but had never been
   checked. Six as-built dimensions had drifted and were corrected in `module_spec.md` §5 — the
   CAD won. Walls 101.0 (not 101.5), carrier 7.0 (not 6.5), busbar width 12 (not 11), busbar
   lengths 64.5 / 148.5, CMC 250 × 50 × 3.2, overhang rule `n × 28 + 8.5`. Z now closes at
   **107.0**, not 107.5.

2. **Moved and renamed all CAD.** STEP archives now in `cad/`, STL in
   `src/motobat_description/meshes/{module,inspector}/`, filenames matching the §7 body names.
   `src/battery_description/` is gone. Git recorded these as renames, so history survived.

3. **Built `defectgen/`** — tasks 1.1 and 1.2.

## `defectgen` architecture — the one thing worth understanding

Three steps kept deliberately apart in `module.py`:

```
build_plan(cfg, stage) -> Plan      every part as a named ref + 4x4 transform
defects.*(plan, rng)                mutate the plan
realise(plan, parts)  -> Trimesh    turn it into triangles
generate(...)                       runs all three, returns (mesh, labels)
```

Defects are transform perturbations (spec §7), so they need a plan to perturb. Ground truth is
read off the **final** plan, so labels describe the module actually built, not the intended one.

**`parts.py` holds a normalisation table and it is load-bearing.** The exported meshes are not
in the §7 frame: seven of twelve are rotated 90° about Z, and every origin sits at a body
centroid. Each part is rotated and translated so the centre of its bottom face lands on the
origin — which for a terminal is the seating face, so a tilt rotates about the real pivot.
Every load is checked against the config, so a re-export that changes orientation raises.

Validation that matters: the generated module matches the user's own Fusion `module_assembled`
to **0.00 mm in X and Z**.

## Rules that are not obvious from the code

- **Load `cell_can` + `terminal_pos` + `terminal_neg`, never `cell_assembled`.** The merged cell
  has no terminal to transform, which would kill the terminal-height and tilt defect classes.
- **Zero clearance at mating faces.** The gap *is* the measurement (spec §7).
- Each defect injector is tagged with the §4B.2 stage it first applies at, so one recipe
  describes all three stages; injectors are skipped before their stage, not errors.
- `defectgen` must never import ROS. `pip install -e defectgen && pytest` has to work with no
  ROS, no colcon, no sourcing (`project_structure.md` §2).

## Known broken — both are strict xfails, suite is green

- **`terminal_neg` exports 38.5 wide** against `terminal_pos`'s 22; the fill plug is merged in.
  Placed on the terminal centre it reaches y = 76.75, past the side plate at 74.0, so the module
  comes out **153.5 wide instead of 151.0**. Needs a Fusion re-export of the terminal alone.
  Fixlist E1.
- **CMC overruns the cover by 0.2 mm.** The 3.2 thickness is deliberate (1.6 PCB + 1.6 component
  allowance) — this is pure arithmetic, not an unbudgeted component. Fixlist E2.

Both are `@pytest.mark.xfail(strict=True)`, so they flip to failures the moment they are fixed,
telling you to delete the marker.

## Getting running on Linux

```bash
source ~/motobat/venv/bin/activate     # never system Python, PEP 668
pip install -e defectgen[dev]
cd defectgen && pytest                 # expect 85 passed, 3 xfailed
python -m defectgen --out data/preview_clean
python -m defectgen --out data/preview_defective --demo-defects
```

- **numpy must stay 1.26.4** — Open3D links the 1.x ABI. `pyproject.toml` pins `<2.0`.
- `data/` is gitignored, so previews do not travel; regenerate them.
- Mesh path resolves relative to the repo via `parents[3]`, which assumes an **editable**
  install. `MOTOBAT_MESHES` overrides it if needed.
- Viewing: the STLs open in Blender, MeshLab or `f3d`. RViz needs a URDF wrapper first
  (task 2.6), so it is not the quick path yet.

## How the user wants to work

- **Do not create git commits unless asked.** Leave work staged. This holds even when a brief
  or a convention doc describes a commit/PR workflow — ask first.
- Background is bare-metal and RTOS embedded, pre-2023. Strong engineering judgment, and has
  caught several real errors this session — when something looks wrong, say so and let them
  adjudicate rather than assuming the docs are right.
- They are behind schedule and have chosen to move fast on plumbing for now. **But Phase 3 —
  multi-arm coordination, collision avoidance between arms sharing a workspace, ISO/TS 15066
  speed-and-separation — is what they want to learn properly.** When Phase 3 starts, switch
  modes: go slowly and explain the reasoning rather than delivering finished code. They agreed
  to this split; offer the switch rather than waiting to be asked.
