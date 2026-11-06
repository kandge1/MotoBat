# MotoBat — Battery Module Specification (VDA 355, PHEV2 cells)

**One question this file answers: what exactly are we building, part by part, and how sure is
each number?**

This is the geometry reference for the Fusion 360 parts and for `defectgen`'s module generator.
Every dimension is tagged:

| Tag | Meaning |
|---|---|
| **S** | Sourced — from a datasheet, standard or paper (link given) |
| **D** | Derived — arithmetic from sourced numbers (working shown) |
| **A** | Assumed — a reasonable engineering value; replace when a better source appears |
| **M** | Measure — take it from a real-cell photo in Fusion (method in §6) |

Units: millimetres. Last updated 1 Nov 2026 — CAD exported and reconciled against §7; six
as-built corrections measured by loading the meshes into `defectgen` (§5); Gazebo mesh-loading
notes added. Previous: 22 Oct 2026, wall height and busbar corrections (§5).

---

## 1. What we are building

A **VDA 355 module**: the standard German/Chinese automotive module envelope, holding twelve VDA
PHEV2-format prismatic cells. The target look is the reference image in the chat of 15 Sep: cells
in a row, finned end plates, side plates, a bottom liquid-cooling plate, a busbar carrier
("support structure"), busbars, a cell module controller, and a cover.

> The **cell module controller** (CMC, also called CSC) is the part of the battery management
> system that lives on the module: it measures each cell group's voltage through the busbars and
> reads temperature sensors, then reports to the pack BMS.

---

## 2. Module envelope — VDA 355

| Parameter | Value | Tag | Source / working |
|---|---|---|---|
| Outer dimensions (L × W × H) | 355 × 151 × 108 | S | [LYTH — VDA355 & MEB590](https://lythbattery.com/what-is-vda355-and-meb590-module%EF%BC%9F/), [DIY Solar Forum](https://diysolarforum.com/threads/has-anyone-considered-building-a-battery-with-vda-355-modules.116912/) |
| Cells | 12 | S | same |
| Configuration | **3P4S** (3 parallel × 4 series) | S | Real CALB VDA355 module: 3P4S, 174 Ah, 14.68 V ([LYTH](https://lythbattery.com/calb-3p4s-58ah-vda355-14-68v-174ah-ev-ncm-module/)). 4S3P and 3S4P are the common real configs. |
| Mass | ≈ 11 kg | S | DIY Solar Forum thread |
| End-plate preload | ≈ 3 kN initial | S | [batterydesign.net — prismatic cells](https://www.batterydesign.net/battery-cell/formats/prismatic-cells/) |
| Length budget for end plates + pads | 355 − 12 × 26.5 = **37** | D | 2 end plates + 11 interface pads |
| Width budget for side plates | 151 − 148 = **3** → ≈ 1.5 per side | D | thin steel/Al side plates |
| Height budget above/below cell can | 108 − 91 = **17** | D | base/cooling interface + terminals + carrier + cover |

**Why 3P4S and not 12S1P:** 3P4S is what real VDA 355 modules use. It also makes the busbar
pattern non-trivial (groups of three cells flip orientation), so a reversed cell is a meaningful
defect.

---

## 3. Cell — VDA PHEV2

| Parameter | Value | Tag | Source / working |
|---|---|---|---|
| Can width × thickness × height | **148.0 × 26.5 × 91.0** | S | [batterydesign.net — VDA prismatic](https://www.batterydesign.net/vda-prismatic-cells/) |
| Real example | CATL NCM27E892-50Ah: 148 × 27 × 92 can; listed 97–98 with terminals; 0.85 kg; 50 Ah; 3.65 V | S | [evlithium](https://www.evlithium.com/nmc-battery/catl-50ah-nmc-battery-cell.html), [dlcpo](https://dlcpo.com/catl-50ah-ncm-battery-cell-prismatic-ternary/) |
| Dimensional tolerance | ±0.5 | S | vendor listings above |
| Can material / wall | Aluminium, ≈ 1.1 | S | batterydesign.net |
| Insulation film | blue PET wrap, ≈ 0.1 | A | visual (the blue in the reference image); negligible geometrically |
| Mass | ≈ 0.85 | S | CATL example |
| Vendor drawing | 27.0 × 148.0 × 92.0, all ±1.0 | S | dimensioned drawing obtained 26 Sep 2026 (§4). Same cell family as the CATL example: 92 can + 6 terminal = 98, matching its listed 97–98. |

**Nominal stays 26.5 × 91, not the drawing's 27.0 × 92.** The two agree inside the ±1.0
tolerance, and the module length budget in §2 is built on 26.5: at 12 × 26.5 = 318 the remaining
37 mm covers 11 pads and both end plates (§5 stack-up). At 27.0 the budget falls to 31 mm, which
squeezes the end plates below what the fins need. Keep `cell_t = 26.5`.

> Model the can as a plain solid — do **not** shell it to the 1.1 mm wall. The wall is internal;
> nothing in this project sees or measures it, and shelling only costs triangles. Likewise the
> 0.1 mm PET wrap is an appearance on the can's four side faces, not a body. Mass belongs in the
> sim description (URDF/SDF/USD take explicit mass and inertia), not in the CAD density.

> **Caveat from batterydesign.net:** "even if it states they are VDA prismatic cells, check the
> dimensions on the specification sheet and on the actual cell" — manufacturers vary terminal
> height and tab thickness inside the same format. That is why the terminal is fully parametric.

---

## 4. Terminals (the most important part for inspection)

The pre-weld inspection measures terminal-top height to **±0.1 mm between adjacent terminals**
(MDPI *Batteries* 6(3):47 via search — confirm from the paper). So the terminal is modelled
exactly, but its dimensions are parameters.

| Parameter | Value | Tag | Source / working |
|---|---|---|---|
| Type | laser-weld pad (flat top, no stud) | S | module lines weld busbars to pads ([IPG](https://www.ipgphotonics.com/solutions/e-mobility-manufacturing-solutions/battery-module-busbar-welding)) |
| Material | + Al; − Al (Cu–Al composite inside the cell) | S | [NB Cell — terminal comparison](https://nbcellenergy.com/lifepo4-prismatic-battery-cell-terminal-comparison/) |
| Terminal pad thickness | ≈ 4 | S | weld study: 1.5 mm 1060 Al busbar on a 4 mm 1060 Al terminal ([ResearchGate](https://www.researchgate.net/publication/379641129_A_modelling_approach_for_laser_welding_of_busbars_to_lithium-ion_prismatic_cell_terminals_to_enhance_failure_prediction)) |
| Height above cap | **6.0 ±0.5** (L1) | S | vendor drawing. Confirms the earlier 5–7 estimate exactly. |
| Pad footprint | **22.0 × 18.5** (L5 × L6) | S | vendor drawing. The earlier 21.5 × 17.5 guess from the LFP-based NB Cell guide was close but is superseded. |
| Centre-to-centre spacing | **115.0 ±1.0** (L7) | S | vendor drawing. C-to-c is the functional dimension — it is what the busbar span must match — and the ±1.0 tolerance fits an overall span rather than a feature size. With W = 148 the centres sit **16.5 from each long edge**, so each 22-wide pad spans 5.5 → 27.5 and leaves a 5.5 margin. That puts the busbar rows near the module's long edges, as in real VDA 355 photos. The 66 / 76 mm web figures were a different format — discard them. |
| Polarity distinction | take the real shapes from the drawing/photo | S | Supersedes the earlier *assumed* chamfer on −. A real asymmetry is strictly better: reversed-cell detection should key on something that exists. |

> **Do not colour-code the terminals in training data.** Both are aluminium; the real cap is two
> metallic terminals, a black fill plug and a blue wrap. Red-for-+ / black-for-− is a schematic
> convention, and a detector trained on it learns a cue that is absent on the line and fails on
> the first real image. Keep two appearance sets: colour-coded for renders and the write-up,
> physically plausible for anything fed to a model.
| Vent | oval **23.0 × 12.5** (L3 × L4), centred on the cap | S | vendor drawing |
| Fill-port plug (black pad beside one terminal) | ends **38.5** from the near edge (L2) | S | vendor drawing. Occlusion and appearance only: L5 and L7 fix the terminals completely, so nothing measured depends on this. |

**Source:** dimensioned vendor drawing for a 148 × 92 PHEV2-class cell, obtained 26 Sep 2026.
It resolves what §6 was written to measure. Note §3 and §4 were originally assembled from
*different* sources — the can from the VDA table, the terminal from an LFP-cell guide — which is
why they did not agree. This drawing makes them one cell.

**Terminal numbering — 4 Nov 2026.** Terminals are numbered like the pins of a 24-pin DIP chip.
View the module from above with the cell-0 end (−X) at the top, so the −Y terminal row is on the
left. **T1 is the top-left terminal.** Numbers run down the left row (T1–T12, cells 0 → 11) and back
up the right row (T13–T24, cells 11 → 0), counter-clockwise. In code: `module.terminal_pin`, and
every terminal label carries its `pin`. The Zivid photos use the same view and are named by pins:
four quadrant shots centred between T3/T4, T9/T10, T15/T16 and T21/T22 (`defectgen.photos`).

**The terminals are rigidly fixed to the can — the CAD split is not a joint.** Separating them is
an *authoring-time* degree of freedom, not a runtime one. The defect is a real manufacturing
variation (cap seating, weld penetration, a proud terminal) that is baked in when the cell is
made; but generating a few hundred modules with that variation distributed across them means
either re-exporting the cell from Fusion once per sample, or exporting the terminal once and
letting the generator place it at `cap_height + ε`. The second is why the split exists. In the
exported scene the transform is set once at scene-generation time and then frozen: no joint,
physics never touches it. Merge the terminals into the can solid and the terminal-height and
terminal-tilt defect classes — the two headline targets of the pre-weld inspection in §8 step 6 —
become ungeneratable.

**CAD structure (see §7).** The can, terminal (+) and terminal (−) are separate **components**,
not bodies in one component: STEP's assembly tree maps from components, so the generator can
address `terminal_pos` by name. The vent and the fill-port plug are *features on the can* — they
never move, so they need no transform of their own. Each terminal's origin goes at the **centre
of its seating face** (where it meets the cap), so a height defect is a pure +Z translation and a
tilt is a pure rotation about the real pivot. An origin at the pad top or the centroid would make
every tilt drag the contact point sideways, injecting two defects where one was intended.

---

## 5. The rest of the module

| Part | Key dimensions | Tag | Notes |
|---|---|---|---|
| **End plates** (×2, finned — "passive cooling structure") | **148 W × 101.0 H × 10.25 T** incl. fins | D | Raised from 91.5 on 22 Oct 2026 (see corrections below): top at z = 105.5, 3 mm above the busbar tops, so the cover closes over everything. Shallow for fins — budget ≈ 4 base + 6 fin. Structurally ample: 3 kN over the 148 × 101.0 face is only 0.20 MPa. *(Height corrected 101.5 → 101.0 on 1 Nov 2026 — measured from the CAD.)* |
| **Interface pads** (×11, between cells) | 148 × 91, **1.5** thick | A | Set by the 28.0 cell pitch (below). Also the better number on its own merits: real modules use compressible foam pads of 1–2 mm+ to take cell swelling; the earlier 0.8 was pure arithmetic. |
| **Side plates** (×2) | **355 L × 101.0 H × 1.5 T** | D | Raised from 91.5 on 22 Oct 2026, same height as the end plates. Lap over the *outboard* faces of the end plates; welded after compression. |
| **Bottom cooling plate** ("liquid cooling structure") | 355 × 151, ≈ 3–5 thick, internal channels | A | channels matter only visually; the thermal gap-filler bead on top of it is a defect class |
| **Thermal gap filler bead** | bead width ≈ 5–8, height ≈ 2 before compression | A | adhesive-bead defect target. **Not a CAD part — generated geometry (§7).** |
| **Busbar carrier** ("support structure", CCS frame) | **333 L × 146.5 W × 7.0 H**, windows over each terminal | A | as built, 27 Sep 2026. Terminals span 14.25 → 340.75, so 333 centred covers them with ≈ 3 spare each end. The 146.5 leaves 0.75 clearance to each side plate — tight on purpose: that is the threshold at which a mis-placed carrier contacts a side plate, which §10 wants geometry behind. Industry supplies carrier + busbars + sense wiring as **one subassembly** (cell contacting system). |
| **Busbars** | **Copper** (C110/Cu-ETP class), **1.5 thick** | S/A | *Material corrected 4 Nov 2026:* the busbars are copper; the earlier "1060 Al" came from a Fusion appearance overriding the material. Thickness still from the weld study above, which was Al-on-Al — a Cu-busbar-on-Al-terminal lap weld is a dissimilar Cu–Al joint (intermetallic-forming, a different process window), so the weld parameters and gap tolerance must be re-sourced for Cu–Al. |
| — busbar pattern (3P4S) | end busbars span 3 terminals; series busbars span 6 terminals (3 + 3) | D | computed from config by the generator |
| — busbar width | **12** | A | as built, 22 Oct 2026. Replaces "≈ 20–25 (≥ pad width)", which had no basis: a busbar narrower than the 22 mm pad sits entirely on aluminium, so the weld cannot miss. Leaves ≈ 5.5 of bare pad each side, which the step-8 gap check uses as a direct height step. |
| — busbar lengths | short (end, 3 terminals) **64.5**; long (series, 6 terminals) **148.5** | B | as built, measured from the CAD 1 Nov 2026. Previously recorded as 63.5 / 147.5. |
| — electrical sizing | not modelled | — | 11 × 1.5 is thin for 150 A real current. Deliberate: this spec serves welds and scans, not current capacity. |
| **Laser weld seams** | one seam per busbar–terminal joint; 2–3 linear stitches ≈ 10–15 long, ≈ 1–1.5 wide | A | post-weld inspection target. See the weld note below §5. |
| **Cell module controller** (PCB) | **250 × 50 × 3.2** incl. components | A | as built, 27 Sep 2026. Runs the carrier's length, seated in the central channel between the two busbar rows at ±57.5, so no clash. Sense tabs to each busbar. |
| **Cover** | 355 × 151 × **2** | A | plastic or Al. Rests on the wall tops: z 105.0 → 107.0. |

### Weld note — it is a lap weld, and the gap is the process variable

**Busbar welding is a lap (penetration) weld**, not a two-sided resistance-style weld: the laser
fires down *through* the 1.5 mm busbar and fuses ≈ 0.5–1 mm into the 4 mm terminal pad beneath.
Aluminium's low resistance makes resistance welding impractical at this thickness.

- On the 11 mm busbar, linear stitches run **along X** (the busbar's length). Use one centreline
  stitch, or two parallel stitches ≈ 2.5 either side of the centreline. Keep ≈ 2 from the busbar
  edges.
- **The pre-weld gap is the critical process variable.** A gap larger than a small fraction of
  the top sheet's thickness causes undercut, porosity or no fusion at all. That is precisely why
  §8 step 8 checks it before the irreversible weld at step 9.

**Constraint on generated seams at the end terminals.** The as-built lengths are `n × 28 + 8.5`,
so each busbar ends **4.25 past the centre** of its outermost terminal and covers **13.5 of that
pad's 18.5** in X. Generated weld seams on an end pad must lie inside the busbar ∩ pad overlap —
this binds the *generator*, not the CAD. (A suggestion to lengthen the busbars to 74.5 / 158.5
was considered and rejected; the as-built lengths stand.)

### Busbar material — copper on aluminium, decided 4 Nov 2026

**Copper busbars on the cells' aluminium terminal pads**, lap-welded with the beam entering
through the copper. Al-on-Al was the alternative and is the more common prismatic practice
(3000-series Al busbars). It was rejected for this project, not for being wrong, for three reasons:

- **Physically real and broader:** a dissimilar Cu–Al joint adds the defect mechanisms that make
  busbar welding hard. Al–Cu intermetallics grow with heat input. Above roughly 3–5 µm they raise
  contact resistance and make the joint brittle (sources disagree on the exact figure). Copper's
  conductivity narrows the process window.
- **Visible heat history:** copper oxidises dark red → brown → black with temperature and time at
  temperature, so discolouration is a real overheating cue. Aluminium shows only soot and a frosted
  band.
- **Harder for the sensors:** bare copper is more reflective than aluminium, so specular dropout
  matters more (task 1.3).

Costs accepted: Cu-on-top needs high power or a green/blue laser in practice, and reviews prefer
entering from the Al side. The weld limits must be sourced for Cu–Al. Busbar material should be a
config parameter, so Al-on-Al stays one switch away.

Sources: [Kumar et al., Warwick](https://wrap.warwick.ac.uk/id/eprint/168639/1/WRAP-In-depth-evaluation-of-laser-welded-similar-and-dissimilar-material-tab-to-busbar-electrical-interconnects-for-electric-vehicle-battery-pack-Kumar-2021.pdf),
[dual-mode tab-to-busbar](https://www.sciencedirect.com/science/article/pii/S2666330924000669),
[tab-to-busbar review](https://pmc.ncbi.nlm.nih.gov/articles/PMC12843912),
[Al–Cu laser welding review](https://www.sciencedirect.com/science/article/abs/pii/S003039922300614X),
[laser braze-welded Al–Cu resistance](https://www.sciencedirect.com/science/article/abs/pii/S152661251300131X),
[E-Mobility Engineering](https://www.emobility-engineering.com/ev-battery-welding/).
Further Cu-on-Al findings (foil studies, robot vs scanner roles, GP4 payload check):
`docs/cu_on_al_laser_welding.md`. **Welding station** (4 Nov 2026): GP25 + MOTOPAC-RL2D,
an 8 kW infrared fibre laser with a 320 µm spot and wobble welding. Infrared couples into copper
worse than green; the package makes up for it with power and wobble (`project_plan.md` §4A.7).

### Stack-up — nothing adds to the envelope

Every part lives *inside* 355 × 151 × 108. The cooling plate is the full footprint and the end
plates and side plates sit **on top of it**, not beside it.

| Axis | Sum | Closes |
|---|---|---|
| X (355) | 10.25 + (12 × 26.5) + (11 × 1.5) + 10.25 | **355.0** ✓ |
| Y (151) | 1.5 + 148 + 1.5 | **151.0** ✓ |
| Z (108) | cooling 4 + walls 101.0 + cover 2 = 107.0 (cells, terminals, carrier, busbars, CMC all inside the walls) | **107.0** ✓, 1.0 spare |

### Cell pitch = 28.0 — the number the carrier is built to

`cell_t + pad_t = 26.5 + 1.5 = 28.0`. **Drive the carrier's window pattern from this parameter,
never from a literal.** The busbar windows must land on it in both directions:

| | Value | Free to change? |
|---|---|---|
| Window pitch along X | **28.0** | Yes — absorbed by `pad_t`, which is tagged **A** |
| Row spacing across Y | **115.0** | **No.** Sourced from the cell drawing (§4), and no part sits between the two terminal rows whose thickness could absorb an error. |

That asymmetry matters. An X-pitch mismatch is fixable by retuning the pad; a Y mismatch is not
fixable at all except by changing the carrier.

**Why 28.0 and not 27.3** (decided 27 Sep 2026): the existing carrier is patterned at 28.0 and is
complex enough that re-patterning it costs more than retuning the parts around it — which were
unbuilt at the time, so the change was free. Note the error would not have been shimmable: a
series busbar spans 6 terminals = 5 pitches, so a 0.7 mm pitch error drifts 3.5 mm *within a
single busbar*, and no placement of that busbar puts all six pads under it.

**Two corrections made 26 Sep 2026.** End plates were listed as "151 × 108", which was impossible
twice over: 151 wide collides with the side plates that already claim 1.5 mm per side, and 108
tall overflows the envelope once it sits on the 4 mm cooling plate. Now **148 × 91**, making the
end plates and side plates a *cell-height cage* consistent with the side plates' ~90.
*(Height superseded 22 Oct 2026 — see below. The width fix stands.)*

**Corrections made 22 Oct 2026.**

1. **Walls too short for the cover.** The cell-height cage (91.5 as built) left everything above
   the cell cap outside it: cell 91 + terminal 6 + busbar 1.5 = **98.5** above the cell bottom,
   7 mm proud of the walls, so a flat cover resting on the walls could not close. End plates and
   side plates are now **101.5** tall (+10), giving **3 mm relief** above the busbar tops. The
   envelope still closes: 4 + 101.5 + 2 = 107.5 of 108. No X/Y clash: the carrier and CMC
   (333 long) fit the 334.5 between end plates, and the busbar rows (Y ≤ ±63 at 11 wide) sit
   inside the side plates at ±74.
2. **Busbar width.** "≈ 20–25 (≥ pad width)" replaced by the as-built **11**, with the as-built
   lengths 63.5 / 147.5. The "≥ pad width" rule had the weld-placement argument backwards.
   Electrical sizing is explicitly out of scope.

**Consequence for inspection — taller walls are occluders.** This follows from §9's rule and
belongs in the view-plan results:

| Target | Horizontal distance to side-plate inner face | Wall height above target | Blocked from outboard below |
|---|---|---|---|
| Terminal pad outer edge (Y = ±68.5, z = 101) | 5.5 | 4.5 | **≈ 39° elevation** |
| Busbar outer edge / exposed pad (Y = ±63) | 11 | 4.5 above pad, 3 above busbar | ≈ 22° (pad), ≈ 15° (busbar) |

So step 6 views of the outer terminal edges must come from within ≈ 50° of vertical, or from
inboard. The step-8 gap check is barely affected.

**Why the side plates are outermost in Y.** They lap over the end plates' outboard faces, which
is the only arrangement where the step-5 weld is physically makeable — two parts cannot be welded
while occupying the same space.

**Do not model the side-plate welds.** §8 step 5 has no inspection against it; nothing ever
measures them. Only the busbar welds (step 9, inspected at step 10) need geometry, and those are
generated (§7).

**Nothing "holds" anything in simulation.** Parts are placed at a pose and stay there — no
clamping, preload, friction or weld strength is simulated, the same stance §11 takes on the
conveyor. The 3 kN at step 4 is a process step with a geometric consequence (pad compression),
not a physics problem; bake it into the pad thickness if wanted. The §10 fixture *locates* parts,
which is what gives mis-placement a real geometric relationship to tooling — but in sim,
placement is assertion.

**The bead eats no height.** The check below has no allowance for it, so cell bottoms sit at
**z = 4**, flush on the cooling plate, with the compressed bead notionally inside that interface.
The uncompressed 2 mm bead will interpenetrate the cells; harmless, since it is only ever shown
in the pre-cell state at steps 1–2. Hide or swap it once cells are placed.

### Height check (D) — an overlapping stack, not a serial sum

**The carrier straddles the terminals, it does not sit on them.** That is what the windows are
for: the terminals pass *up through* the carrier, so the carrier's base rests on the cell caps at
z = 95, overlapping the whole terminal zone. An earlier serial sum here (`… + terminal 6 +
carrier 4 + …`) implied the carrier stacked above the terminals and under-budgeted it.

| z | Part |
|---|---|
| 0 → 4 | cooling plate |
| 4 → 95 | cells (91) |
| 95 → 101 | terminals (6) — through the carrier windows |
| 95 → 102.0 | carrier (7.0) — overlaps the terminal zone |
| 101 → 102.5 | busbars (1.5), on the terminal tops |
| 102.0 → 105.2 | CMC (3.2) — **0.2 over the wall top, see caveat** |
| 4 → 105.0 | end plates and side plates (101.0), on the cooling plate |
| 105.0 → 107.0 | cover (2), resting on the wall tops |

Closes at **107.0**, 1.0 under the 108 envelope. Clearance above the busbars: 2.5.
Independently confirmed 1 Nov 2026 — the generator's assembled mesh and the Fusion
`module_assembled` reference agree on z 0 → 107.0 exactly.

> **CMC — 0.2 over, and the component allowance is already in the number.** The 3.2 is
> deliberate: **1.6 of PCB plus 1.6 of surface-mount component space above it.** So the old
> open item "will the components fit in 1.4 mm?" is answered — they were budgeted from the
> start. What remains is a plain arithmetic overrun: seated on the carrier top at z = 102.0 the
> CMC reaches **105.2** against a wall top of **105.0**, so the cover cannot sit flat.
> It is 0.2 mm, and raising the walls is not available without breaking the VDA 355 height.
> Cheapest fixes: drop the CMC 0.2 into the carrier, or take the component allowance to 1.4.
> Open item — §12, asserted as a strict `xfail` in
> `defectgen/tests/test_budgets.py::test_cmc_clears_the_cover`.

*(Before 22 Oct 2026 this table listed the cover at 106 → 108 with ≈ 1.9 headroom, but had no
row for the walls; with 91.5 walls, the cover had nothing at that height to rest on.)*

**Corrections made 1 Nov 2026 — measured from the CAD.** Building `defectgen` meant loading every
exported mesh and comparing it against this file, which surfaced six as-built values that had
drifted from what was recorded. The CAD wins; these are now the numbers:

| Item | Was | As built |
|---|---|---|
| End and side plate height | 101.5 | **101.0** |
| Busbar carrier thickness | 6.5 | **7.0** |
| Busbar width | 11 | **12** |
| Busbar lengths (end / series) | 63.5 / 147.5 | **64.5 / 148.5** |
| CMC | 333 × 45 × 1.6 | **250 × 50 × 3.2** |
| Busbar overhang rule | `n × 28 + 7.5` | **`n × 28 + 8.5`** |

X and Y are unaffected and still close at 355.0 and 151.0. Z now closes at **107.0** rather than
107.5, which is what makes the CMC clash above a real 0.2 mm overrun rather than a margin.

All six are enforced by `defectgen/tests/`, so the next drift fails a test rather than waiting
for someone to notice.

---

## 6. How to measure the **M** values in Fusion 360

> **Mostly superseded (26 Sep 2026).** The vendor drawing in §4 supplies terminal height, pad
> footprint, vent size and spacing directly. Keep this method for the one item still open —
> the L2 = 38.5 fill-plug span (§12) — and for any future part with no drawing.

Without a manufacturer drawing, calibrate a real photo against a known dimension:

1. Find a straight-on photo of the top (cap) of a PHEV2-class cell (e.g. the CATL 50 Ah product
   photos linked above). Top-down, as little perspective as possible.
2. Fusion: **Insert → Canvas**, place it on the XY plane.
3. Right-click the canvas → **Calibrate**, click the two ends of the cell's long edge, enter
   **148 mm**.
4. Sketch over it: terminal centres, pad size, vent outline, distances to the edges.
5. Cross-check with a second photo, or the side view against the 91 mm height. If two photos
   disagree by more than ~1 mm, average them and tag the value **M (±1 mm)**.

That accuracy is plenty. The detector measures *relative* heights (terminal vs neighbouring
terminal), so exact absolute pad size barely matters to the results.

---

## 7. Fusion → `defectgen` handoff

**One source of truth for numbers:** every value above becomes a **Fusion User Parameter** with
the same name as the generator's config key (e.g. `cell_w = 148`, `terminal_h = 6`). They live in
`defectgen/configs/vda355_3p4s.yaml` when code starts. Change a number in one, change it in both.

**Export conventions** (per part, so the generator can assemble them):

| Rule | Value |
|---|---|
| Format | STEP (archive) + STL (generator / Gazebo) |
| Units | millimetres in the file; the generator converts to metres for Gazebo |
| Axes | Z up; X along the module length (cell stacking direction); Y across |
| Cell origin | centre of the can's bottom face |
| Other parts | origin at a named datum written next to the file (e.g. `busbar_series: origin at centre of underside`) |
| One body per file | cell can, terminal (+), terminal (−), pad, end plate, side plate, cooling plate, carrier, busbar (end), busbar (series), CMC, cover |
| Clearances | **none at mating faces** — see below |

**Model at nominal with zero clearance. Do not subtract a tolerance from mating faces.**

Standard CAD practice is to relieve contacting faces so parts assemble. Here it is actively
wrong, for three reasons:

1. **The gap is the measurement.** A busbar rests on its terminal with zero gap at nominal, and
   *busbar-to-terminal gap* is a defect class checked at §8 step 8. Relieve that face and every
   generated module ships pre-defective. Against a ±0.1 mm adjacent-terminal tolerance, a 0.5 mm
   clearance is five times the signal. (Same argument as fillets at interfaces, smaller number.)
2. **It breaks the budgets**, which close exactly at nominal. 0.5 mm off both X faces of every
   cell loses 12 mm from the stack and walks the carrier windows off the terminals.
3. **Nothing is ever inserted.** Parts are placed by transform; there is no assembly motion to
   clear.

Clearance is required only where something **passes through** something else:

| Interface | Clearance |
|---|---|
| Face-to-face contact (cell↔pad, plate↔cell, busbar↔terminal, parts on the cooling plate) | **zero** |
| Terminal through a carrier window | **required** — size for the terminal plus the full defect perturbation range, or a tilted terminal interferes |
| Ø6 dowel through its hole | **required** |
| Carrier to side plates (0.75/side) | designed, not subtracted — keep |

Where clearance affects results it is a **variable, not a constant**: the §10 nest clearance sets
the threshold at which a mis-placed part touches tooling, so it belongs in the YAML as a swept
parameter, never subtracted into the CAD once.

The generator places these meshes with one transform per instance, and **injects defects mostly
as transform perturbations** (cell pose, terminal height, busbar lift-off). Weld seams, spatter
and **the thermal gap-filler bead** are generated geometry.

**Why the bead is generated, not modelled.** Its defects *are* shape defects — skips and gaps,
thin sections, a wandering path, a late start — and you cannot put a gap in a rigid CAD body by
transforming it. So the dispense path is a parameter and `defectgen` sweeps a bead profile along
it with per-segment width and height. Visual mesh only, **no collision**.

**Compression is never simulated.** §8 dispenses and inspects the bead at step 1; cells do not
arrive until step 3, so the sensor only ever sees it uncompressed. If a post-compression look is
ever wanted for a render, author a second static bead (wider, flatter, slight squeeze-out) and
swap meshes — deformable-body physics here would cost days and inform no measurement, the same
trade §11 refuses for the conveyor belt.

*(Superseded 1 Nov 2026 — this section used to end "until the Fusion parts exist, the generator
uses its own box primitives … so code does not wait on CAD." The parts now exist; see below.)*

### The CAD as exported — 1 Nov 2026

**Filenames are the body names above.** STEP archives live in `cad/module/`; the STL the
generator and Gazebo load lives in `src/motobat_description/meshes/module/`. One name per body,
so a filename, a Fusion User Parameter prefix and a generator config key all read the same —
that is the one-source-of-truth rule applied to paths.

| §7 body | File stem | |
|---|---|---|
| cell can | `cell_can` | ✓ |
| terminal (+) | `terminal_pos` | ✓ |
| terminal (−) | `terminal_neg` | ✓ |
| pad | `interface_pad` | ✓ |
| end plate | `end_plate` | ✓ |
| side plate | `side_plate` | ✓ |
| cooling plate | `cooling_plate` | ✓ |
| carrier | `busbar_carrier` | ✓ |
| busbar (end) | `busbar_end` | ✓ |
| busbar (series) | `busbar_series` | ✓ |
| CMC | `cmc` | ✓ |
| cover | `cover` | ✓ |

Two extra files are references, not bodies: **`cell_assembled`** (can + both terminals as one
mesh) and **`module_assembled`** (the whole module). Useful for renders and for eyeballing a fit
check. **The generator must not load `cell_assembled`** — it loads `cell_can` plus the two
terminals, because that is what lets a terminal be transformed independently.

*(Resolved 1 Nov 2026 — the first export had the two terminals merged into a single
`VDA PHEV2 Prismatic Cell` file. Its STEP held three solids, so the geometry was fine, but the
STL was one binary mesh with no body boundaries, which made the terminal-height and terminal-tilt
defect classes ungeneratable: both are injected by transforming the terminal relative to the can.
Re-exported as three files.)*

The assembly file names parts individually (`Front End Plate v6` / `Back End Plate v6`,
`Left Side plate v2` / `Right Side plate v2`, three `Bus bar long`, two `Bus bar short`), which
confirms the 3P4S busbar count. The generator instances the single exported part per position;
it does not need the left/right variants as separate files.

### Loading these meshes in Gazebo

- **Gazebo cannot load STEP.** It loads STL / OBJ / DAE / glTF. STEP stays the archive format.
- **STL carries no units, and Gazebo assumes metres.** These are exported in millimetres, so the
  SDF/URDF mesh tag needs `<scale>0.001 0.001 0.001</scale>` or the module loads **1000× too
  large**. This is the single most likely first failure when §4B.8 / task 2.6 begins.
- Each part's origin must sit at its named datum above (cell: centre of the can's bottom face),
  so the generator can place parts by transform alone.

### What the generator found when it loaded these — 1 Nov 2026

The exports do **not** satisfy the axes and origin rules above, so `defectgen` normalises them on
load rather than waiting on a re-export. The table lives in `defectgen/src/defectgen/parts.py`.

- **Seven of the twelve bodies are rotated 90° about Z** relative to "X along the module length":
  `cell_can`, both terminals, `cooling_plate`, `busbar_carrier`, both busbars and `cmc`. The other
  four already match.
- **Every origin sits at a body centroid**, not at a named datum. Normalisation puts the centre of
  each part's bottom face on the origin — which for a terminal is the seating face §4 asks for,
  so a height defect stays a pure +Z translation.
- Each load is checked against the config, so a re-export that silently changes orientation or
  size raises instead of producing a subtly wrong module.

**Two parts need re-exporting. Neither blocks the generator, both are recorded in §12.**

1. **`terminal_neg` is 38.5 wide against `terminal_pos`'s 22** — the fill-plug is merged into it.
   *(Corrected 3 Nov 2026.)* This was first read as making the module 153.5 wide. That was a
   generator bug, not the CAD: the part was placed by its bounding-box centre, which sits 8.25
   off the pad centre, and the plug hung outboard past the side plate. Checked against
   `cell_assembled`, the pad sits at the outboard end and the plug runs **inboard** (y 30.0 →
   68.5 on an unflipped cell). Terminals are now placed by their pad centre, and the module is
   151.0 wide. The same check found `+` and `−` on swapped sides, and flipped cells whose terminals
   had been moved without being turned. Both are fixed: a flipped cell is the whole cell rotated
   180°. What is left is minor: a height or tilt defect on a `−` terminal also moves the plug,
   which belongs to the can. Exporting the terminal alone still fixes that.
2. **`cmc` is non-manifold** where its components meet the board. It has no holes, so a ray cannot
   leak through it and nothing downstream is affected — the generator checks for holes rather than
   strict two-manifoldness for exactly this reason. Worth knowing, not worth fixing on its own.

---

## 8. Assembly sequence — start to end (what the robots do)

Modelled on a real prismatic module line: stacking → strapping → terminal positioning → busbar
placement → laser welding → end-of-line test
([line process](https://www.linkedin.com/pulse/prismatic-ev-lithium-battery-module-pack-assembly-line-sam-lau)).

| # | Step | Robot / station | Inspection after it |
|---|---|---|---|
| 1 | Cooling plate into fixture; dispense gap-filler bead | fixture + dispenser (simulated) | **Fixed Ranger3**: bead continuity and width |
| 2 | End plate A into fixture | GP7 | — |
| 3 | Cells 1–12 with pads, orientation per 3P4S pattern | **GP7**, cells located by overhead 2D camera | per-cell placement check → **correction loop, k = cell index** |
| 4 | End plate B, compress to ≈ 3 kN | GP7 + fixture | — |
| 5 | Side plates placed and welded | GP7 places, GP25 welds (simulated) | — |
| 6 | **Pre-carrier inspection** | **HC10DT multi-view** vs **fixed Ranger3** | cell pose, terminal height and tilt, polarity, missing cell |
| 7 | Busbar carrier (CCS subassembly) placed | GP4 | — |
| 8 | **Pre-weld inspection** | HC10DT | busbar pose, busbar-to-terminal gap |
| 9 | Laser weld every busbar–terminal joint | **GP25 + MOTOPAC-RL2D** remote scanner (simulated; `project_plan.md` §4A.7) | — |
| 10 | **Post-weld inspection** | HC10DT | seam continuity, width, spatter |
| 11 | Cell module controller placed, cover placed | GP4 | final visual (2D) |
| 12 | End-of-line electrical test | simulated pass/fail | — |

Reject/rework decisions happen at 6 and 8, **before the irreversible weld at 9**.

---

## 9. What to model in CAD, and at what fidelity

**The rule: model a thing if it can be hit, can block a sensor's view, or gets
measured. Everything else is a box, or is left out.**

Occlusion is the project's central question (§3.7.6 of the plan), so anything
that casts a shadow into a sensor's field of view has to exist in the scene —
but only as a shape, not as detail.

| Thing | Visual mesh | Collision | Notes |
|---|---|---|---|
| Module parts (§3–5) | your CAD, decimated | convex boxes | the only parts that get *measured* |
| Fixture / nest / pallet | simple CAD | boxes | defines where the part is; sets the 26V188-style contact damage |
| Conveyor | boxes | boxes | a frame, two side rails, a belt surface |
| Sensor bodies (Zivid, profilometer, Ranger3, cameras) | **box at datasheet outline** | same box | must exist: they occlude and they collide |
| Sensor bridge over the conveyor | boxes / extrusion profile | boxes | a fixed frame, **not** a motorised gantry |
| Robot grippers and weld head | simple CAD | convex hull | these occlude the arm's own view — the classic reason a view plan fails |
| End stops, clamps, cabling | omit or box | boxes | only where they intrude on a view |
| Robots | vendor URDF meshes | vendor collision | already in `src/_vendor` |

**Do not** put CAD-quality meshes in collision geometry. MoveIt and Gazebo both
want convex primitives; a raw STEP tessellation there will make planning crawl.
Keep triangle counts modest in visual meshes too (decimate in Blender or
`trimesh.simplify`), since the ray-cast sensor models in `defectgen` intersect
against them.

**Sensors: model the body, not the optics.** A floating frame would render
faster, but it would silently remove the housing that blocks a view, and
"where can the sensor actually see from" is the result we are after. A box with
the datasheet's outline and the optical origin placed correctly is enough.

---

## 10. Fixture, datums and pick features

This is the part that makes the line read as real automation rather than a
render. Worth modelling properly, and it is cheap.

**Locating scheme — 3-2-1 on the pallet.** Three pads define the primary plane
(Z, tip, tilt), two dowel pins define X/Y and rotation (one round pin, one
diamond pin so the part is not over-constrained), and one stop defines the last
translation. Every measurement in `defectgen` is expressed in the **pallet
datum frame**, not the world, which mirrors how a real cell reports results.

**Pick features to add to your parts:**

| Part | Pick method | Feature to model |
|---|---|---|
| Prismatic cell | vacuum on the 148 × 91 face, backed up by side clamping | flat, unobstructed pad area; keep the label/vent clear |
| Compression pad | vacuum, small pads | flat top face |
| End plate | vacuum or magnetic (if steel) | flat outer face |
| Side plate | vacuum, two pads | flat outer face |
| Busbar carrier (CCS) | vacuum on the frame's flat ribs + 2 locating pin holes | **2 pin holes**, Ø 6 (A), so the gripper repeats |
| Cover | vacuum, four pads | flat top |
| Cooling plate | vacuum | flat top, clear of the bead path |

Also define, per part, a **TCP offset** (where the gripper's tool point sits
relative to the part origin) and a **grip approach direction**. Both go in the
same YAML as the geometry, so the generator, MoveIt and your CAD agree.

**Why this matters beyond realism:** the nest clearance and the pin engagement
set the threshold at which a mis-placed part *touches tooling* — the mechanism
behind NHTSA 26V188 (plan §3.7.10, experiment §4B.9). Without a modelled nest,
that coupling has no geometry behind it.

---

## 11. Line layout: stations, conveyor, and why the part must move

**The module rides a pallet on a conveyor and indexes between stations.**
This is not decoration:

1. **The fixed line-scan baseline needs relative motion.** A Ranger3 builds a
   3D surface from successive profiles; with a stationary part it measures one
   line. Without a conveyor there is no fixed-sensor baseline, and the central
   comparison of §3.7.6 cannot be run.
2. **Cycle time needs a takt.** "Seconds per module" only means something if
   transport is in the budget alongside capture and motion.
3. It is how real prismatic module lines work.

**Simulate it cheaply.** Do not attempt friction-driven belt physics in Gazebo —
it is a well-known time sink. Attach the pallet to a **prismatic joint** along
the line and drive it with a position controller, or script the pose directly.
The belt surface is visual only.

**Station layout (matches the assembly sequence in §8):**

| Station | Contents | Sensors |
|---|---|---|
| S1 Cell placing | GP7, cell magazine/tray, pallet nest | overhead 2D camera (cell locating) |
| S2 Line-scan gate | a fixed bridge the pallet passes under | **Ranger3 + laser** (the baseline) |
| S3 Inspection cell | HC10DT, pallet stop, safety scanner | wrist: snapshot 3D + profilometer + 2D |
| S4 Busbar + weld | GP4 (carrier, CMC, cover), carrier magazine; **GP25 + MOTOPAC-RL2D** scanner head | (weld monitoring out of scope, §4B.2) |
| S5 Final | cover placing, EOL test stub | 2D camera |

S2 and S3 inspect the **same** module at the same stage, which is what makes
the fixed-vs-arm comparison a controlled experiment rather than two anecdotes.

---

## 12. Still open

- ~~Terminal spacing, pad size, vent size~~ — **resolved 26 Sep 2026** by the vendor drawing (§4).
- ~~L2 = 38.5 ambiguity~~ — **resolved 26 Sep 2026.** L7 is centre-to-centre, so L5 and L7 fix the
  terminals completely and L2 can only describe the fill-port plug. (An earlier reading of L7 as
  outer-edge-to-outer-edge, giving c-to-c ≈ 93, was wrong: it was argued from a symmetry that in
  fact holds under *both* readings, so it decided nothing.)
- The 0.1 mm adjacent-terminal tolerance: confirm from MDPI *Batteries* 6(3):47 directly.
- ~~Wall height vs cover~~ — **resolved 22 Oct 2026**: end and side plates raised to 101.5 (§5).
- ~~Busbar width~~ — **resolved 22 Oct 2026**: as-built 11 × 1.5, lengths 63.5 / 147.5 (§5).
- **CMC clashes with the cover by 0.2 mm.** At 3.2 thick on a carrier top of 102.0 it reaches
  105.2 against a wall top of 105.0. Lower it into the carrier, thin the board, or recess the
  cover. (§5 height check; strict `xfail` in `defectgen/tests/test_budgets.py`.)
- **Re-export `terminal_neg` without the merged fill plug** (§7). No longer affects placement or
  width (3 Nov 2026); only a `−` terminal height/tilt defect also moving the plug. Strict `xfail` in
  `defectgen/tests/test_parts.py`.
- ~~Reconcile 10 exported CAD components against the 12 bodies listed in §7~~ — **resolved
  1 Nov 2026**: the two terminals are merged into the cell file. The STEP holds them as separate
  solids; the STL does not.
- **Export terminal (+) and terminal (−) as their own STLs** (§7). Until then the terminal-height
  and terminal-tilt defect classes cannot be generated, because the cell STL is one merged mesh.
- Weld seam pattern, carrier and CMC dimensions: assumed, refine from photos of real VDA 355
  modules if useful.
- A module-line cycle time for the fixed-vs-arm comparison: not found yet.

**Not found anywhere free:** a CAD model of a PHEV2 cell or VDA 355 module. batterydesign.net's
paid downloads are spreadsheets (cell/module/pack databases, calculators), **not CAD**.
