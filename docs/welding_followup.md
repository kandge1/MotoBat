# Process-driven welds — follow-up project brief and research digest

*Written 9 Nov 2026. **Parked**: this is not MotoBat scope. MotoBat keeps its
idealised weld defects (`defectgen/defects.py`, §4B.2) and the 1.7 baseline
dataset. This file holds the design sketch and the research for a later,
welding-specific project, so none of it is lost or re-searched.*

Companion: [`cu_on_al_laser_welding.md`](cu_on_al_laser_welding.md), the
Cu-on-Al foil digest from 4 Nov. It already establishes that the GP4 cannot
carry a welding head, and that in a remote laser cell the robot places the
head while galvo mirrors draw the seam.

---

## 1. Scope line

| Stays in MotoBat | Goes to the follow-up |
|---|---|
| Idealised weld shapes: missing, short, offset, bowed, drifting, too wide or narrow, crown change | Welds **caused by** process settings and disturbances |
| Gap under the busbar → underfill, undercut, burn-through (already causal, `weld.py`) | Spatter as a time-ordered event, and its interactions with the bead |
| Spatter as independent drops near stitches | Heat tint (needs the 2D camera and a colour model) |
| Out-of-spec scoring against tolerances (1.7) | ISO 13919-2 levels as the tolerances (needs the standard) |
| The baseline dataset (1.7) | New dataset tiers added on top of it |

**Why the line is here:** MotoBat's claims are fixed vs arm coverage and
recall, the correction loop, and %GRR. None of them needs a physical weld
model. A process model is a project of its own, and it is only worth doing
with the calibration data listed in §4.

## 2. Reframing "the robot moved too fast" for *this* cell

The MotoBat weld station is a **GP25 with a MOTOPAC-RL2D remote laser
package**. Yaskawa's trade material says welding speed, laser output,
wobble and angle of incidence are set from the robot pendant ([dempa][dempa]).
In remote laser welding, **the robot places the head and galvo mirrors draw
the seam** (`cu_on_al_laser_welding.md` row 3). So the user's process
variables map onto this cell as follows:

| What you said | What it is in this cell | What it does to the weld (direction, from §4) |
|---|---|---|
| Robot moves too fast / slow | **Scanner seam speed** (not robot speed); robot speed only matters for on-the-fly welding | Faster → narrower bead, slightly less penetration ([Amada][amada], [Rosenthal trend][lia-ros]). Copper spatter falls above ~500 mm/s scanner speed ([scanner Cu][cu-scan]). Very fast → humping ([Nature Comms 2024][hump-psu]) |
| Not straight | **Seam position error** = robot pose error + scanner calibration + part position | Offset or drift. Already modelled as idealised `seam_deviation` / `weld_path_drift` |
| Wobble | **Beam oscillation** (intended): amplitude, frequency, trajectory | Wider bead, gap bridging, less spatter. Trajectory effects are under-studied ([wobble fluence][wobble]) |
| Job wobbles on the conveyor | **Part vibration during the weld** (unintended): spot position jitter, and focus error if vertical | Irregular width and penetration. **No direct study found** (§5) |
| Too much / too little power | **Laser power**, with focus offset (height error from fixture and terminals) | Power window is narrow for Cu/Al. Heat input sets IMC thickness, and with it strength and resistance ([Kumar 2021][kumar], [Al–Cu IMC][alcu-imc]) |
| Discolouration | **Heat tint**: oxide thickness set by peak temperature and time | Colour by interference. Needs the 2D camera; no validated predictive model found ([AWS D18.2 chart][tint-fab]) |

## 3. Design sketch — what the follow-up would build

**Principle: simulate the process as events in time; get the shapes as the
result.** Today the generator draws finished shapes. The follow-up draws a
beam trace (where the spot was, when, at what power) and derives the bead,
the spatter and the tint from it. `weld.py` already lays beads from beam
traces and supports wobble, so this extends what exists.

### 3.1 The model ladder (each rung calibrated by the one above)

1. **L1, analytical.**
   - Width from the Rosenthal moving line source, which predicts width well
     for partial and full penetration ([lia][lia-ros], [Eagar][eagar]).
   - Depth only to first order. It is the weak point of this rung
     ([Giudice 2020][giudice]).
   - Ripple spacing ≈ solidification speed / pool oscillation frequency
     ([Rappaz, NASA][ripple-nasa]).
   - Humping onset from the dimensionless humping index ([Nature Comms
     2024][hump-psu]), with the Rayleigh length/width > π criterion as a
     check ([hump review][hump-pmc]).

   Cheap enough to run per stitch in a dataset.
2. **L2, surrogate.** A fast model fitted to CFD results.
   - TU Wien released **73 high-fidelity CFD simulations of laser beam
     welding** (316L): weld pool and keyhole geometry, process parameters,
     beam intensity profiles, and Python surrogate-modelling code. AGPL-3.0,
     27.7 MB ([Zenodo 20413437][tuwien]).
   - It is steel, not Cu/Al. Its value is the method, not the numbers.
3. **L3, CFD, offline only.**
   - beamWeldFoam (OpenFOAM, SoftwareX 2022) and laserbeamFoam (ray tracing,
     multiple reflections in the keyhole) ([beamWeldFoam][bwf],
     [OpenFOAM keyhole][zenz]).
   - Used to generate tables for L2, never inside the dataset loop.

### 3.2 Spatter as events

- **Ejection** along the beam path:
  - size lognormal, speed Gaussian ([beam-oscillation spatter][spatter-osc]);
  - diameters 50–100 µm at low speed and 100–150 µm at high speed in one
    study ([motion tracking][spatter-track]);
  - in copper, large spatter comes from keyhole pressure bursts
    ([Cu spatter][cu-scan]).
- **Flight:** ballistic, so where a drop lands depends on its velocity and the
  geometry around the stitch.
- **Landing state decides the outcome:**

| Lands on | Outcome to model |
|---|---|
| cold busbar / cap | a drop: today's spatter |
| the path **ahead** of the beam | remelted when the beam arrives: a larger lump or an irregular bead section |
| solidified bead | a bump on the bead |
| an open burn-through hole | a partial plug |

None of these interaction outcomes has published geometry that we found; they
would be assumed (A) until measured.

### 3.3 Heat tint

- **Mechanism:** an oxide film's thickness gives its colour by interference.
  For stainless, colour bands map to approximate thickness, ~20 nm (straw) to
  > 200 nm (black) ([vendor table][tint-cxp], unverified).
- **What the model needs:** the thermal history (L1 gives peak temperature
  and time), an oxidation-kinetics law, and a thin-film colour formula. It is
  then rendered into the Zivid 2D image (§4B.3 defers the 2D camera).
- **Copper:** no temper-colour model found. This is a research item, not an
  implementation item.

### 3.4 What detection would need

MotoBat's detector deliberately treats anything within 0.25 mm of a bead as
part of the bead (§4B.4), so spatter **on** a weld is not reported. Detecting
the §3.2 interactions needs an *expected-bead-surface* model, so a lump stands
out against the bead, not against the busbar. This is the weld-inspection
problem itself, and the main research contribution of a follow-up.

### 3.5 Dataset

New tiers on top of the 1.7 baseline, never replacing it: a process tier
(settings × disturbances × seeds) scored the same way. The baseline stays the
reference that every later model is compared against.

## 4. Research digest — what exists

### 4.1 Path Robotics (the company you named)

- **No open-source code of their own.** Their GitHub org holds about 44–50
  repositories, almost all forks of upstream dependencies: ROS, ceres,
  pytorch3d, protobuf, gRPC, and **ros-industrial/motoman**
  ([org][path-gh], [review][path-api]). The Motoman fork suggests Yaskawa
  robots in their cells; that is an inference, not stated.
- **Their simulator is learned, not physical.**
  - The "Weld World Model" is a neural network trained on "tens of millions of
    welded inches" from their fleet over 8 years.
  - The Obsidian agent is trained by reinforcement learning inside it.
  - Their sensor stack: 2 cameras + 4 lasers around the torch, sub-mm point
    clouds, data before, during and after the weld.
  - First-pass yield of 97%+ is claimed.
  - All of this is company claims ([Path llms-full][path-llms],
    [Obsidian][path-obsidian]).
- **Their method is public through patents.** The "Autonomous welding robots"
  family covers:
  - scanning to a point cloud or mesh;
  - candidate seam identification;
  - path planning as a graph search (Dijkstra / A*) over feasible robot states
    per seam point (US 11648683, 11548162, 11759958, 11801606, 12070867);
  - simulated weld paths, US 12277369 ([Justia][path-patents],
    [US12277369][path-sim]).
- **What it means for us:** Path's world model is trained on data we will
  never have. A **physics- and geometry-based synthetic pipeline with exact
  labels** is the open, inspectable complement to it. That contrast makes a
  good talking point with them.

### 4.2 Open datasets

| Dataset | Process / material | Modality | Size, licence | Use to us |
|---|---|---|---|---|
| WMG / Basile 2025 ([Zenodo 15833960][wmg]) | Remote laser, **AA1050 busbar-to-terminal**, gap × power full factorial | Optical microphone, 183 runs | 2.2 GB, CC BY 4.0 | **Closest to our joint.** Gap and power effects on a battery busbar weld |
| AIMEN SAW repository ([Zenodo 15083865][saw]) | Submerged arc, X-joints | Process data, **3D bead profilometer**, photos, X-ray/UT | 12.7 GB, CC BY 4.0 | Real profilometer bead scans with NDT truth: noise and texture for our sensor models |
| TU Wien LBW CFD ([Zenodo 20413437][tuwien]) | Laser keyhole, 316L | Simulated pool / keyhole geometry + surrogate code | 27.7 MB, AGPL-3.0 | L2 surrogate method |
| GMAW bead width/reinforcement ([figshare][gmaw-fig]) | GMAW | Speed, voltage, current, wire feed → width, reinforcement | open (check licence) | Parameter → bead geometry for arc welding |
| GMAW thermography ([PMC12301811][gmaw-thermo]) | GMAW | Thermal images + laser-line profiles | open | Bead geometry from process signals |
| Fronius TPS/i production ([preprint][fronius]) | Arc, production | 642 weld time series | check | Real process variability |
| LoHi-WELD ([GitHub][lohi]) | Arc | 3,022 bead surface images, pores/stains | public | Surface-defect appearance |
| RIAWELC, SWRD, GDXray ([PMC review][xray]) | Various | X-ray images | public | Internal defects: out of our optical scope |
| Molten pool images ([Mendeley][pool]) | Arc | Pool images, penetration | open | Penetration from vision |

**Gaps found (the opportunity):**
- No public dataset of **laser-welded busbar beads as 3D profilometer point
  clouds**.
- No public **Cu–Al battery process map** (power × speed × wobble → IMC,
  resistance, strength); the papers say one must be assembled from studies
  or a DOE ([Kumar 2021][kumar], [Al–Cu IMC][alcu-imc]).
- No study of **robot or part vibration during welding** on bead geometry
  ([vibration search][vib]).
- No validated **heat-tint predictive model**, especially for copper.

### 4.3 Open-source projects

| Project | What it is | Relation |
|---|---|---|
| [hold_and_weld][hw] | ROS 2 Jazzy dual-arm welding: seams from CAD/mesh, MoveIt 2 + OMPL, weld paths as JSON | Same stack as our Phase 2 lane. Planning only, no weld physics |
| [seam_detection][seam] | ROS 1, PCL: seams from point clouds (RANSAC plane intersection, CAD-to-scan ICP) | Same registration idea as our detector |
| [romi-lab robotic-welding-demo][romi] | ROS 1 + Open3D, V-groove multilayer path planning | Path planning reference |
| [robotic-welding-platform (wwryy)][wwryy] | Isaac Sim synthetic RGB-D/point clouds, domain randomisation, weld path generation | Closest in spirit (synthetic data for welding). No licence; portfolio only |
| [ABatRe-Sim][abatre] | Gazebo simulator for robotic battery **disassembly** | Battery + robot sim, opposite direction |
| [UR10e battery assembly digital twin][ur10e] | RoboDK UI for module assembly | Assembly, no inspection |
| [WAAM multi-robot architecture][waam] | Open-source software for Motoman wire-arc additive manufacturing | Yaskawa + welding-process software |
| [beamWeldFoam / laserbeamFoam][bwf] | OpenFOAM laser welding solvers | L3 |

**Is MotoBat one of a kind?** Robotic welding and weld inspection are
well-trodden, commercially (Path Robotics and others) and academically, and
all the pieces exist above. **No open project combining a simulated battery
module line, labelled 3D inspection, fixed-vs-arm coverage and a correction
loop turned up in these searches.** The top search hit for "open source
robotic battery module assembly" was this repository
([kandge1/MotoBat][motobat]). That is "we found none", not "none exists".

### 4.4 Literature by topic

- **Cu/Al battery tab and busbar laser welding:**
  - Kumar et al. 2021, Warwick:
    - power windows of ~300–1350 W (Cu busbar) and ~300–1125 W (Al busbar);
    - contact resistance is less sensitive to IMC thickness than strength is;
    - Cu tab on Al busbar ≈ 560 N, Al tab on Cu busbar ≈ 520 N
    ([WRAP][kumar]).
  - Review of tab-to-busbar welding methods ([PMC12843912][review]).
  - Francioso 2024 (thicker Cu/Al lap joints for high current), DiMatteo 2019
    (beam oscillation on thin sheet), and Solchenbach & Plapper (IMC < 4 µm,
    51 MPa) ([search digest][alcu-imc]).
  - AIMS variable Al/Cu busbars ([AIMS][aims]).
- **Wobble:** amplitude and frequency are well studied, trajectory less so; a
  fluence model for wobble trajectories, C101 Cu / AA1060, 1070 nm
  ([ScienceDirect][wobble]).
- **Spatter:**
  - the spatter map ([RG][spatter-map]);
  - spatter with beam oscillation ([Physics Procedia][spatter-osc]);
  - spatter motion tracking ([JMP][spatter-track]);
  - copper spatter suppression by high-speed scanner welding
    ([ScienceDirect][cu-scan]);
  - green-laser beam shapes for copper ([Springer][green]).
- **Humping:**
  - Nature Comms 2024 (synchrotron X-ray + CFD, humping index, 1.00 m/s
    onset in 304) ([PSU][hump-psu]);
  - capillary instability ([PMC][hump-pmc]);
  - Mendez, arc-welding defect models ([UAlberta][mendez]).
- **Analytical thermal models:** Rosenthal line source for width
  ([LIA][lia-ros]); Eagar & Tsai ([MIT][eagar]); Giudice 2020 multipoint-line
  keyhole source ([Springer][giudice]).
- **Ripples:** Rappaz et al. (capillary standing waves, spacing = v/f)
  ([NASA][ripple-nasa]); GMAW ripple CFD by Tsai's group ([MST][ripple-mst]);
  Anthony & Cline for laser (not yet read).
- **Conduction / transition / keyhole regimes in aluminium:**
  - mode thresholds ~0.5–1 MW/cm² ([Amada][amada], a vendor blog);
  - porosity at the transition ([Zhao 2003][zhao]);
  - a 6061/5052/5086 RSM thesis ([Lakhal 2023][lakhal]).
- **Weld quality levels:**
  - ISO 13919-2:2021 (Al, Mg, **pure Cu**; levels B/C/D; ≥ 0.5 mm) is
    paywalled ([BSI][iso-bsi]).
  - Its limits are still unsourced, as in §4B.4.
  - ISO 5817 (steel) is the analogue for arc welds ([SSAB copy][iso5817]).
- **Generic arc-welding parameter charts (steel / aluminium MIG, with or
  without a Yaskawa robot):** free handbooks with MIG/MAG data tables from Air
  Products ([PDF][airproducts]) and Miller ([PDF][miller]).
  - Robot-held studies show welding current and speed dominate bead geometry
    ([pulsed GMAW][gmaw-incl]).
  - **Yaskawa publishes no parameter tables for the RL2D package**; the
    pendant sets them ([dempa][dempa]).

## 5. If the follow-up starts — first steps, by dependency

1. **Buy or borrow ISO 13919-2:2021**, which gives real tolerances.
   University libraries usually carry ISO.
2. **Download the WMG AA1050 busbar dataset and the AIMEN profilometer
   set.** The first anchors gap × power for our joint type; the second gives
   real bead-scan texture and noise.
3. **L1 model** (Rosenthal width, ripple spacing, humping check), calibrated
   against Kumar 2021 and the WMG set, with tolerances from step 1.
4. **Event-ordered spatter** (§3.2) and a bead-surface reference in the
   detector (§3.4).
5. **Heat tint last:** it needs the 2D camera, an oxidation law and a colour
   model, with no validated starting point.

The gaps in §4.2 are where a follow-up would add something new rather than
repeat.

## 6. Sources

[dempa]: https://aei.dempa.net/archives/358
[amada]: https://amadaweldtech.com/blog/laser-welding-modes-conduction-transition-keyhole-welding/
[lia-ros]: https://lia.org/node/12522
[cu-scan]: https://www.sciencedirect.com/science/article/abs/pii/S0924013616303211
[hump-psu]: https://pure.psu.edu/en/publications/unveiling-mechanisms-and-onset-threshold-of-humping-in-high-speed/
[hump-pmc]: https://pmc.ncbi.nlm.nih.gov/articles/PMC11538395
[wobble]: https://www.sciencedirect.com/science/article/pii/S0030399226005967
[kumar]: https://wrap.warwick.ac.uk/id/eprint/168639/1/WRAP-In-depth-evaluation-of-laser-welded-similar-and-dissimilar-material-tab-to-busbar-electrical-interconnects-for-electric-vehicle-battery-pack-Kumar-2021.pdf
[alcu-imc]: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9658164/
[tint-fab]: https://www.thefabricator.com/tubepipejournal/article/tubepipeproduction/keeping-stainless-steels-stainless
[tint-cxp]: https://cxp-solutions.com/guides/heat-tint-removal-guide/
[eagar]: https://eagar.mit.edu/publications/Eagar032.pdf
[giudice]: https://springerlink.bibliotecabuap.elogim.com/doi/10.1007/s00170-020-06479-0
[ripple-nasa]: https://ntrs.nasa.gov/api/citations/19990041942/downloads/19990041942.pdf
[ripple-mst]: https://scholarsmine.mst.edu/mec_aereng_facwork/1944
[tuwien]: https://zenodo.org/records/20413437
[bwf]: https://doaj.org/article/7965986e06474e2abecc380b9f4fc566
[zenz]: https://repositum.tuwien.at/bitstream/20.500.12708/189856/1/Zenz-2024-Computers%20and%20Fluids-vor.pdf
[spatter-osc]: https://www.sciencedirect.com/science/article/pii/S1875389213000618/pdf?md5=3fc0286e24693e6eed7ce01bfc72c8c8&pid=1-s2.0-S1875389213000618-main.pdf
[spatter-track]: https://www.sciencedirect.com/science/article/abs/pii/S152661252030236X
[spatter-map]: https://www.researchgate.net/publication/328349850_Laser_welding_The_spatter_map
[green]: https://link.springer.com/article/10.1007/s00170-024-13180-z
[path-gh]: https://github.com/path-robotics
[path-api]: https://github.com/api-evangelist/path-robotics
[path-llms]: https://www.path-robotics.com/llms-full.txt
[path-obsidian]: https://www.path-robotics.com/obsidian
[path-patents]: https://patents.justia.com/assignee/path-robotics-inc
[path-sim]: https://patents.google.com/patent/US12277369
[wmg]: https://zenodo.org/records/15833960
[saw]: https://zenodo.org/records/15083865
[gmaw-fig]: https://figshare.com/articles/dataset/Dataset_of_Reinforcement_and_Width_Estimation_in_GMAW_Weld_Bead_Process/27325698
[gmaw-thermo]: https://pmc.ncbi.nlm.nih.gov/articles/PMC12301811
[fronius]: https://www.preprints.org/manuscript/202503.1190
[lohi]: https://github.com/SylvioBlock/LoHi-Weld
[xray]: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC11079250/
[pool]: https://data.mendeley.com/datasets/k5cd79zt52
[vib]: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC11174035/
[hw]: https://github.com/silanus23/hold_and_weld
[seam]: https://github.com/thillRobot/seam_detection
[romi]: https://github.com/romi-lab/robotic-welding-demo
[wwryy]: https://github.com/wwryy/robotic-welding-platform
[abatre]: https://arxiv.org/pdf/2303.07617
[ur10e]: https://github.com/AlbertoPrats/UR10e-Digital-Twin-System-for-Automated-Robotic-Battery-Assembly
[waam]: https://arxiv.org/pdf/2408.04677
[motobat]: https://github.com/kandge1/MotoBat
[review]: https://pmc.ncbi.nlm.nih.gov/articles/PMC12843912
[aims]: https://www.aimspress.com/aimspress-data/aimsmates/2022/6/PDF/matersci-09-06-053.pdf
[mendez]: https://sites.ualberta.ca/~pmendez/Publications/Papers/2003_wj_defect.pdf
[zhao]: https://modeling.matse.psu.edu/research_files/papers/2003JAP_Zhao.pdf
[lakhal]: https://semaphore.uqar.ca/2906/1/Abdessamad_Lakhal_novembre2023.pdf
[iso-bsi]: https://knowledge.bsigroup.com/products/electron-and-laser-beam-welded-joints-requirements-and-recommendations-on-quality-levels-for-imperfections-aluminium-magnesium-and-their-alloys-and-pure-copper
[iso5817]: https://weldcalc.ssab.com/sisStandards/ISO%205817_Ed_4_2023.pdf
[airproducts]: https://airproducts.co.uk/-/media/files/en/welders-handbook.pdf
[miller]: https://www.millerwelds.com/~/media/miller%20electric/imported%20mam%20assets/literature/2/6/5/educational%20resources%20and%20product%20guide.pdf
[gmaw-incl]: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7321599/

Reference-style links above. Rendered link targets, for plain-text readers:
Yaskawa RL2D package (dempa) · Amada modes blog · LIA Rosenthal · Cu scanner
spatter · Nature Comms humping (PSU) · humping review (PMC) · wobble fluence
model · Kumar 2021 (Warwick) · Al–Cu IMC (PMC) · Fabricator heat tint · CXP
tint table · Eagar & Tsai · Giudice 2020 · Rappaz ripples (NASA) · Tsai ripples
(MST) · TU Wien CFD dataset · beamWeldFoam · Zenz 2024 · spatter with
oscillation · spatter tracking · spatter map · green laser Cu · Path Robotics
GitHub / review / llms-full / Obsidian / patents / US12277369 · WMG AA1050 ·
AIMEN SAW · GMAW figshare · GMAW thermography · Fronius · LoHi-WELD · X-ray
datasets review · molten pool · vibration-assisted welding · hold_and_weld ·
seam_detection · romi-lab · wwryy · ABatRe-Sim · UR10e twin · WAAM · MotoBat ·
tab-to-busbar review · AIMS Al/Cu busbars · Mendez · Zhao 2003 · Lakhal 2023 ·
ISO 13919-2 (BSI) · ISO 5817 · Air Products handbook · Miller guide · pulsed
GMAW torch inclination.
