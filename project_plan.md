# MotoBat — Project Start
 
**Battery Cell Assembly Line: Multi-Robot Inspection & Correction System**
A simulated closed-loop defect detection and correction system for lithium-ion
battery cell manufacturing, built in NVIDIA Isaac Sim.
 
Rhishikesh Kandge — M.S. Electrical and Computer Engineering, The Ohio State University
 
---
 
## Who is building this, and why
 
**Author:** Rhishikesh Kandge — M.S. Electrical and Computer Engineering,
The Ohio State University. Graduating late August 2026.
 
MotoBat is a self-directed research and engineering project — not coursework,
not lab work. Its purpose is a defensible result: a simulated multi-sensor
inspection cell that detects documented structural defect classes in
lithium-ion cell assembly and closes a correction loop before the irreversible
sealing step, with measured detection performance and a characterised
stability boundary.
 
**§3.6 states the claims this project must defend** and what separates a
research artifact from a demo. That section is the standard the work is held
to.
 
### Relevant background
 
- **ROS 2:** tutorials once, ~1 year ago. Currently rebuilding depth.
- **Built:** a vision-driven pick-and-place system on an Elephant Robotics
  myCobot 280 Pi — **no hardcoded coordinates**. The arm located AprilTag-marked
  zones, identified blocks by colour and shape within the pickup zone, and
  assembled composite structures (triangular prism on square = "house").
  That is a complete perception → planning → manipulation loop.
- **Built:** a control system for myCobot Pro 450 arms — highly compliant, with
  significant backlash — achieving **sub-1 mm repeatability**. Compensating
  backlash on a compliant low-cost arm to that tolerance is a genuine controls
  result, and harder than hitting the same number on a rigid industrial arm.
- **Studying:** John J. Craig, *Introduction to Robotics: Mechanics and Control*.
### Gaps this project is meant to close
 
Prior lab work centred on making rudimentary hardware behave, rather than on
the primary research it was meant to support. The planned multi-robot assembly
system was never built. Industrial-grade tooling (Jetson AGX Orin, ROS 2 at
scale, industrial arms, MoveIt2, Isaac Sim) was not part of that experience.
 
MotoBat exists to close exactly that gap: real industrial robot models, real
ROS 2 architecture, real sensor simulation, and a real defect-detection
contribution.
 
### Working principle for this repository
 
> **Every subsystem must be documented with the mathematics that governs it,
> not just the code that implements it.**
 
The goal is to understand *what* is being computed and *why* — from equations
and block diagrams — without reading generated code line by line. Where an
implementation is AI-assisted, the maths must be written down first, so the
code can be checked against it.
 
Practically, that means every module gets:
 
1. The governing equations, with symbols defined
2. A block or data-flow diagram showing where it sits
3. Units and frames stated explicitly
4. A citation to the textbook or paper it comes from
§9 collects the mathematics. §10 lists the study resources.
 
---
 
## What this document is
 
The handoff from environment setup to actual development. It records what is
built and verified, the decisions made and why, and the concrete next milestone:
**one HC10DT arm, planned in RViz via MoveIt2, moving in Isaac Sim.**
 
Read this first if you are picking the project up cold — including Claude Code.
 
**§9 is the mathematics** behind every subsystem, and **§10 the study
resources.** Per the working principle above, implementations are checked
against the equations in §9 — write the maths down before writing the code.
 
---
 
## 0. Current status
 
| Area | State |
|---|---|
| Ubuntu 24.04 LTS + ROS 2 Jazzy | Installed, verified (talker/listener) |
| MoveIt2, ros2_control, rqt, RViz2 | Installed |
| Open3D, PCL, OpenCV, Python 3.11 venv | Installed |
| PyTorch CUDA | **Broken** — see §1.1 |
| Yaskawa GP4/GP7/HC10DT descriptions | Cloned, verified |
| `motobat_description` (4-arm line) | Written, xacro compile-tested |
| Four arms visualized in RViz | Blocked on §1.2 (one-line fix) |
| Isaac Sim | Deferred — hardware repair pending (§1.0) |
| MoveIt config for GP4 / HC10DT | **Next work** — §4A |
| Sensor flange CAD | **Next work** — §4A.4 |
| Defect generation + detection | **Next work** — §4B (prototyped, ~10 MB) |
| Memory upgrade | Decided: 2 x 16 GB DDR5 kit, deferred — §1.0 |
 
> **Current plan:** Dell replaces the motherboard onsite **25 Aug 2026** under
> warranty (§1.0), restoring 16 GB. Until Isaac Sim is viable, the active work
> is **§4A** (RViz2 + Gazebo: arms, IK, sensor flange) and **§4B** (defect
> generation and detection in numpy/Open3D). Both run in 8 GB today. Isaac Sim
> (§4) resumes after the repair and the memory upgrade.
>
> **Target: 1 Oct 2026** (§3.5). The bar the work is held to is **§3.6**.
 
---
 
## 1. Known issues to clear first
 
### 1.0 HARDWARE: dead DIMM slot — 8 GB ceiling
 
**Machine:** Dell G15 5530 · i9-13900HX (24C/32T) · RTX 4060 Laptop (8 GB VRAM)
· BIOS 1.34.0 · Service Tag `BZC6524` · purchased in India, in use in the US.
 
**Finding: memory slot DIMM A is dead. Both RAM modules are healthy.**
 
Diagnosed by physically swapping the modules between slots and re-reading
`dmidecode` both times:
 
| | Before swap | After swap |
|---|---|---|
| DIMM A | `No Module Installed` | `No Module Installed` |
| DIMM B | 8 GB, serial `442BC272` | 8 GB, serial **`442BC255`** |
 
The serial in slot B changed, proving a *different* physical module was moved
there — and it works. Slot A reads empty with either module in it. Confirmed
independently by SMBIOS type 19: `Memory Array Mapped Address → Range Size: 8 GB`.
 
Both modules are Micron `MTC4C10163S1SC48BA1`, 8 GB DDR5-4800, Rank 1.
 
**Conclusion:** motherboard fault, not a memory fault. **Buying RAM would not
fix this** — a new module in slot A would also read as absent. The board
supports 64 GB across 2 slots, but only slot B is functional, so the machine
is capped at whatever a single SODIMM holds.
 
A bluescreen some months earlier is the likely moment slot A failed. The system
has been silently running on one module since; nothing announces the loss.
 
**This almost certainly caused the earlier multi-day Isaac Sim debugging.**
Isaac Sim was being run on 7.4 GB visible RAM against a documented 32 GB
minimum, while believing the machine had 16 GB. Every OOM and freeze looked
like an Isaac Sim problem. It was not.
 
**Diagnostic worth reusing:**
 
```bash
free -h                                    # what Linux sees
sudo dmidecode -t memory | grep -E "Locator|Size|Serial"
```
 
If firmware reports `No Module Installed` for a populated slot, the fault is
below the OS. Swapping modules between slots distinguishes a bad module from a
bad slot — the serial number is what proves which module is where.
 
**Do not bother with memtest86+ here.** Both modules are proven good by the
swap test, and memtest cannot test a socket that reports nothing.
 
#### Warranty path — bought in India, used in the US
 
Roughly one year of warranty remains, but it was purchased in India and Dell
service contracts are **region-bound**: "the service contract purchased for
that computer at the point of sale is valid only in the country or region in
which the computer was purchased."
 
Dell's international support covers travel of **under six months**. Beyond
that, it is a permanent relocation and needs a transfer.
 
**Steps, in order:**
 
1. **Check current coverage.** dell.com/support → enter Service Tag `BZC6524`.
   Note the exact service plan name — the plan type decides everything below.
2. **File an Ownership/Warranty Transfer** at Dell's Ownership Transfer page,
   selecting the country/region relocation reason ("permanently relocated to
   another country for 6 months or more"). Requires the Service Tag and both
   old and new owner details; keep the India proof of purchase ready.
   **~15 business days** for international transfers. Dell representatives
   cannot file it for you — privacy policy requires the owner to submit.
3. **Then request service** through US Dell support with the transferred tag.
**Manage expectations on the service type.** Onsite next-business-day service
— an engineer coming to your home, as is common with Dell India consumer
plans — **is unlikely to carry over.** Dell states that "Return to Depot,
Complete Care, Same Day, and Parts Only contracts are not always available for
transfer outside of the region the coverage was purchased in." Realistically,
expect **mail-in depot service** in the US: ship the laptop to a Dell facility,
lose it for one to two weeks.
 
**If the transfer is refused,** paid options are a Dell out-of-warranty depot
repair or a local shop board-level repair. Motherboard replacement on a G15
runs roughly $300–500 with labor.
 
#### The decision this forces
 
DDR5 SODIMM prices rose sharply through 2026 — 2×16 GB now runs around $400.
Combined with a dead slot that makes new RAM useless, **buying memory is off
the table.** The realistic options:
 
| Option | Gets you | Cost | Downside |
|---|---|---|---|
| **Warranty repair** | Back to 16 GB, dual-channel | Free if approved | ~15 business days transfer + 1–2 weeks depot; laptop gone |
| **Paid board repair** | Back to 16 GB | ~$300–500 | Cost; still under 32 GB |
| **Single 32 GB SODIMM in slot B** | 32 GB single-channel | High at current prices | Single-channel bandwidth penalty; verify G15 supports a single 32 GB module |
| **Remote/lab machine** | Full 32 GB+ capability | Free if OSU provides | Requires access; container workflow |
| **Stay at 8 GB** | Nothing | Free | Isaac Sim not realistically usable |
 
#### RESOLVED — Dell onsite repair approved
 
Dell approved a **free onsite motherboard replacement in the US**, despite the
machine being purchased in India. Better than the region-bound policy predicted;
worth noting that the documented policy is not always the outcome — it was worth
asking.
 
Post-repair the machine returns to **16 GB dual-channel**. Still under Isaac
Sim's documented 32 GB minimum, but workable for single-arm scenes with
simplified collision geometry.
 
**After the technician leaves, verify the fix:**
 
```bash
free -h                                              # expect ~15 Gi total
sudo dmidecode -t memory | grep -E "Locator|Size|Serial"
```
 
Both DIMM A and DIMM B should report 8 GB with distinct serials. If DIMM A
still reads `No Module Installed`, the board swap did not resolve it — raise it
before the case closes.
 
While waiting, **§4A** and **§4B** are the work plan. Neither needs Isaac Sim.
 
#### Memory upgrade — decision and slot math
 
Post-repair the machine has **two working slots, both holding Dell's 8 GB
DDR5 modules**. Any upgrade must account for that:
 
| Purchase | Install | Result | Notes |
|---|---|---|---|
| 1 x 16 GB now | pull one 8 GB | **24 GB** | Intel Flex Mode; nets only +8 GB |
| 1 x 16 GB later | pull the other 8 GB | **32 GB** | both 8 GB modules now spare |
| **2 x 16 GB kit at once** | pull both 8 GB | **32 GB** | matched pair, dual-channel |
 
**Recommendation: save for the 2 x 16 GB kit and buy it in one purchase.**
The incremental path reaches the same 32 GB but passes through a month of
mismatched memory and nets only +8 GB for the first payment. Timing favours
waiting: §4A and §4B are 6-8 weeks of work that run comfortably in 16 GB, so
the kit arrives about when Isaac Sim actually becomes the bottleneck.
 
**DDR5 ONLY — DDR4 is not an option.** A 32 GB DDR4 kit at half the price is
not a viable substitute: DDR4 and DDR5 SODIMMs differ in pin count, notch
position, and voltage regulation (DDR5 carries its PMIC on the module). A DDR4
module will not physically seat in this machine. Confirmed by `dmidecode`:
`Type: DDR5`, `Configured Voltage: 1.1 V`. Buying DDR4 would be money lost.
 
Buying notes:
 
- Buy a **kit**, not two separate modules — kits are validated as a matched pair
- Spec: DDR5 SODIMM, 4800 MT/s or faster, 1.1 V, non-ECC
- Faster kits (5600) clock down to platform speed; fine, but no benefit
- **Keep the 8 GB modules.** They are the known-good fallback for the same
  swap test that diagnosed the dead slot
- Even 32 GB is Isaac Sim's *minimum* (64 GB ideal). The single-arm milestone
  will be fine; the full four-station line will still be tight
**Worth doing before spending anything:** ask the OSU ECE department about GPU
workstation access. A lab machine with 64 GB beats any laptop configuration and
is exactly the scenario Isaac Sim's container mode targets (§4.0).
 
### 1.1 PyTorch is broken
 
The installer ran `pip install torch --index-url .../cu124` (got `2.6.0+cu124`),
then ran `pip install stable-baselines3[extra]` **without** the index URL. Pip
resolved torch from PyPI, found `2.13.0`, and upgraded over the CUDA build.
 
Result: `torchvision 0.21.0` and `torchaudio 2.6.0` both pin `torch==2.6.0` and
are broken, plus ~4 GB of redundant CUDA 13 wheels alongside the cu124 ones.
 
```bash
./fix_torch.sh
```
 
Reinstalls all four together in one resolve, removes the stray CUDA 13 runtime,
verifies `torch.cuda.is_available()`. Does not block RViz or Isaac Sim work —
only the Section 5.4 RL stretch goal.
 
**Lesson worth keeping:** any `pip install` that can pull `torch` as a transitive
dependency must either carry the same `--index-url` or use `--no-deps`.
 
### 1.2 `bootstrap_rviz.sh` fails with "AMENT_TRACE_SETUP_FILES: unbound variable"
 
`set -u` collides with `/opt/ros/jazzy/setup.bash`, which reads undefined
variables intentionally. Fixed in the current bundle — the script now uses
`set -Eeo pipefail` without `-u`.
 
**This applies to every shell script in the project that sources ROS.** Do not
use `set -u` in anything that sources a ROS setup file.
 
### 1.3 Minor, already handled
 
- `ros-jazzy-industrial-msgs` does not exist in apt; vcs pulls it from source.
- protobuf version mismatch with `grpcio-tools` — only matters when generating
  gRPC stubs. `fix_torch.sh` pins it.
- numpy correctly held at `1.26.4`. **Do not upgrade to 2.x** — Open3D wheels
  and several Jazzy Python packages still link the 1.x ABI.
---
 
## 2. Environment
 
**Ubuntu 24.04 LTS + ROS 2 Jazzy.** Not 26.04/Lyrical: Isaac Sim 6.0 supports
22.04/24.04 only, and its ROS 2 bridge names Jazzy on 24.04 as the recommended
configuration. Jazzy is supported to May 2029 — past both project phases.
 
### Layout
 
```
~/motobat/
├── venv/                     Python 3.11 (matches Isaac Sim's interpreter)
├── motobat_env.sh            sourced from ~/.bashrc
└── ros2_ws/
    └── src/
        ├── motobat_description/   4-arm line, launch files, RViz config
        ├── motobat_bringup/       (empty) launch composition, Isaac scene setup
        ├── motobat_msgs/          (empty) DefectReport, ScanResult, CorrectionCmd
        ├── motobat_stacker/       Station 1 — GP4  (MoveIt config: §4A.1)
        ├── motobat_welder/        (empty) Station 2 — GP4
        ├── motobat_inspector/     Station 3 — HC10DT (§4A.2, flange §4A.4)
        ├── motobat_sealer/        (empty) Station 4 — GP7
        └── _vendor/               cloned deps — gitignore this
```
 
Aliases: `mb-venv` `mb-ws` `mb-build`
 
### Python: two interpreters, on purpose
 
System Python is 3.12. The venv is **3.11** because Isaac Sim 5.x/6.x embed
CPython 3.11 — importing Open3D from 3.12 into Isaac Sim's interpreter produces
ABI errors that look like Open3D bugs but aren't.
 
Use `mb-venv` for Open3D/PyTorch work. Never `pip install` into system Python;
24.04 blocks it via PEP 668 and overriding will eventually break apt.
 
### Hardware
 
Dell G15 5530 · i9-13900HX (24C/32T) · RTX 4060 Laptop (8 GB VRAM) ·
driver 595.84 · 854 GB free. GPU, driver, and disk all pass.
 
**RAM is 8 GB against Isaac Sim's documented 32 GB minimum, and cannot be
upgraded — slot DIMM A is dead (§1.0).** 35 GB of swap is configured, which
prevents hard OOM kills but does not make the machine adequate.
 
This is *the* binding constraint on the project. §4.0 covers what it means for
sequencing.
 
The CPU is excellent (24 cores) and the GPU meets the 8 GB VRAM minimum — this
is a memory-capacity problem specifically, not a weak machine.
 
---
 
## 3. What is built and verified
 
### `motobat_description`
 
Four arms in one scene, each instantiated from the Yaskawa xacro macros with a
distinct prefix so they share one TF tree without name collisions.
 
| Station | Role | Arm | Prefix | Macro |
|---|---|---|---|---|
| 1 | Stacker | GP4 | `s1_` | `motoman_gp4` |
| 2 | Tab Welder | GP4 | `s2_` | `motoman_gp4` |
| 3 | Inspector | HC10DT | `s3_` | `motoman_hc10dt` |
| 4 | Can Sealer | GP7 | `s4_` | `motoman_gp7` |
 
**Verified by compile test:** 39 links, 38 joints, 24 movable (4 × 6), single
`world` root, no missing joint limits.
 
```bash
tar xzf motobat_rviz_bundle.tar.gz
./bootstrap_rviz.sh
 
# sliders for all 24 joints
ros2 launch motobat_description view_line.launch.py
 
# or scripted motion (terminal 1, then terminal 2)
ros2 launch motobat_description view_line.launch.py gui:=false
ros2 run motobat_description demo_line_motion.py
```
 
> **`demo_line_motion.py` is a smoke test, not architecture.** It publishes
> `/joint_states` directly — no physics, no collision checking, no controllers.
> It proves the descriptions, TF tree, and meshes are correct. Delete it once
> real controllers exist so it does not quietly become load-bearing.
 
### Robot descriptions
 
`ros-industrial/motoman` is **ROS 1 only**. ROS 2 descriptions come from the
[`gavanderhoorn/motoman`](https://github.com/gavanderhoorn/motoman/tree/ros2)
fork, `ros2` branch — ament_cmake packages with meshes.
 
**On the HC10:** the fork ships `hc10`, `hc10dt`, and `hc10dt_b10` — there is no
`hc10dtp`. DTP is the IP67-rated variant of the DT with identical kinematics and
reach, so `hc10dt.xacro` is correct. Worth one sentence in Section 2 of the
proposal so it does not read as a spec mismatch.
 
---
 
## 3.5 Timeline to a portfolio-ready result — target 1 Oct 2026
 
**Scoped goal:** simulated structural defects in a battery cell stack, plus one
HC10DT carrying a sensor suite that scans them and reports what it found.
 
**That goal is achievable by 1 October.** The full four-station line is not, and
should not be attempted on this schedule. Narrowing to one arm is the right
call regardless of time — it is the honest scope for a solo six-week build, and
a working single-station system demonstrates more than a half-built line.
 
| Week | Dates | Focus | Depends on |
|---|---|---|---|
| 1 | Aug 21–27 | §4B.1–4B.3 stack generator, defect injection, simulated scan | nothing — pure numpy, runs on 8 GB today |
| 2 | Aug 28–Sep 3 | §4B.4–4B.5 Open3D detection + labelled validation set | week 1 |
| 3 | Sep 4–10 | §4A.1–4A.2 GP4 and HC10DT MoveIt configs, marker IK in RViz | 16 GB (post-repair) |
| 4 | Sep 11–17 | §4A.4 flange CAD, sensor placement study, URDF frames | week 3 + §4B.0 scan-window requirement |
| 5 | Sep 18–24 | Isaac Sim install, HC10DT import, OmniGraph, MoveIt bridge | **RAM — see risk below** |
| 6 | Sep 25–Oct 1 | Integration, results, README, demo video | all above |
 
### The one real risk: weeks 5–6
 
The memory upgrade (§1.0) is deferred roughly two months, which lands **after**
this deadline. Weeks 5–6 therefore run Isaac Sim on **16 GB against a documented
32 GB minimum**.
 
A single HC10DT with simplified collision meshes is far lighter than Isaac Sim's
worst case, and 35 GB of swap is configured, so it will probably run — slowly.
But this is the part of the plan that can slip.
 
**Primary mitigation: rent a cloud GPU for weeks 5–6 (§4.0b).** 40–80 hours on
an L4 instance costs roughly $20–60 — an order of magnitude less than the RAM
kit, and it removes the hardware purchase from the critical path entirely.
 
**Fallback, decided in advance:** if Isaac Sim will not cooperate by ~Sep 24,
ship the Gazebo version. Gazebo with a depth camera on the HC10DT produces the
same demonstration — arm scans defective stack, detector reports the defect — at
lower sensor fidelity. Swap the hardware interface, keep everything else (§4A.3).
 
This fallback exists because of how the work is ordered: **§4B is the actual
research contribution and completes in week 2, independent of any simulator.**
Even in the worst case, there is a real result with real precision/recall
numbers by early September.
 
### Working pace
 
Six weeks at a sustainable pace beats four weeks of 16-hour days followed by
two weeks of exhaustion. This is stated as an engineering observation, not
advice about life: most of what remains is debugging — TF frames, joint-name
mismatches, ICP convergence, Isaac Sim's ActionGraph. Debugging while tired is
where days disappear, and the failure modes in §4.4 are exactly the kind that
look like software bugs and are not.
 
The schedule above assumes roughly 6–8 focused hours a day and has slack in it.
It does not require 16-hour days, and the project will finish sooner without
them.
 
---
 
## 3.6 What makes this read as research rather than a demo
 
The goal is work that stands up next to a published paper. That is a specific
bar, and it is **not** about scope, polish, or how impressive the simulation
looks. A four-station line with no measurements impresses no one who reads
carefully. A single arm with a defensible quantitative claim does.
 
The currency is: **a claim someone could disagree with, defended with numbers.**
 
### The claims MotoBat can actually defend
 
Three, all reachable within the §3.5 timeline:
 
**1. A negative result, already found.**
Top-down height mapping is insufficient for lateral electrode misalignment —
the primary defect class and the Nissan 26V188 fault mode. Measured in §4B.0:
a 0.8 mm layer shift produced a −2.3 µm max-Z change and **zero** footprint
change, while lateral envelope deviation gave a clean monotonic signal
(+200 µm at 0.2 mm shift, rising linearly to +2000 µm at 2.0 mm).
 
Negative results with a mechanism are exactly what papers are built from. This
one says something specific about sensing modality selection that a practitioner
could act on, and it was found by building rather than assuming.
 
**2. A sensor-placement requirement derived, not guessed.**
The scan window must exceed the nominal part envelope, because material that has
moved outside the expected footprint is invisible to a scan covering only that
footprint. This converts flange design (§4A.4) from a modelling exercise into a
requirement with a numerical justification — and if the four-sensor suite does
*not* fit within that constraint, **that is also a publishable finding.**
 
**3. A closed-loop stability characterisation.**
The correction loop (§9.9) has a derived stability bound, `0 < K < 2/g`. A sweep
of K showing convergence, critical damping, and divergence — plus open-loop vs
closed-loop defect rate — is a controls result, not a demo. Most simulation
projects show one tuned gain that works. Showing the boundary demonstrates the
system is understood.
 
### What turns these into a research artifact
 
| Deliverable | Why it matters | When |
|---|---|---|
| Labelled defect dataset + generator | Reproducible by others; the generator *is* a contribution | Week 2 |
| Precision/recall per defect class, with error bars | Benchmarkable against the 62.07 % Hi-Pot and 93.82 % YOLOv4 figures (Section 1.2) | Week 2 |
| Minimum detectable defect per class | Directly comparable to the 0.5 mm production threshold (Section 1.1) | Week 2 |
| K-sweep: convergence → divergence | Controls result with a derived bound | Week 6 |
| Open-loop vs closed-loop defect rate | The headline claim of the whole project | Week 6 |
| Public repo, documented maths (§9), reproducible runs | The difference between "I built a thing" and "here is a method" | Throughout |
 
### The write-up is not optional
 
The single highest-leverage hour in this project is spent on the README and
results write-up, not on code. Structure it as a paper:
 
1. **Problem** — pre-seal geometric inspection, grounded in NHTSA 26V188
2. **Method** — architecture, the maths from §9, the detection pipeline
3. **Results** — the tables above, with plots
4. **Limitations** — simulation-only, geometric defects only, single station
5. **Future work** — carbon-fibre layup generalisation (proposal Section 6)
Stating limitations plainly is what separates research writing from marketing.
It also pre-empts the first question any reviewer or interviewer will ask.
 
### Publication paths, honestly assessed
 
| Path | Feasible by Oct? | Notes |
|---|---|---|
| **arXiv preprint (cs.RO)** | **Yes** | No peer review. Needs one endorsement for a first submission — any robotics author can give it, **not necessarily the advisor**. Produces a citable artifact with a permanent link. |
| **Workshop paper** | Yes, next cycle | 4–6 pages, far higher acceptance than the main track, real venue. ICRA/IROS workshop CfPs open periodically — track them. |
| **Full conference paper** (ICRA/IROS) | No | Main-track deadlines and quality bar do not fit six weeks of solo work. A rushed rejection helps nothing. Target a later cycle with this work as the foundation. |
 
**Recommended:** finish the project, write it properly, post the preprint. Then
extend toward a workshop paper for the next cycle. The preprint is a real,
linkable, citable thing — and unlike a LinkedIn post, it is permanent.
 
### On the comparison to other students' papers
 
A published paper and a rigorous engineering project are different currencies,
and they trade differently depending on the buyer.
 
- **Industry robotics roles** — a working system with quantitative results, a
  clean repo, and a demo video is often *worth more* than a paper. Hiring
  managers can evaluate a repo directly; most cannot evaluate a paper outside
  their subfield.
- **Academic RA / PhD applications** — papers weigh more, but a substantial
  independent project with a preprint is legible evidence of research capability,
  and it demonstrates something a co-authored paper does not: the ability to
  scope, execute, and finish a project alone.
The scoped MotoBat result — a simulated multi-sensor inspection cell that
detects real documented defect classes and closes a correction loop, with
measured precision/recall and a stability characterisation — is not a small
project. Done properly and written honestly, it competes.
 
---
 
## 4A. ACTIVE WORK PLAN — RViz2 + Gazebo (no Isaac Sim)
 
**This is what to work on now.** Everything here runs in 8 GB and none of it is
throwaway: the MoveIt configs, the flange, and the controller wiring all carry
directly into Isaac Sim afterwards.
 
The strategic point: once `MoveIt2 → FollowJointTrajectory → Gazebo` works,
swapping Gazebo for Isaac Sim is a **hardware-interface change, not a rewrite**
— the same seam that makes the MotoROS2 hardware story true (§6). Doing Gazebo
first de-risks Isaac Sim rather than delaying it.
 
### 4A.1 — GP4 with MoveIt2 in RViz (start here)
 
Goal: drag the interactive marker in RViz, hit Plan & Execute, watch the GP4
follow.
 
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
 
Load `motoman_gp4_support/urdf/gp4.xacro`. Then:
 
- Generate the self-collision matrix
- One planning group: `gp4_arm`, kinematic chain `base_link` → `tool0`
- Add named poses: `home`, `ready`
- Output to `src/motobat_stacker/config/`
**Use the analytic IK solver, not KDL.** The GP4 has an ortho-parallel basis
and spherical wrist, and the vendor package ships the parameters for it:
 
```
motoman_gp4_support/config/opw_parameters_gp4.yaml
    a1: 0.000   a2: -0.015   b: 0.000
    c1: 0.330   c2: 0.260    c3: 0.290   c4: 0.072
    joint_offsets:          [0, 0, deg(-90), 0, 0, deg(180)]
    joint_sign_corrections: [1, 1, -1, -1, -1, -1]
```
 
Install `moveit_opw_kinematics_plugin` and set it in `kinematics.yaml` instead
of the default `KDLKinematicsPlugin`. Closed-form IK is faster and returns all
8 solution branches rather than one numerically-converged answer — the same
choice a real integrator makes, and worth a sentence in the proposal.
 
### 4A.2 — HC10DT: expect this to be harder, and know why
 
**The HC10DT is NOT an ortho-parallel/spherical-wrist arm, so OPW does not
apply.** Verified from the URDF joint origins:
 
| Joint | GP4 origin (xyz) | HC10DT origin (xyz) |
|---|---|---|
| `joint_4_r` | `0.290 0 0.015` | `0 0 -0.500` |
| `joint_5_b` | `0 0 0` | **`0 0.162 0`** ← wrist offset |
| `joint_6_t` | `0 0 0` | `0 0 0` |
 
On the GP4, joints 4/5/6 intersect at a point — a spherical wrist, which is
what makes the closed-form OPW solution valid. On the HC10DT, `joint_5_b` is
offset 162 mm in Y, so the wrist axes do **not** intersect. This is normal for
collaborative arms and is precisely why the vendor package ships no
`opw_parameters_hc10*.yaml` (confirmed: `motoman_hc10_support` has no `config/`
directory at all).
 
Consequences to plan for:
 
- Use **KDL** (default) or build an IKFast solution for the HC10DT
- Numerical IK is slower and can fail to converge near singularities
- Do not spend hours hunting for a missing OPW file — it does not exist, and
  the geometry is the reason
This asymmetry is itself a finding worth reporting: it is a concrete,
measurable difference between the industrial GP-series and collaborative
HC-series arms, and it reinforces the Section 2.1 argument that the two classes
suit different tasks.
 
### 4A.3 — Gazebo with ros2_control
 
Gazebo Harmonic is installed (`ros-jazzy-ros-gz`). Add a `ros2_control` block
to the arm URDF using `gz_ros2_control/GazeboSimSystem`, configure a
`joint_trajectory_controller`, and launch MoveIt2 against it.
 
**This is the rehearsal for Isaac Sim.** The controller config, the action
interface, and the MoveIt setup are all reused; only the hardware plugin
changes (`GazeboSimSystem` → `JointStateTopicSystem`, §4.2).
 
One arm at a time. Gazebo with a single arm is comfortable in 8 GB.
 
### 4A.4 — Custom sensor flange for the HC10DT
 
The highest-value design work available right now, because it converts a
Section 4 *assertion* into a verified result.
 
Section 4 of the proposal states the Inspector carries four sensors. That is
currently an assumption. Building the flange tests it:
 
- Do the SICK microScan3, multiScan136, Inspector83x, and ZED 2i physically
  fit on one HC10DT wrist?
- Do they occlude each other's fields of view?
- Is the total mass within the HC10's 10 kg payload, including the flange?
- Are the standoff distances compatible with each sensor's working range?
**Any answer is a real result.** If they do not fit, discovering that now — and
reporting it with the geometry to prove it — is far better than assuming it
works.
 
Build it **parametrically** (FreeCAD or Blender), with sensor poses as
variables rather than baked geometry, so layouts can be swept rather than
remodelled.
 
> **The URDF is ground truth, not the STEP file.** Every sensor TF frame flows
> from the `<origin xyz rpy>` values in the xacro. A 5 mm or 2° mismatch
> between CAD and URDF silently corrupts every point-cloud registration
> downstream, and it will look like an algorithm bug. Derive the URDF origins
> from the CAD numerically; do not eyeball them.
 
Deliverables:
 
1. Parametric flange model (STEP + source)
2. `motobat_description/urdf/inspector_flange.xacro` — flange + four sensor
   frames, attached to `s3_tool0`
3. Mass/payload check against 10 kg
4. FOV/occlusion study — even a simple ray-cast or a documented sketch
5. TF tree verified in RViz: `ros2 run tf2_tools view_frames`
### 4A.5 — Deferred: simulated network switch for DDS
 
Considered and **deliberately deferred.** Recording the reasoning so it is not
revisited by accident:
 
- It targets Cyclone DDS, which §5 concludes is likely the wrong RMW for this
  project (NVIDIA recommends FastDDS for single-machine Isaac Sim; MotoROS2
  *requires* FastDDS). Building infrastructure around Cyclone deepens a
  commitment that is about to be reversed.
- DDS discovery already works across a real network. Two machines and
  `ROS_DOMAIN_ID` give genuine multi-host behaviour; a software switch mostly
  models one's own model of a switch.
- It does not advance the stated contribution — closed-loop geometric defect
  detection. It is adjacent infrastructure that could absorb weeks.
**Redirect that interest into latency instrumentation instead.** Section 3.2
promises detection-and-correction latency benchmarked against the 1.5–2.5 s/unit
industrial cycle time. Measuring where time actually goes — sensor publish →
registration → correction command — using `rosbag2` timestamps and QoS
experiments produces a number the proposal *needs*, while teaching real DDS
behaviour. Same curiosity, directly load-bearing.
 
### 4A.6 — Suggested order
 
| # | Task | Rough effort | Blocks |
|---|---|---|---|
| 1 | GP4 MoveIt config + OPW IK, marker-driven in RViz | 1–2 days | everything |
| 2 | HC10DT MoveIt config (KDL) | 1–2 days | flange work |
| 3 | Gazebo + `ros2_control` + `joint_trajectory_controller`, one arm | 2–3 days | Isaac Sim swap |
| 4 | Flange CAD + sensor placement study | 3–5 days | sensor sim |
| 5 | Flange → URDF with exact frames, TF verified in RViz | 1–2 days | registration work |
 
Steps 1–3 rehearse the full Isaac Sim control path. Step 4 is where the
genuinely novel engineering is.
 
---
 
## 4B. ACTIVE WORK PLAN — defect generation and detection (no simulator)
 
Runs entirely in numpy/Open3D on the current laptop. **Prototyped and measured:
a 96-layer stack with defect injection plus a simulated profilometer sweep uses
~10 MB and 0.23 s.** Nothing here is blocked by RAM, Gazebo, or Isaac Sim.
 
This is the project's actual contribution (Section 3 of the proposal). Building
it first — against synthetic scans, before any simulator exists — means the
detector is already validated when Isaac Sim arrives, and the simulator becomes
a *rendering* problem rather than an algorithm problem.
 
### 4B.0 — A finding from the prototype: height maps do not work
 
While probing feasibility, the first detector measured **stack height** and was
nearly blind to lateral misalignment: a 0.8 mm layer shift produced a −2.3 µm
max-Z change and **zero** footprint change. Obvious in hindsight — shifting
layers sideways does not change how tall the stack is.
 
Switching the metric to **lateral envelope deviation** produced a clean,
monotonic signal:
 
| Layer shift | Scanned X-extent | Δ vs reference | Protruding points |
|---|---|---|---|
| 0.0 mm | 66.450 mm | +0 µm | 0 |
| 0.2 mm | 66.650 mm | +200 µm | 3,840 |
| 0.5 mm | 66.950 mm | +500 µm | 9,600 |
| 0.8 mm | 67.250 mm | +800 µm | 15,360 |
| 1.2 mm | 67.650 mm | +1200 µm | 23,040 |
| 2.0 mm | 68.450 mm | +2000 µm | 38,400 |
 
*(24 layer-pairs, 66.5 × 48.0 mm footprint, 1.5 mm cathode inset, defect
injected from layer 12 onward, 50 µm scan pitch.)*
 
**Two consequences, both load-bearing:**
 
1. **A top-down height map is insufficient for the primary defect class.**
   Misalignment — the Nissan 26V188 fault mode and the project's main target —
   must be detected by comparing the lateral envelope against reference
   geometry. This is precisely what point-cloud-versus-mesh registration does,
   and it sharpens the Section 1.2 argument for geometric inspection over
   electrical test.
2. **The scan window must extend beyond the nominal part envelope.** Material
   that has moved outside the expected footprint is invisible to a scan
   covering only the expected footprint. This is a hard sensor-placement
   requirement and feeds directly into the flange design (§4A.4): FOV and
   standoff must cover **part + tolerance band**, not just the part.
Point 2 is why §4B should run before or alongside §4A.4 — it converts a flange
design guess into a derived requirement.
 
### 4B.1 — Procedural stack generator
 
A parametric electrode stack, defects off by default. Realistic dimensions from
the prismatic-cell literature cited in the proposal:
 
```
footprint        66.5 x 48.0 mm
layer pairs      24 (anode / sep / cathode / sep)
anode thickness  90 um
cathode          75 um
separator        20 um
cathode inset    1.5 mm  (anode overhang, per Section 1.4)
stack height     ~4.92 mm
```
 
Everything a variable, nothing hard-coded — the generator is also the reference
geometry source, so "known good" and "scanned part" come from one model.
 
### 4B.2 — Defect injection
 
One function per fault class from the Section 1.4 taxonomy, each with a
severity parameter so datasets sweep rather than sample:
 
| Defect | Parameter | In scope |
|---|---|---|
| Electrode misalignment (transfer) | `shift_x`, `shift_y`, `from_layer` | **primary** — Nissan 26V188 |
| Overhang violation | cathode inset per layer | **primary** |
| Stack thickness / layer count | add, drop, or duplicate layers | **primary** |
| Wrinkle / uneven pressure | sinusoidal Z perturbation | partial |
 
Every generated stack carries its ground-truth label — defect type, magnitude,
affected layers. **That label set is what makes precision/recall measurable**,
which Section 3.2 promises and cannot deliver without it.
 
### 4B.3 — Simulated scan
 
Convert stack geometry to a point cloud the way the Inspector's sensors would:
 
- Configurable scan pitch (50–200 µm) to match profilometer resolution
- Gaussian sensor noise (~5 µm σ) so the detector is not tuned to clean data
- Scan window sized to **part + tolerance band** (see §4B.0)
- Optional occlusion and grazing-angle dropout for realism
Output plain `.ply` / numpy arrays — the same format Isaac Sim's RTX Lidar will
publish via `sensor_msgs/PointCloud2`. **When the simulator arrives, only the
point-cloud source changes; the detector does not.**
 
### 4B.4 — Detection pipeline (Open3D)
 
1. Load scan, load reference geometry from the same generator
2. Coarse align, then ICP refinement
3. Envelope deviation per §4B.0 — not height
4. Classify: defect type, magnitude, affected layers
5. Emit a `DefectReport` (see §7 — define `motobat_msgs` alongside this)
Runs in the Python 3.11 venv (`mb-venv`), not system Python.
 
### 4B.5 — Validation: the numbers the proposal needs
 
Generate a labelled dataset — a few hundred stacks across defect types and
severities — and measure:
 
- **Precision / recall per defect class**, benchmarked against the 62.07 %
  Hi-Pot-alternative recall and 93.82 % YOLOv4 accuracy cited in Section 1.2
- **Minimum detectable defect** per class, versus the 0.5 mm positional error
  Section 1.1 identifies as production-critical
- **Detection latency**, against the 1.5–2.5 s/unit cycle time (Section 3.2)
- **Sensitivity to scan pitch and noise** — which directly informs which
  sensors the flange actually needs
> These are real, defensible results produced without a simulator. If Isaac Sim
> were to slip further, §4B alone still constitutes a substantive contribution.
 
### 4B.6 — How this meets the simulator later
 
Once Isaac Sim is running, the generator emits **USD/mesh** instead of only
point clouds, so defective stacks are importable as scene geometry and the
HC10DT scans them with RTX Lidar. The detector is unchanged — it was written
against `PointCloud2`-shaped data from the start.
 
Gazebo is the intermediate step: import a generated stack as a mesh, mount a
depth camera on the arm, and confirm the pipeline works on simulator-produced
clouds before Isaac Sim's higher-fidelity sensors.
 
**Order of fidelity:** synthetic scan → Gazebo depth camera → Isaac Sim RTX
Lidar. The detector is written once.
 
---
 
## 4. Isaac Sim milestone — HC10DT planned from RViz (after repair)
 
The target: open RViz, drag the MoveIt interactive marker to a pose, hit Plan &
Execute, and watch the HC10DT move in Isaac Sim. This is the first real
closed-loop between planner and simulator, and every later station reuses it.
 
### 4.0 Where to run Isaac Sim — decide before installing
 
The 8 GB ceiling (§1.0) makes this a real decision rather than a default.
 
**Isaac Sim 4.5 does not help.** Its documented minimum is 32 GB RAM / 8 GB
VRAM — *identical* to 6.0. There is no lighter tier. Downgrading also loses
Ubuntu 24.04 support (4.5 lists 20.04/22.04) and the Jazzy-recommended bridge
configuration. It costs a day and buys nothing.
 
**The container does not reduce memory requirements either.** NVIDIA does not
differentiate RAM requirements between container and workstation installs —
same application, different wrapper. The container solves *deployment*
problems, not memory pressure.
 
But the container **is** the right tool for the option that actually works:
 
> **Run Isaac Sim on a lab/remote machine, stream to the laptop.**
> This is exactly what the container is built for — headless servers with
> WebRTC streaming to a thin client. The laptop runs ROS 2, MoveIt2, RViz2,
> and Open3D locally (all comfortable in 8 GB); the remote box runs only
> Isaac Sim.
 
Container limitations to plan around:
 
- GUI mode needs a real local display and is not recommended — expect to work
  through WebRTC streaming, one browser tab or client at a time.
- Python apps and standalone examples run headless only.
- Tutorials needing the Content Browser require a Nucleus connection.
Streaming adds friction to interactive work — building an OmniGraph by hand,
tuning an imported URDF. It is workable, not pleasant.
 
**Recommended sequencing given the hardware situation:**
 
1. **Now, on the laptop:** everything that does not need Isaac Sim — the RViz
   work in §3, `motobat_msgs` definitions, the Open3D registration pipeline
   against synthetic point clouds, cell/fixture geometry in Blender. This is
   genuinely a lot of the project and none of it is blocked.
2. **In parallel:** file the Dell warranty transfer (§1.0) and ask the OSU ECE
   department about GPU workstation access. Both have long lead times, so
   start them today.
3. **When a capable machine is available:** §4.1–4.3 as written, container
   install if remote, workstation install if the laptop is repaired.
**Do not attempt a serious Isaac Sim install on 8 GB.** It will technically
launch on swap and then thrash badly enough that every problem looks like a
software bug. That failure mode has already cost multiple days once.
 
### 4.0b Cloud GPU — the recommended path, with the caveat that matters
 
Cloud is very likely the right answer for weeks 5–6, but **not for the reason
usually given**, and the naive architecture does not work.
 
#### Correcting a common misdiagnosis
 
A frequent claim is *"Isaac Sim's hard constraint is GPU/VRAM, not system RAM."*
**For this machine that is backwards**, and it matters:
 
| Resource | This laptop | Isaac Sim 6.0 minimum | Verdict |
|---|---|---|---|
| VRAM | 8 GB (RTX 4060) | 8 GB | **meets minimum** |
| System RAM | 7.4 GB → 16 GB post-repair | **32 GB** | **below minimum** |
 
The GPU is not the bottleneck. System RAM is, and it was diagnosed precisely
(§1.0). Advice premised on the opposite would lead to buying GPU capacity to
solve a memory problem.
 
Also note: the 32 GB DDR5 kit is roughly **$400**, not the ~$1,000 figure that
sometimes circulates. The comparison below uses the real number.
 
#### The cost comparison, with current figures
 
Isaac Sim work is confined to **weeks 5–6** (§3.5) — realistically 40–80 hours
of actual simulator time, not continuous use.
 
| Option | System RAM | VRAM | Cost |
|---|---|---|---|
| AWS `g6.2xlarge` (L4) | 32 GiB | 22.4 GiB | **$0.98/hr** |
| Runpod L4 | 24 GB | 24 GB | **$0.49/hr** |
| Vast.ai L4 | 63 GB | 24 GB | **$0.33/hr** |
| 32 GB DDR5 kit (local) | 32 GB | 8 GB (unchanged) | **~$400 one-time** |
 
At 60 hours: **$20–60 on cloud**, versus $400 for RAM that still leaves the
laptop at 8 GB VRAM and only just at Isaac Sim's RAM minimum.
 
**For this project's timeline, cloud wins clearly.** It also removes weeks 5–6
from the critical path — the deadline no longer depends on a hardware purchase.
 
#### The architecture caveat — do not split ROS 2 across the internet
 
The obvious layout — Isaac Sim in the cloud, ROS 2 and MoveIt2 local — **does
not work well**, and this is the part usually omitted:
 
- DDS discovery relies on multicast, which does not traverse the public internet
- The Isaac Sim container **requires `--network=host`** (the streaming SDK binds
  its UDP media socket to a real interface), so container-level port mapping
  cannot paper over it
- Even with a VPN (Tailscale/ZeroTier) or a Zenoh bridge, WAN latency sits
  inside the `FollowJointTrajectory` control loop
**Correct layout: run the entire stack on the cloud instance** — Isaac Sim,
ROS 2, MoveIt2, RViz2 — and stream only the display back. All DDS traffic stays
on `localhost` inside one machine. The laptop becomes a thin client.
 
```
  ┌─ cloud instance (32-64 GB RAM, L4) ──────────────┐
  │  Isaac Sim  ⇄  ROS 2 / MoveIt2 / RViz2           │      WebRTC
  │  all DDS traffic on localhost                     │  ═══════════▶  laptop
  └───────────────────────────────────────────────────┘               (display)
```
 
#### Practical setup notes
 
- **Ports:** 49100/TCP (WebRTC signalling), 47998/UDP (media), 8210/TCP if using
  the Docker Compose web viewer. Open these in the provider's firewall.
- **`--network=host` is mandatory** — see above.
- **Mount the cache volumes.** Shader compilation takes 10–30 minutes on first
  launch; without persistent `cache/`, `computecache/`, `config/`, and `data/`
  volumes, that cost is paid **every session**, which is ruinous on hourly
  billing. Pre-create them and `chown -R 1234:1234`.
- **Driver caveat:** NVIDIA notes the newest drivers are not always fully
  supported for livestreaming. Prefer a provider image with a known-good driver
  over the absolute latest.
- **Cost discipline:** bill is wall-clock, not compute. **Stop the instance when
  not actively simulating.** Keep the repo in git so a destroyed instance costs
  nothing but setup time.
#### Recommended plan
 
1. **Weeks 1–4: change nothing.** §4B is numpy, §4A is RViz2/Gazebo — all
   comfortable on the repaired 16 GB laptop. No cloud, no purchase.
2. **Week 5: rent for a few hours first.** Runpod or Vast.ai, cheapest tier,
   prove the container launches and streams before committing further.
3. **Weeks 5–6: run the full stack in the cloud**, laptop as thin client.
4. **Defer the RAM purchase entirely** until after 1 Oct. If cloud works, that
   $400 may be better spent elsewhere, or not at all.
This makes the memory upgrade (§1.0) genuinely optional rather than blocking,
and removes the single largest schedule risk in §3.5.
 
### 4.1 Architecture
 
```
RViz (MoveIt Motion Planning plugin)
        │  interactive marker → plan request
        ▼
   move_group  ──────────────────────────────┐
        │  FollowJointTrajectory action      │  planning scene,
        ▼                                     │  collision checking
   joint_trajectory_controller               │
        │  (ros2_control)                    │
        ▼                                     │
   TopicBasedSystem hardware interface       │
        │                                     │
        │  publishes /isaac_joint_commands   │
        │  subscribes /isaac_joint_states    │
        ▼                                     │
   Isaac Sim OmniGraph ActionGraph           │
        │  ROS2 Subscribe JointState  ───────┘
        │  Articulation Controller
        │  ROS2 Publish JointState
        ▼
   HC10DT articulation in the USD stage
```
 
The seam is deliberate. `joint_trajectory_controller` exposes exactly the
`control_msgs/FollowJointTrajectory` action that **MotoROS2 exposes on real
Yaskawa hardware** (see §6). Swapping Isaac Sim for a YRC1000 later means
changing the hardware interface, not the planning stack.
 
### 4.2 The bridge package
 
Two options, both released for Jazzy:
 
| | Package | Plugin class |
|---|---|---|
| **Recommended** | `topic_based_hardware_interfaces` (ros-controls) | `joint_state_topic_hardware_interface/JointStateTopicSystem` |
| Legacy | `topic_based_ros2_control` (PickNik) | `topic_based_ros2_control/TopicBasedSystem` |
 
Prefer the **ros-controls** one. The PickNik package is marked UNMAINTAINED on
the ROS index; the functionality moved upstream into `ros2_control` under
Simulator Integrations. Most tutorials still show the old name — translate as
you read them.
 
### 4.3 Steps
 
**1 — Install Isaac Sim 6.0.** ~30 GB. Not scripted deliberately: the installer
is version-sensitive enough that a wrapper rots faster than it helps.
 
```bash
# https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html
cd ~/isaacsim && ./post_install.sh && ./isaac-sim.selector.sh
```
 
First launch compiles shaders for 10–30 minutes. **It is not hung.** Then
uncomment `ISAAC_SIM_PATH` in `~/motobat/motobat_env.sh`.
 
**2 — Import the HC10DT into Isaac Sim.** Use the URDF Importer extension on
`hc10dt.xacro` (expand to plain URDF with `xacro` first). Save as USD.
 
Expect to fix after import: joint drive stiffness/damping, joint limits, and
collision meshes. **The visual meshes are too detailed to use as collision
geometry** — replace with convex hulls or primitives, or physics will crawl.
 
**3 — Build the OmniGraph.** Three nodes, wired to the articulation:
 
- `ROS2 Subscribe JointState` → topic `/isaac_joint_commands`
- `Articulation Controller` → targeting the HC10DT prim
- `ROS2 Publish JointState` → topic `/isaac_joint_states`
Follow the [Joint Control tutorial](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_manipulation.html)
— it covers this graph directly.
 
**4 — Generate the MoveIt config.**
 
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
 
Load `hc10dt.xacro`. Define one planning group (`inspector_arm`, all six joints),
generate the self-collision matrix, add a home pose. Output goes to
`src/motobat_inspector/config/`.
 
**5 — Add the `ros2_control` block** to the URDF used by MoveIt:
 
```xml
<ros2_control name="HC10DTIsaac" type="system">
  <hardware>
    <plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin>
    <param name="joint_commands_topic">/isaac_joint_commands</param>
    <param name="joint_states_topic">/isaac_joint_states</param>
  </hardware>
  <!-- one <joint> block per joint_1_s … joint_6_t -->
</ros2_control>
```
 
**6 — Launch and test.** `move_group` + `joint_trajectory_controller` + RViz.
Drag the marker, Plan & Execute, watch Isaac Sim.
 
### 4.4 Where this will actually go wrong
 
Budget debugging time for these — they are the known failure modes, not
hypotheticals:
 
- **Joint name mismatch.** MoveIt's names must match Isaac Sim's articulation
  DOF names exactly. Check `ros2 topic echo /isaac_joint_states` against
  `ros2 param get /move_group robot_description`.
- **Trajectory following is inconsistent** even when connected — a documented
  Isaac Sim + `topic_based_ros2_control` issue. Usually joint drive gains being
  too soft to track the commanded trajectory. Tune stiffness/damping on the
  articulation first.
- **Nothing moves, no error.** Almost always the ActionGraph is not ticking, or
  the Articulation Controller points at the wrong prim.
- **RMW mismatch.** See §5 — this one is silent and wastes hours.
---
 
## 5. Open decision: Cyclone DDS vs FastDDS
 
`motobat_env.sh` currently sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` per
Section 5.3 of the proposal. **There are now two independent arguments against
that choice:**
 
1. **NVIDIA recommends FastDDS** for single-machine Isaac Sim — it uses shared
   memory for large `PointCloud2` traffic, which is essentially everything the
   Inspector does.
2. **MotoROS2 requires FastDDS** and does not support Cyclone. Keeping Cyclone
   means the stated hardware migration path needs an RMW change to work.
**Recommendation:** switch to FastDDS (comment the line out — it is the default)
and note the reversal in the proposal as an evidence-driven change. That reads
better than a spec nobody revisited.
 
If point clouds arrive truncated or drop under load, this is the first thing to
check — before debugging registration code.
 
---
 
## 6. MotoROS2: what it is and is not
 
**MotoROS2 cannot run on this machine.** It is a MotoPlus application that runs
*on* a Yaskawa controller (DX200 / YRC1000 / YRC1000micro). No PC build, no
simulation mode.
 
The MotoPlus SDK is Yaskawa's paid, license-gated C SDK, needed only to *build*
MotoROS2 from source. Yaskawa publishes prebuilt `.out` binaries. **The SDK is
not the barrier — the controller is.**
 
### Why this barely matters
 
MotoROS2 exposes itself to ROS 2 as a standard
`control_msgs/FollowJointTrajectory` action server plus `sensor_msgs/JointState`
publishing — the same interface MoveIt2 drives in Isaac Sim.
 
```
Simulation:  MoveIt2 → FollowJointTrajectory → Isaac Sim controller
Hardware:    MoveIt2 → FollowJointTrajectory → micro-ROS Agent → MotoROS2 → arm
```
 
Planning, inspection, and the correction loop are unchanged across that swap.
 
### The discipline that keeps this true
 
- Command arms **only** via `FollowJointTrajectory` actions
- Read state **only** from `/joint_states`
- **Never** call Isaac Sim's Python articulation API from a control node
- Isaac-Sim-specific code lives in `motobat_bringup` only
Breaking this welds the project to the simulator and invalidates the
sim-to-hardware claim.
 
### Proposal wording
 
Section 5.1 currently reads as though MotoROS2 is part of the running stack.
Replace with:
 
> The system targets the standard ROS 2 manipulation interfaces
> (`control_msgs/FollowJointTrajectory`, `sensor_msgs/JointState`) that
> MotoROS2 implements on real Yaskawa controllers. Arms are driven through
> ros2_control and MoveIt2 against Isaac Sim's articulation controller, with the
> command interface built against the genuine MotoROS2 contract — making
> sim-to-hardware migration a controller swap rather than a rewrite.
 
Section 2.2's Yaskawa-over-FANUC argument survives intact and is now the basis
of the migration path.
 
Add to Section 6 (future work): real controllers enforce motion limits, speed
overrides, and safety interlocks that Isaac Sim ignores. Trajectories valid in
simulation may be rejected outright by a YRC1000.
 
---
 
## 7. Roadmap after the Isaac Sim milestone
 
*(§4A is the current work. This is what follows once Isaac Sim is running.)*
 
1. **Sensor array on the HC10DT** — SICK microScan3 / multiScan136 / Inspector83x,
   ZED 2i, mounted on the flange from §4A.4. RTX Lidar OmniGraph →
   `sensor_msgs/PointCloud2`. Verify in RViz2.
2. **TF2 tree for the sensors** — already defined by §4A.4's flange URDF;
   verify it survives the move into Isaac Sim. Point clouds from different
   passes cannot fuse without correct frames.
3. **`motobat_msgs`** — `DefectReport`, `ScanResult`, `CorrectionCmd`. Define
   these before writing nodes that pass them around.
4. **Open3D registration pipeline** — ICP scan-vs-reference. Runs in the venv,
   not system Python.
5. **Cell and fixture geometry** — Blender/FreeCAD, procedurally generated so
   defect variants can be scripted rather than modeled by hand.
6. **Defect injection** — geometric, per Section 3.1.
7. **Remaining three stations** — reuse the HC10DT pattern. Needs the
   hardware situation resolved (§1.0); not viable on 8 GB.
8. **Latency instrumentation** — measure sensor publish → registration →
   correction command against the 1.5–2.5 s/unit industrial cycle time
   (Section 3.2 of the proposal). See §4A.5 for why this replaces the
   simulated-switch idea.
9. **Correction loop** — the actual contribution. Everything above is scaffolding.
---
 
## 8. Repo conventions
 
**`.gitignore`:**
 
```gitignore
build/
install/
log/
src/_vendor/
venv/
*.pyc
__pycache__/
```
 
`_vendor/` is cloned by the bootstrap script — never commit it.
 
**Commits:** the history is evidence of the development process (Section 5.7 of
the proposal). Write messages that explain *why*, not *what*.
 
**Before writing a control node,** re-read §6. That boundary is the whole
sim-to-hardware argument.
 
---
 
## 9. The mathematics — what each subsystem actually computes
 
Per the working principle above: equations first, then code. Every symbol is
defined, every frame and unit stated. Craig's *Introduction to Robotics* chapter
numbers are given where they apply, since that is the current study text.
 
### 9.1 Rigid-body transforms and the TF tree
 
Everything in this project is bookkeeping about frames. A pose of frame *B*
expressed in frame *A* is a homogeneous transform:
 
```
        ⎡ R_AB   p_AB ⎤
T_AB =  ⎢             ⎥        R ∈ SO(3),  p ∈ ℝ³
        ⎣  0ᵀ      1  ⎦
```
 
Composition chains along the kinematic tree, and inversion is cheap:
 
```
T_AC = T_AB · T_BC          T_AB⁻¹ = ⎡ Rᵀ   −Rᵀp ⎤
                                     ⎣ 0ᵀ     1  ⎦
```
 
**Why it matters here:** a sensor point measured in the sensor frame becomes a
point in the world frame by `p_world = T_world,tool · T_tool,sensor · p_sensor`.
The middle transform is the flange geometry from §4A.4. **An error there
propagates into every point of every scan** — which is why the URDF origins must
be derived from CAD numerically, not eyeballed.
 
*Craig ch. 2. Modern Robotics ch. 3 (as SE(3) and screw theory).*
 
### 9.2 Forward kinematics
 
Joint angles → end-effector pose. With Denavit–Hartenberg parameters
(αᵢ₋₁, aᵢ₋₁, dᵢ, θᵢ):
 
```
                 ⎡ cθᵢ            −sθᵢ           0          aᵢ₋₁         ⎤
  ᵢ⁻¹Tᵢ    =     ⎢ sθᵢcαᵢ₋₁    cθᵢcαᵢ₋₁    −sαᵢ₋₁    −sαᵢ₋₁dᵢ      ⎥
                 ⎢ sθᵢsαᵢ₋₁    cθᵢsαᵢ₋₁     cαᵢ₋₁     cαᵢ₋₁dᵢ      ⎥
                 ⎣ 0              0              0          1            ⎦
 
  ⁰T₆ = ⁰T₁ ¹T₂ ²T₃ ³T₄ ⁴T₅ ⁵T₆
```
 
*Craig ch. 3.* Modern Robotics uses the product-of-exponentials form instead,
which avoids DH's frame-assignment ambiguity:
 
```
  T(θ) = e^{[S₁]θ₁} e^{[S₂]θ₂} ⋯ e^{[S₆]θ₆} · M
```
 
where Sᵢ is the screw axis of joint *i* in the base frame and *M* is the
zero-configuration pose. Worth learning both — MoveIt and most ROS tooling are
DH/URDF-shaped, but PoE is cleaner to reason about.
 
### 9.3 Inverse kinematics — and why the two arms differ
 
Pose → joint angles. This is where the GP4 and HC10DT genuinely diverge (§4A.2).
 
**GP4 — closed-form (OPW).** The arm has an ortho-parallel basis and a
**spherical wrist**: joints 4, 5, 6 axes intersect at a single point. That
permits kinematic decoupling — position from the first three joints, orientation
from the last three:
 
```
  p_wrist = p_tool − d₆ · R_tool · ẑ        (wrist centre)
  θ₁,θ₂,θ₃ ← geometric solution from p_wrist
  R₃₆ = R₀₃ᵀ · R_tool                        (orientation residual)
  θ₄,θ₅,θ₆ ← Euler extraction from R₃₆
```
 
Yields up to **8 discrete solutions** (shoulder left/right × elbow up/down ×
wrist flip), all in closed form, in microseconds. Parameters ship in
`opw_parameters_gp4.yaml`.
 
**HC10DT — numerical.** `joint_5_b` is offset 162 mm in Y, so the wrist axes do
**not** intersect and the decoupling above is invalid. Solve iteratively with
damped least squares (Levenberg–Marquardt):
 
```
  Δθ = Jᵀ(J Jᵀ + λ²I)⁻¹ · e        e = pose error twist ∈ ℝ⁶
```
 
The damping λ is what keeps the solution finite near singularities, where **J**
loses rank and the undamped pseudo-inverse `Jᵀ(JJᵀ)⁻¹` blows up. Slower, may
fail to converge, no guarantee of finding all solutions.
 
*Craig ch. 4. The DLS form is Craig ch. 5 + Siciliano ch. 3.*
 
### 9.4 Velocity kinematics and the Jacobian
 
The Jacobian maps joint rates to end-effector twist:
 
```
  ⎡ v ⎤             ⎡ ẑᵢ₋₁ × (p_n − pᵢ₋₁) ⎤
  ⎢   ⎥ = J(θ) θ̇     Jᵢ = ⎢                      ⎥   (revolute joint i)
  ⎣ ω ⎦             ⎣ ẑᵢ₋₁                 ⎦
```
 
Static force duality — the same matrix, transposed:  `τ = Jᵀ · F`
 
**Singularity measure.** Yoshikawa's manipulability index:
 
```
  w(θ) = √( det( J(θ) J(θ)ᵀ ) )
```
 
`w → 0` at a singularity. Worth logging during scan trajectories: if the
Inspector's sweep passes near `w ≈ 0`, IK will struggle and motion will look
erratic. This is a measurable, reportable quantity, not a vague warning.
 
*Craig ch. 5. Modern Robotics ch. 5.*
 
### 9.5 Trajectory generation
 
MoveIt hands `joint_trajectory_controller` a time-parameterised path. For a
point-to-point move, a quintic keeps acceleration continuous:
 
```
  θ(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
```
 
with coefficients fixed by θ(0), θ(T), θ̇(0), θ̇(T), θ̈(0), θ̈(T). Trapezoidal
velocity profiles are the alternative — faster, but with acceleration
discontinuities that a real controller feels.
 
**Relevant to Section 3.2 of the proposal:** cycle time is bounded below by
these profiles plus the velocity/acceleration limits in the URDF. Latency
budget = plan time + trajectory duration + detection time.
 
*Craig ch. 7. Modern Robotics ch. 9.*
 
### 9.6 Control — the joint_trajectory_controller loop
 
The controller tracks the commanded trajectory. Independent-joint PID:
 
```
  τ = Kp·e + Ki∫e dt + Kd·ė           e(t) = θ_desired(t) − θ_actual(t)
```
 
The model-based form, which is what a real industrial controller approximates:
 
```
  τ = M(θ)·θ̈_d  +  C(θ,θ̇)·θ̇  +  G(θ)  +  Kp·e + Kd·ė
       ⎣inertia⎦  ⎣Coriolis⎦   ⎣gravity⎦  ⎣  feedback  ⎦
```
 
**This connects directly to prior myCobot work.** Achieving sub-1 mm on a
compliant, backlash-prone arm meant compensating for terms this model assumes
away — joint compliance and hysteresis. The industrial arms here are stiff
enough that PID on a good trajectory suffices, which is *why* industrial arms
hit tolerance more easily. Worth stating explicitly in the writeup: it shows
awareness of what the hardware is doing for you.
 
*Craig ch. 9–10. Modern Robotics ch. 8, 11.*
 
### 9.7 Point cloud registration — ICP
 
The core of the Inspector. Given scanned points **P** = {pᵢ} and reference
**Q** = {qⱼ}, find the rigid transform aligning them:
 
```
  minimise over (R, t):    Σᵢ ‖ (R·pᵢ + t) − q_{c(i)} ‖²
```
 
where `c(i)` is the closest point in **Q**. Iterate: (1) find correspondences,
(2) solve for (R,t), (3) apply, repeat until convergence.
 
**Closed-form inner solve** (Arun / Horn, via SVD):
 
```
  p̄ = (1/n)Σpᵢ ,  q̄ = (1/n)Σqᵢ            (centroids)
  H  = Σ (pᵢ − p̄)(qᵢ − q̄)ᵀ                 (3×3 cross-covariance)
  H  = U Σ Vᵀ                               (SVD)
  R  = V · diag(1, 1, det(VUᵀ)) · Uᵀ        (det term prevents reflection)
  t  = q̄ − R·p̄
```
 
**Point-to-plane** converges faster on surfaces like an electrode stack, since
it penalises only motion along the surface normal:
 
```
  minimise    Σᵢ ( (R·pᵢ + t − qᵢ) · n̂ᵢ )²
```
 
**What to know before trusting it:** ICP is a local optimiser. It needs a decent
initial guess or it converges to a wrong minimum — which is why §4B.4 does
coarse alignment (FPFH features or RANSAC) before refinement. A defect that
shifts material can also bias the fit itself, so registration should be
performed against features known to be unaffected.
 
*Szeliski, Computer Vision, ch. 13. Open3D ICP tutorial.*
 
### 9.8 Defect metric — envelope deviation
 
From §4B.0, the empirical finding: height maps are blind to lateral
misalignment. The metric that works:
 
```
  d(p) = signed distance from scanned point p to the reference surface
 
  Defect flagged when   |d(p)| > τ   over a connected region of area > A_min
```
 
Two thresholds, both requiring justification in the writeup: **τ** from sensor
noise (roughly 3σ, so ~15 µm for a 5 µm-σ profilometer) and **A_min** from the
smallest defect that matters physically.
 
Per-class specialisations:
 
```
  Misalignment      Δextent = extent(scan) − extent(reference)      [§4B.0]
  Overhang          w_over = min distance, cathode edge → anode edge
  Thickness         Δh = h_measured − N·(t_anode + t_cath + 2·t_sep)
```
 
**Detection statistics** — the numbers Section 3.2 promises:
 
```
  Precision = TP / (TP + FP)          Recall = TP / (TP + FN)
  F₁ = 2·P·R / (P + R)
```
 
Benchmark against the 62.07 % Hi-Pot recall and 93.82 % YOLOv4 accuracy from
Section 1.2. Also report **minimum detectable defect** per class against the
0.5 mm threshold from Section 1.1.
 
### 9.9 The correction loop
 
The project's actual contribution. Given a measured misalignment **e** at layer
*k*, the correction applied to layer *k+1*:
 
```
  u_{k+1} = −K · e_k                    (proportional correction)
```
 
With integral action across layers, if error accumulates systematically:
 
```
  u_{k+1} = −Kp·e_k − Ki · Σⱼ₌₀..k e_j
```
 
**Stability condition.** This is a discrete-time loop indexed by layer, not
time. With plant gain *g* (commanded offset → achieved offset), the error
recursion is `e_{k+1} = (1 − gK)·e_k`, so convergence requires:
 
```
  |1 − g·K| < 1      ⟺      0 < K < 2/g
```
 
Over-correction (`K > 2/g`) oscillates and diverges — layers alternating past
the target. **Worth demonstrating deliberately in the results:** a sweep of *K*
showing convergence, critical damping, and divergence is far more convincing
than one tuned value, and it proves the loop is understood rather than guessed.
 
Open-loop vs closed-loop defect rate over the sweep is the headline result.
 
---
 
## 10. Study resources
 
Selected for this project specifically. Free where possible.
 
### 10.1 Robotics fundamentals
 
| Resource | Use | Cost |
|---|---|---|
| **Craig, *Introduction to Robotics*** (current text) | DH parameters, FK/IK, Jacobians, control. Matches ROS/URDF conventions. Ch. 2–5, 7, 9 are the ones that matter here | book |
| **Lynch & Park, *Modern Robotics*** | Screw theory / product of exponentials — cleaner than DH once it clicks. **Free PDF, free video lectures, free Coursera audit, Python library** | **free** |
| **Siciliano et al., *Robotics: Modelling, Planning and Control*** | Deeper on dynamics and control; the reference to reach for when Craig is too brief | book |
 
**Suggested approach:** stay with Craig as the primary — it maps directly onto
what MoveIt and URDF expect. Use Modern Robotics' video lectures for the same
topics from a second angle; the free Coursera audit covers ch. 3–6, which is
exactly §9.1–9.4 above.
 
### 10.2 Perception and point clouds
 
- **Szeliski, *Computer Vision: Algorithms and Applications*** — free PDF from
  the author's site. Ch. 13 covers 3D reconstruction and registration.
- **Open3D ICP tutorial** — worked examples for point-to-point and point-to-plane
- **Li et al. (2021), "A Tutorial Review on Point Cloud Registrations"** —
  survey covering coarse-to-fine pipelines and where each method fails
- **PCL tutorials** — even when using Open3D, the PCL docs explain the
  algorithms well
### 10.3 Battery manufacturing — start here
 
- **PEM RWTH Aachen & VDMA, *Production Process of a Lithium-Ion Battery Cell***
  — **free PDF**, the authoritative industry guide. Covers electrode
  manufacturing, cell assembly (including stacking and tab welding), and
  finishing, across pouch/cylindrical/prismatic. **Read this first** — it is the
  process the project models.
- **VDMA Battery Production** — updated editions and process guides
- **"Advanced Electrode Processing for Lithium-Ion Battery Manufacturing"** —
  US DOE / OSTI, free
- The papers already cited in the proposal's References — particularly the
  defect taxonomy sources behind Section 1.4
### 10.4 ROS 2 and MoveIt2
 
- **Official ROS 2 Jazzy tutorials** — worth redoing given the year gap; the
  executor and QoS material matters more here than in the tutorial pick-and-place
- **MoveIt2 tutorials** — especially Setup Assistant and the move_group interface
- **ros2_control docs** — hardware interfaces and controller lifecycle, directly
  relevant to §4A.3 and the Isaac Sim swap
- **Articulated Robotics (YouTube)** — the clearest practical ROS 2 + Gazebo
  material available
### 10.5 What to read, in order, for this project
 
1. PEM/VDMA battery guide — understand the process being modelled *(week 1)*
2. Craig ch. 2–3 — transforms and FK, alongside §9.1–9.2 *(week 1–2)*
3. Szeliski ch. 13 + Open3D ICP — before writing §4B.4 *(week 2)*
4. Craig ch. 4–5 — IK and Jacobians, before §4A MoveIt work *(week 3)*
5. MoveIt2 Setup Assistant docs — during §4A.1 *(week 3)*
6. Craig ch. 7, 9 — trajectories and control, during §4A.3 *(week 4)*
Reading paired with the subsystem being built beats reading ahead — the
equations in §9 mean more once the thing they describe is misbehaving.
 
---
 
## References
 
- [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [Isaac Sim ROS 2 installation](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_ros.html)
- [Isaac Sim joint control tutorial](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_manipulation.html)
- [MoveIt: command a simulated Isaac robot](https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html)
- [topic_based_hardware_interfaces](https://control.ros.org/master/doc/topic_based_hardware_interfaces/doc/index.html)
- [Yaskawa MotoROS2](https://github.com/Yaskawa-Global/motoros2)
- [gavanderhoorn/motoman (ros2)](https://github.com/gavanderhoorn/motoman/tree/ros2)
**Study resources**
 
- [Modern Robotics (Lynch & Park) — free PDF, videos, code](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- [PEM RWTH Aachen / VDMA — Production Process of a Lithium-Ion Battery Cell (free PDF)](https://www.pem.rwth-aachen.de/global/show_document.asp?id=aaaaaaaaabdqbtk)
- [VDMA — Production Process of a Li-Ion Battery Cell, 2026 edition](https://www.vdma.eu/documents/34570/35405938/Production_Process_of_a_Lithium-Ion_Battery_Cell_2026.pdf)
- [Open3D ICP registration tutorial](https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html)
- [Li et al. — A Tutorial Review on Point Cloud Registrations (2021)](https://onlinelibrary.wiley.com/doi/10.1155/2021/9953910)
- [Advanced Electrode Processing for Li-Ion Battery Manufacturing (DOE/OSTI)](https://www.osti.gov/servlets/purl/2575264)
**Hardware / warranty**
 
- [Dell Ownership Transfer and Product Registration](https://www.dell.com/support/kbdoc/en-us/000125103/ownership-transfer-and-dell-product-registration-guides)
- [Dell support when traveling internationally](https://www.dell.com/support/contents/en-us/article/contact-information/international-support-services/dell-international-support)
- [Isaac Sim 4.5 requirements](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/requirements.html) (same 32 GB minimum as 6.0)
- [Isaac Sim container installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html)
 
