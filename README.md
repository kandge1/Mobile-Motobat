# Mobile-MotoBat — Project Plan

**Mobile welding with Yaskawa arms on rough-terrain bases: perception that keeps the weld on the seam when the ground does not hold still, and an energy plan that keeps a fleet of them running.**

A Yaskawa HC10DTP on an AgileX Ranger chassis welds structural joints. It is tested in three environments, each harder than the last:

1. **Shop floor:** flat, lit, mapped, one robot.
2. **Urban construction or rocket-assembly site:** uneven, cluttered and partly outdoors. Two robots work together: one picks and places parts, the other welds.
3. **Lunar surface:** terrain built from real Lunar Reconnaissance Orbiter elevation data, 1/6 g, vacuum, hard lighting and no GPS.

On top of the three environments sits a **fleet energy layer**: a shared charging station, battery degradation models, and long-horizon Monte Carlo simulation, coupled to the weld-quality and inspection models from MotoBat.

Same software stack, working principle and evidence standard as MotoBat. This is MotoBat's mobile follow-up, not a replacement.

Rhishikesh Kandge — M.S. Electrical and Computer Engineering, The Ohio State University

*Plan rewritten 11 Nov 2026 (v2). Nothing is built yet. Section references like MB §3.6 point into MotoBat's `docs/project_plan.md`. MB status §… points into MotoBat's `docs/status_update.md`.*

---

## Who is building this, and why

**Author:** Rhishikesh Kandge — M.S. ECE, The Ohio State University.

A self-directed project, like MotoBat. Its purpose is **a defensible result: a claim someone could disagree with, backed by numbers** (MB §3.6), not an impressive-looking simulation.

### Relevant background (carried over)

- **MotoBat:** a simulated battery-module line with a Yaskawa HC10DT inspector. It includes `defectgen` (a mesh-native generator, 13 defect classes, and ray-cast sensor models frozen at params hash `779e2255dc4c`), MoveIt2 and Gazebo control paths, a CAD-derived sensor flange, and a closed-loop correction with a derived stability bound (MB §9.9).
- **swarm_project:** vision-guided pick-and-place on a myCobot 280 Pi, plus the Elephant Robotics myAGV. The vendor driver stack had to be repaired before it worked. That's why this plan picks bases whose drivers you can read and fix.
- **Optimization theory:** used here for the fleet energy layer (§5).
- **Sub-1 mm repeatability** on a compliant, backlash-heavy myCobot Pro 450.

### Gaps this project closes that MotoBat does not

| Gap | Where it is closed |
|---|---|
| Mobile robotics: Nav2, localization, a 4WS chassis controller | Phase 1 |
| Base pose uncertainty propagating into tool accuracy | Core research question (§3) |
| Multi-robot coordination across separate bases | Phase 2 |
| Perception on unstructured terrain and under hard lighting | Phases 2–3 |
| Energy-aware fleet scheduling and battery lifetime | Phase 3 (§5) |
| Writing a robot model and a ros2_control controller from scratch | Phase 1 (§2.1) |
| Cloud-scale simulation: Isaac Sim, OmniLRS | Phase 3 |

### Working principle (unchanged from MotoBat)

> **Every subsystem must be documented with the mathematics that governs it, not just the code that implements it.**

Every module gets its governing equations with symbols defined, a block or data-flow diagram, units and frames stated explicitly, and a citation. Write the maths before the code. §10 collects the maths.

---

## What this document is

The founding plan for a new repository: decisions, the reasons for them, open items, and the schedule (§9). It follows MotoBat's house style:

- **S** = sourced, **D** = derived, **A** = assumed, **M** = to be measured, **TBD** = not yet known. **No figure is invented.**
- Decisions are recorded with their reasons and the alternatives that were rejected.
- When a decision is superseded, strike it through with the date. Don't delete it.

**What changed in v2 (11 Nov 2026):**

- Added the fleet energy layer (§5).
- Project Chrono is now the laptop path for lunar terrain physics (§4.3).
- NASA's Lunar Autonomy Challenge simulator is an optional benchmark, pending access (§4.3).
- The phases are reorganised: **Phases 1–2 on the laptop in Gazebo, Phase 3 full-scale in the cloud.**
- Added a dated schedule (§9).

---

## 0. Current status

| Area | State |
|---|---|
| **Gate: MotoBat finished** | **Not yet.** MotoBat is in its Phase 1 (task 1.4, detection) and Phase 2 (tasks 2.5–2.7). Phases 3–4 have not started. This project's Phase 1 doesn't start before the gate (§9.1) |
| Base | **Decided:** AgileX Ranger (full size) — §2.1 |
| Arm | **Decided:** Yaskawa HC10DTP, both welder and picker — §2.2 |
| Weld process | **Decided:** GMAW for Stages 1–2, laser beam welding for Stage 3 — §2.3 |
| Software stack | **Same as MotoBat**, plus Nav2, Chrono, and the fleet layer — §2.4 |
| Where things run | **Decided:** Phases 1–2 on the laptop (Gazebo, Chrono, numpy). Phase 3 in the cloud (Isaac Sim, OmniLRS) — §6 |
| Ranger simulation model | **Doesn't exist** for the full-size Ranger in Gazebo Harmonic or Isaac Sim. Build it (task 1.6) |
| Lunar Autonomy Challenge simulator | **Not public.** Access request to APL pending (task 0.2) |
| Hardware purchases | **None.** Simulation only |

---

## 1. Constraints (read MotoBat's docs before changing these)

### 1.1 Hardware: the same laptop as MotoBat

Dell G15 5530 · i9-13900HX (24C/32T) · RTX 4060 Laptop (**8 GB VRAM**) · **16 GB DDR5, dual-channel** (after the 31 Aug 2026 motherboard replacement, MB §1.0) · 35 GB swap · Ubuntu 24.04 · ROS 2 Jazzy · Python 3.11 venv.

**Runs on it** (some shown by MotoBat already): ROS 2, MoveIt2, RViz2, Open3D, numpy/trimesh, `defectgen`, Gazebo Harmonic (one arm at RTF ≈ 1.0, MB status 2.2), and, per §4.3 and §5, Project Chrono with ray-traced lidar and the fleet Monte Carlo.

**Doesn't run on it:** Isaac Sim at this project's scale (§1.2).

**Budget:** none for hardware. Cloud time only in Phase 3 (§6.3).

### 1.2 CORRECTION to MB §4.0b: Isaac Sim needs 16 GB of VRAM

MB §4.0b says the laptop's 8 GB of VRAM meets the minimum and that RAM is the only bottleneck. **That's no longer true.** The current requirements page lists:

| | Minimum | Good | Ideal |
|---|---|---|---|
| RAM | 32 GB | 64 GB | 64 GB |
| GPU | GeForce RTX 4080 | GeForce RTX 5080 | RTX PRO 6000 Blackwell |
| VRAM | **16 GB** | 16 GB | 48 GB |

Its footnote warns that GPUs with less than 16 GB of VRAM may not run complex scenes that render more than 16 MP per frame. The Isaac Sim 5.0 page, which is the version OmniLRS targets, lists the same 32 GB / 16 GB minimum (S). Isaac Lab needs more on top of that (S).

**Consequences:**

- The laptop is below the minimum on **both** RAM and VRAM. A RAM kit fixes only one of the two.
- All Isaac Sim and OmniLRS work happens in Phase 3, on a rented GPU, **with the whole ROS 2 stack on the same instance** (MB §4.0b's architecture: do not split DDS across the internet).
- **Action for MotoBat:** add a dated correction to MB §4.0b (task 0.3).

### 1.3 Scope discipline

- **MotoBat finishes first** (§9.1).
- **Simulation only.** Real hardware appears only as datasheet numbers.
- **No weld-pool physics.** Beads are geometry generated from the torch or beam trace, as in `defectgen/weld.py`.
- **No quantitative result rests on wheel–soil physics alone.** Disturbances are injected and labelled (§3). Validated terramechanics (Chrono, Phase 2) is the exception, and it's used to *set the range* of those disturbances.

---

## 2. Decisions so far

### 2.1 Base: AgileX Ranger (full size), not the Clearpath Ridgeback

**Decision (11 Nov 2026):** the AgileX Ranger, the full-size 4-wheel-drive / 4-wheel-steer model.

| | AgileX Ranger | Clearpath Ridgeback |
|---|---|---|
| Payload | **150 kg** (S; sources differ by configuration, so verify) | 100 kg (S) |
| Size | 1228 × 876 × 475 mm, wheelbase 560 mm, track 890 mm (S) | 960 × 793 × 296 mm, 135 kg (S) |
| Drive | 4WD + 4WS, suspension, T-slot deck (S) | Mecanum, indoor (S) |
| Interface | CAN; open `ranger_ros2` driver (S) | Native ROS 2 Jazzy (S) |
| Simulation | **None for the full-size model.** AgileX's `ugv_gazebo_sim` covers the Ranger Mini v1–v3 on Gazebo Classic only (S) | `ros-jazzy-clearpath-simulator` on Gazebo Harmonic (S) |

**Why the Ranger:**

1. **Payload honesty.** The HC10DTP alone is 58 kg (S). The Ridgeback's 100 kg is exceeded once the rest of the equipment is on board.
2. **The research question needs uneven ground.** On a level floor, mobile welding reduces to docking accurately.
3. **Building the model is something to show:** a URDF from datasheet dimensions, suspension, and a 4WS controller.

**Costs accepted:** a hand-written model (task 1.6), with mass, inertia and suspension stiffness A until sourced. Probably no stock 4WS controller for Jazzy (task 1.7: check, then write one). Vendor risk is handled by the simulation replacing the driver.

**Rejected:** Ridgeback (over payload, flat floors only). MiR, Omron MD, OTTO and Robotnik (closed APIs or no public model, and level floors). Ranger Mini (payload too low).

### 2.2 Arm: Yaskawa HC10DTP

**Decision:** the HC10DTP is the welder. In Stage 2, a second HC10DTP on a second Ranger is the picker.

| Item | Value |
|---|---|
| Payload / reach / mass | 10 kg / 1379 mm / 58 kg (S) |
| Controller | YRC1000micro, which accepts 1- or 3-phase supply (S). **Mass and power TBD** |
| Welding pedigree | ArcWorld HC: a portable cell built around one HC10DTP, with Power and Force Limiting and hand-guided teaching, IP67 (S) |
| MoveIt | Reuse `motobat_hc10dt_moveit_config`, KDL, chosen by measurement (MB status 2.1) |

**Same arm for both robots:** this follows MotoBat's two-GP4 reasoning. Identical arms mean identical control code.

**Open:** MotoBat measured its flange as **DT, not DTP** (MB fixlist §0). Check that the DTP's kinematics and flange match the HC10DT description (task 1.8).

**Rejected:** the GP25 + MOTOPAC-RL2D package. Its 18.3 kg head exceeds 10 kg, and an 8 kW laser can't be enclosed on an open site (ISO 11553). GP-series arms in general, because they're non-collaborative and need fencing a mobile robot can't carry.

### 2.3 Weld process: arc on Earth, laser beam on the Moon

| Stage | Process | Why |
|---|---|---|
| 1, 2 | **GMAW** (MIG) | Yaskawa's own HC10DTP pairing (ArcWorld HC). Seam trackers for robotic GMAW are standard products (§7) |
| 3 | **Laser beam welding** | Gas-shielded arc welding depends on shielding gas. *Vulkan* (Soyuz-6, 1969) tested electron beam, plasma arc and gas metal arc welding in orbit, and found electron beam the most suitable (S). NASA Marshall now lists the lunar surface among its in-space laser-beam-welding cases (vacuum, 0.17 g), **in collaboration with Ohio State on parabolic-flight experiments** (S) |

The Stage 3 switch is itself a finding to report: the perception pipeline shouldn't care which process follows the seam. If it does, that's a result.

### 2.4 Software stack

**Unchanged from MotoBat:** Ubuntu 24.04 · ROS 2 Jazzy · MoveIt2 · ros2_control · Open3D / PCL / OpenCV · Python 3.11 venv · PyTorch · Gazebo Harmonic + `ros_gz` · Isaac Sim 6.x (cloud) · Foxglove · rosbag2.

**MotoBat decisions carried forward:**

- **FastDDS, not Cyclone** (MB §5: NVIDIA recommends it for single-machine Isaac Sim, and MotoROS2 requires it). Don't set `RMW_IMPLEMENTATION`.
- **The MotoROS2 contract** (MB §6): arms are commanded only through `FollowJointTrajectory` and read only from `/joint_states`. Simulator-specific code lives only in `*_bringup`.
- **Sensor models live once, in a ray-cast library** (MB §4B.3). Simulators supply poses and raw rays or depth.
- **One `main` branch; separation by package** (MB `project_structure.md` §1).

**New:**

| Tool | Role | Phase |
|---|---|---|
| Nav2, `robot_localization`, `slam_toolbox` / 3D lidar SLAM | Navigation, EKF, mapping | 1 |
| ros2_control 4WS controller (written or adapted) | Ranger drive | 1 |
| **Project Chrono** (PyChrono, Chrono::Vehicle SCM, Chrono::Sensor) | Deformable lunar terrain + ray-traced lidar on the laptop (§4.3) | 2 |
| Gazebo DEM heightmaps | Real lunar elevation tiles, rigid terrain, lunar gravity (§4.3) | 2 |
| Isaac Sim 6.x | Two-robot scenes at full sensor load, outdoor lighting (§4.2) | 3 |
| **OmniLRS** | Photoreal lunar scenes on Isaac Sim (§4.3) | 3 |
| Lunar Autonomy Challenge simulator | Optional benchmark, if APL grants access (§4.3) | 3 |
| **NREL BLAST-Lite** | Battery degradation models (§5.3) | 3 |
| numpy / SimPy, HiGHS or OR-Tools | Fleet simulation and scheduling optimization (§5) | 3 |
| Isaac ROS (`nvblox`, `visual_slam`) | Optional GPU mapping and VSLAM, both list Jazzy support (S) | 3 |

### 2.5 Design principle, extended from MotoBat

MotoBat: *use the arm for coverage and the sensor for measurement* (MB §3.7.6).

Mobile-MotoBat adds: **use the base for reach and the seam scan for accuracy.** After the base stops and settles, the wrist sensor measures the seam **in the tool frame**, and the weld path is built from that measurement. Base pose error must never enter the weld path directly. §10.2 shows why: a 1° tilt nobody measured moves the tool about 24 mm at full reach.

---

## 3. The research question and the claims to defend

### 3.1 The questions

> **Q1 (perception).** How large a base disturbance (tilt, settling, suspension give under arm load, localization error) can a scan-then-weld mobile welder absorb before the weld path leaves tolerance, and how much does re-scanning the seam after the base settles buy compared with trusting the base's pose estimate?
>
> **Q2 (energy, §5).** For a fleet sharing one charger, what inspection effort and charging policy minimise the cost of energy, downtime and battery wear **together with the cost of weld defects that get through and the rework they cause**, and how does that optimum move from shop floor to construction site to the Moon?

### 3.2 Claims this project can defend

1. **An error budget, decomposed.** Weld-path error broken into localization, tilt and settling, suspension compliance, arm repeatability, and seam-sensor error, each one injected alone.
2. **A characterised boundary.** Sweep each disturbance. Report where open-loop (trust the base pose) fails and where scan-then-weld fails.
3. **A cross-robot correction loop (Stage 2).** The picker places part *k*. The welder's scan measures the fit-up error *e_k*. The correction is applied to part *k+1*. This is MB §9.9's loop closed across two robots on two moving bases, with a re-derived stability bound (§10.6).
4. **What breaks on the Moon.** 1/6 g, low sun, no GNSS. Dynamic tip-over margin gets about 6× worse (§10.4).
5. **The energy–quality coupling (new).** Prior fleet work treats a task as a fixed energy cost. Here the same task's inspection effort sets both its energy and its probability of letting a defect through, and every defect that gets through means rework, which costs energy, time and battery wear. The result is a Pareto front, with Monte Carlo confidence bands, comparing at least a threshold-charging baseline and a battery-health-aware baseline from the literature (§5.1).

### 3.3 What makes it a research artifact (as in MB §3.6)

| Deliverable | Why |
|---|---|
| Disturbance + joint generators, labelled dataset | Reproducible. Others can benchmark against them |
| Error budget per stage, with confidence intervals | The headline numbers for Q1 |
| Boundary plots and the cross-robot K-sweep | Controls results, not a demo |
| Positioning test modelled on NIST's RMMA | NIST's artifact-based test method measures mobile-manipulator positioning within about 2 mm measurement uncertainty (S). A benchmark reviewers recognise |
| Energy–quality Pareto front with Monte Carlo confidence bands | The headline result for Q2 |
| Lunar Autonomy Challenge benchmark numbers (if access is granted) | Directly comparable with published work on the same simulator (§4.3) |
| Write-up structured as a paper | Problem, method, results, limitations, future work |

### 3.4 Precedent: say what exists

- **Shimizu Robo-Welder (2018):** welds steel columns on high-rise sites after measuring the groove with a laser. Two robots usually work one column. It ran on night shifts for safety (S). Close to Stage 2, but **fixed to the column**. Here, the base moves.
- **Yaskawa MH12 on Clearpath OTTO 1500 (2016):** a Yaskawa arm on a mobile base, on a factory floor (S).
- **Mobile-manipulator accuracy:** about 1.67 cm mean end-effector error while tracking against odometry in one study (S). Precise-parking work argues that parking accuracy is the key to process automation (S). This project argues **measure the seam instead of trusting the park**, and has to show it.
- **Fleet energy:** battery-health-aware fleet scheduling exists. A 2026 preprint jointly optimises task assignment, charging decisions, charger access and degradation across a fleet (S). Its authors note that fleet methods mostly treat battery level as a feasibility constraint, and a related paper says fleet methods rely on simplified, distance-proportional energy estimates (S). **I found no work that couples task quality (inspection, weld defects, rework) to fleet energy. That doesn't prove none exists. Do a proper literature search before claiming it (task 3.13).**

---

## 4. The three stages

Each stage lists its scenario, environment, perception tasks, simulator, what gets measured, and an exit criterion. **A stage is done when its exit criterion is met, not when its world looks good.**

### 4.1 Stage 1 — Shop floor (Phase 1, laptop)

**Scenario:** one Ranger + HC10DTP welder. Steel fillet and butt joints on fixtures. Drive to a work pose → stop → scan the seam → check fit-up → weld.

**Environment:** a flat floor with small injected tilt and settling. Good lighting. A static map.

**Perception:** 2D/3D lidar localization (EKF with odometry and IMU) to about 5 cm at the work pose. Coarse part localization with the snapshot 3D camera. Seam measurement with the wrist laser line sensor (MotoBat's profilometer model). The weld path is built in the tool frame. Fit-up gate: if the joint is out of tolerance, report it and don't weld.

**Simulator:** Gazebo Harmonic.

**Measured:** weld-path error per injected disturbance, open-loop vs scan-then-weld. **Energy per task (drive, arm, scan, compute, arc) logged for §5.**

**Exit:** Stage 1 error budget table and boundary plot, with sensor parameters frozen and hashed before the results were looked at.

### 4.2 Stage 2 — Construction or rocket-assembly site, two robots (Phase 2 laptop, Phase 3 cloud)

**Scenario:** a large steel structure (frame, tower, gantry) in an assembly hall or on an open site. **Robot P** (picker) carries plates and brackets from staging and holds or tacks them in place. **Robot W** (welder) scans the fit-up and welds.

> **Scope honesty:** "rocket assembly" here means steel structures and brackets. Propellant-tank barrels are typically friction-stir welded, a different process that isn't modelled. *(General knowledge, not sourced here. Cite before publishing.)*

**New compared with Stage 1:**

- **Uneven, cluttered ground.**
- **Two robots** sharing a map and handing off.
- **A CAD/BIM prior map.** Shimizu's robots compare live laser data with BIM (S). Here the structure's CAD is the reference mesh, which is MotoBat's `(mesh, labels)` design rule applied to a new domain.
- **Partly outdoor:** sun and shadow on shiny steel, dust as a dropout model.

**Perception added:** 3D lidar localization against the CAD prior. Fit-up inspection (gap, misalignment, missing tack), reusing MotoBat's detector. The cross-robot correction loop. Safety zones with people as dynamic obstacles (no certification claimed).

**Simulator:** Phase 2 runs Gazebo on the laptop, headless if needed, with reduced sensor rates. **Phase 3 runs Isaac Sim in the cloud** at full sensor load: RTX lidar digital twins, outdoor lighting, two articulated robots.

**Ground truth:** in the real world, a robotic total station tracking a prism on the tool (S). In simulation, the simulator's own pose.

**Exit:** error budget under Stage 2 disturbances, plus the cross-robot K-sweep (convergence → critical point → divergence).

### 4.3 Stage 3 — Lunar surface (Phase 2 laptop, Phase 3 cloud)

**Scenario:** the same two-robot task on lunar terrain.

**Terrain data (all S):**

| Source | Resolution | Notes |
|---|---|---|
| LOLA-based south-pole DEMs (NASA GSFC) | 5 m/pixel | Geodetic reference. Shape-from-shading products sharpen it with LROC NAC images |
| LROC NAC DTMs | 2–5 m/pixel | Stereo-derived. A USGS south-pole DEM has about 4 m average vertical accuracy |
| JAXA LUPEX candidate-site DTMs | 1 m/pixel | Six south-pole sites (Hemmi & Inoue 2025) |

**Important limit (D):** even 1 m/pixel can't resolve wheel-scale terrain, because the rover spans about one pixel. Real DEMs set the large-scale slopes and craters. Rocks and small craters are added procedurally, which is NASA's own approach: fractal expansion of the DEM and insertion of rocks and craters based on size-frequency distribution models (S). **Say which scale is real and which is generated.**

**Three simulators, three jobs:**

| Simulator | Job | Where | Why |
|---|---|---|---|
| **Project Chrono** (Chrono::Vehicle + SCM + Chrono::Sensor) | **Mobility on deformable regolith, 3D lidar scans, 1/6 g** | Laptop, Phase 2 | A NASA-funded study used a Chrono digital twin of VIPER on deformable SCM terrain (S). Chrono's terramechanics have been validated against NASA SLOPE-lab tests with lunar soil simulants (S). Its lidar is ray-traced and needs a Turing-or-newer NVIDIA GPU and driver R580+ (S); the RTX 4060 qualifies. Its simulations revealed that decades of Earth-based rover tests were misleading, because they reduced rover weight but not the terrain's own response to gravity (S). **Speed:** SCM runs at RTF ≤ 1.0 on smooth deformable terrain, but 30–40 with movable rocks (S), so use it for batch runs, not interactive ones |
| **Gazebo Harmonic** with DEM heightmaps | Rigid real terrain + `gpu_lidar` + full ROS stack | Laptop, Phase 2 | Gazebo reads GeoTIFF DEMs through GDAL (S). A Space ROS demo already loads a 5 m/pixel lunar DEM (Shoemaker) into Gazebo (S). **Crop the tiles.** NASA Ames found an 8K, 213 MB DEM took about 5 minutes to load (S) |
| **OmniLRS** (on Isaac Sim) | Photoreal low-sun lighting, procedural rocks, two-robot scenes | Cloud, Phase 3 | Real LRO DEMs (usually 5 m/pixel), procedural environments, ROS 2, Docker, terrain deformation in some environments (S). **Version risk:** its README lists a migration to Isaac Sim **5.0**. Pin it in its own container if it doesn't run on 6.x |
| *Lunar Autonomy Challenge simulator* (optional) | Benchmark against published work | Laptop GPU fits CARLA's requirements (S); Phase 3 | §4.3.1 |

#### 4.3.1 The NASA Lunar Autonomy Challenge simulator — optional, access pending

- **What it is:** NASA, JHU APL, Caterpillar and Embodied AI built it on a custom version of CARLA (Unreal Engine). It shows a digital twin of NASA's IPEx robot on a 40 m × 40 m lunar terrain patch, with 8 cameras, an IMU and battery management (S). Published results exist on it: Stanford's first-place stack (open-source code) and MIT's LunarLoc (S).
- **Access:** **not public.** The package was given only to selected teams (S). The competition ran Nov 2024 – May 2025. OSU wasn't a selected team. → **Task 0.2: email APL with a faculty advisor and a one-paragraph research use case.** Don't ask former teams for their copy.
- **Lidar:** the challenge interface exposes only the cameras and IMU. The underlying CARLA interface has more (S), and CARLA itself has a ray-cast lidar and, since 0.9.16, native ROS 2 publishing as `sensor_msgs/PointCloud2` (S). **Unverified:** which CARLA version the challenge build uses, and whether the lidar sensor is still included in it. First test if access is granted: spawn a lidar on the IPEx and echo the topic into the Open3D pipeline.
- **Limits:** no arm (CARLA is a vehicle simulator), a small map, and lidar traced against simple collision shapes (fine for navigation, not for seam measurement).
- **Usable now:** MIT's LunarLoc datasets from this simulator are public, with a playback module (S). They're camera data, not lidar.

**New in Stage 3:** gravity 1.62 m/s² (dynamic tip-over and traction, §10.4). No GNSS: VIPER's navigation hardware was stereo navigation cameras, hazard cameras, a star tracker, an IMU, and LED lights tuned for permanently shadowed regions (S). Low sun at the pole (S). VIPER was the first planetary rover with headlights (S). Vacuum, so laser beam welding. Regolith settling during the weld comes from Chrono, then is injected and labelled.

**Exit:** the lunar error budget, plus a list of what broke compared with Stage 2 and why.

---

## 5. Fleet energy layer (Phase 3)

### 5.1 The question and the gap

Q2 (§3.1). Battery-health-aware fleet scheduling is an active area. A 2026 preprint jointly optimises task assignment, service order, charging decisions, charging mode, charger access and degradation across a fleet (S). It builds on the observations that fast charging intensifies cycling wear and that idling at high state of charge accelerates calendar aging (S). The same preprint notes that most fleet planning treats battery level only as a feasibility constraint (S). A related paper notes that fleet methods rely on simplified, distance-proportional energy estimates (S). **The contribution here is to couple each task's energy to its quality outcome**, using MotoBat's inspection and weld models.

### 5.2 Architecture: physics measured once, Monte Carlo samples it

**Never run Gazebo, Chrono or Isaac inside the Monte Carlo loop.** Phases 1–3 measure distributions once. The fleet simulation samples from them.

```
Phase 1–3 physics sims ──▶ surrogates (lookup tables / fitted distributions)
   E_drive(d, terrain), E_arm(trajectory), E_scan(n_views), E_compute(pipeline),
   E_arc(seam length), P_escape(n_views, disturbance), P_reject(fit-up)
                               │
                               ▼
Fleet discrete-event simulation (numpy / SimPy)
   robots × tasks × charger × policy × seeds  ──▶  BLAST-Lite degradation
                               │
                               ▼
Optimization: day-ahead MILP (HiGHS / OR-Tools) or policy-parameter search
   ──▶ Pareto front: throughput · energy · battery life · defect escape · rework
```

### 5.3 Models

| Model | Content | Source / status |
|---|---|---|
| Energy per task | Drive + arm + scan motion + compute + sensors/lights + arc | Measured in Phases 1–3 (M). **Compute alone has been measured at 33% of an AMR's total energy** (S), so perception choices matter to the battery |
| Arc energy | Arc power × arc-on time | Welding power source datasheet **TBD**. Reference point: Fronius's Ignis Battery welder packs 750 Wh, about 30 minutes of TIG at 100 A, and weighs 12 kg (S). It's a manual MMA/TIG unit, not GMAW, but it's a real sanity check |
| Quality | P(defect escapes inspection \| views, disturbance), P(fit-up reject) | MotoBat's precision/recall machinery (MB status 1.8), re-measured for structural joints (M) |
| Rework | Re-drive + re-scan + grind/re-weld energy and time | A, stated |
| Battery degradation | Calendar (SOC, temperature) + cycle (throughput, C-rate) | **NREL BLAST-Lite:** open-source models identified from lab aging data, including fast simplified simulations (S). It ships an LFP prismatic model, and its docs suggest pack life is 20–30% shorter than cell life (S). **The Ranger's pack chemistry and capacity are TBD** |
| Charger | One shared station, charge-rate curve, queueing | A |
| Stage 3 power | Solar availability and lunar night (no grid) | To source |

### 5.4 Policies to compare

1. Threshold charging (charge below x% SOC): the industry baseline.
2. Battery-health-aware scheduling, a reimplementation of the 2026 preprint's idea (S).
3. **Quality-coupled policy (this project):** inspection effort and charging chosen jointly.

### 5.5 Monte Carlo design and compute

- Seeds × task-arrival processes × disturbance draws × policies, over 1–5 simulated years.
- Report means with confidence intervals. Pin the number of runs with a convergence check, not a guess.
- **Cost (M, preliminary):** a toy version (4 robots, 1-minute steps, 1 simulated year, vectorised numpy) took **44.5 s for 1000 runs on a 2-core test machine**. The laptop's 24 cores run batches in parallel, so tens of thousands of multi-year runs is minutes to hours, with negligible RAM. **The optimizer is the expensive part:** fleet scheduling can only be solved quickly for small instances (S). Solve once per simulated day, or optimise policy parameters offline. Never call the solver at every step.
- **This runs on the laptop.** It sits in Phase 3 because it needs Phase 3's surrogates, not because it needs the cloud.

### 5.6 A possible tie-back to MotoBat (hypothesis, source it first)

MotoBat inspects busbar welds on battery modules. If a weld defect raises contact resistance and speeds up pack aging, MotoBat's defect classes become an **input** to the fleet's degradation model, and the two projects close a loop. **Unverified. Search the literature before claiming it (task 3.14).**

---

## 6. Where things run

### 6.1 Laptop (Phases 1–2)

| Work | Fits? |
|---|---|
| numpy/Open3D: joint and disturbance generators, seam models, error budget | ✅ |
| Gazebo Harmonic: Ranger + HC10DTP, Nav2, Stage 1. Stage 2 with two robots headless, reduced sensor rates | ✅ (two robots: M, check RTF) |
| Gazebo with cropped lunar DEMs + `gpu_lidar` | ✅ |
| Chrono: SCM regolith + ray-traced lidar | ✅ GPU qualifies (S). Batch speed, not real time, with rocks |
| Fleet Monte Carlo + BLAST-Lite | ✅ (§5.5) |

### 6.2 Cloud (Phase 3)

| Work | Instance |
|---|---|
| Isaac Sim Stage 1 port + Stage 2 at full sensor load | L4 (24 GB VRAM), whole ROS 2 stack on the instance |
| OmniLRS Stage 3 | Same, in a separate container if it's pinned to Isaac Sim 5.x |
| Large Monte Carlo sweeps (optional) | CPU instance, only if the laptop is too slow |

### 6.3 Cloud budget (D)

GPU prices as recorded in MB §4.0b (re-check before renting): Vast.ai L4 $0.33/hr · Runpod L4 $0.49/hr · AWS `g6.2xlarge` $0.98/hr. **Phase 3 estimate: 150–250 GPU hours, about $50–245.** Rules: rent for a few hours first to prove the container launches and streams. Stop instances when idle. Mount the shader caches (MB §4.0b).

### 6.4 Onboard computer (paper design only)

| Option | Key figures (S) | Fit |
|---|---|---|
| **Jetson AGX Orin 64 GB** | 275 TOPS, 64 GB | Default for the paper design |
| Jetson AGX Thor (T5000) | 2070 FP4 TFLOPS (sparse), 128 GB at 273 GB/s, 4 × 25 GbE, up to 130 W. $3,499 at launch (Aug 2025); resellers now list it higher | Only if learned perception is in scope |

Also on board, all **TBD** in the power and payload budget: YRC1000micro (AC supply, so an inverter), welding power source + wire feeder, gas cylinder (Stages 1–2), CAN bridge. **The compute's power draw enters §5 directly.**

---

## 7. Sensors

Rule from MB §3.7.3: **to detect a feature of size *D*, a sensor needs noise σ ≲ *D*/3 and sample spacing ≲ *D*/2.**

### 7.1 On the welder's arm

| Sensor | Role | Sim model | Example products (S) |
|---|---|---|---|
| **Laser line seam sensor** | Seam finding and tracking, fit-up gate, bead check | MotoBat's profilometer model (MB §4B.3), re-pinned to a seam-tracker datasheet | Meta Vision SLS (±0.1 mm, 50 mm FOV); wenglor weCat3D MLZL (Yaskawa interface); Servo-Robot AUTO-TRAC (joint validation before welding, automatic joint search) |
| Snapshot 3D camera | Coarse part and joint localization | MotoBat's Zivid-class model | Zivid 2+ class |

**Payload (D, partial):** Zivid 1.0 kg (S) + seam sensor TBD + torch and dress TBD + mount TBD, against 10 kg.

**Specular steel** remains the biggest sim-to-real gap (MB §4B.3). Model the dropout or state it as a limitation.

### 7.2 On the picker's arm

Snapshot 3D camera for grasping. Joint torques (the HC series' PFL) for contact-based placement.

### 7.3 On the base

| Sensor | Role | Sim | Stages |
|---|---|---|---|
| **3D lidar** | Localization, mapping, obstacles | Gazebo `gpu_lidar` · Chrono::Sensor lidar · Isaac RTX lidar with Ouster/HESAI/SICK twins (S) · CARLA ray-cast (LAC) | 1–3 |
| **IMU** | Base tilt. **The key sensor for claim 1** | Simulator IMU + noise | 1–3 |
| Wheel encoders, steering angles | Odometry | Joint states | 1–3 |
| Stereo camera | VSLAM / VIO | RGB pair; `isaac_ros_visual_slam` (S) | 2–3 |
| Safety laser scanner | Protective fields | 2D lidar (no certification claimed) | 1–2 |
| RTK GNSS / total-station prism | Absolute position / ground truth | Simulator pose + noise | 2 |
| Star tracker; mast stereo + LED lights | Navigation without GNSS, under low sun | Attitude + noise; cameras + lights | 3 |

The Livox Mid-360's non-repetitive scan pattern isn't stock in Isaac Sim (S). Use an Ouster.

**Not used:** lidar or passive stereo for seam measurement (fails the σ rule). GNSS on the Moon.

---

## 8. What each simulator can and can't do here

| | Use it for | Don't use it for |
|---|---|---|
| **Gazebo Harmonic** | ROS 2 plumbing, Nav2, MoveIt, TF, two robots headless, DEM heightmaps, lunar gravity | Deformable soil, photoreal lighting |
| **Chrono** | Deformable regolith (SCM, CRM), 1/6-g mobility, ray-traced lidar, settling under arm load | Being the ROS front end. Check its URDF import and ROS 2 bridge before committing (task 2.6) |
| **Isaac Sim 6.x** | URDF → articulation; Jazzy bridge; `isaacsim.ros2.control` (6.1) with multiple robots (S); RTX lidar twins; lighting; Replicator synthetic data; parallel instances (S); WebRTC streaming | Weld physics. Seam-sensor fidelity. Quantitative wheel–soil numbers (PhysX wheeled-robot contact is known to misbehave, S) |
| **OmniLRS** | Photoreal lunar scenes, procedural rocks, multi-robot | Validated terramechanics. Its deformation is qualitative here |
| **LAC simulator** | Navigation and mapping benchmark, battery-limited operation | Anything with an arm |
| **`defectgen` / seam models** | **All metrology numbers** | — |

---

## 9. Schedule

### 9.1 How it is sequenced (MotoBat's lesson)

MotoBat began with a dated six-week plan and changed to dependency ordering on 15 Sep (MB §3.5). **This plan has both.** The dates below are **targets for pacing, not deadlines.** When a target slips, the dependency order holds and the dates move. Re-date the table when the gate passes.

**Gate:** MotoBat's Phase 4 is done: results, README written as a paper, preprint posted. **Estimated mid-February 2027 (A).** The estimate is MotoBat's remaining tasks (Phase 1: 1.4–1.12; Phase 2: 2.5–2.7; Phase 3: 3.0–3.6; Phase 4: 4.1–4.5) at its November 2026 pace. **If the gate passes later, every date below shifts by the same amount.**

**Never trade away** the Phase 1 error budget or the Phase 3 energy–quality Pareto front. Everything else is presentation around them.

### 9.2 Timeline (targets)

| Week | Dates | Phase | Focus | Depends on |
|---|---|---|---|---|
| — | now → gate | 0 | Zero-cost items only: APL email, compatibility checker, MotoBat correction (§9.3). **No Mobile-MotoBat code** | — |
| 1 | 15–21 Feb 2027 | 1 | Joint generator (1.1); disturbance generator (1.2) | Gate |
| 2 | 22–28 Feb | 1 | Seam sensor model re-pinned and **frozen** (1.3); seam extraction → tool-frame path (1.4) | 1.1–1.2 |
| 3 | 1–7 Mar | 1 | Error-budget harness, first boundary plot **without a simulator** (1.5) | 1.3–1.4 |
| 4 | 8–14 Mar | 1 | Ranger URDF/xacro + suspension (1.6); 4WS kinematics written (§10.5) | — (parallel to 1.5) |
| 5 | 15–21 Mar | 1 | 4WS controller (1.7); HC10DTP mount + DT/DTP check (1.8); **tip-over check** (1.9) | 1.6 |
| 6 | 22–28 Mar | 1 | Nav2 + EKF + SLAM on a shop map (1.10) | 1.7 |
| 7 | 29 Mar–4 Apr | 1 | Stage 1 end to end in Gazebo; energy-per-task logging (1.11–1.12) | 1.5, 1.10 |
| 8 | 5–11 Apr | 1 | **Stage 1 results** (1.13) · *buffer* | 1.11 |
| 9 | 12–18 Apr | 2 | Structural-site world from CAD + uneven ground (2.1); second robot (2.2) | Phase 1 |
| 10 | 19–25 Apr | 2 | Picker task + cross-robot loop in Gazebo (2.3–2.4) | 2.2 |
| 11 | 26 Apr–2 May | 2 | Stage 2 laptop-scale results + **cross-robot K-sweep** (2.5) | 2.4 |
| 12 | 3–9 May | 2 | Chrono: Ranger on SCM at 1/6 g, lidar (2.6–2.7) | 1.6 |
| 13 | 10–16 May | 2 | Lunar DEM tiles in Gazebo + Chrono; settling ranges → disturbance generator (2.8–2.9) | 2.7 |
| 14 | 17–23 May | 2 | Stage 3 laptop-scale results (2.10) · *buffer* | 2.9 |
| 15 | 24–30 May | 3 | **Cloud bring-up:** rent hours, container, streaming (3.1); Isaac Stage 1 port + Gazebo agreement check (3.2) | Phase 2 |
| 16 | 31 May–6 Jun | 3 | Isaac Stage 2, full sensor load (3.3) | 3.2 |
| 17 | 7–13 Jun | 3 | Isaac Stage 2 results (3.4) | 3.3 |
| 18 | 14–20 Jun | 3 | OmniLRS bring-up + version check; south-pole tile (3.5–3.6) | 3.1 |
| 19 | 21–27 Jun | 3 | OmniLRS Stage 3 runs (3.7); LAC benchmark **if access was granted** (3.8) | 3.6, 0.2 |
| 20 | 28 Jun–4 Jul | 3 | Energy surrogates from Phases 1–3 (3.9); BLAST-Lite integration (3.10) | 1.12, 3.4, 3.7 |
| 21 | 5–11 Jul | 3 | Fleet discrete-event sim + baselines (3.11); literature searches (3.13–3.14) | 3.10 |
| 22 | 12–18 Jul | 3 | Quality-coupled policy + optimizer (3.12) | 3.11 |
| 23 | 19–25 Jul | 3 | **Monte Carlo campaign → Pareto fronts per stage** (3.15) | 3.12 |
| 24 | 26 Jul–1 Aug | 3 | *buffer*: Phase 3 is where slips land | — |
| 25 | 2–8 Aug | 4 | README as a paper; figures (4.1) | Phase 3 |
| 26 | 9–15 Aug | 4 | Demo video; clean-clone quick-start check (4.2–4.3) | 4.1 |
| 27 | 16–22 Aug | 4 | arXiv preprint (4.4); MSFC/OSU email with results (4.5) | 4.1 |

### 9.3 Task tables (as in MB `status_update.md`; run them from there once the repo exists)

#### Phase 0 — Before the gate (zero-cost only)

| # | Task | § | Output |
|---|---|---|---|
| 0.1 | Finish MotoBat Phases 1–4 | MB status | **The gate** |
| 0.2 | Email JHU APL for Lunar Autonomy Challenge simulator access (faculty advisor CC'd, one-paragraph use case) | 4.3.1 | Yes, no, or terms |
| 0.3 | Add the §1.2 VRAM correction to MB §4.0b | 1.2 | One dated line in MotoBat |
| 0.4 | Run NVIDIA's Isaac Sim Compatibility Checker on the laptop and record the result | 1.2 | Screenshot in `docs/` |
| 0.5 | Create the repo from §11's structure; commit this plan | 11 | Empty workspace that builds |

#### Phase 1 — numpy core + Stage 1, laptop, Gazebo

| # | Task | § | Output |
|---|---|---|---|
| 1.1 | Structural joint generator (fillet, butt, lap; gap, mismatch, bevel, tacks), `(mesh, labels)`, defects off by default | 4.2 | `seamgen/joints.py` |
| 1.2 | Base disturbance generator: tilt, settling, suspension compliance, localization error; labelled and sweepable | 3 | `seamgen/disturb.py` |
| 1.3 | Seam sensor model: reuse `defectgen`'s profilometer, re-pin to a seam-tracker datasheet. **Freeze and hash** | 7.1 | params hash in `tests/` |
| 1.4 | Seam extraction (RANSAC planes → seam line) → weld path in the tool frame | 10.3 | `seamgen/seam.py` |
| 1.5 | Error-budget harness: one disturbance at a time, then combinations; open-loop vs scan-then-weld | 3.2 | **first boundary plot** |
| 1.6 | Ranger URDF/xacro from published dimensions; suspension as prismatic spring-dampers; mass/inertia A | 2.1 | `mm_description` |
| 1.7 | 4WS controller: check for an existing Jazzy one, otherwise write it from §10.5 | 10.5 | `mm_ranger_control` |
| 1.8 | Mount the HC10DTP; verify DT vs DTP kinematics and flange | 2.2 | note in fixlist |
| 1.9 | **Tip-over check at full reach**, static and dynamic (§10.4) | 10.4 | pass/fail per pose |
| 1.10 | Nav2 + EKF + SLAM on a shop map | 4.1 | `mm_navigation` |
| 1.11 | Stage 1 scenario end to end in Gazebo | 4.1 | launch + rosbag |
| 1.12 | Log energy per task component (drive, arm, scan, compute, arc) | 5.3 | CSV for §5 |
| 1.13 | **Stage 1 error budget + boundary plot** | 4.1 | `git tag -a v0.1-stage1` |

#### Phase 2 — Stages 2–3 at laptop scale (Gazebo + Chrono)

| # | Task | § | Output |
|---|---|---|---|
| 2.1 | Structural-site world from CAD; uneven ground; clutter | 4.2 | world SDF |
| 2.2 | Second Ranger + HC10DTP (picker), namespaced; measure two-robot RTF | 4.2 | RTF recorded |
| 2.3 | Picker task: grasp from staging, place, hold or tack | 4.2 | — |
| 2.4 | Cross-robot correction loop (welder scan → picker offset) | 10.6 | — |
| 2.5 | **Stage 2 error budget + cross-robot K-sweep** | 3.2 | `v0.2-stage2-laptop` |
| 2.6 | Chrono bring-up: PyChrono, check its URDF import and ROS 2 bridge, Ranger on SCM terrain | 4.3 | decision note |
| 2.7 | Chrono::Sensor lidar on the Ranger; point clouds into the Open3D pipeline | 4.3 | — |
| 2.8 | Lunar DEM tile (cropped) in Gazebo with lunar gravity; procedural rocks and craters on top | 4.3 | world SDF |
| 2.9 | Chrono settling/sinkage under arm load at 1/6 g → **sets the ranges** in the disturbance generator | 1.3 | ranges with sources |
| 2.10 | **Stage 3 error budget, laptop scale**; what broke compared with Stage 2 | 4.3 | `v0.3-stage3-laptop` |

#### Phase 3 — Full scale in the cloud + fleet energy layer

| # | Task | § | Output |
|---|---|---|---|
| 3.1 | Cloud bring-up: a few rented hours; container launches and streams; shader caches mounted | 6.2 | runbook in `docs/` |
| 3.2 | Isaac Stage 1 port; **agreement check with Gazebo** (geometry must agree; a disagreement is a bug, not a result) | 4.1 | — |
| 3.3 | Isaac Stage 2 with RTX lidar twins, outdoor lighting, both robots; choose the topic bridge or `isaacsim.ros2.control` | 4.2, 8 | — |
| 3.4 | **Stage 2 full-scale results** | 3.2 | `v0.4-stage2-isaac` |
| 3.5 | OmniLRS bring-up; check against Isaac Sim 6.x, otherwise pin 5.x in its own container | 4.3 | — |
| 3.6 | Import a south-pole tile (LOLA / NAC / LUPEX), state real vs generated scale | 4.3 | — |
| 3.7 | **OmniLRS Stage 3 runs:** low sun, mast stereo + lights, star tracker; laser welding process | 4.3 | — |
| 3.8 | *If access granted:* LAC simulator: spawn lidar on IPEx, ROS 2 → Open3D; navigation/mapping benchmark vs published results | 4.3.1 | comparison table |
| 3.9 | Energy and quality surrogates from Phase 1–3 logs (E per component, P_escape, P_reject) | 5.2 | `fleet/surrogates/` |
| 3.10 | BLAST-Lite integration; check it installs in the project's Python 3.11 venv (its docs use 3.12) | 5.3 | — |
| 3.11 | Fleet discrete-event sim + baselines (threshold charging; battery-health-aware) | 5.4 | `fleet/` |
| 3.12 | Quality-coupled policy; day-ahead MILP or policy search | 5.4 | — |
| 3.13 | **Literature search:** quality-coupled fleet energy, to confirm or kill the novelty claim | 3.4 | note |
| 3.14 | Literature search: weld defects → busbar resistance → pack aging (§5.6) | 5.6 | keep or drop |
| 3.15 | **Monte Carlo campaign:** convergence-checked run counts, Pareto fronts per stage | 5.5 | `v0.5-fleet` |

#### Phase 4 — Write-up

| # | Task | § |
|---|---|---|
| 4.1 | README as a paper: Problem → Method → Results → **Limitations** → Future work | 3.3 |
| 4.2 | Demo video: shop → site → Moon, with the Pareto front | — |
| 4.3 | Clean-clone quick-start check in a fresh container | 11 |
| 4.4 | arXiv preprint (cs.RO); then a workshop paper next cycle (MB §3.6) | — |
| 4.5 | Email NASA MSFC's in-space welding group (OSU collaboration) with results | 2.3 |

### 9.4 The one real risk: Phase 3

Phase 3 is the only phase that depends on things outside the laptop: cloud availability, OmniLRS's version, APL's answer. **Mitigations decided in advance:**

- **The core results don't depend on it.** The Stage 1–3 error budgets exist at laptop scale by the end of Phase 2, and the fleet layer runs on the laptop.
- **If Isaac fights back by week 17,** report Stage 2 at laptop scale and state it in Limitations.
- **If OmniLRS won't run by week 19,** Stage 3 stands on Chrono + Gazebo.
- **If APL says no,** task 3.8 is dropped and nothing else changes.
- **Week 24 is buffer.** Use it before cutting anything.

### 9.5 Working pace

Same rule as MB §3.5: a sustainable pace with slack beats sprints followed by burnout. Most of what remains is debugging (TF, controller gains, contact, version pinning), and debugging while tired is where days disappear. The schedule assumes MotoBat's November 2026 pace and contains three buffer weeks.

---

## 10. The mathematics (stubs; write in full before the matching code)

### 10.1 Frames

`world → base_footprint → base_link (after suspension) → arm_base → … → tool0 → seam_sensor`. Keep two trees: the **estimated** one (EKF) and the **true** one (simulator). The error budget is their difference, measured at the torch tip.

### 10.2 Why the base pose can't be trusted for the weld (D)

Tool shift ≈ *L*·tan(δθ):

| Tilt δθ | at *L* = 1.379 m (max reach) | at *L* = 1.2 m |
|---|---|---|
| 0.1° | 2.4 mm | 2.1 mm |
| 0.5° | 12.0 mm | 10.5 mm |
| 1.0° | 24.1 mm | 20.9 mm |
| 2.0° | 48.2 mm | 41.9 mm |

To keep the tilt term under 0.1 mm at full reach, tilt would have to be known to about **15 arcseconds (0.004°)**. That's beyond what a base IMU and EKF give on uneven ground, and it's the quantitative case for scan-then-weld. Make it the first figure.

### 10.3 Seam extraction

Stitched profiles in the tool frame → fit the joint model (for example two RANSAC planes for a fillet, refined by least squares; their intersection line is the seam) → poses along that line at the torch's work and travel angles. Cite the methods, as MB §9.7 does for ICP.

### 10.4 Tip-over stability, and the Moon (D)

Static: the combined centre of mass must project inside the support polygon (560 × 890 mm, S). Gravity scales both the tipping and the restoring moments, **so static margins don't depend on g.** Dynamic: arm inertial forces *m·a* don't scale with *g*, but the restoring moment *m_base·g·d* does. At 1.62 vs 9.81 m/s², the same motion uses about **6× more** of the margin, and traction drops by the same factor. The consequences are slower trajectories, outriggers, or both. Masses are TBD.

### 10.5 4WS kinematics

Wheel *i* at **r**_i, base twist (*v_x*, *v_y*, ω): **v**_i = (*v_x* − ω·*r_iy*, *v_y* + ω·*r_ix*). Steering angle = atan2(**v**_i). Speed = |**v**_i| / wheel radius. Handle the ±π flip by reversing the wheel rather than rotating it 180°.

### 10.6 Cross-robot correction loop

`u_{k+1} = −K·e_k`. Recursion: `e_{k+1} = (1 − g·K)·e_k + w_k`, where *w_k* is the picker's re-park disturbance. Stability still needs `0 < K < 2/g`. Steady-state variance: `Var(e) = σ_w² / (1 − (1 − gK)²)` (D, for white *w_k*). Derive it, then show it in the sweep.

### 10.7 Localization

EKF state; process model (4WS kinematics + IMU); measurements (lidar scan matching, VSLAM, GNSS in Stage 2, star tracker in Stage 3). §10.2 depends on the tilt covariance.

### 10.8 Battery state and degradation

SOC: `SOC_{t+1} = SOC_t − P_t·Δt / (V_nom·Q(t))` (with charger input as a negative *P*). Capacity fade *Q(t)* from BLAST-Lite's calendar + cycle model (S). Write out the specific model's equations from its source before using it.

### 10.9 The fleet optimization

Decision variables: task assignment, sequence, inspection effort *n_views*, charge start/stop, charger slot. Objective: weighted throughput, energy, degradation, expected defect-escape cost and rework cost. Constraints: SOC bounds, charger capacity, precedence (picker → welder). Write the MILP with every symbol defined before the code. Linearise any bilinear aging term, as the 2026 preprint does (S).

---

## 11. Repository structure (MotoBat's pattern)

**The repo root is the colcon workspace. One long-lived branch: `main`.** Separation is by package. The stage is a launch argument.

```
mobile-motobat/                       ← git repo AND colcon workspace
├── README.md                         paper-structured front door
├── LICENSE                           Apache-2.0 (as MotoBat)
├── mm_env.sh                         sourced from ~/.bashrc; FastDDS (default)
├── docs/
│   ├── project_plan.md               this file
│   ├── status_update.md              what to do next (task tables from §9.3)
│   ├── fixlist.md                    what is known to be wrong
│   └── results/
├── seamgen/                          Phase 1: joints, disturbances, seam models (numpy/Open3D, no ROS)
├── fleet/                            Phase 3: surrogates, discrete-event sim, BLAST-Lite, optimizer (no ROS)
├── chrono/                           Phase 2: PyChrono scenes (Ranger on SCM, lidar)
└── src/
    ├── mm_description/               Ranger xacro, HC10DTP mount, sensor frames
    ├── mm_ranger_control/            4WS controller
    ├── mm_navigation/                Nav2, EKF, SLAM
    ├── mm_seam/                      seam → weld path node
    ├── mm_coordination/              picker/welder logic, cross-robot loop
    ├── mm_interfaces/                SeamMeasurement, FitUpReport, WeldPath, PlacementCorrection, EnergyLog
    ├── mm_hc10dtp_moveit_config/
    └── mm_bringup/                   Gazebo / Isaac / OmniLRS launch; the only simulator-specific code
```

```bash
ros2 launch mm_bringup stage.launch.py stage:=1 sim:=gazebo
ros2 launch mm_bringup stage.launch.py stage:=2 sim:=isaac robots:=picker,welder
python -m fleet.run --stage 3 --policy coupled --seeds 1000
```

**Reusing MotoBat:** pin `defectgen` as a dependency (a submodule or a tagged pip install). **Don't copy its sensor models.** They're frozen and hashed.

**`seamgen/` and `fleet/` must not import ROS**, so a reviewer can `pip install -e seamgen && pytest` (as MotoBat requires).

---

## 12. Open items (source before results are claimed)

1. Ranger payload (sources disagree), mass, inertia, suspension stiffness, **battery chemistry and capacity**.
2. YRC1000micro mass and power draw.
3. GMAW power source + wire feeder: mass, power and arc efficiency (one real unit).
4. Seam sensor head mass and FOV (one real product).
5. **GMAW fillet weld positioning tolerance.** To source from a standard or a seam-tracker application note.
6. HC10DT vs HC10DTP kinematics and flange.
7. A 4WS controller for Jazzy: does one exist?
8. Chrono: URDF import, ROS 2 bridge, SCM speed with the Ranger at 1/6 g.
9. OmniLRS on Isaac Sim 6.x.
10. LAC simulator access (APL), its CARLA version, lidar availability.
11. BLAST-Lite on Python 3.11.
12. Lunar power model (solar, night) for Stage 3 energy.
13. Novelty check for claim 5 (task 3.13).
14. The weld-defect → pack-aging hypothesis (task 3.14).
15. Who at OSU works with NASA MSFC on in-space laser welding.

---

## 13. Risks

| Risk | Mitigation |
|---|---|
| Starting before MotoBat is finished | Gate (§9.1); Phase 0 is zero-cost only |
| Ranger model takes weeks | Rigid chassis + 4 suspension prismatics. Fidelity lives in the disturbances |
| Wheel–soil physics produces numbers nobody can defend | Chrono sets the *ranges*; results come from labelled, injected disturbances |
| Two robots too slow in Gazebo on 16 GB | Headless, reduced sensor rates; full scale in Phase 3 |
| Cloud costs creep | Phases 1–2 and the fleet layer need no cloud; stop idle instances |
| OmniLRS version mismatch | Separate pinned container; fallback to Chrono + Gazebo (§9.4) |
| APL says no | Task 3.8 is dropped; nothing else depends on it |
| The fleet layer balloons | Surrogates only; the optimizer runs per simulated day, not per step |
| The novelty claim for Q2 doesn't survive the literature search | Then the contribution is the measured surrogates plus the Stage comparison. Still a result |
| Vendor driver quality (again) | The sim replaces the driver |

---

## References

**Bases**
- [Generation Robots — AgileX Ranger 4WS4WD](https://www.generationrobots.com/en/404081-mobile-ranger-4ws-4wd-robot.html) · [Ranger overview (payload discrepancy)](https://www.asiasatellite.co/Agilex-RANGER-RANGER.htm) · [AgileX GitHub links](https://indrorobotics.notion.site/GitHub-642cf05343844be392c252fd4e598eb9) · [ugv_gazebo_sim models](https://gitee.com/galaxy1229/ugv_gazebo_sim)
- [Ridgeback specs (MYBOTSHOP)](https://www.mybotshop.de/Clearpath-Ridgeback_1) · [Clearpath Simulator (Jazzy)](https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install)

**Arms and welding**
- [Yaskawa Education Program flyer (HC10DTP)](https://www.yaskawa.eu.com/Global%20Assets/Downloads/Brochures_Catalogues/Robotics/EducationProgram/Flyer_EducationProgram_E.pdf) · [YRC1000micro supply](https://www.yaskawa.de/_downloads/download_d10703) · [ArcWorld HC](https://www.automate.org/robotics/news/arcworld-hc-a-flexible-human-collaborative-workcell-for-robotic-welding/boa)
- [Yaskawa + Clearpath OTTO (2016)](https://www.automate.org/robotics/news/yaskawa-and-clearpath-partner-to-develop-mobile-manipulation-solution/aph) · [Shimizu Robo-Welder](https://insights.globalspec.com/article/8757/insights.globalspec.com/help/privacy) · [AP / Courthouse News](https://courthousenews.com/construction-robots-weld-bolt-lift-to-beat-worker-shortage/)
- [Fronius Ignis Battery](https://www.fronius.com/en-us/usa/welding-technology/product-information/ignis-battery) · [Ignis Battery 12 kg](https://www.fronius.com/ignisbattery)

**Seam sensors**
- [Meta Vision Smart Laser Pilot](https://www.azorobotics.com/News.aspx?newsID=643) · [Meta A700](https://emeraldinsight.com/insight/content/doi/10.1108/ir.2003.04930caf.003/full/html) · [wenglor weCat3D MLZL](https://img0.hospitalityhub.com.au/automated-weld-seam-tracking-wecat3d-mlzl-2d-3d-profile-sensors-enable-exact-seam-placement-in-robot-welding-cells/f/24120) · [Servo-Robot AUTO-TRAC](https://servo-robot.com/?p=1746)

**Isaac Sim, Isaac ROS, compute**
- [Isaac Sim requirements (latest)](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html) · [Isaac Sim 5.0 requirements](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/requirements.html) · [Isaac Lab install](https://isaac-sim.github.io/IsaacLab/v3.0.0-beta2/source/setup/installation/index.html)
- [Isaac Sim 6.0 release notes](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/overview/release_notes.html) · [6.1 release notes](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/overview/release_notes.html) · [ROS 2 install](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_ros.html)
- [RTX Lidar assets](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/assets/usd_assets_nonvisual_sensors.html) · [Livox Mid-360 thread](https://forums.developer.nvidia.com/t/livox-mid360/283074)
- PhysX wheeled issues: [bumpy plane](https://forums.developer.nvidia.com/t/weird-collisions-between-plane-and-mobile-wheeled-robot/350329) · [stuck](https://forums.developer.nvidia.com/t/vehicle-gets-unexpectedly-stuck-in-environment/262359) · [friction](https://forums.developer.nvidia.com/t/friction-cannot-be-updated-in-real-time-resulting-in-incorre-friction-only-applies-to-the-current-startup-time/311101)
- [Isaac ROS nvblox](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox) · [Visual SLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam) · [Isaac ROS 4.0 for Thor](https://forums.developer.nvidia.com/t/nvidia-isaac-ros-4-0-for-thor-has-arrived/352109)
- [Jetson AGX Thor (CNX)](https://www.cnx-software.com/2025/08/19/3499-nvidia-jetson-agx-thor-developer-kit-2070-tops-jetson-t5000-som-for-robotics-and-edge-ai/?amp=1) · [ServeTheHome](https://www.servethehome.com/nvidia-jetson-agx-thor-developer-kit-blackwell-for-robotics/2/) · [Newegg Jetson lineup](https://www.newegg.com/insider/?p=90886)

**Chrono, Gazebo and lunar simulators**
- [POLAR-Sim: Chrono VIPER on SCM (arXiv 2309.12397)](https://arxiv.org/pdf/2309.12397) · [Chrono lunar sensor simulation (arXiv 2410.04371)](https://arxiv.org/pdf/2410.04371) · [Chrono terramechanics validated against NASA SLOPE (arXiv 2405.11001)](https://arxiv.org/pdf/2405.11001) · [Chrono VIPER gravity finding](https://sciencedaily.com/releases/2025/07/250726234412.htm) · [Chrono::Sensor install requirements](https://api.projectchrono.org/module_sensor_installation.html)
- [Gazebo DEM heightmaps](https://gazebosim.org/api/sim/10/heightmap_dem.html) · [Space ROS lunar terrain demo](https://black-queen-531f.myworkers1.workers.dev/space-ros/docs/pull/36/files) · [NASA Ames lunar Gazebo, ROSCon 2018](https://ntrs.nasa.gov/api/citations/20210010714/downloads/ROSCon2018.pdf) · [NASA synthetic lunar terrain](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20190027571.pdf)
- [OmniLRS paper](https://arxiv.org/abs/2309.08997v1) · [OmniLRS repository](https://github.com/AntoineRichard/LunarSim)
- [LAC documentation](https://lunar-autonomy-challenge.jhuapl.edu/Challenge-Documentation/index.php) · [NASA LAC page](https://www.nasa.gov/directorates/stmd/student-lunar-autonomy-challenge) · [Selected teams](https://www.nasa.gov/directorates/stmd/lunar-autonomy-challenge-selected-teams) · [Winners](https://www.nasa.gov/directorates/stmd/top-prize-awarded-in-lunar-autonomy-challenge-to-virtually-map-moons-surface) · [Stanford LAC code](https://github.com/Stanford-NavLab/lunar_autonomy_challenge) · [Stanford paper](https://arxiv.org/html/2603.17232) · [LunarLoc (MIT)](https://openreview.net/pdf/adee90539eb9bd95c2832d19a45de5770c549ff8.pdf) · [APU on LAC battery](https://amuedge.com/nasas-lunar-autonomy-challenge-lineup-includes-student-team/)
- [CARLA requirements](https://carla.readthedocs.io/start_quickstart/) · [CARLA 0.9.16 native ROS 2](https://carla.org/2025/09/16/release-0.9.16) · [CARLA ROS 2 sensor topics](https://carla-ue5.readthedocs.io/en/latest/ros2_native_sensors/)

**Lunar data and missions**
- [NASA PGDA south-pole topography](https://pgda.gsfc.nasa.gov/products/104) · [USGS LRO south-pole DEM](https://astrogeology.usgs.gov/search/map/moon_lro_south_pole_dem) · [overview (≈4 m vertical)](https://astrogeology.usgs.gov/ckan/dataset/ae25c2c4-2d44-425c-afd5-4d0d8a6eccf5/resource/3e0a1e62-99d3-437e-9b6b-319d91b64989/download/lunar_south_pole_20130408_overview.pdf) · [LROC derived data](https://ode.rsl.wustl.edu/Mars/pagehelp/Content/Missions_Instruments/Lunar%20Reconnaissance%20Orbiter%20(LRO)/LROC/Derived/Intro.htm) · [JAXA LUPEX DTMs](https://jlpeda.jaxa.jp/en/product/archive/detail_14/)
- [VIPER navigation hardware (LSIC 2024)](https://ntrs.nasa.gov/api/citations/20240013903/downloads/LSIC_2024-Tech%20Infusion_Panel-VIPER_Mob-HW.pdf) · [VIPER mast and headlights](https://www.nasa.gov/?p=634089)

**Welding in space**
- [Welding in space milestones](https://ntrs.nasa.gov/api/citations/20230011078/downloads/NASA%2020230011078.pdf) · [MSFC in-space LBW, lunar case, OSU collaboration](https://ntrs.nasa.gov/api/citations/20250000018/downloads/AIAA-SciTech-ISW-ecosystem_2025-01-02.pdf) · [MSFC LBW in thermal vacuum (FabTech 2025)](https://ntrs.nasa.gov/api/citations/20250009729/downloads/AWS-FabTech_2025_2025-09-09.pdf)

**Fleet energy and batteries**
- [Fleet-level battery-health-aware scheduling (arXiv 2603.22731)](https://arxiv.org/pdf/2603.22731) · [abstract](https://www.catalyzex.com/paper/fleet-level-battery-health-aware-scheduling)
- [Auction allocation with energy-aware trajectories (arXiv 2603.21545)](https://arxiv.org/pdf/2603.21545.pdf) · [Scheduling + control for fleets (arXiv 2510.23129)](https://arxiv.org/pdf/2510.23129v1)
- [TCM / MTC / AMR compute energy (NSF PAR)](https://par.nsf.gov/biblio/10502396) · [Maintenance-aware fleet management (IEEE)](https://ieeexplore.ieee.org/document/10323097)
- [NREL BLAST-Lite (PyPI)](https://pypi.org/project/BLAST-Lite/) · [repository metadata](https://science.ecosyste.ms/projects/76536)

**Mobile manipulator accuracy**
- [NIST TN 2108 — RMMA](https://nvlpubs.nist.gov/nistpubs/TechnicalNotes/NIST.TN.2108.pdf) · [NIST AMS 100-46](https://nvlpubs.nist.gov/nistpubs/ams/NIST.AMS.100-46.pdf) · [Precise parking (arXiv 2302.04486)](https://arxiv.org/pdf/2302.04486) · [Whole-body EE tracking 1.67 cm (arXiv 2206.15298)](https://arxiv.org/pdf/2206.15298)