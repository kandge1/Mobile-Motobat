# Mobile-MotoBat — Project Plan

**Mobile welding with Yaskawa arms on rough-terrain bases: perception that keeps the weld on the seam when the ground does not hold still.**

A Yaskawa HC10DTP on an AgileX Ranger chassis welds structural joints. It is evaluated in three settings, each harder than the last:

1. **Shop floor:** flat, lit, mapped, with one robot.
2. **Urban construction or rocket-assembly site:** uneven, cluttered and partly outdoors. Two robots work together: one picks and places parts, the other welds them.
3. **Lunar surface (extended goal):** terrain built from real Lunar Reconnaissance Orbiter elevation data, 1/6 g, vacuum, hard lighting and no GPS.

The project carries over MotoBat's software stack, working principle and standard of evidence. It is the mobile follow-up to MotoBat, not a replacement for it.

Rhishikesh Kandge — M.S. Electrical and Computer Engineering, The Ohio State University

*Plan written 11 Nov 2026. Nothing is built yet. Section references like MB §3.6 point into MotoBat's `docs/project_plan.md`.*

---

## Who is building this, and why

**Author:** Rhishikesh Kandge — M.S. ECE, The Ohio State University.

This is a self-directed project, like MotoBat. Its purpose is a defensible result, a claim someone could disagree with, backed by numbers (MB §3.6). An impressive-looking simulation is not the goal.

### Relevant background (carried over)

- **MotoBat:** a simulated battery-module line with a Yaskawa HC10DT inspector, a ray-cast sensor-model library (`defectgen`), MoveIt2 configs and a Gazebo control path. It has frozen sensor models (params hash `779e2255dc4c`) and a closed-loop correction with a derived stability bound (MB §9.9).
- **swarm_project:** vision-guided pick-and-place on a myCobot 280 Pi, plus the Elephant Robotics myAGV. This involved repairing a vendor driver stack that did not work out of the box. That experience is the reason this plan favours bases with drivers you can read and fix.
- **Sub-1 mm repeatability** on a compliant, backlash-heavy myCobot Pro 450.

### Gaps this project closes that MotoBat does not

| Gap | Where it is closed |
|---|---|
| Mobile robotics: Nav2, localization, a non-holonomic or 4WS chassis controller | Stage 1 |
| Base pose uncertainty propagating into tool accuracy | Core research question (§3) |
| Multi-robot coordination across separate bases | Stage 2 |
| Perception on unstructured terrain and under hard lighting | Stages 2–3 |
| Writing a robot model and a ros2_control controller from scratch | Ranger 4WS model (§2.1) |

### Working principle (unchanged from MotoBat)

> **Every subsystem must be documented with the mathematics that governs it, not just the code that implements it.**

Every module gets its governing equations with symbols defined, a block or data-flow diagram, units and frames stated explicitly, and a citation. Write the maths before the code, so the code can be checked against it. §9 collects the maths.

---

## What this document is

This is the founding plan for a new repository. It records the decisions made so far, why they were made, what is still open, and the order in which to do the work. It follows MotoBat's house style:

- **S** = sourced, **D** = derived, **A** = assumed, **M** = to be measured, **TBD** = not yet known. **No figure is invented.** If a number is TBD, it stays TBD until there is a source.
- Decisions are recorded with their reasons and the alternatives that were rejected.
- When a decision is superseded, strike it through with the date. Don't delete it.

---

## 0. Current status

| Area | State |
|---|---|
| **Gate: MotoBat finished** | **Not yet.** §4B (defect detection) and the closed loop are still open. This project does not start before that gate (§8, Phase 0) |
| Base choice | **Decided:** AgileX Ranger (full size, not the Mini) — §2.1 |
| Arm choice | **Decided:** Yaskawa HC10DTP, the welder and, in Stage 2, the picker as well — §2.2 |
| Weld process per stage | **Decided:** GMAW for Stages 1–2, laser beam welding for Stage 3 — §2.3 |
| Software stack | **Same as MotoBat** (ROS 2 Jazzy, MoveIt2, ros2_control, Open3D, Gazebo Harmonic first, Isaac Sim in the cloud) — §2.4 |
| Ranger simulation model | **Does not exist** for the full-size Ranger in Gazebo Harmonic or Isaac Sim. Build it (Phase 2) |
| Isaac Sim on the laptop | **Not viable** under current requirements. **This corrects MB §4.0b** — see §1.2 |
| Hardware purchases | **None planned.** Simulation only. §6 is a paper design for the onboard computer |

---

## 1. Constraints (read MotoBat's docs before changing these)

### 1.1 Hardware: the same laptop as MotoBat

Dell G15 5530 · i9-13900HX (24C/32T) · RTX 4060 Laptop (**8 GB VRAM**) · **16 GB DDR5, dual-channel** (since the 31 Aug 2026 motherboard replacement, MB §1.0) · 35 GB swap · Ubuntu 24.04 · ROS 2 Jazzy · Python 3.11 venv.

What runs comfortably on it, as MotoBat has already shown: ROS 2, MoveIt2, RViz2, Open3D, numpy/trimesh, `defectgen`, and Gazebo Harmonic with one arm.

What does not run on it: Isaac Sim at the scale this project needs (§1.2).

**Budget: none for hardware.** The memory upgrade (2 × 16 GB DDR5 SODIMM kit, about $400, MB §1.0) is still optional. As §1.2 explains, it would no longer be enough on its own to make Isaac Sim viable.

### 1.2 CORRECTION to MB §4.0b: Isaac Sim's minimum is now 16 GB of VRAM

MB §4.0b says the laptop's 8 GB of VRAM "meets minimum" and that system RAM is the only bottleneck. **That was true of older Isaac Sim releases. It is not true of the current one.** The current requirements page lists:

| | Minimum | Good | Ideal |
|---|---|---|---|
| RAM | 32 GB | 64 GB | 64 GB |
| GPU | GeForce RTX 4080 | GeForce RTX 5080 | RTX PRO 6000 Blackwell |
| VRAM | **16 GB** | 16 GB | 48 GB |
| Storage | 50 GB SSD | 500 GB SSD | 1 TB NVMe |

Its footnote says that GPUs with less than 16 GB of VRAM may not be able to run complex scenes that render more than 16 MP per frame. Isaac Lab adds more RAM and VRAM on top of that for training. (S — Isaac Sim requirements page, "latest", checked 11 Nov 2026.)

**Consequences:**

- The laptop is below the minimum on **both** RAM (16 vs 32 GB) and VRAM (8 vs 16 GB). A RAM kit fixes only one of the two.
- A two-robot scene with lidars, cameras and a wrist profilometer is exactly the sensor-heavy case the footnote warns about.
- **Isaac Sim work happens on a rented cloud GPU, with the whole ROS 2 stack on the same instance** (MB §4.0b's architecture, which still holds: do not split DDS across the internet). An L4 has 24 GB of VRAM, so it meets the minimum.
- **Action for MotoBat:** add a dated correction line to MB §4.0b. It belongs there, not only here.

Cloud costs as recorded in MB §4.0b (re-check before renting): AWS `g6.2xlarge` (L4) $0.98/hr · Runpod L4 $0.49/hr · Vast.ai L4 $0.33/hr. **Stop the instance whenever you are not actively simulating**, and mount the shader-cache volumes (MB §4.0b practical notes).

### 1.3 Scope discipline

- **MotoBat finishes first** (Phase 0). This project's strength comes from reusing a working scan → detect → correct pipeline. Two half-built projects are worth less than one finished one.
- **Simulation only.** Real hardware appears only as datasheet numbers that pin the simulated models.
- **No weld-pool physics.** Beads are geometry generated from a beam or torch trace, as in `defectgen/weld.py` (MB status_update item 2). No simulator in this stack models the arc or the melt pool.

---

## 2. Decisions so far

### 2.1 Base: AgileX Ranger (full size), not the Clearpath Ridgeback

**Decision (11 Nov 2026):** the AgileX Ranger, the full-size 4-wheel-drive / 4-wheel-steer model (not the Ranger Mini).

| | AgileX Ranger | Clearpath Ridgeback |
|---|---|---|
| Payload | **150 kg** (S; sources differ by configuration, so verify on AgileX's datasheet) | 100 kg (S) |
| Size | 1228 × 876 × 475 mm, wheelbase 560 mm, track 890 mm (S) | 960 × 793 × 296 mm, 135 kg (S) |
| Drive | 4WD + 4WS, suspension, aluminium T-slot deck (S) | Omnidirectional mecanum, indoor (S) |
| Interface | CAN; open `ranger_ros2` driver (S) | Native ROS 2 Jazzy (S) |
| Simulation | **None for the full-size Ranger.** AgileX's `ugv_gazebo_sim` covers the Ranger Mini v1–v3 on Gazebo Classic only (S) | Official `ros-jazzy-clearpath-simulator` on Gazebo Harmonic (S) |
| Terrain | Indoor and outdoor | Level indoor floors |

**Why the Ranger:**

1. **Payload honesty.** The HC10DTP alone is 58 kg (S). With its controller, sensors and welding equipment, the Ridgeback's 100 kg is exceeded. A reviewer who checks the numbers would catch that.
2. **It makes the research question possible.** On a level floor, mobile welding reduces to docking accurately. On uneven ground the base tilts, settles, and gives on its suspension while the arm moves. That turns the project into a perception problem (§3).
3. **Building the model is a skill to show.** A URDF from published dimensions, suspension joints, and a 4WS controller in ros2_control are all things to demonstrate in an interview.

**Costs accepted:**

- The model must be written by hand (Phase 2). Mass, inertia and suspension stiffness are **TBD** and must be estimated and stated as assumptions (A).
- I'm not aware of a stock 4WS (swerve) controller for Jazzy's ros2_controllers. **Check before writing one.** If none exists, writing it is part of the project. Document its kinematics in §9.5.
- Vendor risk: AgileX is a research-rover company, like Elephant Robotics. Read `ranger_ros2` before trusting it. In simulation this matters less, because the sim replaces the driver.

**Rejected:** Ridgeback (over payload, flat floors only). Industrial AMRs such as MiR600/1350, Omron MD, OTTO 1500 and Robotnik RB-ROBOUT (closed fleet APIs or no public simulation model; also, a 500 kg base on level floors gives up the terrain question). Ranger Mini (payload far too low).

### 2.2 Arm: Yaskawa HC10DTP

**Decision:** the HC10DTP is the welder. In Stage 2 a second HC10DTP on a second Ranger is the picker.

| Item | Value |
|---|---|
| Payload / reach | 10 kg / 1379 mm (S) |
| Mass | 58 kg (S) |
| Controller | YRC1000micro, which accepts 1-phase or 3-phase supply (S). **Its mass is TBD — do not invent it** |
| Welding pedigree | Yaskawa's ArcWorld HC is a portable collaborative welding cell built around one HC10DTP, with Power and Force Limiting and hand-guided teaching, rated IP67 (S) |
| MoveIt | MotoBat already has `motobat_hc10dt_moveit_config`. HC10DT kinematics are non-spherical-wrist, so the solver is KDL rather than OPW (MB §4A.2) |

**Why the same arm for both robots in Stage 2:** this follows MotoBat's own reasoning for two GP4s. Identical arms keep the control and planning code identical. The picker's job (placing plates and brackets of a few kg) fits inside 10 kg minus the gripper.

**Open item:** MotoBat measured its flange as **DT, not DTP** (MB fixlist §0). Before reusing the URDF, check that the DTP's kinematics and flange match the HC10DT description, and record the result.

**Rejected:** the GP25 + MOTOPAC-RL2D laser package from MB §4A.7. Its 18.3 kg head exceeds the HC10DTP's 10 kg payload, and an 8 kW infrared laser on an open, uneven site cannot be enclosed and interlocked the way ISO 11553 expects. GP-series arms in general are non-collaborative, which on a site with people around means fencing that a mobile robot can't carry.

### 2.3 Weld process: arc welding on Earth, laser beam welding on the Moon

| Stage | Process | Why |
|---|---|---|
| 1, 2 | **GMAW** (MIG arc welding) | The process Yaskawa already pairs with the HC10DTP (ArcWorld HC). Seam-tracking sensors for robotic GMAW are standard products (§5) |
| 3 | **Laser beam welding** | Gas-shielded arc welding depends on shielding gas, which is a problem in vacuum. Beam processes need no gas. *Vulkan* (Soyuz-6, 1969) tested electron beam, plasma arc and gas metal arc welding in orbit, and concluded that electron beam welding was the most suitable (S). NASA Marshall now lists the lunar surface among its in-space laser-beam-welding cases (vacuum, 0.17 g), **in collaboration with Ohio State on parabolic-flight experiments** (S) |

**The Stage 3 process change is itself a finding to write up:** the perception pipeline should not care which process follows the seam. If it does care, that is a result.

**Opportunity:** NASA MSFC's in-space welding work collaborates with OSU. That is worth one email from an OSU student with a working simulation (§11).

### 2.4 Software stack: MotoBat's, plus mobility

**Unchanged from MotoBat:** Ubuntu 24.04 · ROS 2 Jazzy · MoveIt2 · ros2_control · Open3D / PCL / OpenCV · Python 3.11 venv · PyTorch · Gazebo Harmonic + `ros_gz` · Isaac Sim 6.x (cloud) · Foxglove · rosbag2.

**Carry these MotoBat decisions forward:**

- **RMW: FastDDS, not Cyclone.** MB §5 already recommends switching: NVIDIA recommends FastDDS for single-machine Isaac Sim, and MotoROS2 requires it. Start this repo on FastDDS (the default). Don't set `RMW_IMPLEMENTATION` in the env script.
- **The MotoROS2 contract** (MB §6): command arms only through `control_msgs/FollowJointTrajectory`, and read state only from `/joint_states`. Isaac-specific code lives only in `*_bringup`.
- **Sensor models live once, in a ray-cast library**, not in the renderer (MB §4B.3). Gazebo and Isaac supply poses and raw rays or depth.
- **One `main` branch; separation by package** (MB `project_structure.md` §1).

**New for mobility:**

| Package / tool | Role |
|---|---|
| Nav2 | Navigation to a work pose |
| `robot_localization` | EKF fusing wheel odometry, IMU and lidar or visual odometry |
| `slam_toolbox` (2D) or a 3D lidar SLAM | Mapping. Stage 2 adds a CAD/BIM prior map (§4.2) |
| `ranger_ros2` | Read it for message types and the CAN protocol. The sim replaces it |
| ros2_control 4WS controller | Written or adapted (§2.1) |
| Isaac ROS (`isaac_ros_nvblox`, `isaac_ros_visual_slam`) | Optional, GPU-accelerated mapping and VSLAM. Both list ROS 2 Jazzy support (S). Use only where they replace something slower |
| OmniLRS | Stage 3 lunar environments on Isaac Sim (§4.3) |

### 2.5 Design principle, extended from MotoBat

MotoBat: *use the arm for coverage and the sensor for measurement* (MB §3.7.6).

Mobile-MotoBat adds: **use the base for reach and the seam scan for accuracy.** Base pose error must never enter the weld path directly. After the base stops and settles, the wrist sensor measures the seam **in the tool frame** and the weld path is built from that measurement. The base only has to get the seam into the sensor's field of view and the arm's workspace.

§9.2 shows why this isn't optional: a 1° tilt nobody measured moves the tool about 24 mm at full reach.

---

## 3. The research question and the claims to defend

### 3.1 The question

> **How large a base disturbance (tilt, settling, suspension give under arm load, localization error) can a scan-then-weld mobile welder absorb before the weld path leaves tolerance, and how much does re-scanning the seam after the base settles buy compared with trusting the base's pose estimate?**

It is asked three times, in three environments (§4). The answer is a **stability boundary**, the same kind of result as MotoBat's K-sweep (MB §9.9).

### 3.2 Claims this project can defend

1. **An error budget, decomposed.** Weld-path error is broken down into localization, base tilt and settling, suspension compliance under arm motion, arm repeatability, and seam-sensor error. Each is measured by injecting it alone. Most mobile-manipulation demos report one end-to-end number.
2. **A characterised boundary.** Sweep each injected disturbance. Report where open-loop (trust the base pose) fails and where scan-then-weld fails. Either way it's a result.
3. **A cross-robot correction loop (Stage 2).** The picker places part *k*. The welder's seam scan measures the fit-up error *e_k*. The correction is applied to the picker's placement of part *k+1*. This is MB §9.9's loop, `u_{k+1} = −K·e_k`, closed **across two robots on two moving bases**. The stability bound `0 < K < 2/g` must be re-derived here, because the plant gain now includes both bases' localization (§9.6).
4. **What breaks on the Moon (Stage 3).** Under 1/6 g, hard sun angles and no GNSS, which parts of the pipeline degrade and by how much? Dynamic tip-over is one expected change (§9.4).

### 3.3 What makes it a research artifact (as in MB §3.6)

| Deliverable | Why it matters |
|---|---|
| Disturbance generator + labelled dataset | Reproducible. Others can benchmark against it |
| Error budget table per stage, with confidence intervals | The headline numbers |
| Boundary plots (open-loop vs scan-then-weld) | A controls result, not a demo |
| Cross-robot K-sweep | Shows the loop is understood, not just tuned once |
| A positioning test modelled on NIST's RMMA | NIST publishes an artifact-based test method for mobile-manipulator positioning uncertainty, within about 2 mm measurement uncertainty (S). Reproducing it in simulation gives a benchmark reviewers recognise |
| Write-up structured as a paper | Problem, method, results, limitations, future work (MB §3.6) |

### 3.4 Precedent: say what exists

- **Shimizu Robo-Welder (2018):** robots that weld steel columns on high-rise sites. They measure the groove shape with a laser, then weld. Two robots usually work one column. They were deployed for night shifts for safety reasons (S). That is close to Stage 2. **The difference to claim:** Shimizu's welders are fixed to the column, while here the base moves and the ground is part of the error budget.
- **Yaskawa MH12 on Clearpath OTTO 1500 (2016):** a Yaskawa arm on a mobile base, for machine tending on a factory floor (S).
- **Academic:** mobile manipulators with continuous base motion report centimetre-level end-effector error. One example is 1.67 cm mean error tracking against odometry (S). Precise-parking approaches argue that parking accuracy is the key to process automation (S). This project argues the opposite: **measure the seam instead of trusting the park.** That needs to be stated and then shown.

---

## 4. The three stages

Each stage lists its scenario, what's new compared with the stage before, the environment model, the perception tasks, what gets measured, and an exit criterion. **A stage is done when its exit criterion is met, not when its world looks good.**

### 4.1 Stage 1 — Shop floor

**Scenario:** one Ranger + HC10DTP welder. Steel fillet and butt joints on fixtures and tables around a mapped shop. The robot drives to a work pose, stops, scans the seam, and welds.

**Environment:** flat concrete floor with mild, known unevenness, injected as small tilt and settling disturbances. Good lighting. A static map.

**Perception tasks:**

1. Localize to about 5 cm at the work pose (2D lidar SLAM / AMCL, plus wheel odometry and IMU through an EKF).
2. Find the part with the wrist snapshot 3D sensor (carried over from MotoBat: Zivid-class stop-and-shoot).
3. Measure the seam with the wrist laser line sensor. This is a short sweep and works exactly like MotoBat's profilometer model (MB §4B.3).
4. Build the weld path in the tool frame. Check fit-up (gap, mismatch) before welding. If it is out of tolerance, report it and do not weld.

**Measured:** weld-path error against ground truth, per injected disturbance. Open-loop vs scan-then-weld.

**Simulator:** Gazebo Harmonic on the laptop first. Then port to Isaac Sim in the cloud only once the result exists, the same way MotoBat sequences it.

**Exit criterion:** the Stage 1 error budget table and boundary plot exist, with sensor-model parameters frozen and hashed before the results were looked at (MB §4B.3 rule).

### 4.2 Stage 2 — Construction or rocket-assembly site, two robots

**Scenario:** a large steel structure, such as a frame, a support tower or a gantry, in an assembly hall or on an open site. **Robot P (picker)** carries plates and brackets from a staging area and holds or tacks them in position. **Robot W (welder)** scans the fit-up and welds.

> **Scope honesty:** "rocket assembly" here means steel structures and brackets around the vehicle. Propellant-tank barrels are typically friction-stir welded, a different process this project does not model. *(General knowledge, not yet sourced here. Cite before publishing.)*

**What's new:**

- **Uneven, cluttered ground.** Gravel, ramps, plates and cables. The base's tilt and settling become large and variable.
- **Two robots, one task.** Coordination, a shared map, and handoff timing.
- **A CAD/BIM prior.** Shimizu's robots compare live laser data with BIM (S). Here the structure's CAD is the reference mesh, which is MotoBat's design rule 2 (MB §4B, "narrow `Part` seam": the generator returns `(mesh, labels)`, and detection never learns which domain it is in). **The new domain is a structural joint generator.**
- **Partly outdoor.** Sunlight and shadows on shiny steel. Dust as a dropout model.

**Perception tasks (added to Stage 1):**

1. 3D lidar localization against the CAD/BIM prior.
2. Fit-up inspection before welding: gap, misalignment, missing tack. These are geometric defect classes, so MotoBat's detector carries over.
3. The cross-robot correction loop (§3.2 claim 3).
4. Safety-zone monitoring with people present (modelled as dynamic obstacles; no certification claimed).

**Ground truth:** in the real world this is a robotic total station tracking a prism on the tool (S: this is how mobile-manipulator studies measure end-effector accuracy). In simulation the simulator's own pose is ground truth, but state what the real measurement would be.

**Simulator:** Isaac Sim in the cloud. This is the stage that justifies it: two articulated robots, RTX lidar digital twins, outdoor lighting, and many sensors.

**Exit criterion:** the error budget under Stage 2 disturbances, plus a cross-robot K-sweep showing convergence, the critical point and divergence.

### 4.3 Stage 3 (extended goal) — Lunar surface

**Scenario:** the same two-robot task on terrain built from real lunar elevation data.

**Terrain data (all S):**

| Source | Resolution | Notes |
|---|---|---|
| LOLA-based south-pole DEMs (NASA GSFC) | 5 m/pixel | The geodetic reference. Shape-from-shading products sharpen it using LROC NAC images |
| LROC NAC DTMs | 2–5 m/pixel | Stereo-derived. Includes a USGS south-pole DEM with about 4 m average vertical accuracy |
| JAXA LUPEX candidate-site DTMs | 1 m/pixel | Six south-pole sites, with quality assessments (Hemmi & Inoue 2025) |

**Important limit (D):** even the 1 m/pixel data can't resolve wheel-scale terrain. A 1228 mm-long rover spans about one pixel. Real DEMs set the **large-scale** slopes and craters. Rocks and small craters must be **procedurally added on top**, which is what OmniLRS does. It upscales DEMs and overlays craters and rocks. Say in the write-up which scale is real and which is generated.

**Simulator:** **OmniLRS**, an open-source lunar simulator built on Isaac Sim. It uses real LRO DEMs (usually 5 m/pixel), offers procedural environments and multi-robot support, has ROS 2 integration and Docker installation, and can deform terrain and leave wheel traces in some environments (S). **Version risk:** its README lists a migration to Isaac Sim **5.0**. Check whether it runs on 6.x before planning around it. Pinning Isaac Sim 5.x for Stage 3 is acceptable if necessary.

**What's new:**

- **Gravity 1.62 m/s².** Static tip-over margins are unchanged, but **dynamic** margins get about 6× worse (§9.4). Traction falls with gravity, so wheel slip increases.
- **No GNSS.** VIPER's navigation hardware was stereo navigation cameras, aft and hazard cameras, a star tracker, an IMU, and LED lights tuned for permanently shadowed regions (S). Model that sensor class.
- **Lighting.** At the south pole the sun stays low on the horizon and shadows are extreme (S). VIPER was the first planetary rover with headlights (S). Model low sun angles and add light sources to the robot.
- **Vacuum.** Switch the process to laser beam welding (§2.3). No convective cooling. The weld bead model needs a thermal note, not a new simulator.
- **Regolith.** Use OmniLRS's deformation as a qualitative effect. Inject sinkage and settling as labelled disturbances for the quantitative results, the same rule as the warning in the base-choice discussion: **do not let wheel–soil physics carry the numbers.**

**Exit criterion:** the error budget under lunar conditions, plus a list of what broke compared with Stage 2 and why.

---

## 5. Sensors

Rule carried over from MB §3.7.3: **to detect a feature of size *D*, a sensor needs noise σ ≲ *D*/3 and sample spacing ≲ *D*/2.** Every sensor below has a job it can actually do by that rule, or it's not on the list.

### 5.1 On the arm (welder)

| Sensor | Role | Sim model | Example products (S) | Stages |
|---|---|---|---|---|
| **Laser line seam sensor** | Seam finding and tracking, fit-up check, post-weld bead check | MotoBat's line-profilometer model: a single-row ray fan plus motion, with both occlusions, specular dropout, and noise from the datasheet (MB §4B.3) | Meta Vision SLS (±0.1 mm tracking, 50 mm FOV, Ethernet to the robot controller); wenglor weCat3D MLZL (robot interfaces include Yaskawa); Servo-Robot AUTO-TRAC (joint validation before welding, automatic joint search) | 1–3 |
| **Snapshot 3D camera** | Finds the part and joint coarsely before the seam scan | MotoBat's Zivid-class depth model | Zivid 2+ class (MB §3.7.3) | 1–3 |

**Payload check (D, partial):** Zivid 1.0 kg (S) + seam sensor **TBD** + GMAW torch and cable dress **TBD** + mount **TBD**, against 10 kg. Do this sum before any CAD, as MotoBat had to (MB §4A.4).

**Specular steel and aluminium** remain the biggest sim-to-real gap (MB §4B.3). Laser seam sensors exist specifically for shiny surfaces (Meta's scanning-spot design is one example, S). Model the dropout, or state it as a limitation.

### 5.2 On the picker's arm (Stage 2)

| Sensor | Role | Sim model |
|---|---|---|
| Snapshot 3D camera | Grasp pose of plates and brackets in the staging area | Depth camera |
| Wrist force/torque (built into the HC series' PFL) | Contact-based placement against the structure | Joint torque from the simulator |

### 5.3 On the base

| Sensor | Role | Sim model | Notes | Stages |
|---|---|---|---|---|
| **3D lidar** | Localization, mapping, obstacles | Isaac Sim RTX Lidar. Ouster OS0/OS1/OS2, HESAI, SICK and SLAMTEC configs ship with it (S). Gazebo `gpu_lidar` | The Livox Mid-360 has a non-repetitive scan pattern that isn't stock in Isaac Sim (S). Pick an Ouster for the sim | 1–3 |
| **IMU** | Base tilt. **The most important sensor for claim 1** | Simulator IMU plus a noise model | Tilt resolution sets the open-loop baseline (§9.2) | 1–3 |
| Wheel encoders and steering angles | Odometry | Joint states | Slip makes them unreliable in Stages 2–3 | 1–3 |
| Stereo camera | VSLAM / VIO | RGB stereo pair | `isaac_ros_visual_slam` supports Jazzy and stereo or multi-camera setups (S) | 2–3 |
| Safety laser scanner | Protective fields around the robot | 2D lidar | SICK microScan3 (MB §3.7.3). **No safety certification is claimed in simulation** | 1–2 |
| RTK GNSS | Absolute position outdoors | Ground-truth pose plus noise | Stage 2 open sites only | 2 |
| Total-station prism | **Ground truth** (real-world equivalent) | The simulator's pose | Used to state how the real experiment would be measured | 2 |
| Star tracker | Absolute attitude without GNSS | Attitude plus noise | VIPER carried one (S) | 3 |
| Stereo nav cams on a mast + LED lights | Navigation under low sun and in shadow | Cameras plus Isaac lights | VIPER's arrangement (S) | 3 |

### 5.4 Sensors deliberately not used

| Sensor | Why not |
|---|---|
| Lidar for seam measurement | Centimetre-class range noise against a sub-millimetre seam fails the σ ≲ D/3 rule (the same reasoning as for MotoBat's multiScan136) |
| Passive stereo for seam measurement | ≳ 3 mm error at best (MB §3.7.3) |
| GNSS on the Moon | No constellation |

---

## 6. Processors and compute

### 6.1 What runs where during the project (no purchases)

| Work | Where | Fits? |
|---|---|---|
| Disturbance generator, seam-scan models, detector, error budget (numpy / Open3D) | Laptop | ✅ (MotoBat has shown this scale) |
| Ranger URDF, 4WS controller, MoveIt2, Nav2, Gazebo Harmonic, Stage 1 | Laptop | ✅ Expect slower than real time with the profilometer step rate (MB §4B.3 note) |
| Isaac Sim, Stage 1 port and Stage 2 | Cloud L4 instance, whole stack on the instance | ❌ on the laptop (§1.2) · ✅ in the cloud |
| OmniLRS, Stage 3 | Cloud, Docker | Same, plus the version check (§4.3) |
| Isaac Lab (optional RL) | Cloud, more VRAM | Requires more than base Isaac Sim (S) |

### 6.2 Onboard computer (paper design only)

The write-up should say what would run on the robot and show the latency budget fits. **Do not buy one.**

| Option | Key figures (S) | Fit |
|---|---|---|
| **Jetson AGX Orin 64 GB** | 275 TOPS, 64 GB | Enough for lidar localization, nvblox, VSLAM and the seam pipeline. The default choice for the paper design |
| **Jetson AGX Thor (T5000)** | 2070 FP4 TFLOPS (sparse), 128 GB LPDDR5X at 273 GB/s, 4 × 25 GbE, up to 130 W. Dev kit $3,499 at launch (Aug 2025); resellers now list it higher | Only if learned perception (for example a segmentation model on the seam images) is in scope. Isaac ROS 4.0 launched for Thor (S) |

**Also on board, and part of the power budget:**

- **YRC1000micro** controller (needs AC: an inverter from the Ranger's battery, rating **TBD**).
- **Welding power source and wire feeder** (mass and power **TBD**. This will likely be the largest single item in the payload and power budget. **Source it before claiming the 150 kg budget closes.**)
- Shielding-gas cylinder (Stages 1–2).
- Ranger CAN ↔ compute bridge.

**Latency budget (to measure in sim, M):** sensor capture → seam extraction → path generation → `FollowJointTrajectory` goal. Report it the way MotoBat reports detection latency.

---

## 7. What you can do in Isaac Sim, and what you can't

### 7.1 Use it for

| Capability | How it's used here |
|---|---|
| **URDF import → articulation** | Ranger + HC10DTP as one articulation, or two linked by a fixed joint. The URDF importer was rebuilt in 6.0 (S). Expect to tune joint drives and replace visual meshes with convex hulls for collision (MB §4.3) |
| **ROS 2 Jazzy bridge** | Jazzy on Ubuntu 24.04 is the recommended setup (S) |
| **ros2_control inside Isaac** | Isaac Sim 6.1 adds `isaacsim.ros2.control`, an in-process controller manager that supports Jazzy, multiple robots and sensors (S). That could replace the topic-based bridge in MB §4.2. Evaluate it, and keep the MotoROS2 contract either way |
| **RTX lidar digital twins** | Ouster, HESAI, SICK and more ship as configured assets (S). The lidar API moved to `isaacsim.sensors.experimental.rtx` in 6.0, so most online tutorials show the old one (S) |
| **Cameras, depth, IMU** | Snapshot 3D, stereo, IMU |
| **Multi-robot scenes** | Two Rangers with namespaced ROS 2 graphs (Stage 2) |
| **Terrain from data** | DEM → heightfield mesh → USD, with physics materials. Gravity is a scene parameter (1.62 m/s² for Stage 3) |
| **Lighting** | Low sun angles, hard shadows, robot-mounted lights |
| **Synthetic data (Replicator)** | Labelled seam images and point clouds, if a learned seam detector is added. OmniLRS showed a rock detector trained on its synthetic data came within about 5% of one trained on real data (S) |
| **Isaac Lab (stretch)** | An RL policy for disturbance-aware base placement, compared against the analytic method. Needs more VRAM |
| **Parallel instances** | 6.0 supports several Isaac Sim instances on one machine (S). Useful for sweeps on a cloud box |
| **Livestreaming** | WebRTC client to the laptop (S) |

### 7.2 Don't use it for

| Not this | Use instead |
|---|---|
| Arc or weld-pool physics | Bead geometry from the torch trace (`defectgen/weld.py` approach) |
| Seam-sensor fidelity (specular metal, multipath) | The ray-cast sensor models, written once (MB §4B.3 and its stated limitations) |
| Quantitative wheel–soil results | Labelled, injected disturbances. PhysX contact on rough terrain is known to misbehave: robots bumping on flat planes, getting stuck, friction not updating (S, NVIDIA forums) |
| A lunar regolith model | OmniLRS's deformation for visuals only, as qualitative behaviour |

---

## 8. Work plan, sequenced by dependency (no dates, as MotoBat ended up)

### Phase 0 — Gate: finish MotoBat

- MotoBat §4B results (precision/recall, minimum detectable defect), the closed-loop K-sweep, and the README written as a paper.
- Add the §1.2 VRAM correction to MB §4.0b.
- **Do not start Phase 1 until this is done.**

### Phase 1 — Disturbances and seams, in numpy (no simulator)

The contribution comes first, exactly as MB §4B did.

1. **Structural joint generator:** fillet, butt and lap joints as watertight meshes. Parameters: plate thickness, gap, misalignment, bevel, tack positions. Each defect is labelled and **off by default**. Same `(mesh, labels)` interface as `defectgen`.
2. **Base disturbance generator:** tilt (roll/pitch), settling (Δz over time), suspension compliance under arm load (a spring-damper per corner, parameters A), and localization error (x, y, yaw). Each is labelled and sweepable.
3. **Seam sensor model:** reuse MotoBat's line-profilometer model. Re-pin it to a seam-tracker datasheet. **Freeze and hash the parameters before looking at any result.**
4. **Seam extraction + weld path in the tool frame** (§9.3).
5. **Error budget harness:** inject one disturbance at a time, then combinations. Open-loop vs scan-then-weld.

**Exit:** a first boundary plot with no simulator involved. If Phases 2–5 slip, this alone is a result.

### Phase 2 — Ranger model and Stage 1 in Gazebo (laptop)

1. Ranger URDF/xacro from published dimensions. Mass and inertia A. Suspension as prismatic joints with spring-dampers.
2. 4WS ros2_control controller (or adapt an existing one). Kinematics written first (§9.5).
3. Mount the HC10DTP on the deck. **Run a tip-over check at full reach before any motion** (§9.4).
4. Nav2 + EKF + 2D/3D SLAM on a shop map.
5. Move the Phase 1 models into the TF chain. Run the Stage 1 scenario end to end.

**Exit:** Stage 1 criterion (§4.1).

### Phase 3 — Isaac Sim (cloud): port Stage 1, build Stage 2

1. Rent for a few hours first, and prove the container launches and streams (MB §4.0b recommended plan).
2. Import the robots. Decide between the topic bridge and `isaacsim.ros2.control` (§7.1).
3. Reproduce Stage 1 and compare with Gazebo. The two should agree on geometry. Disagreement is a bug, not a result.
4. Stage 2 world: a structure from CAD, uneven ground, clutter, outdoor lighting.
5. Second robot (the picker) and the cross-robot loop.

**Exit:** Stage 2 criterion (§4.2).

### Phase 4 — Stage 3 lunar (extended goal)

1. Check OmniLRS against Isaac Sim 6.x (§4.3). If it doesn't work, pin 5.x in a separate container.
2. Import a south-pole DTM tile. Overlay procedural rocks and craters, and state which scale is real.
3. Lunar gravity, lighting and sensor suite. Switch the process to laser beam welding.
4. Re-run the error budget.

**Exit:** Stage 3 criterion (§4.3).

### Phase 5 — Write-up

Paper structure as in MB §3.6. Preprint first. The Limitations section carries every "not modelled" item from §7.2 and §5.1.

---

## 9. The mathematics (stubs; write in full before the matching code)

### 9.1 Frames

`world → base_footprint → base_link (after suspension) → arm_base → … → tool0 → seam_sensor`

Each link is a homogeneous transform **T** ∈ SE(3). The base's pose is **estimated** (EKF output), not known. Keep two trees in mind: the estimated one the robot uses, and the true one the simulator knows. **The error budget is the difference between them, measured at the torch tip.**

### 9.2 Why the base pose can't be trusted for the weld (D)

A small tilt δθ at the arm base moves a tool at horizontal reach *L* by about *L*·tan(δθ):

| Tilt δθ | Tool shift at *L* = 1.379 m (HC10DTP max reach) | at *L* = 1.2 m |
|---|---|---|
| 0.1° | 2.4 mm | 2.1 mm |
| 0.5° | 12.0 mm | 10.5 mm |
| 1.0° | 24.1 mm | 20.9 mm |
| 2.0° | 48.2 mm | 41.9 mm |

To keep the tilt term alone under 0.1 mm at full reach, the tilt would have to be known to about **15 arcseconds (0.004°)**. That is far beyond what a base IMU and EKF will give you on uneven ground. **This is the quantitative case for scan-then-weld**, and it should be the first figure in the write-up.

### 9.3 Seam extraction and the weld path

Profiles from the line sensor → stitched point cloud in the **tool** frame → fit the joint model (for example two planes for a fillet, whose intersection line is the seam) → weld path as a sequence of poses along that line, with the torch at work and travel angles. Cite the fitting method (RANSAC plane fit + least squares), as MB §9.7 does for ICP.

### 9.4 Tip-over stability, and why the Moon is harder (D)

Static: the robot stays upright while the combined centre of mass projects inside the support polygon (the Ranger's wheelbase 560 mm × track 890 mm, S). Gravity multiplies **both** the tipping moment and the restoring moment, **so static margins don't depend on g.**

Dynamic: arm acceleration adds inertial forces *m·a* that **don't scale with g**, while the restoring moment does (*m_base·g·d*). At 1.62 m/s² instead of 9.81 m/s², the same arm motion uses up about **6× more** of the tip-over margin. Traction (∝ *g*) drops by the same factor. **Consequence:** Stage 3 needs slower arm trajectories, or outriggers, or both. This is a design result, not just a parameter change.

Masses are TBD (Ranger mass, payload stack). Fill them in before running this.

### 9.5 4WS kinematics

For each wheel *i* at position **r**_i in the base frame, with commanded base twist (*v_x*, *v_y*, ω): wheel velocity **v**_i = (*v_x* − ω·*r_iy*, *v_y* + ω·*r_ix*). The steering angle is atan2 of that vector and the wheel speed is its norm divided by the wheel radius. Handle the ±π flip (reverse the wheel rather than rotating it 180°). Write this before the controller.

### 9.6 The cross-robot correction loop

`u_{k+1} = −K·e_k`, with *e_k* the fit-up error the welder measures on part *k* and *u* the offset applied to the picker's next placement. Recursion: `e_{k+1} = (1 − g·K)·e_k + w_k`, where *g* is the plant gain (commanded offset → achieved offset) and *w_k* is new disturbance from the picker's own localization each time it re-parks. Stability still needs `0 < K < 2/g`. What's new is that *w_k* is no longer small, so the **steady-state error variance** depends on K. Derive it, then show it in the sweep.

### 9.7 Localization

EKF state, process model (4WS kinematics + IMU), measurement models (lidar scan match, VSLAM, GNSS in Stage 2, star tracker in Stage 3). Write the covariance propagation, because §9.2 depends on the tilt covariance.

---

## 10. Repository structure (MotoBat's pattern)

**The repository root is the colcon workspace. One long-lived branch: `main`.** Separation is by package. The stage is a launch argument, not a branch.

```
mobile-motobat/                       ← git repo AND colcon workspace
├── README.md                         paper-structured front door
├── LICENSE                           Apache-2.0 (as MotoBat)
├── mm_env.sh                         sourced from ~/.bashrc; FastDDS (default)
├── docs/
│   ├── project_plan.md               this file
│   ├── status_update.md              what to do next
│   ├── fixlist.md                    what is known to be wrong
│   └── results/
├── seamgen/                          Phase 1: joint generator, disturbances, seam models (numpy/Open3D)
│                                     depends on MotoBat's defectgen sensor models; don't copy them
└── src/
    ├── mm_description/               Ranger xacro, HC10DTP mount, sensor frames
    ├── mm_ranger_control/            4WS controller (ros2_control)
    ├── mm_navigation/                Nav2, EKF, SLAM configs
    ├── mm_seam/                      seam extraction → weld path node
    ├── mm_coordination/              Stage 2 picker/welder task logic, cross-robot loop
    ├── mm_interfaces/                SeamMeasurement, FitUpReport, WeldPath, PlacementCorrection
    ├── mm_hc10dtp_moveit_config/
    └── mm_bringup/                   Gazebo and Isaac launch; the only place sim-specific code lives
```

```bash
ros2 launch mm_bringup stage.launch.py stage:=1 sim:=gazebo
ros2 launch mm_bringup stage.launch.py stage:=2 sim:=isaac robots:=picker,welder
```

**Reusing MotoBat:** add `defectgen` as a pinned dependency (a git submodule or pip install from a tagged commit). **Don't copy its sensor models.** They were frozen and hashed, and a copy would drift.

**Commits:** explain *why*, not *what* (MB §8).

---

## 11. Open items (to source before results are claimed)

1. **Ranger payload figure.** Sources say 150 kg but disagree by configuration. Get it from AgileX's datasheet.
2. **Ranger mass, inertia and suspension stiffness.** Ask AgileX or estimate (A). Needed for §9.4.
3. **YRC1000micro mass and power draw.** Datasheet.
4. **GMAW power source + wire feeder mass and power.** Pick one real unit and use its datasheet.
5. **Seam sensor head mass and FOV.** Pick one product (§5.1).
6. **Weld seam tolerance for GMAW fillets.** The positioning tolerance the torch must hold. **To source** (a welding standard or a seam-tracker application note). Do not assume.
7. **HC10DT vs HC10DTP kinematics and flange** (§2.2).
8. **A 4WS controller for Jazzy.** Does one exist?
9. **OmniLRS on Isaac Sim 6.x.** Does it run?
10. **NASA MSFC / OSU in-space laser welding contact.** Who at OSU? One email, after Stage 1 results exist.

---

## 12. Risks

| Risk | Mitigation |
|---|---|
| Starting before MotoBat is finished | Phase 0 gate |
| Ranger model takes weeks | Keep it simple: rigid chassis + 4 suspension prismatics. Fidelity lives in the injected disturbances, not the chassis model |
| Wheel–soil physics produces numbers nobody can defend | Results come only from labelled, injected disturbances. Terrain physics is for demos |
| Cloud costs creep | Phase 1 and 2 need no cloud. Stop instances when idle. Persistent shader cache |
| OmniLRS version mismatch | Separate container pinned to its Isaac Sim version |
| Scope creep across three stages | Each stage has an exit criterion. Stage 3 is explicitly an extended goal |
| Vendor driver quality (again) | The sim replaces the driver. Read `ranger_ros2` before any hardware decision |

---

## References

**Bases**
- [Generation Robots — AgileX Ranger 4WS4WD (size, wheelbase, track, 150 kg)](https://www.generationrobots.com/en/404081-mobile-ranger-4ws-4wd-robot.html)
- [AgileX Ranger overview (payload discrepancy note)](https://www.asiasatellite.co/Agilex-RANGER-RANGER.htm)
- [AgileX ranger_ros2 / ranger_ros / ugv_sdk links](https://indrorobotics.notion.site/GitHub-642cf05343844be392c252fd4e598eb9)
- [AgileX ugv_gazebo_sim model list (Ranger Mini only)](https://gitee.com/galaxy1229/ugv_gazebo_sim)
- [Clearpath Ridgeback specs (MYBOTSHOP)](https://www.mybotshop.de/Clearpath-Ridgeback_1) · [Clearpath Simulator, Jazzy install](https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install)

**Arms and welding**
- [Yaskawa Education Program flyer (HC10DTP 10 kg, 1379 mm, 58 kg)](https://www.yaskawa.eu.com/Global%20Assets/Downloads/Brochures_Catalogues/Robotics/EducationProgram/Flyer_EducationProgram_E.pdf)
- [Yaskawa GP/YRC1000micro overview (1- or 3-phase supply)](https://www.yaskawa.de/_downloads/download_d10703)
- [ArcWorld HC (HC10DTP welding cell)](https://www.automate.org/robotics/news/arcworld-hc-a-flexible-human-collaborative-workcell-for-robotic-welding/boa)
- [Yaskawa + Clearpath OTTO mobile manipulator (2016)](https://www.automate.org/robotics/news/yaskawa-and-clearpath-partner-to-develop-mobile-manipulation-solution/aph)
- [Shimizu Robo-Welder (GlobalSpec)](https://insights.globalspec.com/article/8757/insights.globalspec.com/help/privacy) · [Shimizu robots, AP / Courthouse News](https://courthousenews.com/construction-robots-weld-bolt-lift-to-beat-worker-shortage/)

**Seam sensors**
- [Meta Vision Smart Laser Pilot (±0.1 mm, 50 mm FOV)](https://www.azorobotics.com/News.aspx?newsID=643)
- [Meta A700 scanning-spot sensor for shiny surfaces](https://emeraldinsight.com/insight/content/doi/10.1108/ir.2003.04930caf.003/full/html)
- [wenglor weCat3D MLZL weld seam tracker](https://img0.hospitalityhub.com.au/automated-weld-seam-tracking-wecat3d-mlzl-2d-3d-profile-sensors-enable-exact-seam-placement-in-robot-welding-cells/f/24120)
- [Servo-Robot AUTO-TRAC/PWP](https://servo-robot.com/?p=1746)

**Isaac Sim, Isaac ROS, compute**
- [Isaac Sim requirements (latest)](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [Isaac Lab installation (RAM/VRAM)](https://isaac-sim.github.io/IsaacLab/v3.0.0-beta2/source/setup/installation/index.html)
- [Isaac Sim 6.0 release notes](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/overview/release_notes.html) · [6.1 release notes (isaacsim.ros2.control)](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/overview/release_notes.html)
- [Isaac Sim ROS 2 installation (Jazzy recommended)](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_ros.html)
- [RTX Lidar assets](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/assets/usd_assets_nonvisual_sensors.html) · [isaacsim.sensors.rtx deprecation](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/deprecated/isaacsim.sensors.rtx/docs/index.html) · [Livox Mid-360 thread](https://forums.developer.nvidia.com/t/livox-mid360/283074)
- PhysX wheeled-robot issues: [bumpy on flat plane](https://forums.developer.nvidia.com/t/weird-collisions-between-plane-and-mobile-wheeled-robot/350329) · [stuck vehicle](https://forums.developer.nvidia.com/t/vehicle-gets-unexpectedly-stuck-in-environment/262359) · [friction update](https://forums.developer.nvidia.com/t/friction-cannot-be-updated-in-real-time-resulting-in-incorre-friction-only-applies-to-the-current-startup-time/311101)
- [Isaac ROS nvblox](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox) · [Isaac ROS Visual SLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam) · [Isaac ROS 4.0 for Thor](https://forums.developer.nvidia.com/t/nvidia-isaac-ros-4-0-for-thor-has-arrived/352109)
- [Jetson AGX Thor dev kit, $3,499 (CNX)](https://www.cnx-software.com/2025/08/19/3499-nvidia-jetson-agx-thor-developer-kit-2070-tops-jetson-t5000-som-for-robotics-and-edge-ai/?amp=1) · [ServeTheHome (273 GB/s, 25 GbE)](https://www.servethehome.com/nvidia-jetson-agx-thor-developer-kit-blackwell-for-robotics/2/) · [Newegg Jetson lineup (AGX Orin 275 TOPS / 64 GB)](https://www.newegg.com/insider/?p=90886)

**Lunar**
- [OmniLRS paper (arXiv 2309.08997)](https://arxiv.org/abs/2309.08997v1) · [OmniLRS / LunarSim repository](https://github.com/AntoineRichard/LunarSim)
- [NASA PGDA — shape-from-shading enhanced south-pole topography](https://pgda.gsfc.nasa.gov/products/104)
- [USGS LRO south-pole DEM](https://astrogeology.usgs.gov/search/map/moon_lro_south_pole_dem) · [overview (≈4 m vertical accuracy)](https://astrogeology.usgs.gov/ckan/dataset/ae25c2c4-2d44-425c-afd5-4d0d8a6eccf5/resource/3e0a1e62-99d3-437e-9b6b-319d91b64989/download/lunar_south_pole_20130408_overview.pdf)
- [LROC derived data (NAC DTMs 2–5 m/px)](https://ode.rsl.wustl.edu/Mars/pagehelp/Content/Missions_Instruments/Lunar%20Reconnaissance%20Orbiter%20(LRO)/LROC/Derived/Intro.htm)
- [JAXA LUPEX 1 m/px south-pole DTMs](https://jlpeda.jaxa.jp/en/product/archive/detail_14/)
- [VIPER navigation hardware (NASA LSIC 2024)](https://ntrs.nasa.gov/api/citations/20240013903/downloads/LSIC_2024-Tech%20Infusion_Panel-VIPER_Mob-HW.pdf) · [VIPER mast and headlights (NASA)](https://www.nasa.gov/?p=634089)

**Welding in space**
- [NASA — welding in space milestones (Welding Journal)](https://ntrs.nasa.gov/api/citations/20230011078/downloads/NASA%2020230011078.pdf)
- [NASA MSFC — in-space laser beam welding, lunar surface case, OSU collaboration](https://ntrs.nasa.gov/api/citations/20250000018/downloads/AIAA-SciTech-ISW-ecosystem_2025-01-02.pdf)
- [NASA MSFC — LBW in thermal vacuum and microgravity (FabTech 2025)](https://ntrs.nasa.gov/api/citations/20250009729/downloads/AWS-FabTech_2025_2025-09-09.pdf)

**Mobile manipulator accuracy**
- [NIST TN 2108 — RMMA test method](https://nvlpubs.nist.gov/nistpubs/TechnicalNotes/NIST.TN.2108.pdf) · [NIST AMS 100-46](https://nvlpubs.nist.gov/nistpubs/ams/NIST.AMS.100-46.pdf)
- [Mobile manipulator framework, precise parking (arXiv 2302.04486)](https://arxiv.org/pdf/2302.04486)
- [Reconfigurable base, whole-body EE tracking 1.67 cm (arXiv 2206.15298)](https://arxiv.org/pdf/2206.15298)
