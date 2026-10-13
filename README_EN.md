# RoboWorld

> 🌐 **Language**: **English** (current) · [简体中文](./README.md)

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Python](https://img.shields.io/badge/Python-3.9%2B-blue.svg)](https://python.org)
[![Tests](https://img.shields.io/badge/tests-1846%20passed-brightgreen.svg)](#testing)
[![Platform](https://img.shields.io/badge/platform-Linux%20%7C%20macOS%20%7C%20Windows-lightgrey.svg)](#installation-and-quick-start)

**Open-source robot programming and simulation platform**

A desktop programming and simulation tool for industrial robots. RoboWorld brings robot modeling, 3D scene editing, trajectory planning, collision checking, program playback and controller code generation into a single workspace — quick enough to try the offline-programming workflow in minutes, structured enough for an engineering team to keep building on.

Open it and you get a 3D viewport, a robot/part library, a device tree, a property editor, path/code preview and an export panel: load a URDF, drag the joints to check poses, edit taught points, validate the trajectory, and export the program for your target controller.

[⬇️ Download & Install](#installation-and-quick-start) · [🖼 UI and Features](#ui-and-features) · [📖 User manual (Chinese)](docs/manual/INDEX.md) · [GitHub repository](https://github.com/cModules/RoboWorld) · [Open an Issue](https://github.com/cModules/RoboWorld/issues)

**Keywords**: industrial robotics · offline programming · robot simulation · robotics digital twin · trajectory planning · motion planning · inverse kinematics · collision checking · multi-robot coordination · force control · CAD/CAM · PLC · URDF · Python · PySide6

---

## UI and Features

The RoboWorld window is laid out around one complete robot-programming workflow:

- **3D viewport** — inspect robots, grippers, workpieces and the cell layout; drag joint sliders to check pose and reachability.
- **Scene library & device tree** — search and load URDF models; manage the assembly relationships between robots, tools, conveyors, bins and pallets.
- **Property editor** — edit object poses, joint angles, model parameters and mount relations; a gripper can be snapped straight onto the robot flange.
- **Code preview & path teaching** — edit waypoints and motion types, record / play / stop a trajectory, and see the path drawn in the viewport as you go.
- **Export & packaging** — run reachability, joint-limit, collision and cycle-time checks, then generate programs for UR, KUKA, ABB, FANUC and more.
- **Workstation demo** — load the pick-and-place cell with one click and exercise IK, gripper IO, workpiece following, trajectory playback and multi-brand code export.

## Feature Walkthrough

The GUI is just `main.py`: load any URDF, pick devices in the left-hand **library**, and edit properties, scene and code in the right-hand tabs.

### Main view — 3D viewport + library + joint control
![Main view](docs/screenshots/main_ur5e.png)

- The left **library** lists every model under `assets/models/`, with search and filters by brand / payload / reach; click an entry to load it.
- The middle is the OpenGL viewport: once UR5e is loaded you can drag joints and inspect trajectory lines and the selected object.
- The right-hand **Properties** tab shows specs for the selected object plus six **joint sliders** (limits come from the model); dragging moves the robot live. Selecting a gripper or workpiece also lets you edit its **pose (XYZ + RPY)** and **model parameters** directly.

### Path code editor (code preview) — edit taught points / generate robot code
![Path code](docs/screenshots/path_code.png)

- **Code preview** tab: text editor plus Record / Play / Stop / Clear.
- Live parsing: every `movej` / `movel` it recognizes is drawn as connected waypoints in the 3D viewport.
- Inside that tab sits the **Path teaching** sub-page, which manages waypoint semantics (approach / pick / place) and workpiece bindings.

### Offline programming (export) — program validation + cycle-time estimate + playback
![Offline programming](docs/screenshots/offline_programming.png)

- **Export** tab: program summary (cycle time, TCP path, waypoint count), **Validate (F7)** for reachability / limits / collisions, **Play** to run continuously, **Stop** to interrupt.
- Backed by real kinematics rather than a mock solver.

### Vendor CAD — JAKA Zu5 production meshes
![JAKA Zu5 production robot](docs/screenshots/jaka_zu5_real.png)

- Loads JAKA Zu5 from `assets/models/jaka/zu5/zu5_real.urdf`; the meshes come from the **official STEP** files (toolchain in `tools/build_vendor_urdf.py`, details in [docs/manual/VENDOR_CAD_SETUP.md](docs/manual/VENDOR_CAD_SETUP.md), Chinese).
- Joint limits follow the **real Zu5 specification** (`joint_2 -85..265°`, `joint_3 ±175°`, `joint_4 -85..265°`) instead of a generic ±360°.
- This is the reference integration; KUKA / ABB / FANUC / Estun go through the same pipeline — drop a STEP into `assets/vendor/<brand>/step/` and run the tool.

### Pick-and-place workstation — one-click demo (S1–S5)
![Pick-and-place workstation](docs/screenshots/grasping_workstation.png)

- **File → Pick-and-place demo (one click)**: builds a standard cell (UR5e + vacuum gripper with TCP + conveyor with 4 parts + bin + pallet) and loads an 8-point, five-pick demo program — validation, cycle time (status bar) and playback start automatically.
- The same pipeline headlessly: `python examples/grasping_workstation_demo.py` → template scene → IK demo points → gripper IO → cycle time → UR / KUKA / ABB / FANUC code, all through the real API.

![Playback](docs/screenshots/grasping_playback.png)

- During playback, the taught PICK/PLACE semantics drive the gripper: parts attach to the flange, follow the TCP, and are left on the pallet where they are released.
- The same dataset rendered in a browser lives in **deploy/web** — built by `tools/build_web_site.py`.

![Offline programming](docs/screenshots/grasping_offline_programming.png)

- The **Export** panel validates the demo program against the real scene (5.4 s cycle time, 638 TCP waypoints, 8 motions + 5 IO instructions) and exports every brand's code from that single program definition.

### Vision-triggered picking — calibration → vision → pick in one command (v0.3.4)

```bash
python examples/vision_pick_demo.py                    # default noise
python examples/vision_pick_demo.py --noise 1.0 --code # stronger calibration noise + print the code
```

- **TCP calibration**: 4-point method solves the suction-cup offset; 6-point method touches along tool +X/+Z to recover the orientation.
- **3-point pallet frame**: drop positions are computed in the pallet's local frame — rotate the pallet 15° and everything follows, so no more magic constants in the program.
- **Hand-eye calibration** (eye-to-hand, real UR5e configuration + multi-orientation wrist poses): with 0.15 mm / 0.03° acquisition noise the extrinsic error stays around sub-millimetre level.
- **Vision localization**: detected part pixels are back-projected through the extrinsic calibration onto the support plane → pick pose; the closed-loop positioning error stays around sub-millimetre level. Then IK → program → validation (0 errors) → brand-specific code, with the calibration summary written into the code header automatically.
- The calibration set is persisted to `examples/assets/grasping_workstation_calibration.json` and can be loaded and reused by both the GUI and scripts.

---

## Supported Robots and Controllers

| Brand | Controller | Post-processor | Production CAD |
|---|---|---|---|
| Universal Robots | PolyScope 5 / 6 | `ur_polyscope` | ✅ UR3 / UR5 / UR10 / UR3e / UR5e / UR10e / UR16e |
| JAKA | Zu App | `jaka_zu` | ✅ **Zu3 / Zu5 / Zu7 / Zu12 / Zu18 / Zu20 / Zu30** (vendor STEP → meshes) |
| KUKA | KRC4 | `kuka_krc4` | 🔄 pipeline ready, awaiting STEP |
| ABB | IRC5 | `abb_irc5` | 🔄 pipeline ready, awaiting STEP |
| FANUC | R-30iB | `fanuc_r30ib` | 🔄 pipeline ready, awaiting STEP |
| Estun | ER | `estun_er` | 🔄 pipeline ready, awaiting STEP |
| Inovance (汇川) | InoTeach | `huichuan_inoteach` | 🔄 post-processor only |

> **Adding a brand takes < 30 minutes**: drop the STEP into `assets/vendor/<brand>/step/`, run `tools/build_vendor_urdf.py --brand <brand>`, and write one Jinja2 template. See [docs/manual/post_dev_guide.md](docs/manual/post_dev_guide.md) (Chinese).

---

## Installation and Quick Start

### 1. Get the code

- **Run from source**: download a ZIP from the [GitHub repository](https://github.com/cModules/RoboWorld), or run:

```bash
git clone https://github.com/cModules/RoboWorld.git
cd RoboWorld
```

- **Desktop installer**: download the installer matching your system from [Releases](https://github.com/cModules/RoboWorld/releases); if no installer is published yet, please run from source.

### 2. Prepare Python (conda recommended)
```bash
conda create -y -n robotworld python=3.11
conda activate robotworld
```
Any virtualenv works too — Python **3.9 to 3.12** are supported.

### 3. Install dependencies (editable install)
```bash
pip install -e ".[gui]"
```
This installs the core dependencies (numpy / scipy / jinja2 / pyyaml / pydantic) **and** the GUI extras (PySide6 / PyOpenGL), and installs RoboWorld's layered packages (`domain` / `service` / `adapter` / `app` / `support`) in editable mode. No C++ compilation needed — the pure-Python kinematics fallback ships with the package and is guarded by 1846 tests.

Behind a slow PyPI mirror:
```bash
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
```

### 4. Start the GUI
```bash
python main.py assets/models/ur/ur5e/ur5e.urdf
```
- **macOS / Windows** with a desktop: a window opens right away.
- **Headless Linux** (CI / sandbox): start a virtual display with `Xvfb :99 -screen 0 1600x1000x24`, then `DISPLAY=:99 python main.py …`.

### 4.1 Run in the cloud / a container (Cloud Studio · noVNC)

The repository ships `.ide/Dockerfile` to build a "cloud dev container": Python 3.11 + RoboWorld runtime/test/GUI dependencies on top of `node:20`, with `Xvfb + x11vnc + websockify + noVNC` so that `main.py` runs on a virtual desktop reachable from a browser.

- To avoid OOM when installing GUI dependencies on **memory-constrained builders** such as Cloud Studio, GUI deps use `PySide6-Essentials` (QtCore/QtGui/QtWidgets/QtOpenGLWidgets only — no 3D/Charts add-ons), together with `PIP_NO_BUILD_ISOLATION` / `PIP_PREFER_BINARY`, a domestic mirror and retry timeouts, lowering peak memory and dependency-resolution failures.
- Build and run:

```bash
docker build -f .ide/Dockerfile -t robotworld .
docker run -p 6080:6080 robotworld      # open http://localhost:6080 to see the main.py desktop
```

> The container boots the noVNC stack (`Xvfb → main.py → x11vnc → websockify/noVNC`) via `.ide/start.sh`; the IDE itself is started separately by the Cloud Studio platform, so the two do not conflict.

### 5. Your first program (no GUI, script only)
```python
# Same source as examples/olp_full_workflow.py: the whole pipeline, no GUI
import numpy as np
from domain.robot import Robot
from domain.link import FileLink, SimLink
from service.program import ProgramBuilder
from service.debug import DebugSession

# 1) Load the robot (URDF)
robot = Robot("assets/models/ur/ur5e/ur5e.urdf", name="UR5")

# 2) Write the program. The target pose comes from the **forward kinematics** of a
#    real joint configuration, so it stays reachable for any brand — no per-brand
#    hand-tuned Cartesian coordinates.
pose_b = robot.forward_kinematics(np.array([30.0, -30.0, 30.0, -30.0, 30.0, 0.0]))
prog = (
    ProgramBuilder(robot)
    .moveJ([0.0] * robot.dof, speed=40)     # joint interpolation
    .moveL(pose_b, speed=60, blend=10)      # linear interpolation (blend = radius, mm)
    .build()
)

# 3) Validate: reachability / joint limits / singularities / collisions + cycle time
report = prog.validate()
print(report.ok, report.num_motions, f"{report.cycle_time:.2f}s")

# 4) Generate code and deploy. FileLink exports offline (swap in ControllerLink for real hardware)
print(prog.to_post("ur_polyscope"))        # URScript
link = FileLink("out/")
link.connect()
print(prog.deploy(link, "ur_polyscope", name="pick_and_place").handle)

# 5) Online debugging: play on a simulated link and compare feedback against the plan
sim = SimLink(robot)
sim.connect()
summary = DebugSession(prog.simulate(), sim, robot=robot).run()
print(summary["max_pos_deviation_mm"], "mm /", summary["max_rot_deviation_deg"], "deg")
```

---

## Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│  GUI layer — main.py (PySide6 + OpenGL)                            │
│  ────────────────────────────────────────────────────────────────  │
│  Left tabs: (1) Scene library  (2) Device tree                     │
│  Right tabs: Properties / Code preview / Export (Teach page)       │
│  GLWidget viewport · Joint sliders · Pose editor · Path editor     │
├────────────────────────────────────────────────────────────────────┤
│  Python API — layered packages (domain / service / adapter)        │
│  ────────────────────────────────────────────────────────────────  │
│  domain: Robot · Pose · Link · kinematics / collision              │
│  service: ProgramBuilder · Program · TrajectoryPlanner             │
│  service: DebugSession · SimulationWorld · scene & project         │
│  adapter: post (Jinja2) · plugins · robot_library                  │
├────────────────────────────────────────────────────────────────────┤
│  Kinematics core                                                   │
│  ────────────────────────────────────────────────────────────────  │
│  Pure Python FK/IK/Jacobian (default; used by CI and tests)        │
│  Optional C++ Pinocchio/FCL core (built via CMakeLists.txt)        │
├────────────────────────────────────────────────────────────────────┤
│  Post-processor ecosystem — Jinja2 templates                       │
│  ────────────────────────────────────────────────────────────────  │
│  ur_polyscope · kuka_krc4 · abb_irc5 · fanuc_r30ib                 │
│  estun_er · jaka_zu · huichuan_inoteach                            │
│  + plugin brands (adapter.plugins)                                 │
└────────────────────────────────────────────────────────────────────┘
```

Key extension points:
- **The `Link` interface** — swap `FileLink` (offline) for `SimLink` (simulation) or a real `ControllerLink` (RTDE / PROFINET / vendor SDK) without touching application code.
- **`_convert_motion(motion)` in each brand template** — override `_convert_spline` to keep splines when the controller supports them natively; the default degrades to PTP/LIN.

---

## Testing

```bash
pytest tests/    # 1846 tests covering kinematics / post-processing / link (including real Socket and SSH paths with a stubbed controller) / PLC↔robot IO maps / dwell-time timeline / circular-arc editing in the trajectory editor / scene drag-and-drop placement / scale and performance benchmarks / interference-check sampling conventions and geometry caching / incremental recomputation (content-addressed cache, bit-exactness, start-point-in-key regression guard) / debug / vendor CAD / calibration and hand-eye / multi-robot / force control / digital twin / plugin interface / cloud channel / program diffing / envelope tolerance / i18n / real-machine connection panel / real-machine CAD pipeline / interference-result caching / torque-result caching / cache observability / UI architecture acceptance / process sequence (sequence model, seven validations, robot+PLC outputs, JSON round-trip, swimlane editor offscreen behavior) / dynamic-timing UX / part library and preset scenes / project files `.rwproj` (compatible with `.robotproj` / `.core`; six-section envelope, per-section fault tolerance, unknown-section fidelity, atomic writes, GUI open/save with dirty flag) / session support (recent files, auto-recovery, recovery prompt, snapshot cleanup on clean exit) / project templates / packaging configuration
```
CI workflow lives in `.github/workflows/ci.yml`.

---

## Contributing

Robot engineers, researchers and integrators are all welcome.

- **Add a brand** — the easiest entry point: write one Jinja2 template and drop a STEP into `assets/vendor/<brand>/step/`. See [docs/manual/post_dev_guide.md](docs/manual/post_dev_guide.md) (Chinese).
- **Improve core algorithms** — IK solvers, path planning, calibration routines.
- **Hardware validation reports** — results from running the generated URScript / KRL / RAPID on a real controller.

See [CONTRIBUTING.md](CONTRIBUTING.md) for details.

---

## License

**This project is licensed under the Apache License 2.0.**

Code and documentation in this repository are released under Apache-2.0; third-party robot CAD models under `assets/models/` may carry their own licenses — defer to the `README.md` in each subdirectory. The full license text is in [`LICENSE`](LICENSE).

---

## Acknowledgements

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) — INRIA / LAAS-CNRS
- [FCL](https://github.com/flexible-collision-library/fcl) — Flexible Collision Library
- [OMPL](https://ompl.kavrakilab.org/) — Open Motion Planning Library
- [RoboDK](https://robodk.com/) — source of inspiration and an industry benchmark
- Everyone who validated generated code on real controllers

---

**Built with ❤️ by the open-source robotics community.**
