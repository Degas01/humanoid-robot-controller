# MyController — mc_rtc Humanoid Robot Controller

---

## Overview

This controller demonstrates the core building blocks of whole-body humanoid control using mc_rtc:

- **Posture regulation** — keeps the robot near its default half-sitting configuration
- **Center of Mass (CoM) control** — maintains balance by regulating the CoM position
- **End-effector control** — moves the right wrist along a sinusoidal vertical trajectory
- **Contact management** — declares and enforces foot contacts with the ground

The target robot is the **JVRC1** humanoid, using mc_rtc's built-in JVRC1 robot module.

---

## Controller Architecture

```
MyController
├── Constraints
│   ├── ContactConstraint       — foot contacts stay valid
│   └── KinematicsConstraint    — joint limits respected
│
└── Tasks (solved jointly by QP solver)
    ├── PostureTask             — joint-space regulation (weight: 10, stiffness: 1000)
    ├── CoMTask                 — Cartesian CoM regulation (weight: 10, stiffness: 1000)
    └── EndEffectorTask (R_WRIST_Y_S)
                                — sinusoidal z-axis hand motion (weight: 5, stiffness: 1000)
```

At each control tick (`run()`), the QP solver computes joint torques/velocities that satisfy all constraints and minimise the weighted sum of task errors. The right-hand target is updated every tick to produce a smooth sinusoidal oscillation (±10 cm at 1 rad/s).

---

## Dependencies

| Package | Version tested |
|---------|---------------|
| Ubuntu (WSL2) | 20.04 LTS |
| mc_rtc (libmc-rtc-dev) | 2.12.0 |
| mc-rtc-data | 1.0.7 |
| mc-rtc-utils | 2.12.0 |
| CMake | ≥ 3.1 |
| GCC | 9.4 |

### Install mc_rtc on Ubuntu 20.04

```bash
curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' | sudo -E bash
sudo apt install libmc-rtc libmc-rtc-dev mc-rtc-data mc-rtc-utils mc-rtc-gui
```

---

## Building the Controller

```bash
cd MyController
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

After installation the controller plugin is available at:

```
/usr/lib/x86_64-linux-gnu/mc_controller/MyController.so
```

---

## Running the Controller

### 1. Configure mc_rtc

Create or edit `~/.config/mc_rtc/mc_rtc.yaml`:

```yaml
MainRobot: JVRC1

Enabled:
  - MyController

RobotModulePaths:
  - /usr/lib/x86_64-linux-gnu/mc_robots

ControllerModulePaths:
  - /usr/lib/x86_64-linux-gnu/mc_controller
```

### 2. Launch the controller

```bash
mc_rtc_ticker
```

Expected output:

```
[info] Loading controller MyController
[success] MyController init done
```

### 3. Visualise (optional)

In a second terminal:

```bash
mc_rtc_gui
```

This opens the mc_rtc stand-alone GUI showing live task targets, robot state, and solver output.

---

## Design Decisions

**Why PostureTask as a regularisation term?**
Without a posture task, the QP has an under-constrained null space — joints drift freely. The PostureTask at low weight acts as a soft regulariser that keeps joint configurations human-like without interfering with higher-priority Cartesian tasks.

**Why CoMTask for balance?**
JVRC1 is a floating-base robot. Controlling the CoM position relative to the support polygon is the simplest approach to passive balance in a quasi-static regime. A full stabiliser (e.g. LIPMStabilizer) would be needed for dynamic motion but is out of scope here.

**Why a sinusoidal end-effector trajectory in `run()`?**
Updating the target every tick (rather than setting it once in the constructor) demonstrates how a controller reacts dynamically to changing references — the pattern used when tracking sensor input, GUI interaction, or FSM transitions in production controllers.

**Task weight choices:**
- PostureTask: weight 10 — low, acts as regulariser only
- CoMTask: weight 10 — moderate, balance is important but should not block hand motion
- EndEffectorTask: weight 5 — lower than CoM so balance is preserved if the two conflict

---

## Key mc_rtc Concepts Used

| Concept | API |
|---------|-----|
| Controller base class | `mc_control::MCController` |
| QP solver access | `solver()` |
| Joint regularisation | `mc_tasks::PostureTask` |
| CoM balance | `mc_tasks::CoMTask` |
| Cartesian end-effector | `mc_tasks::EndEffectorTask` |
| Contact constraints | `contactConstraint`, `solver().setContacts()` |
| Kinematics constraints | `kinematicsConstraint` |
| Logging | `mc_rtc::log::success()` |
| Plugin registration | `CONTROLLER_CONSTRUCTOR` macro |

---

## Possible Extensions

- **FSM controller**: use `mc_control::fsm::Controller` to sequence states (e.g. `HalfSit → ReachForObject → ReturnToStand`)
- **LIPM stabiliser**: replace CoMTask with `mc_tasks::lipm_stabilizer::StabilizerTask` for robust dynamic balance
- **Bilateral symmetry**: add a second EndEffectorTask for the left hand, creating a synchronised reaching motion
- **GUI integration**: expose the hand target via `mc_rtc::gui::Point3D` so it can be dragged interactively in mc_rtc_gui

---

## Notes on WSL2

`mc_rtc_ticker` may segfault on WSL2 Ubuntu 20.04 due to missing real-time kernel features that mc_rtc's internal timing loop relies on. The controller compiles, installs, and loads correctly; the runtime issue is environment-specific and does not affect the controller logic. Running on native Ubuntu or in Choreonoid resolves it.

---

## Author

Giacomo Demetrio Masone  
Technical assessment — CNRS-AIST Joint Robotics Laboratory  
April 2026
