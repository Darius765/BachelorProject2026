# BachelorProject2026
# Franka-Haply Haptic Teleoperation

A haptic teleoperation system that connects a **Haply Inverse3** and **VerseGrip** to a simulated **Franka Panda** robot arm using MuJoCo. The user controls the robot arm's position and orientation using the Haply devices, with force feedback reflecting contact forces from the simulation.

---

## Hardware

- **Haply Inverse3** — haptic device providing position control and force feedback
- **Haply VerseGrip (Wireless Stylus)** — provides orientation control and button input
- **Franka Panda** — simulated robot arm (real hardware integration planned)

---

## Requirements

### System
- Windows 10/11 or Ubuntu 24.04
- Python 3.10 or higher

### Python packages
```bash
pip install mujoco numpy websockets
```

### Haply SDK
Download and install the Haply Inverse Service from:
https://develop.haply.co

Start the service before running the simulation:
```bash
# Linux
sudo systemctl start haply-inverse-service

# Windows
# The service starts automatically after installation
```

---

## Project Structure

```
python_sim/
├── main.py                  # Main simulation loop
├── safety.py                # Safety limits (joint, workspace, velocity, force, estop)
├── wipe_scene.py            # Scene generator for wipe task
├── nut_pole_scene.py        # Scene generator for nut assembly task
├── drawer_scene.py          # Scene generator for drawer task
├── empty_scene.py           # Scene generator for empty table
└── tasks/
    ├── base_task.py         # Base class for all tasks
    ├── wipe_task.py         # Table wiping task
    ├── nut_pole_task.py     # Nut assembly task
    └── drawer_task.py       # Drawer task

models/
├── panda.xml                # Base Franka Panda model
├── panda_wipe.xml           # Wipe task scene (generated)
├── panda_nut.xml            # Nut assembly scene (generated)
├── panda_drawer.xml         # Drawer task scene (generated)
├── panda_empty.xml          # Empty table scene (generated)
└── assets/                  # Textures and meshes
```

---

## Setup

### 0. Get the Franka Panda model
Clone the MuJoCo Menagerie and copy the Franka model into the models folder:

```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git
cp -r mujoco_menagerie/franka_emika_panda/* models/
```

> The MuJoCo Menagerie is developed by Google DeepMind and licensed under Apache 2.0.

### 0b. Get robosuite assets (for nut assembly task only)
The nut assembly task requires texture files from robosuite:

```bash
git clone https://github.com/ARISE-Initiative/robosuite.git
mkdir -p models/assets/textures
cp robosuite/robosuite/models/assets/textures/brass-ambra.png models/assets/textures/
cp robosuite/robosuite/models/assets/textures/steel-scratched.png models/assets/textures/
```

> Robosuite is developed by the ARISE Initiative and licensed under MIT.

### 1. Generate scene files
Before running for the first time, generate the MuJoCo scene XML files:

```bash
cd python_sim
python wipe_scene.py
python nut_pole_scene.py
python drawer_scene.py
python empty_scene.py
```

### 2. Configure the simulation
At the top of `main.py`, set your desired task and force feedback:

```python
task_option = "0"  # "0"=empty, "1"=wipe, "2"=nut, "3"=drawer
force_feedback_enabled = True  # set False to disable force feedback
```

### 3. Start the Haply service
Make sure the Haply Inverse Service is running and the devices are connected before starting the simulation.

### 4. Run the simulation
```bash
cd python_sim
python main.py
```

---

## Controls

| Input | Action |
|-------|--------|
| **Inverse3 position** | Controls robot arm end-effector position |
| **VerseGrip orientation** | Controls robot arm end-effector orientation |
| **Button A** | Open gripper |
| **Button B** | Close gripper |
| **Button C (short press)** | Reset arm to home position |
| **Button C (hold)** | Clutch — arm holds position while Haply repositions |

---

## Tasks

### Wipe Task
The robot arm must wipe 25 markers off a table surface. Markers disappear when the fingers touch them while in contact with the table. 

### Nut Assembly Task
Pick up the square nut and place it over the square peg. The nut must be lowered onto the peg and touch the table to complete the task.

### Drawer Task
Three-stage task:
1. Pull the drawer open
2. Pick up the bowl and place it inside the drawer
3. Close the drawer with the bowl inside

Completion times are printed for each stage.

### Empty scene (Familiarization environment)
An empty table with two blue cubes. Use the gripper to pick up and manipulate the cube. Useful for practice and calibration.

---

## Safety System

The safety system in `safety.py` enforces the following limits:

- **Joint limits** — hard clamps at Franka's physical joint limits with soft margin warning
- **Joint velocity limits** — caps the rate of change of joint angles
- **Force limits** — triggers emergency stop if external forces exceed safe thresholds

---

## Force Feedback

Contact forces are detected using MuJoCo's constraint force system (`qfrc_constraint`) and mapped back to the Haply Inverse3 via the `set_cursor_force` WebSocket command. Force feedback can be disabled by setting `force_feedback_enabled = False` in `main.py`.

For the nut assembly task, nut-peg contact forces are amplified to provide noticeable feedback when placing the nut on the peg.

---

## Architecture

```
Haply Inverse3 ──(WebSocket ws://localhost:10001)──► Python bridge
                                                          │
                                              ┌───────────┴───────────┐
                                              │     main.py           │
                                              │  ┌─────────────────┐  │
                                              │  │  Coordinate     │  │
                                              │  │  Transform      │  │
                                              │  └────────┬────────┘  │
                                              │           │           │
                                              │  ┌────────▼────────┐  │
                                              │  │   IK Solver     │  │
                                              │  │  (Jacobian)     │  │
                                              │  └────────┬────────┘  │
                                              │           │           │
                                              │  ┌────────▼────────┐  │
                                              │  │  Safety Limits  │  │
                                              │  └────────┬────────┘  │
                                              │           │           │
                                              │  ┌────────▼────────┐  │
                                              │  │  PD Controller  │  │
                                              │  └────────┬────────┘  │
                                              │           │           │
                                              │  ┌────────▼────────┐  │
                                              │  │  MuJoCo Sim     │  │
                                              │  └────────┬────────┘  │
                                              └───────────┼───────────┘
                                                          │
                                              Force feedback to Inverse3
```

---
