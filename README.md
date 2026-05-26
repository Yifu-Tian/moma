# Mobile Manipulator Charging Demo

This repository contains a REMANI-Planner based mobile-manipulator demo for a
charging-port reaching task. The scene has two car-like cuboid obstacles with a
narrow gap between them. The planner drives the mobile base and arm so that the
end-effector reaches the charger target.

The code is based on the original REMANI-Planner project. The charging demo adds:

- a deterministic charging scene map;
- a visible charger and target marker;
- one normal start scenario and one obstacle-blocked start scenario;
- command-line launch arguments for switching normal and blocked initial poses;
- an optional lightweight 2D MP4 recorder, so the demo can be checked without RViz.

## Quick Start

Tested environment:

- Ubuntu 20.04
- ROS Noetic
- `catkin_make`

Install basic dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  libompl-dev \
  libeigen3-dev \
  python3-opencv \
  python3-rospkg

cd /usr/include
sudo ln -sf eigen3/Eigen Eigen
sudo ln -sf eigen3/unsupported unsupported
```

Create a workspace and clone this repository:

```bash
mkdir -p ~/moma_ws/src
cd ~/moma_ws/src
git clone https://github.com/Yifu-Tian/moma.git
cd ~/moma_ws
```

Build the packages needed by the charging demo:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES="quadrotor_msgs;remani_diffusion_msgs;diffusion_arm_planner;map_generator;plan_env;mm_config;fake_mm;local_sensing_node;traj_utils;mm_controller;path_searching;traj_opt;remani_planner"
source devel/setup.bash
```

The whitelist is intentional. It avoids building unrelated legacy RViz plugin
targets that are not required by this demo.

## Recommended Evaluation Commands

These commands are the preferred pair for checking the demo. Both use the same
recording launch file; only the initial base pose changes. The first case starts
from an open approach pose. The second case starts above the upper vehicle, so
the target is partially blocked by the obstacle layout and the base must move
around before the arm reaches the charger.

```bash
cd ~/moma_ws
source devel/setup.bash

# Case 1: normal charging approach, no RViz, record a top-down MP4.
roslaunch remani_planner exp_charging_record.launch record_duration:=25.0

# Case 2: obstacle-blocked start, no RViz, record a top-down MP4.
roslaunch remani_planner exp_charging_record.launch \
  init_x:=-1.30 \
  init_y:=1.15 \
  init_yaw:=-90 \
  record_duration:=35.0
```

Each run prints the final base position, final end-effector position, target
position, final end-effector-to-target distance, and success status. The MP4 also
shows the live `ee-target` distance. By default, success means:

```text
final ee-target distance <= 0.15 m
```

## Run the RViz Demo

Normal charging scenario:

```bash
cd ~/moma_ws
source devel/setup.bash
roslaunch remani_planner exp_charging.launch
```

Obstacle-blocked start scenario:

```bash
cd ~/moma_ws
source devel/setup.bash
roslaunch remani_planner exp_charging.launch \
  init_x:=-1.30 \
  init_y:=1.15 \
  init_yaw:=-90
```

The demo auto-triggers the target after a short delay. In RViz, the blue cuboid is
the charger, and the green sphere is the end-effector target.

## Run Without RViz and Record MP4

Normal scenario:

```bash
cd ~/moma_ws
source devel/setup.bash
roslaunch remani_planner exp_charging_record.launch record_duration:=25.0
```

Blocked-start scenario:

```bash
cd ~/moma_ws
source devel/setup.bash
roslaunch remani_planner exp_charging_record.launch \
  init_x:=-1.30 \
  init_y:=1.15 \
  init_yaw:=-90 \
  record_duration:=35.0
```

Videos are written to:

```text
~/moma_ws/src/moma/output/
```

The 2D recorder uses a top-down projection:

- `base`: mobile base position;
- `end-effector`: arm end-effector position;
- gray dashed line: base trajectory;
- red/blue line: end-effector trajectory;
- brown line/points: planner visualization;
- `charger`: physical charging connector;
- `target`: planner goal point.

## Set the Initial Base Pose

You can set the mobile base initial pose directly from the launch command. No code
edit is needed.

```bash
roslaunch remani_planner exp_charging.launch \
  init_x:=-2.2 \
  init_y:=1.2 \
  init_yaw:=-90
```

For MP4 recording:

```bash
roslaunch remani_planner exp_charging_record.launch \
  init_x:=-2.2 \
  init_y:=1.2 \
  init_yaw:=-90 \
  record_duration:=25.0
```

Coordinate convention:

- `init_x`: along the long direction of the two vehicles, in meters;
- `init_y`: across the gap between the two vehicles, in meters;
- `init_yaw`: mobile base heading, in degrees;
- `init_j1` to `init_j6`: initial arm joint angles, in degrees.

The two cuboid vehicles are approximately:

- `x` range: `[-0.75, 1.25]`
- upper vehicle: `y` range `[0.25, 0.95]`
- lower vehicle: `y` range `[-0.95, -0.25]`
- gap: `y` range `[-0.25, 0.25]`

For a blocked-start test, a useful initial pose is:

```bash
roslaunch remani_planner exp_charging.launch \
  init_x:=-1.30 \
  init_y:=1.15 \
  init_yaw:=-90
```

## Useful Files

- Charging map: `mm_simulator/map_generator/src/map_generator.cpp`
- Charging helper marker and auto-trigger node:
  `remani_planner/plan_manage/src/charging_demo_helper.cpp`
- Demo configs:
  `remani_planner/plan_manage/config/exp_charging_param.yaml`
- Demo launch files:
  `remani_planner/plan_manage/launch/exp_charging.launch`
  `remani_planner/plan_manage/launch/exp_charging_record.launch`
- 2D recorder:
  `remani_planner/plan_manage/scripts/charging_scene_recorder.py`

## Troubleshooting

If `roslaunch` cannot find the package, source the workspace again:

```bash
cd ~/moma_ws
source devel/setup.bash
```

If the planner repeatedly reports `goal is not free`, the target is probably too
close to an occupied obstacle cell or inside the safety margin. Use the provided
charging config defaults first, then adjust the charger/target carefully.

If RViz is slow, use the `*_record.launch` files to generate a 2D MP4 without
opening RViz.

---

# Original REMANI-Planner

This demo is built on top of the original REMANI-Planner:

- Project: https://github.com/SYSU-STAR/REMANI-Planner
- Paper: https://ieeexplore.ieee.org/document/10610192

Please refer to the original project for the full method description, citation,
license, and upstream usage instructions.
