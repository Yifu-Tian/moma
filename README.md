# Mobile Manipulator Charging Demo

This repository contains a REMANI-Planner based mobile-manipulator demo for a
charging-port reaching task. The scene has two car-like cuboid obstacles with a
narrow gap between them. The planner drives the mobile base and arm so that the
end-effector reaches the charger target.

The code is based on the original REMANI-Planner project. The charging demo adds:

- a deterministic charging scene map;
- a visible charger and target marker;
- one normal start scenario and one obstacle-blocked start scenario;
- command-line launch arguments for the initial base pose;
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
roslaunch remani_planner exp_charging_blocked.launch
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
roslaunch remani_planner exp_charging_blocked_record.launch record_duration:=25.0
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
roslaunch remani_planner exp_charging_blocked.launch \
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
  `remani_planner/plan_manage/config/exp_charging_blocked_param.yaml`
- Demo launch files:
  `remani_planner/plan_manage/launch/exp_charging.launch`
  `remani_planner/plan_manage/launch/exp_charging_blocked.launch`
  `remani_planner/plan_manage/launch/exp_charging_record.launch`
  `remani_planner/plan_manage/launch/exp_charging_blocked_record.launch`
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

**RE**al-time Whole-body Motion Planning for Mobile **MANI**pulators Using Environment-adaptive Search and Spatial-temporal Optimization

![top](attachment/top.png)

## News

- **Jan 29, 2024**: REMANI-Planner is accepted to [ICRA 2024](https://2024.ieee-icra.org/).

## Introduction

REMANI-Planner presents a motion planning method capable of generating high-quality, safe, agile and feasible trajectories for mobile manipulators in real time.

![system_overview](./attachment/system_overview.png)

**Authors**: [Chengkai Wu](https://chengkaiwu.me/)\*, [Ruilin Wang](https://github.com/Ruilin-W)\*, [Mianzhi Song](https://robotics-star.com/), [Fei Gao](http://zju-fast.com/fei-gao/), [Jie Mei](https://scholar.google.com/citations?user=tyQm5IkAAAAJ&hl=zh-CN) and [Boyu Zhou](https://robotics-star.com/)$^{\dagger}$.

**Institutions**: [STAR Group](https://robotics-star.com/), [HITSZ MAS Lab](https://hitsz-mas.github.io/mas-lab-website/) and [ZJU FAST Lab](http://zju-fast.com/).

**Video**: [YouTube](https://www.youtube.com/watch?v=iYdAEZ3z11s), [Bilibili](https://www.bilibili.com/video/BV1Wz4y1V7vL).

**Paper**: [Real-time Whole-body Motion Planning for Mobile Manipulators Using Environment-adaptive Search and Spatial-temporal Optimization](https://ieeexplore.ieee.org/document/10610192), 2024 IEEE International Conference on Robotics and Automation (ICRA).

```
@INPROCEEDINGS{10610192,
  author={Wu, Chengkai and Wang, Ruilin and Song, Mianzhi and Gao, Fei and Mei, Jie and Zhou, Boyu},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Real-time Whole-body Motion Planning for Mobile Manipulators Using Environment-adaptive Search and Spatial-temporal Optimization}, 
  year={2024},
  volume={},
  number={},
  pages={1369-1375},
  keywords={Service robots;Dynamics;Transportation;Real-time systems;Planning;Safety;Complexity theory},
  doi={10.1109/ICRA57147.2024.10610192}}
```

If you find this work useful or interesting, please kindly give us a star ⭐, thanks!😀

## Setup

Compiling tests passed on Ubuntu 20.04 with ROS installed.

### Prerequisites

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with Noetic)

```
sudo apt install libompl-dev libeigen3-dev
cd /usr/include
sudo ln -sf eigen3/Eigen Eigen
sudo ln -sf eigen3/unsupported unsupported
```

### Compiling and Running

```
cd ${your catkin workspace}/src
git clone -b master --single-branch https://github.com/SYSU-STAR/REMANI-Planner.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
```

1. Navigating in dense cuboids map

```
source devel/setup.bash
roslaunch remani_planner exp0.launch
```

You should see the simulation in rviz. You can use the `2D Nav Goal` to send a trigger to start navigation.

<p align="center">
  <img src="./attachment/exp0_0.gif"/>
</p>


2. Navigating through a bridge

```
source devel/setup.bash
roslaunch remani_planner exp1.launch
```

<p align="center">
  <img src="./attachment/exp0_1.gif"/>
</p>


## Customize your own Mobile Manipulator (MM)

1. Make the following adjustments in the `remani_planner/mm_config/src/mm_config.cpp` file:
   - Modify the `getAJointTran` function to calculate the homogeneous transformation between different frames of the manipulator.
   - Modify the `setLinkPoint` function to set the position of collision spheres in the respective frame.
   - Modify the `getMMMarkerArray` function based on the urdf file to generate the marker array for MM visualization.
2. Adapt the `mm_param.yaml` file located in the `remani_planner/remani_planner/config/` directory to configure parameters specific to your MM.



**Note**: We have provided an example for the [UR5](https://www.universal-robots.com/products/ur5-robot/) in our code. To use the UR5, simply follow these steps:

1. Open the `mm_param.yaml` file located in the `remani_planner/plan_manage/config/` directory.
2. Locate the `parameter` for `FastArmer` and comment it out by adding a "#" symbol at the beginning of the line.
3. Uncomment the `parameter` for `UR5` by removing the "#" symbol at the beginning of the line.
4. You are now ready to conduct above experiments with a mobile base incorporating the UR5 configuration.

<p align="center">
  <img src="./attachment/exp1_0.gif" width = "400" height = "225"/>
  <img src="./attachment/exp1_1.gif" width = "400" height = "225"/>
</p>

## Acknowledgements

We use [MINCO](https://github.com/ZJU-FAST-Lab/GCOPTER) as our trajectory representation.

We borrow the framework from [AutoTrans](https://github.com/SYSU-STAR/AutoTrans).

We would like to thank colleagues at [Huawei](https://www.huawei.com/en/) for their support for this work: Zehui Meng and Changjin Wang.

## License

The source code is released under the [GPLv3](https://www.gnu.org/licenses/) license.

## Maintenance

For any technical issues, please contact Chengkai Wu([chengkaiwuu@gmail.com](mailto:chengkaiwuu@gmail.com)) or Ruilin Wang(Ruilinin@outlook.com).
