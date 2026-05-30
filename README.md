# Mobile Manipulator Charging Demo

This repository contains a REMANI-Planner based mobile-manipulator demo for a
charging-port reaching task. The scene has two car-like cuboid obstacles with a
narrow gap between them. The planner drives the mobile base and arm so that the
end-effector reaches the charger target.

The code is based on the original REMANI-Planner project. 

## Charging Demo

The current demo keeps the REMANI whole-body planner as the trajectory
generator, uses the MOMA/Piper mobile-manipulator model, and selects the final
docking state online. For each preset target trigger, the planner samples a
small set of candidate base poses near the charger, solves position IK for the
arm so the end-effector tip reaches the charger point, rejects colliding
candidates, and sends the selected base + arm state to REMANI for whole-body
trajectory generation.

Build:

```bash
cd ~/remani_ws/remani_mpd_ws
catkin_make
source devel/setup.bash
```

Run the default charging scene:

```bash
cd ~/remani_ws/remani_mpd_ws && source devel/setup.bash && roslaunch remani_planner exp_charging.launch
```

Run with a custom initial base pose:

```bash
cd ~/remani_ws/remani_mpd_ws && source devel/setup.bash && roslaunch remani_planner exp_charging.launch init_x:=1.8 init_y:=-1.2 init_yaw:=0
```

Run without RViz for log-only validation:

```bash
cd ~/remani_ws/remani_mpd_ws && source devel/setup.bash && roslaunch remani_planner exp_charging.launch init_x:=1.8 init_y:=-1.2 init_yaw:=0 show_rviz:=false
```

Useful validation starts:

```bash
roslaunch remani_planner exp_charging.launch init_x:=-3.0 init_y:=0.0 init_yaw:=0
roslaunch remani_planner exp_charging.launch init_x:=1.8 init_y:=0.0 init_yaw:=0
roslaunch remani_planner exp_charging.launch init_x:=1.8 init_y:=-1.2 init_yaw:=0
roslaunch remani_planner exp_charging.launch init_x:=-1.0 init_y:=1.15 init_yaw:=-90
```

The successful trigger log should contain a line similar to:

```text
[charging_demo] IK docking goal selected: base=(...), yaw=... deg, tip_err=... m, q_deg=[...].
```

## Quantitative Evaluation

Run the batch evaluator to compare the modified charging method with the
original REMANI baseline on the same charging scene and robot model. The
baseline disables the charging-specific IK docking target selection, base-only
frontend, arm activation term, approach-side filtering, and stricter
manipulator safe-distance check, then uses the fixed REMANI waypoint target.

```bash
cd ~/remani_ws/remani_mpd_ws
source devel/setup.bash
rosrun remani_planner run_charging_metrics_batch.py
```

Quick single-case comparison:

```bash
cd ~/remani_ws/remani_mpd_ws
source devel/setup.bash
rosrun remani_planner run_charging_metrics_batch.py --case blocked_upper --repeat 1
```

The default batch runs 8 initial base poses, 5 repeated trials per pose, and 2
methods (`modified` and `remani_original`). This is 80 runs total, so it can
take a while. For a shorter smoke test:

```bash
rosrun remani_planner run_charging_metrics_batch.py --case blocked_upper --case right_lower --repeat 2 --duration 45
```

The evaluator writes:

```text
output/metrics/batch_*/summary.csv
output/metrics/batch_*/summary.md
output/metrics/batch_*/aggregate_summary.csv
output/metrics/batch_*/aggregate_summary.md
```

Reported metrics:

- `success`: final end-effector distance to the charging target is below the threshold.
- `final_ee_error_m`: final end-effector target error.
- `min_arm_obstacle_distance_m`: minimum distance between arm/gripper proxy points and obstacle point cloud, excluding the charger contact region.
- `base_path_length_m`: executed base path length.
- `base_yaw_total_variation_rad`: accumulated base heading change.
- `arm_motion_l1_rad`: accumulated joint motion.
- `replan_count`: number of planner replan attempts parsed from the launch log.

`summary.*` contains every run. `aggregate_summary.*` contains per-case,
per-method means and standard deviations across repeated trials.

## Original vs Modified REMANI

The following examples compare the original REMANI behavior with the modified
charging-demo planner. `org1` and `org2` are original REMANI runs. `dev1` and
`dev2` are modified-version runs using the MOMA/Piper mobile manipulator model
and the charging-scene planner changes.

| Original REMANI | Modified version |
| --- | --- |
| **org1**<br><img src="attachment/remani_org1.gif" width="420"> | **dev1**<br><img src="attachment/remani_dev1.gif" width="420"> |
| **org2**<br><img src="attachment/remani_org2.gif" width="420"> | **dev2**<br><img src="attachment/remani_dev2.gif" width="420"> |

# Original REMANI-Planner

This demo is built on top of the original REMANI-Planner:

- Project: https://github.com/SYSU-STAR/REMANI-Planner
- Paper: https://ieeexplore.ieee.org/document/10610192

Please refer to the original project for the full method description, citation,
license, and upstream usage instructions.
