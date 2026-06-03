# Mobile Manipulator Charging Demo



The code is based on the original REMANI-Planner project. 

## Charging Demo


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
