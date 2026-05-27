# Charging Demo Baselines

## 2026-05-27 Before Base-Only / Whole-Body Switching

- Purpose: baseline for comparing the original always-whole-body charging demo against a staged base-only + whole-body planner.
- Command:

```bash
cd ~/remani_ws/remani_mpd_ws && source devel/setup.bash && roslaunch remani_planner exp_charging.launch init_x:=1.2 init_y:=1.3 init_yaw:=0
```

- Video: `src/REMANI-Planner/attachment/20260527_032053.mp4`
- Notes: user observed this case as a candidate for before/after comparison because replanning can be frequent and the resulting trajectory is not always visually optimal.
