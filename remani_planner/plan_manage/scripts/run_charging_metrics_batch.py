#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import random
import re
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path


CASES = [
    ("normal_far", 1.20, 1.30, 0.0),
    ("blocked_upper", -1.00, 1.15, -90.0),
    ("right_center", 1.80, 0.00, 0.0),
    ("right_lower", 1.80, -1.20, 0.0),
    ("left_far", -2.20, -0.75, 0.0),
    ("near_port", -1.55, 0.55, -45.0),
    ("lower_open", -1.40, -1.35, 90.0),
    ("upper_left_open", -1.65, 1.35, -45.0),
]


def distance_to_box(px, py, box):
    x0, x1, y0, y1 = box
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    return (dx * dx + dy * dy) ** 0.5


def base_check_points(x, y, yaw_deg):
    # Mirrors the 2D part of MMConfig::getCarPts() for the charging demo
    # base model: a 0.5m x 0.5m footprint sampled with 0.10m check balls.
    length = 0.5
    width = 0.5
    check_radius = 0.10
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)

    hx = length / 2.0 - check_radius
    hy = width / 2.0 - check_radius
    local = [(hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy)]
    corners = [(x + c * lx - s * ly, y + s * lx + c * ly) for lx, ly in local]
    pts = list(corners)
    for a, b in zip(corners, corners[1:] + corners[:1]):
        seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
        d = check_radius
        while d < seg_len:
            ratio = d / seg_len
            pts.append((a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1])))
            d += check_radius
    return pts


def valid_charging_start(x, y, yaw_deg, extra_margin):
    # Keep random starts consistent with GenerateChargingScene() and
    # MMConfig::checkCarObsCollision(..., safe=false). The charging map uses
    # two 2.0m x 0.70m vehicles centered at (0.25, +/-0.60), leaving a 0.50m
    # passage between their inner faces. A start is invalid when any base check
    # ball is within mobile_base_check_radius + grid_map resolution of a vehicle.
    vehicle_boxes = [
        (-0.75, 1.25, 0.25, 0.95),
        (-0.75, 1.25, -0.95, -0.25),
    ]
    safe_dist = 0.10 + 0.05 + extra_margin
    for px, py in base_check_points(x, y, yaw_deg):
        if any(distance_to_box(px, py, box) < safe_dist for box in vehicle_boxes):
            return False
    return True


def make_random_cases(count, seed, start_margin):
    rng = random.Random(seed)
    cases = []
    attempts = 0
    while len(cases) < count and attempts < count * 200:
        attempts += 1
        x = rng.uniform(-2.40, 2.30)
        y = rng.uniform(-1.75, 1.75)
        yaw = rng.choice([-180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0, 180.0])
        if not valid_charging_start(x, y, yaw, start_margin):
            continue
        cases.append((f"random_{len(cases):02d}", round(x, 3), round(y, 3), yaw))
    if len(cases) < count:
        raise RuntimeError(f"Could only sample {len(cases)} valid random starts after {attempts} attempts.")
    return cases

VARIANTS = {
    "modified": {
        "flexible_goal_enabled": "true",
        "base_only_frontend_enabled": "true",
        "arm_activation_enabled": "true",
        "arm_activation_collision_skip_weight": "1.10",
        "max_seach_time": "1.5",
    },
    "remani_original": {
        "flexible_goal_enabled": "false",
        "ik_goal_enabled": "false",
        "min_approach_standoff": "-999.0",
        "base_only_frontend_enabled": "false",
        "arm_activation_enabled": "false",
        "manipulator_safe_collision_check": "false",
        "arm_activation_collision_skip_weight": "0.85",
        "max_seach_time": "0.5",
    },
}

COMPARISON_METRICS = [
    "success_rate",
    "final_ee_error_m_mean",
    "min_arm_obstacle_distance_m_mean",
    "base_path_length_m_mean",
    "base_yaw_total_variation_rad_mean",
    "arm_motion_l1_rad_mean",
    "replan_count_mean",
    "candidate_mani_collision_count_mean",
    "candidate_car_collision_count_mean",
    "time_to_first_plan_since_first_log_s_mean",
    "time_to_first_trajectory_since_first_log_s_mean",
    "trajectory_execution_time_s_mean",
    "plan_success_count_mean",
    "plan_total_time_ms_sum_mean",
    "plan_total_time_ms_mean",
    "plan_init_time_ms_mean",
    "plan_optimize_time_ms_mean",
    "optimization_failure_count_mean",
    "astar_no_path_count_mean",
    "base_only_frontend_active_count_mean",
    "base_only_frontend_rejected_count_mean",
    "ik_goal_selected_rate",
]

RUN_PARAM_FIELDS = [
    "flexible_goal_enabled",
    "ik_goal_enabled",
    "min_approach_standoff",
    "base_only_frontend_enabled",
    "arm_activation_enabled",
    "manipulator_safe_collision_check",
    "arm_activation_collision_skip_weight",
    "try_astar_times",
    "max_seach_time",
    "random_seed",
    "safe_margin",
    "safe_margin_mani",
    "map_resolution",
    "random_start_seed",
    "random_start_margin",
]

DEMO_NODE_NAMES = [
    "/charging_demo_helper",
    "/fake_mm",
    "/map_generator",
    "/mm_controller_node",
    "/model_vis",
    "/moma_robot_state_bridge",
    "/moma_robot_state_publisher",
    "/remani_planner_node",
    "/rviz",
]


def repo_root():
    return Path(__file__).resolve().parents[3]


def workspace_root():
    return repo_root().parents[1]


def output_root():
    return workspace_root() / "output" / "metrics"


def run_shell(cmd, cwd, log_path=None, wait=True):
    log_file = open(log_path, "w") if log_path else subprocess.DEVNULL
    proc = subprocess.Popen(
        ["bash", "-lc", cmd],
        cwd=str(cwd),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
    )
    if wait:
        code = proc.wait()
        if log_path:
            log_file.close()
        return code
    return proc, log_file


def run_quiet(cmd, cwd, timeout=5):
    try:
        return subprocess.run(
            ["bash", "-lc", cmd],
            cwd=str(cwd),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
            check=False,
        ).returncode
    except subprocess.TimeoutExpired:
        return 124


def matching_demo_processes(ws):
    ws = Path(ws)
    patterns = [
        "roslaunch remani_planner exp_charging.launch",
        str(ws / "devel/lib/remani_planner/charging_demo_helper"),
        str(ws / "devel/lib/remani_planner/remani_planner_node"),
        str(ws / "devel/lib/fake_mm/fake_mm"),
        str(ws / "devel/lib/fake_mm/model_vis"),
        str(ws / "devel/lib/map_generator/map_generator"),
        str(ws / "devel/lib/mm_controller/mm_controller_node"),
        "moma_robot_state_bridge.py",
        "charging_demo_metrics.py",
        "__name:=moma_robot_state_publisher",
    ]
    current_pid = os.getpid()
    current_pgid = os.getpgrp()
    proc = subprocess.run(
        ["ps", "-eo", "pid=,pgid=,cmd="],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )
    matches = []
    for line in proc.stdout.splitlines():
        parts = line.strip().split(None, 2)
        if len(parts) != 3:
            continue
        pid, pgid, cmd = int(parts[0]), int(parts[1]), parts[2]
        if pid == current_pid or pgid == current_pgid:
            continue
        if "run_charging_metrics_batch.py" in cmd:
            continue
        if any(pattern in cmd for pattern in patterns):
            matches.append((pid, pgid, cmd))
    return matches


def kill_demo_matches(ws, sig):
    for pid, pgid, cmd in matching_demo_processes(ws):
        try:
            if "roslaunch remani_planner exp_charging.launch" in cmd:
                os.killpg(pgid, sig)
            else:
                os.kill(pid, sig)
        except ProcessLookupError:
            pass
        except PermissionError:
            pass


def cleanup_demo_runtime(ws):
    node_args = " ".join(DEMO_NODE_NAMES)
    run_quiet(f"source devel/setup.bash && rosnode kill {node_args}", ws, timeout=4)
    time.sleep(0.5)
    for sig, wait_s in ((signal.SIGINT, 1.0), (signal.SIGTERM, 1.0), (signal.SIGKILL, 0.5)):
        if not matching_demo_processes(ws):
            break
        kill_demo_matches(ws, sig)
        time.sleep(wait_s)


def stop_process(proc, log_file=None):
    try:
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    proc.wait(timeout=5)
    finally:
        if log_file:
            log_file.close()


def latest_metric_json(out_dir, run_name, variant):
    files = sorted(out_dir.glob(f"{run_name}_{variant}_*.json"), key=lambda p: p.stat().st_mtime)
    return files[-1] if files else None


def first_ros_time(text, pattern):
    match = re.search(r"\[(?:INFO|WARN|ERROR)\]\s+\[([0-9.]+)\]:[^\n]*" + pattern, text)
    return float(match.group(1)) if match else ""


def last_ros_time(text, pattern):
    matches = re.findall(r"\[(?:INFO|WARN|ERROR)\]\s+\[([0-9.]+)\]:[^\n]*" + pattern, text)
    return float(matches[-1]) if matches else ""


def parse_plan_times(text):
    rows = []
    for match in re.finditer(
        r"total time:\s*([0-9.]+),\s*init:\s*([0-9.]+),\s*optimize:\s*([0-9.]+),"
        r"\s*avg_time:\s*([0-9.]+),\s*count_success:\s*([0-9]+)",
        text,
    ):
        rows.append({
            "total_ms": float(match.group(1)),
            "init_ms": float(match.group(2)),
            "optimize_ms": float(match.group(3)),
            "avg_ms": float(match.group(4)),
            "count_success": int(match.group(5)),
        })
    return rows


def parse_param_dump(text, name, default=""):
    match = re.search(rf"\* /remani_planner_node/{re.escape(name)}:\s*([^\n\r]+)", text)
    if match:
        return match.group(1).strip()
    return default


def mean_value(values):
    nums = [float(value) for value in values if value not in ("", None)]
    return sum(nums) / len(nums) if nums else ""


def sum_value(values):
    nums = [float(value) for value in values if value not in ("", None)]
    return sum(nums) if nums else ""


def parse_log(log_path):
    text = log_path.read_text(errors="ignore") if log_path.exists() else ""
    replan_count = len(re.findall(r"\[replan\s+\d+\]", text))
    plan_times = parse_plan_times(text)
    first_log_time = first_ros_time(text, r".*")
    first_plan_time = first_ros_time(text, r"total time:")
    first_trajectory_time = first_ros_time(text, r"Receive the trajectory")
    last_stop_time = last_ros_time(text, r"Stop execute the trajectory")
    time_to_first_plan_since_first_log = (
        first_plan_time - first_log_time
        if first_log_time != "" and first_plan_time != ""
        else ""
    )
    time_to_first_trajectory_since_first_log = (
        first_trajectory_time - first_log_time
        if first_log_time != "" and first_trajectory_time != ""
        else ""
    )
    trajectory_execution_time = (
        last_stop_time - first_trajectory_time
        if first_trajectory_time != "" and last_stop_time != ""
        else ""
    )
    max_continuous_failures = ""
    failures = [int(v) for v in re.findall(r"continous_failures_count:\s*([0-9]+)", text)]
    if failures:
        max_continuous_failures = max(failures)
    return {
        "replan_count": replan_count,
        "candidate_mani_collision_count": len(re.findall(r"mani collision at time", text)),
        "candidate_car_collision_count": len(re.findall(r"car collision at time", text)),
        "receive_trajectory_count": len(re.findall(r"Receive the trajectory", text)),
        "stop_execute_count": len(re.findall(r"Stop execute", text)),
        "ik_goal_selected": bool(re.search(r"IK docking goal selected", text)),
        "ik_goal_fallback_count": len(re.findall(r"No collision-free IK docking candidate found", text)),
        "time_to_first_plan_since_first_log_s": time_to_first_plan_since_first_log,
        "time_to_first_trajectory_since_first_log_s": time_to_first_trajectory_since_first_log,
        "trajectory_execution_time_s": trajectory_execution_time,
        "plan_success_count": len(plan_times),
        "plan_total_time_ms_sum": sum_value(p["total_ms"] for p in plan_times),
        "plan_total_time_ms_mean": mean_value(p["total_ms"] for p in plan_times),
        "plan_init_time_ms_mean": mean_value(p["init_ms"] for p in plan_times),
        "plan_optimize_time_ms_mean": mean_value(p["optimize_ms"] for p in plan_times),
        "plan_last_avg_time_ms": plan_times[-1]["avg_ms"] if plan_times else "",
        "max_continuous_failures": max_continuous_failures,
        "optimization_failure_count": len(re.findall(r"The optimization result is", text)),
        "kinodynamic_feasibility_failure_count": len(re.findall(r"opt failed", text)),
        "astar_no_path_count": len(re.findall(r"Kino Astar NO_PATH|open set empty, no path|KinoAstar: .* is not free", text)),
        "base_only_frontend_active_count": len(re.findall(r"base-only frontend active", text)),
        "base_only_frontend_rejected_count": len(re.findall(r"base-only frontend rejected|base-only frontend disabled", text)),
        "candidate_goal_count_logged": len(re.findall(r"IK docking candidate", text)),
        "logged_safe_margin": parse_param_dump(text, "optimization/safe_margin"),
        "logged_safe_margin_mani": parse_param_dump(text, "optimization/safe_margin_mani"),
        "logged_manipulator_safe_collision_check": parse_param_dump(text, "optimization/manipulator_safe_collision_check"),
        "logged_max_seach_time": parse_param_dump(text, "search/max_seach_time"),
        "logged_try_astar_times": parse_param_dump(text, "search/try_astar_times"),
        "logged_random_seed": parse_param_dump(text, "search/random_seed"),
    }


def run_case(ws, out_dir, log_dir, case, variant, repeat_idx, params, duration, seed_base, run_context):
    name, x, y, yaw = case
    run_name = f"{name}_r{repeat_idx}"
    log_path = log_dir / f"{run_name}_{variant}.log"
    metric_log_path = log_dir / f"{run_name}_{variant}_metrics.log"
    run_params = dict(params)
    run_params["random_seed"] = str(seed_base + repeat_idx)
    param_args = " ".join(f"{k}:={v}" for k, v in run_params.items())
    launch_cmd = (
        "source devel/setup.bash && "
        "roslaunch remani_planner exp_charging.launch "
        f"init_x:={x} init_y:={y} init_yaw:={yaw} show_rviz:=false {param_args}"
    )
    metrics_cmd = (
        "source devel/setup.bash && "
        "rosrun remani_planner charging_demo_metrics.py "
        f"_run_name:={run_name} _variant:={variant} _duration:={duration:.1f}"
    )

    print(f"[metrics] {variant:15s} {name:16s} repeat={repeat_idx} init=({x}, {y}, {yaw})", flush=True)
    cleanup_demo_runtime(ws)
    launch_proc, launch_file = run_shell(launch_cmd, ws, log_path, wait=False)
    try:
        time.sleep(1.0)
        run_shell(metrics_cmd, ws, metric_log_path, wait=True)
    finally:
        stop_process(launch_proc, launch_file)
        cleanup_demo_runtime(ws)

    metric_path = latest_metric_json(out_dir, run_name, variant)
    if metric_path is None:
        row = {
            "case": name,
            "variant": variant,
            "repeat": repeat_idx,
            "init_x": x,
            "init_y": y,
            "init_yaw": yaw,
            "success": False,
            "metric_json": "",
        }
    else:
        row = json.loads(metric_path.read_text())
        row.update({
            "case": name,
            "variant": variant,
            "repeat": repeat_idx,
            "init_x": x,
            "init_y": y,
            "init_yaw": yaw,
            "metric_json": str(metric_path),
        })
    row.update(parse_log(log_path))
    for field in RUN_PARAM_FIELDS:
        if field in run_context:
            row[field] = run_context[field]
        elif field in run_params:
            row[field] = run_params[field]
    row["safe_margin"] = row.get("logged_safe_margin") or row.get("safe_margin", "")
    row["safe_margin_mani"] = row.get("logged_safe_margin_mani") or row.get("safe_margin_mani", "")
    row["manipulator_safe_collision_check"] = (
        row.get("logged_manipulator_safe_collision_check")
        or row.get("manipulator_safe_collision_check", "")
    )
    row["try_astar_times"] = row.get("logged_try_astar_times") or row.get("try_astar_times", "")
    row["max_seach_time"] = row.get("logged_max_seach_time") or row.get("max_seach_time", "")
    row["launch_log"] = str(log_path)
    return row


def parse_extra_params(items):
    params = {}
    for item in items or []:
        if ":=" in item:
            key, value = item.split(":=", 1)
        elif "=" in item:
            key, value = item.split("=", 1)
        else:
            raise ValueError(f"Invalid --extra-param '{item}', expected key:=value")
        key = key.strip()
        if not key:
            raise ValueError(f"Invalid --extra-param '{item}', empty key")
        params[key] = value.strip()
    return params


def write_csv(path, rows):
    fields = [
        "case", "variant", "repeat", "init_x", "init_y", "init_yaw", "success",
        *RUN_PARAM_FIELDS,
        "final_ee_error_m", "min_arm_obstacle_distance_m", "base_path_length_m",
        "base_yaw_total_variation_rad", "arm_motion_l1_rad", "duration_s",
        "replan_count", "candidate_mani_collision_count", "candidate_car_collision_count",
        "receive_trajectory_count", "stop_execute_count", "ik_goal_selected",
        "ik_goal_fallback_count", "time_to_first_plan_since_first_log_s",
        "time_to_first_trajectory_since_first_log_s",
        "trajectory_execution_time_s", "plan_success_count", "plan_total_time_ms_sum",
        "plan_total_time_ms_mean", "plan_init_time_ms_mean", "plan_optimize_time_ms_mean",
        "plan_last_avg_time_ms", "max_continuous_failures", "optimization_failure_count",
        "kinodynamic_feasibility_failure_count", "astar_no_path_count",
        "base_only_frontend_active_count", "base_only_frontend_rejected_count",
        "candidate_goal_count_logged",
        "base_samples", "joint_samples", "ee_samples", "metric_json", "launch_log",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def mean_std(values):
    nums = []
    for value in values:
        if value in ("", None):
            continue
        try:
            nums.append(float(value))
        except (TypeError, ValueError):
            continue
    if not nums:
        return "", ""
    mean = sum(nums) / len(nums)
    var = sum((x - mean) ** 2 for x in nums) / len(nums)
    return mean, var ** 0.5


def write_aggregate_csv(path, rows):
    metrics = [
        "success",
        "ik_goal_selected",
        "final_ee_error_m",
        "min_arm_obstacle_distance_m",
        "base_path_length_m",
        "base_yaw_total_variation_rad",
        "arm_motion_l1_rad",
        "replan_count",
        "candidate_mani_collision_count",
        "candidate_car_collision_count",
        "receive_trajectory_count",
        "stop_execute_count",
        "ik_goal_fallback_count",
        "time_to_first_plan_since_first_log_s",
        "time_to_first_trajectory_since_first_log_s",
        "trajectory_execution_time_s",
        "plan_success_count",
        "plan_total_time_ms_sum",
        "plan_total_time_ms_mean",
        "plan_init_time_ms_mean",
        "plan_optimize_time_ms_mean",
        "plan_last_avg_time_ms",
        "max_continuous_failures",
        "optimization_failure_count",
        "kinodynamic_feasibility_failure_count",
        "astar_no_path_count",
        "base_only_frontend_active_count",
        "base_only_frontend_rejected_count",
        "candidate_goal_count_logged",
    ]
    groups = {}
    for row in rows:
        groups.setdefault((row["case"], row["variant"]), []).append(row)

    fields = ["case", "variant", "runs"]
    for metric in metrics:
        if metric in ("success", "ik_goal_selected"):
            fields.append(f"{metric}_rate" if metric != "success" else "success_rate")
        else:
            fields.extend([f"{metric}_mean", f"{metric}_std"])
    fields.extend(RUN_PARAM_FIELDS)

    agg_rows = []
    for (case, variant), items in sorted(groups.items()):
        out = {"case": case, "variant": variant, "runs": len(items)}
        for metric in metrics:
            if metric in ("success", "ik_goal_selected"):
                field = f"{metric}_rate" if metric != "success" else "success_rate"
                out[field] = sum(1.0 for item in items if item.get(metric) is True) / max(1, len(items))
            else:
                mean, std = mean_std(item.get(metric) for item in items)
                out[f"{metric}_mean"] = mean
                out[f"{metric}_std"] = std
        for field in RUN_PARAM_FIELDS:
            values = [str(item.get(field, "")) for item in items if item.get(field, "") not in ("", None)]
            out[field] = values[0] if values and all(value == values[0] for value in values) else ";".join(sorted(set(values)))
        agg_rows.append(out)

    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(agg_rows)
    return agg_rows


def fmt(value, digits=3):
    if value in ("", None):
        return "-"
    if isinstance(value, bool):
        return "yes" if value else "no"
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def write_markdown(path, rows):
    by_case = {}
    for row in rows:
        by_case.setdefault(row["case"], {}).setdefault(row["variant"], []).append(row)

    lines = [
        "# Charging Demo Metrics",
        "",
        "| Case | Variant | Run | Success | EE error (m) | Min arm-obstacle dist (m) | Base path (m) | Yaw variation (rad) | Arm motion L1 (rad) | Replans |",
        "| --- | --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for case, variants in by_case.items():
        for variant in ("modified", "remani_original"):
            items = variants.get(variant)
            if not items:
                continue
            for row in sorted(items, key=lambda item: item.get("repeat", 0)):
                lines.append(
                    f"| {case} | {variant} | {fmt(row.get('repeat'), 0)} | {fmt(row.get('success'))} | "
                    f"{fmt(row.get('final_ee_error_m'))} | {fmt(row.get('min_arm_obstacle_distance_m'))} | "
                    f"{fmt(row.get('base_path_length_m'))} | {fmt(row.get('base_yaw_total_variation_rad'))} | "
                    f"{fmt(row.get('arm_motion_l1_rad'))} | {fmt(row.get('replan_count'), 0)} |"
                )
    path.write_text("\n".join(lines) + "\n")


def write_aggregate_markdown(path, agg_rows):
    lines = [
        "# Charging Demo Aggregate Metrics",
        "",
        "| Case | Variant | Runs | Success rate | EE err mean | Min dist mean | Base path mean | Yaw var mean | Arm motion mean | Replans mean | First traj from log start (s) | Plan time sum (ms) | Opt fails | A* no path |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in agg_rows:
        lines.append(
            f"| {row['case']} | {row['variant']} | {row['runs']} | "
            f"{fmt(row.get('success_rate'))} | "
            f"{fmt(row.get('final_ee_error_m_mean'))} | "
            f"{fmt(row.get('min_arm_obstacle_distance_m_mean'))} | "
            f"{fmt(row.get('base_path_length_m_mean'))} | "
            f"{fmt(row.get('base_yaw_total_variation_rad_mean'))} | "
            f"{fmt(row.get('arm_motion_l1_rad_mean'))} | "
            f"{fmt(row.get('replan_count_mean'))} | "
            f"{fmt(row.get('time_to_first_trajectory_since_first_log_s_mean'))} | "
            f"{fmt(row.get('plan_total_time_ms_sum_mean'))} | "
            f"{fmt(row.get('optimization_failure_count_mean'))} | "
            f"{fmt(row.get('astar_no_path_count_mean'))} |"
        )
    path.write_text("\n".join(lines) + "\n")


def write_comparison_csv(path, agg_rows, baseline="remani_original", candidate="modified"):
    by_case = {}
    for row in agg_rows:
        by_case.setdefault(row["case"], {})[row["variant"]] = row

    fields = ["case", "runs_baseline", "runs_candidate"]
    for metric in COMPARISON_METRICS:
        fields.extend([f"{baseline}_{metric}", f"{candidate}_{metric}", f"delta_{metric}"])
    fields.extend([f"baseline_{field}" for field in RUN_PARAM_FIELDS])
    fields.extend([f"candidate_{field}" for field in RUN_PARAM_FIELDS])

    rows = []
    for case, variants in sorted(by_case.items()):
        if baseline not in variants or candidate not in variants:
            continue
        b = variants[baseline]
        c = variants[candidate]
        out = {
            "case": case,
            "runs_baseline": b.get("runs", ""),
            "runs_candidate": c.get("runs", ""),
        }
        for metric in COMPARISON_METRICS:
            bv = b.get(metric, "")
            cv = c.get(metric, "")
            out[f"{baseline}_{metric}"] = bv
            out[f"{candidate}_{metric}"] = cv
            try:
                out[f"delta_{metric}"] = float(cv) - float(bv)
            except (TypeError, ValueError):
                out[f"delta_{metric}"] = ""
        for field in RUN_PARAM_FIELDS:
            out[f"baseline_{field}"] = b.get(field, "")
            out[f"candidate_{field}"] = c.get(field, "")
        rows.append(out)

    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    return rows


def write_comparison_markdown(path, comparison_rows, baseline="remani_original", candidate="modified"):
    lines = [
        "# Modified vs Original Comparison",
        "",
        "| Case | Original success | Modified success | Original replans | Modified replans | Original first traj from log start (s) | Modified first traj from log start (s) | Original plan sum (ms) | Modified plan sum (ms) | Original min dist | Modified min dist |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in comparison_rows:
        lines.append(
            f"| {row['case']} | "
            f"{fmt(row.get(f'{baseline}_success_rate'))} | "
            f"{fmt(row.get(f'{candidate}_success_rate'))} | "
            f"{fmt(row.get(f'{baseline}_replan_count_mean'))} | "
            f"{fmt(row.get(f'{candidate}_replan_count_mean'))} | "
            f"{fmt(row.get(f'{baseline}_time_to_first_trajectory_since_first_log_s_mean'))} | "
            f"{fmt(row.get(f'{candidate}_time_to_first_trajectory_since_first_log_s_mean'))} | "
            f"{fmt(row.get(f'{baseline}_plan_total_time_ms_sum_mean'))} | "
            f"{fmt(row.get(f'{candidate}_plan_total_time_ms_sum_mean'))} | "
            f"{fmt(row.get(f'{baseline}_min_arm_obstacle_distance_m_mean'))} | "
            f"{fmt(row.get(f'{candidate}_min_arm_obstacle_distance_m_mean'))} |"
        )
    path.write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Run charging demo metrics for modified and original REMANI baseline.")
    parser.add_argument("--duration", type=float, default=55.0)
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--seed-base", type=int, default=2)
    parser.add_argument("--case", action="append", help="Run only selected case; repeatable. Supports generated random_NN cases.")
    parser.add_argument("--variant", action="append", choices=list(VARIANTS.keys()), help="Run only selected variant; repeatable.")
    parser.add_argument("--random-starts", type=int, default=0, help="Append N random free-space starts in the charging scene.")
    parser.add_argument("--random-start-seed", type=int, default=17, help="Seed used for random start generation.")
    parser.add_argument("--random-start-margin", type=float, default=0.05,
                        help="Extra margin on top of the demo collision check when sampling random starts. "
                             "The default filters borderline poses that KinoAstar later reports as start-not-free.")
    parser.add_argument("--extra-param", action="append", default=[], help="Override launch arg for all variants, e.g. flexible_goal_enabled:=false")
    args = parser.parse_args()

    ws = workspace_root()
    out_dir = output_root()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = out_dir / f"batch_{stamp}"
    log_dir = run_dir / "logs"
    run_dir.mkdir(parents=True, exist_ok=True)
    out_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    requested_cases = set(args.case or [])
    cases = [c for c in CASES if not requested_cases or c[0] in requested_cases]
    if args.random_starts > 0:
        random_cases = make_random_cases(args.random_starts, args.random_start_seed, args.random_start_margin)
        cases = random_cases if not requested_cases else cases + [c for c in random_cases if c[0] in requested_cases]
    if requested_cases and not cases:
        raise RuntimeError(f"No requested cases matched: {', '.join(sorted(requested_cases))}")
    variants = [v for v in VARIANTS if args.variant is None or v in args.variant]
    extra_params = parse_extra_params(args.extra_param)
    rows = []
    run_context = {
        "safe_margin": "0.03",
        "safe_margin_mani": "0.04",
        "map_resolution": "0.10",
        "random_start_seed": str(args.random_start_seed),
        "random_start_margin": str(args.random_start_margin),
    }
    for case in cases:
        for repeat_idx in range(args.repeat):
            for variant in variants:
                params = dict(VARIANTS[variant])
                params.update(extra_params)
                rows.append(run_case(ws, out_dir, log_dir, case, variant, repeat_idx, params,
                                     args.duration, args.seed_base, run_context))
                time.sleep(1.0)

    csv_path = run_dir / "summary.csv"
    md_path = run_dir / "summary.md"
    aggregate_csv_path = run_dir / "aggregate_summary.csv"
    aggregate_md_path = run_dir / "aggregate_summary.md"
    write_csv(csv_path, rows)
    write_markdown(md_path, rows)
    agg_rows = write_aggregate_csv(aggregate_csv_path, rows)
    write_aggregate_markdown(aggregate_md_path, agg_rows)
    comparison_csv_path = run_dir / "comparison_modified_vs_original.csv"
    comparison_md_path = run_dir / "comparison_modified_vs_original.md"
    comparison_rows = write_comparison_csv(comparison_csv_path, agg_rows)
    write_comparison_markdown(comparison_md_path, comparison_rows)
    print(f"[metrics] CSV: {csv_path}", flush=True)
    print(f"[metrics] Markdown: {md_path}", flush=True)
    print(f"[metrics] Aggregate CSV: {aggregate_csv_path}", flush=True)
    print(f"[metrics] Aggregate Markdown: {aggregate_md_path}", flush=True)
    if comparison_rows:
        print(f"[metrics] Comparison CSV: {comparison_csv_path}", flush=True)
        print(f"[metrics] Comparison Markdown: {comparison_md_path}", flush=True)


if __name__ == "__main__":
    main()
