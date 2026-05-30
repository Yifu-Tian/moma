#!/usr/bin/env python3
import argparse
import csv
import json
import os
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

VARIANTS = {
    "modified": {
        "flexible_goal_enabled": "true",
        "base_only_frontend_enabled": "true",
        "arm_activation_enabled": "true",
        "arm_activation_collision_skip_weight": "1.10",
    },
    "remani_original": {
        "flexible_goal_enabled": "false",
        "ik_goal_enabled": "false",
        "min_approach_standoff": "-999.0",
        "base_only_frontend_enabled": "false",
        "arm_activation_enabled": "false",
        "manipulator_safe_collision_check": "false",
        "arm_activation_collision_skip_weight": "0.85",
    },
}


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


def stop_process(proc, log_file=None):
    if proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5)
    if log_file:
        log_file.close()


def latest_metric_json(out_dir, run_name, variant):
    files = sorted(out_dir.glob(f"{run_name}_{variant}_*.json"), key=lambda p: p.stat().st_mtime)
    return files[-1] if files else None


def parse_log(log_path):
    text = log_path.read_text(errors="ignore") if log_path.exists() else ""
    replan_count = len(re.findall(r"\[replan\s+\d+\]", text))
    return {
        "replan_count": replan_count,
        "candidate_mani_collision_count": len(re.findall(r"mani collision at time", text)),
        "candidate_car_collision_count": len(re.findall(r"car collision at time", text)),
        "receive_trajectory_count": len(re.findall(r"Receive the trajectory", text)),
        "stop_execute_count": len(re.findall(r"Stop execute", text)),
        "ik_goal_selected": bool(re.search(r"IK docking goal selected", text)),
    }


def run_case(ws, out_dir, log_dir, case, variant, repeat_idx, params, duration, seed_base):
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
    launch_proc, launch_file = run_shell(launch_cmd, ws, log_path, wait=False)
    time.sleep(3.0)
    run_shell(metrics_cmd, ws, metric_log_path, wait=True)
    stop_process(launch_proc, launch_file)

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
    row["launch_log"] = str(log_path)
    return row


def write_csv(path, rows):
    fields = [
        "case", "variant", "repeat", "init_x", "init_y", "init_yaw", "success",
        "final_ee_error_m", "min_arm_obstacle_distance_m", "base_path_length_m",
        "base_yaw_total_variation_rad", "arm_motion_l1_rad", "duration_s",
        "replan_count", "candidate_mani_collision_count", "candidate_car_collision_count",
        "receive_trajectory_count", "stop_execute_count", "ik_goal_selected",
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
        "final_ee_error_m",
        "min_arm_obstacle_distance_m",
        "base_path_length_m",
        "base_yaw_total_variation_rad",
        "arm_motion_l1_rad",
        "replan_count",
        "candidate_mani_collision_count",
        "candidate_car_collision_count",
    ]
    groups = {}
    for row in rows:
        groups.setdefault((row["case"], row["variant"]), []).append(row)

    fields = ["case", "variant", "runs"]
    for metric in metrics:
        if metric == "success":
            fields.append("success_rate")
        else:
            fields.extend([f"{metric}_mean", f"{metric}_std"])

    agg_rows = []
    for (case, variant), items in sorted(groups.items()):
        out = {"case": case, "variant": variant, "runs": len(items)}
        for metric in metrics:
            if metric == "success":
                out["success_rate"] = sum(1.0 for item in items if item.get("success") is True) / max(1, len(items))
            else:
                mean, std = mean_std(item.get(metric) for item in items)
                out[f"{metric}_mean"] = mean
                out[f"{metric}_std"] = std
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
        "| Case | Variant | Runs | Success rate | EE err mean | Min dist mean | Base path mean | Yaw var mean | Arm motion mean | Replans mean |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
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
            f"{fmt(row.get('replan_count_mean'))} |"
        )
    path.write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Run charging demo metrics for modified and original REMANI baseline.")
    parser.add_argument("--duration", type=float, default=55.0)
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--seed-base", type=int, default=2)
    parser.add_argument("--case", action="append", choices=[c[0] for c in CASES], help="Run only selected case; repeatable.")
    parser.add_argument("--variant", action="append", choices=list(VARIANTS.keys()), help="Run only selected variant; repeatable.")
    args = parser.parse_args()

    ws = workspace_root()
    out_dir = output_root()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = out_dir / f"batch_{stamp}"
    log_dir = run_dir / "logs"
    run_dir.mkdir(parents=True, exist_ok=True)
    out_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    cases = [c for c in CASES if args.case is None or c[0] in args.case]
    variants = [v for v in VARIANTS if args.variant is None or v in args.variant]
    rows = []
    for case in cases:
        for repeat_idx in range(args.repeat):
            for variant in variants:
                rows.append(run_case(ws, out_dir, log_dir, case, variant, repeat_idx, VARIANTS[variant],
                                     args.duration, args.seed_base))
                time.sleep(1.0)

    csv_path = run_dir / "summary.csv"
    md_path = run_dir / "summary.md"
    aggregate_csv_path = run_dir / "aggregate_summary.csv"
    aggregate_md_path = run_dir / "aggregate_summary.md"
    write_csv(csv_path, rows)
    write_markdown(md_path, rows)
    agg_rows = write_aggregate_csv(aggregate_csv_path, rows)
    write_aggregate_markdown(aggregate_md_path, agg_rows)
    print(f"[metrics] CSV: {csv_path}", flush=True)
    print(f"[metrics] Markdown: {md_path}", flush=True)
    print(f"[metrics] Aggregate CSV: {aggregate_csv_path}", flush=True)
    print(f"[metrics] Aggregate Markdown: {aggregate_md_path}", flush=True)


if __name__ == "__main__":
    main()
