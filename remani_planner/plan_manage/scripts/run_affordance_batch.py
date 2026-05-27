#!/usr/bin/env python3
import csv
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


CASES = [
    ("normal_far", 1.20, 1.30, 0.0),
    ("blocked_upper", -1.00, 1.15, -90.0),
    ("left_far", -2.20, -0.75, 0.0),
    ("right_gap_entry", 1.60, -0.35, 180.0),
    ("near_port", -1.55, 0.55, -45.0),
    ("lower_open", -1.40, -1.35, 90.0),
    ("right_center_gap", 1.70, 0.00, 180.0),
    ("upper_left_open", -1.65, 1.35, -45.0),
    ("far_left_gap", -2.50, 0.00, 0.0),
    ("far_right_gap", 2.20, 0.00, 180.0),
    ("far_upper_open", -2.40, 1.45, -45.0),
    ("far_lower_open", -2.40, -1.45, 45.0),
]


def repo_root():
    return Path(__file__).resolve().parents[3]


def workspace_root():
    return repo_root().parents[1]


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


def latest_csv(output_dir, case_name):
    files = sorted(output_dir.glob(f"arm_affordance_{case_name}_*.csv"), key=lambda p: p.stat().st_mtime)
    return files[-1] if files else None


def summarize_csv(path, case):
    with path.open() as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {
            "case": case[0],
            "init_x": case[1],
            "init_y": case[2],
            "init_yaw": case[3],
            "status": "no_samples",
        }

    numeric = []
    for row in rows:
        item = {}
        for k, v in row.items():
            try:
                item[k] = float(v)
            except ValueError:
                item[k] = math.nan
        numeric.append(item)

    trigger = next((r for r in numeric if r.get("switch_triggered", 0.0) >= 0.5), None)
    ready_count = sum(1 for r in numeric if r.get("switch_ready", 0.0) >= 0.5)
    last = numeric[-1]
    max_s = max(r["S"] for r in numeric)
    max_reach = max(r["reachability"] for r in numeric)
    max_clearance = max(r["clearance"] for r in numeric)
    max_manip = max(r["manipulability"] for r in numeric)
    summary = {
        "case": case[0],
        "init_x": case[1],
        "init_y": case[2],
        "init_yaw": case[3],
        "status": "triggered" if trigger else "not_triggered",
        "samples": len(numeric),
        "ready_samples": ready_count,
        "max_S": max_s,
        "final_S": last["S"],
        "max_reachability": max_reach,
        "final_reachability": last["reachability"],
        "max_clearance": max_clearance,
        "final_clearance": last["clearance"],
        "max_manipulability": max_manip,
        "final_manipulability": last["manipulability"],
        "final_base_x": last["base_x"],
        "final_base_y": last["base_y"],
        "final_target_dist_xy": last["target_dist_xy"],
        "csv": str(path),
        "png": str(path.with_suffix(".png")),
    }
    if trigger:
        summary.update({
            "switch_t": trigger["t"],
            "switch_base_x": trigger["base_x"],
            "switch_base_y": trigger["base_y"],
            "switch_base_yaw": trigger["base_yaw"],
            "switch_S": trigger["S"],
            "switch_reachability": trigger["reachability"],
            "switch_clearance": trigger["clearance"],
            "switch_manipulability": trigger["manipulability"],
            "switch_target_dist_xy": trigger["target_dist_xy"],
        })
    else:
        summary.update({
            "switch_t": "",
            "switch_base_x": "",
            "switch_base_y": "",
            "switch_base_yaw": "",
            "switch_S": "",
            "switch_reachability": "",
            "switch_clearance": "",
            "switch_manipulability": "",
            "switch_target_dist_xy": "",
        })
    return summary


def write_summary(summary_path, summaries):
    fields = [
        "case", "init_x", "init_y", "init_yaw", "status", "samples", "ready_samples",
        "switch_t", "switch_base_x", "switch_base_y", "switch_base_yaw",
        "switch_S", "switch_reachability", "switch_clearance", "switch_manipulability",
        "switch_target_dist_xy", "max_S", "final_S", "max_reachability",
        "final_reachability", "max_clearance", "final_clearance", "max_manipulability",
        "final_manipulability", "final_base_x", "final_base_y", "final_target_dist_xy",
        "csv", "png",
    ]
    with summary_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summaries)


def main():
    root = repo_root()
    ws = workspace_root()
    output_dir = root / "output"
    log_dir = output_dir / "affordance_batch_logs"
    output_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    summaries = []
    cases = CASES
    if len(sys.argv) > 1:
        wanted = set(sys.argv[1:])
        cases = [case for case in CASES if case[0] in wanted]

    for case in cases:
        name, x, y, yaw = case
        print(f"[batch] case={name} init=({x}, {y}, {yaw})", flush=True)
        launch_log = log_dir / f"{stamp}_{name}_launch.log"
        monitor_log = log_dir / f"{stamp}_{name}_monitor.log"

        launch_cmd = (
            "source devel/setup.bash && "
            "roslaunch remani_planner exp_charging.launch "
            f"init_x:={x} init_y:={y} init_yaw:={yaw} "
            "show_rviz:=false enable_affordance_monitor:=false"
        )
        launch_proc, launch_file = run_shell(launch_cmd, ws, launch_log, wait=False)
        time.sleep(2.0)

        monitor_cmd = (
            "source devel/setup.bash && "
            "rosrun remani_planner arm_affordance_monitor.py "
            f"_run_name:={name} _duration:=38.0 _rate:=1.0 _seed_count:=16 "
            "_switch_s:=0.55 _switch_reachability:=0.8 _switch_clearance:=0.03 "
            "_switch_manipulability:=0.15 _switch_hold:=1"
        )
        code = run_shell(monitor_cmd, ws, monitor_log, wait=True)
        stop_process(launch_proc, launch_file)

        csv_path = latest_csv(output_dir, name)
        if csv_path is None:
            summaries.append({
                "case": name,
                "init_x": x,
                "init_y": y,
                "init_yaw": yaw,
                "status": f"monitor_failed_{code}",
            })
            print(f"[batch] {name}: no CSV", flush=True)
            continue
        summary = summarize_csv(csv_path, case)
        summaries.append(summary)
        print(
            f"[batch] {name}: {summary['status']} "
            f"switch_t={summary.get('switch_t', '')} max_S={summary.get('max_S', ''):.3f} "
            f"final_S={summary.get('final_S', ''):.3f}",
            flush=True,
        )
        time.sleep(1.0)

    summary_path = output_dir / f"affordance_batch_summary_{stamp}.csv"
    write_summary(summary_path, summaries)
    print(f"[batch] summary: {summary_path}", flush=True)


if __name__ == "__main__":
    main()
