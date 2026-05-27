#!/usr/bin/env python3
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rospy
import rospkg
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

try:
    from scipy.optimize import least_squares
    from scipy.spatial import cKDTree
except Exception as exc:
    raise RuntimeError("arm_affordance_monitor.py requires scipy") from exc


class PiperKinematics:
    def __init__(self):
        self.alpha = np.array([0.0, -math.pi / 2.0, 0.0, math.pi / 2.0, -math.pi / 2.0, math.pi / 2.0])
        self.a = np.array([0.0, 0.0, 0.28503, -0.02198, 0.0, 0.0])
        self.d = np.array([0.123, 0.0, 0.0, 0.25075, 0.0, 0.091])
        self.theta_offset = np.deg2rad([0.0, -172.2135102, -102.7827493, 0.0, 0.0, 0.0])
        self.q_min = np.array([-2.618, 0.0, -2.967, -1.745, -1.22, -2.0944])
        self.q_max = np.array([2.168, 3.14, 0.0, 1.745, 1.22, 2.0944])
        self.folded = np.deg2rad([0.0, 60.0, -120.0, 0.0, 0.0, 0.0])
        self.base_to_arm = np.eye(4)
        self.base_to_arm[:3, 3] = [0.15, 0.0, 0.435]

    def joint_tf(self, i, theta):
        th = theta + self.theta_offset[i]
        ca, sa = math.cos(self.alpha[i]), math.sin(self.alpha[i])
        ct, st = math.cos(th), math.sin(th)
        t = np.eye(4)
        t[0, 0] = ct
        t[0, 1] = -st
        t[0, 3] = self.a[i]
        t[1, 0] = ca * st
        t[1, 1] = ca * ct
        t[1, 2] = -sa
        t[1, 3] = -sa * self.d[i]
        t[2, 0] = sa * st
        t[2, 1] = sa * ct
        t[2, 2] = ca
        t[2, 3] = ca * self.d[i]
        return t

    @staticmethod
    def car_tf(base):
        x, y, yaw = base
        c, s = math.cos(yaw), math.sin(yaw)
        t = np.eye(4)
        t[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
        t[:3, 3] = [x, y, 0.0]
        return t

    def chain(self, base, q):
        ts = []
        t = self.car_tf(base).dot(self.base_to_arm)
        for i in range(6):
            t = t.dot(self.joint_tf(i, q[i]))
            ts.append(t.copy())
        return ts

    def fingertip(self, base, q):
        t = self.chain(base, q)[-1]
        p = t.dot(np.array([0.0, 0.0, 0.1358, 1.0]))
        return p[:3]

    def collision_points(self, base, q):
        local = [
            [np.array([0.0, 0.0, -0.0615, 1.0])],
            [np.array([0.0712575, 0.0, 0.0, 1.0]), np.array([0.142515, 0.0, 0.0, 1.0]), np.array([0.2137725, 0.0, 0.0, 1.0])],
            [np.array([-0.007327, -0.083583, 0.0, 1.0]), np.array([-0.014653, -0.167167, 0.0, 1.0])],
            [],
            [np.array([0.0, -0.0455, 0.0, 1.0])],
            [np.array([0.0, 0.0, 0.04, 1.0]), np.array([0.0, 0.04, 0.12, 1.0]), np.array([0.0, -0.04, 0.12, 1.0])],
        ]
        pts = []
        contact_flags = []
        for i, t in enumerate(self.chain(base, q)):
            for j, p_local in enumerate(local[i]):
                pts.append(t.dot(p_local)[:3])
                contact_flags.append(i == 5 and j >= 1)
        return np.asarray(pts), np.asarray(contact_flags, dtype=bool)

    def joint_margin(self, q):
        span = self.q_max - self.q_min
        normalized = np.minimum((q - self.q_min) / span, (self.q_max - q) / span) * 2.0
        return float(np.clip(np.min(normalized), 0.0, 1.0))


class ArmAffordanceMonitor:
    def __init__(self):
        self.kin = PiperKinematics()
        self.duration = float(rospy.get_param("~duration", 35.0))
        self.rate_hz = float(rospy.get_param("~rate", 1.0))
        self.seed_count = int(rospy.get_param("~seed_count", 16))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.03))
        self.safe_dist = float(rospy.get_param("~safe_dist", 0.03))
        self.contact_radius = float(rospy.get_param("~contact_radius", 0.08))
        self.clearance_clip = float(rospy.get_param("~clearance_clip", 0.30))
        self.w_reach = float(rospy.get_param("~w_reach", 0.5))
        self.w_clearance = float(rospy.get_param("~w_clearance", 0.3))
        self.w_manip = float(rospy.get_param("~w_manip", 0.2))
        self.switch_s = float(rospy.get_param("~switch_s", 0.55))
        self.switch_reachability = float(rospy.get_param("~switch_reachability", 0.8))
        self.switch_clearance = float(rospy.get_param("~switch_clearance", 0.03))
        self.switch_manipulability = float(rospy.get_param("~switch_manipulability", 0.15))
        self.switch_hold = int(rospy.get_param("~switch_hold", 1))
        self.publish_marker = bool(rospy.get_param("~publish_marker", True))
        self.retrigger_target = bool(rospy.get_param("~retrigger_target", True))

        pkg_path = Path(rospkg.RosPack().get_path("remani_planner"))
        output_dir = Path(rospy.get_param("~output_dir", str(pkg_path.parents[1] / "output")))
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_name = str(rospy.get_param("~run_name", "")).strip()
        prefix = f"arm_affordance_{run_name}_{stamp}" if run_name else f"arm_affordance_{stamp}"
        self.csv_path = output_dir / f"{prefix}.csv"
        self.plot_path = output_dir / f"{prefix}.png"

        self.base = None
        self.current_q = self.kin.folded.copy()
        self.target = None
        self.cloud_tree = None
        self.rows = []
        self.start_time = time.monotonic()
        self.last_sample = rospy.Time(0)
        self.switch_count = 0
        self.switch_row = None

        rospy.Subscriber("/mm/car/odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/mm/mani/joint_state", JointState, self.joint_cb, queue_size=1)
        rospy.Subscriber("/map_generator/global_cloud", PointCloud2, self.cloud_cb, queue_size=1)
        rospy.Subscriber("/charging_demo/markers", MarkerArray, self.marker_cb, queue_size=1)
        self.marker_pub = rospy.Publisher("/arm_affordance/switch_marker", MarkerArray, queue_size=1, latch=True)
        self.switch_pub = rospy.Publisher("/arm_affordance/whole_body_enabled", Bool, queue_size=1, latch=True)
        self.target_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.timer_cb)
        rospy.loginfo("[arm_affordance] writing CSV to %s", self.csv_path)
        self.switch_pub.publish(Bool(data=False))

    def publish_retrigger(self):
        if not self.retrigger_target:
            return
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)
        rospy.logwarn("[arm_affordance] retriggered REMANI preset target for staged whole-body planning.")

    @staticmethod
    def yaw_from_quat(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.base = np.array([p.x, p.y, self.yaw_from_quat(msg.pose.pose.orientation)])

    def joint_cb(self, msg):
        if len(msg.position) >= 6:
            self.current_q = np.asarray(msg.position[:6], dtype=float)

    def cloud_cb(self, msg):
        pts = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            if z <= 1.3:
                pts.append((x, y, z))
        if pts:
            arr = np.asarray(pts, dtype=np.float32)
            stride = max(1, int(len(arr) / 30000))
            self.cloud_tree = cKDTree(arr[::stride])

    def marker_cb(self, msg):
        for marker in msg.markers:
            if marker.ns == "charging_port" and marker.id == 1:
                p = marker.pose.position
                self.target = np.array([p.x, p.y, p.z], dtype=float)

    def seed_list(self):
        seeds = [self.current_q.copy(), self.kin.folded.copy()]
        rng = np.random.default_rng(7)
        while len(seeds) < self.seed_count:
            alpha = rng.random()
            noise = rng.normal(0.0, 0.45, 6)
            q = alpha * self.current_q + (1.0 - alpha) * self.kin.folded + noise
            seeds.append(np.clip(q, self.kin.q_min, self.kin.q_max))
        return seeds

    def solve_ik(self, base):
        solutions = []
        if self.target is None:
            return solutions
        for seed in self.seed_list():
            def residual(q):
                pos = self.kin.fingertip(base, q)
                joint_reg = 0.015 * (q - self.kin.folded)
                return np.r_[12.0 * (pos - self.target), joint_reg]

            res = least_squares(
                residual,
                np.clip(seed, self.kin.q_min + 1e-5, self.kin.q_max - 1e-5),
                bounds=(self.kin.q_min, self.kin.q_max),
                max_nfev=120,
                ftol=1e-4,
                xtol=1e-4,
                gtol=1e-4,
            )
            err = float(np.linalg.norm(self.kin.fingertip(base, res.x) - self.target))
            if err <= self.goal_tolerance:
                solutions.append((res.x, err))
        return solutions

    def solution_clearance(self, base, q):
        if self.cloud_tree is None or self.target is None:
            return 0.0, False
        pts, contact = self.kin.collision_points(base, q)
        dists, _ = self.cloud_tree.query(pts, k=1)
        contact_ok = np.logical_and(contact, np.linalg.norm(pts - self.target[None, :], axis=1) <= self.contact_radius)
        checked = ~contact_ok
        if not np.any(checked):
            return self.clearance_clip, True
        clearance = float(np.min(dists[checked]))
        return clearance, clearance >= self.safe_dist

    def compute_score(self):
        if self.base is None or self.target is None or self.cloud_tree is None:
            return None
        solutions = self.solve_ik(self.base)
        clearances = []
        manips = []
        for q, err in solutions:
            clearance, collision_free = self.solution_clearance(self.base, q)
            clearances.append(clearance)
            manips.append(self.kin.joint_margin(q))

        reachability = len(solutions) / max(1, self.seed_count)
        clearance_score = 0.0 if not clearances else float(np.clip(np.mean(clearances) / self.clearance_clip, 0.0, 1.0))
        manipulability = 0.0 if not manips else float(np.mean(manips))
        score = self.w_reach * reachability + self.w_clearance * clearance_score + self.w_manip * manipulability
        best_err = min([err for _, err in solutions], default=float("nan"))
        dist_xy = float(np.linalg.norm(self.base[:2] - self.target[:2]))
        valid_solutions = int(sum(c >= self.safe_dist for c in clearances))
        switch_ready = (
            score >= self.switch_s
            and reachability >= self.switch_reachability
            and (float(np.mean(clearances)) if clearances else 0.0) >= self.switch_clearance
            and manipulability >= self.switch_manipulability
        )
        return {
            "t": time.monotonic() - self.start_time,
            "base_x": self.base[0],
            "base_y": self.base[1],
            "base_yaw": self.base[2],
            "target_dist_xy": dist_xy,
            "solutions": len(solutions),
            "valid_solutions": valid_solutions,
            "reachability": reachability,
            "clearance": float(np.mean(clearances)) if clearances else 0.0,
            "clearance_score": clearance_score,
            "manipulability": manipulability,
            "best_ik_error": best_err,
            "S": score,
            "switch_ready": int(switch_ready),
            "switch_count": self.switch_count,
            "switch_triggered": 0,
        }

    @staticmethod
    def color(r, g, b, a=1.0):
        from std_msgs.msg import ColorRGBA
        return ColorRGBA(r=r, g=g, b=b, a=a)

    def publish_switch_marker(self, row):
        if not self.publish_marker:
            return

        markers = MarkerArray()

        base = Marker()
        base.header.frame_id = "world"
        base.header.stamp = rospy.Time.now()
        base.ns = "arm_affordance_switch"
        base.id = 0
        base.type = Marker.CYLINDER
        base.action = Marker.ADD
        base.pose.position.x = row["base_x"]
        base.pose.position.y = row["base_y"]
        base.pose.position.z = 0.035
        base.pose.orientation.w = 1.0
        base.scale.x = 0.55
        base.scale.y = 0.55
        base.scale.z = 0.07
        base.color = self.color(1.0, 0.08, 0.05, 0.75)
        base.lifetime = rospy.Duration(0)
        markers.markers.append(base)

        arrow = Marker()
        arrow.header.frame_id = "world"
        arrow.header.stamp = base.header.stamp
        arrow.ns = "arm_affordance_switch"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = row["base_x"]
        arrow.pose.position.y = row["base_y"]
        arrow.pose.position.z = 0.12
        arrow.pose.orientation.z = math.sin(row["base_yaw"] * 0.5)
        arrow.pose.orientation.w = math.cos(row["base_yaw"] * 0.5)
        arrow.scale.x = 0.45
        arrow.scale.y = 0.07
        arrow.scale.z = 0.07
        arrow.color = self.color(1.0, 0.08, 0.05, 0.95)
        arrow.lifetime = rospy.Duration(0)
        markers.markers.append(arrow)

        text = Marker()
        text.header.frame_id = "world"
        text.header.stamp = base.header.stamp
        text.ns = "arm_affordance_switch"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = row["base_x"]
        text.pose.position.y = row["base_y"]
        text.pose.position.z = 0.75
        text.pose.orientation.w = 1.0
        text.scale.z = 0.18
        text.color = self.color(0.85, 0.0, 0.0, 1.0)
        text.text = (
            "whole-body start\n"
            f"S={row['S']:.2f}, t={row['t']:.1f}s\n"
            f"({row['base_x']:.2f}, {row['base_y']:.2f})"
        )
        text.lifetime = rospy.Duration(0)
        markers.markers.append(text)

        self.marker_pub.publish(markers)

    def timer_cb(self, _event):
        elapsed = time.monotonic() - self.start_time
        if elapsed > self.duration:
            self.finish()
            rospy.signal_shutdown("affordance monitor complete")
            return
        row = self.compute_score()
        if row is None:
            return
        if row["switch_ready"]:
            self.switch_count += 1
        else:
            self.switch_count = 0
        row["switch_count"] = self.switch_count
        if self.switch_row is None and self.switch_count >= self.switch_hold:
            row["switch_triggered"] = 1
            self.switch_row = dict(row)
            self.publish_switch_marker(row)
            self.publish_retrigger()
            self.switch_pub.publish(Bool(data=True))
            rospy.logwarn(
                "[arm_affordance] WHOLE-BODY SWITCH trigger at t=%.1f base=(%.3f, %.3f, %.2f) S=%.3f reach=%.2f clearance=%.3f manip=%.2f",
                row["t"], row["base_x"], row["base_y"], row["base_yaw"], row["S"],
                row["reachability"], row["clearance"], row["manipulability"],
            )
        self.rows.append(row)
        rospy.loginfo(
            "[arm_affordance] t=%.1f S=%.3f reach=%.2f clearance=%.3f manip=%.2f valid=%d/%d ready=%d hold=%d",
            row["t"], row["S"], row["reachability"], row["clearance"], row["manipulability"],
            row["valid_solutions"], self.seed_count, row["switch_ready"], row["switch_count"],
        )

    def finish(self):
        if not self.rows:
            rospy.logwarn("[arm_affordance] no samples recorded")
            return
        fields = list(self.rows[0].keys())
        with self.csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            t = [r["t"] for r in self.rows]
            plt.figure(figsize=(10, 6))
            plt.plot(t, [r["S"] for r in self.rows], label="S", linewidth=2.5)
            plt.plot(t, [r["reachability"] for r in self.rows], label="reachability")
            plt.plot(t, [r["clearance_score"] for r in self.rows], label="clearance score")
            plt.plot(t, [r["manipulability"] for r in self.rows], label="manipulability")
            plt.axhline(self.switch_s, color="tab:red", linestyle="--", linewidth=1.2, label="switch S threshold")
            if self.switch_row is not None:
                plt.axvline(self.switch_row["t"], color="tab:red", linestyle=":", linewidth=1.6, label="switch trigger")
            plt.xlabel("time [s]")
            plt.ylabel("score")
            plt.ylim(-0.02, 1.05)
            plt.grid(True, alpha=0.3)
            plt.legend()
            plt.tight_layout()
            plt.savefig(self.plot_path, dpi=180)
            rospy.loginfo("[arm_affordance] saved plot to %s", self.plot_path)
        except Exception as exc:
            rospy.logwarn("[arm_affordance] failed to save plot: %s", exc)
        rospy.loginfo("[arm_affordance] saved CSV to %s", self.csv_path)


if __name__ == "__main__":
    rospy.init_node("arm_affordance_monitor")
    monitor = ArmAffordanceMonitor()
    rospy.spin()
