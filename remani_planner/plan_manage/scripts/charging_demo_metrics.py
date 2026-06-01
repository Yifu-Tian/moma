#!/usr/bin/env python3
import json
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rospy
import rospkg
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


class ChargingDemoMetrics:
    def __init__(self):
        self.run_name = rospy.get_param("~run_name", "charging")
        self.variant = rospy.get_param("~variant", "modified")
        self.duration = float(rospy.get_param("~duration", 45.0))
        self.success_threshold = float(rospy.get_param("~success_threshold", 0.08))
        self.map_max_points = int(rospy.get_param("~map_max_points", 18000))

        default_output_dir = Path(rospkg.RosPack().get_path("remani_planner")).parents[3] / "output" / "metrics"
        self.output_dir = Path(rospy.get_param("~output_dir", str(default_output_dir)))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_path = self.output_dir / f"{self.run_name}_{self.variant}_{stamp}.json"

        self.base_samples = []
        self.joint_samples = []
        self.ee_samples = []
        self.map_points = None
        self.robot_markers = {}
        self.target = self.param_vec("charging_demo/goal_position", [-0.60, 0.20, 0.45])
        self.port_surface = self.param_vec("charging_demo/port_surface_position", [-0.60, 0.25, 0.45])
        self.port_contact_radius = float(rospy.get_param("charging_demo/port_contact_radius", 0.08))
        self.start_time = time.monotonic()

        rospy.Subscriber("/mm/car/odom", Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber("/mm/mani/joint_state", JointState, self.joint_cb, queue_size=20)
        rospy.Subscriber("/model_vis/vis_mm", MarkerArray, self.robot_cb, queue_size=5)
        rospy.Subscriber("/charging_demo/markers", MarkerArray, self.charger_cb, queue_size=3)
        rospy.Subscriber("/map_generator/global_cloud", PointCloud2, self.map_cb, queue_size=1)
        rospy.loginfo("[charging_metrics] writing %s", self.output_path)

    @staticmethod
    def param_vec(name, default):
        values = rospy.get_param(name, default)
        if len(values) < len(default):
            return np.asarray(default, dtype=float)
        return np.asarray(values[:len(default)], dtype=float)

    @staticmethod
    def yaw_from_quat(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def elapsed(self):
        return time.monotonic() - self.start_time

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = self.yaw_from_quat(msg.pose.pose.orientation)
        self.base_samples.append((self.elapsed(), p.x, p.y, yaw))

    def joint_cb(self, msg):
        if not msg.position:
            return
        self.joint_samples.append((self.elapsed(), tuple(msg.position)))

    def charger_cb(self, msg):
        for marker in msg.markers:
            if marker.ns == "charging_port" and marker.id == 1:
                p = marker.pose.position
                self.target = np.asarray([p.x, p.y, p.z], dtype=float)

    def map_cb(self, msg):
        pts = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            if z <= 1.25:
                pts.append((x, y, z))
        if not pts:
            return
        arr = np.asarray(pts, dtype=np.float32)
        if len(arr) > self.map_max_points:
            stride = int(math.ceil(len(arr) / float(self.map_max_points)))
            arr = arr[::stride]
        self.map_points = arr

    def robot_cb(self, msg):
        for marker in msg.markers:
            key = (marker.ns, marker.id)
            if marker.action == Marker.DELETE:
                self.robot_markers.pop(key, None)
            elif marker.action == Marker.DELETEALL:
                self.robot_markers.clear()
            else:
                self.robot_markers[key] = marker
        ee = self.ee_position()
        if ee is not None:
            self.ee_samples.append((self.elapsed(), ee[0], ee[1], ee[2]))

    def ee_position(self):
        mesh = [m for m in self.robot_markers.values() if m.type == Marker.MESH_RESOURCE]
        gripper = [m for m in mesh if m.id in (19, 20)]
        if len(gripper) < 2:
            gripper = [m for m in mesh if m.id in (18, 19, 20)]
        if not gripper:
            return None
        pts = np.asarray([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for m in gripper], dtype=float)
        return pts.mean(axis=0)

    def arm_marker_points(self):
        pts = []
        for marker in self.robot_markers.values():
            if marker.type != Marker.MESH_RESOURCE:
                continue
            p = marker.pose.position
            # Ignore base/wheel markers and keep only elevated arm/gripper proxies.
            if p.z < 0.16:
                continue
            pts.append((p.x, p.y, p.z))
        if not pts:
            return None
        return np.asarray(pts, dtype=np.float32)

    def allowed_port_contact_mask(self, pts):
        axis = self.target - self.port_surface
        axis_len_sq = float(axis.dot(axis))
        if axis_len_sq < 1.0e-9:
            return np.linalg.norm(pts - self.target, axis=1) <= self.port_contact_radius
        rel = pts - self.port_surface
        t = np.clip(rel.dot(axis) / axis_len_sq, 0.0, 1.0)
        closest = self.port_surface + t[:, None] * axis
        return np.linalg.norm(pts - closest, axis=1) <= self.port_contact_radius

    def min_arm_obstacle_distance(self):
        arm_pts = self.arm_marker_points()
        if arm_pts is None or self.map_points is None or len(self.map_points) == 0:
            return None

        cloud = self.map_points
        keep = ~self.allowed_port_contact_mask(cloud.astype(float))
        cloud = cloud[keep]
        if len(cloud) == 0:
            return None

        best = float("inf")
        chunk = 3000
        for i in range(0, len(cloud), chunk):
            diff = arm_pts[:, None, :] - cloud[None, i:i + chunk, :]
            dist = np.sqrt(np.sum(diff * diff, axis=2))
            best = min(best, float(dist.min()))
        return best

    @staticmethod
    def base_path_length(samples):
        if len(samples) < 2:
            return 0.0
        total = 0.0
        for a, b in zip(samples[:-1], samples[1:]):
            total += math.hypot(b[1] - a[1], b[2] - a[2])
        return total

    @staticmethod
    def yaw_total_variation(samples):
        if len(samples) < 2:
            return 0.0
        total = 0.0
        for a, b in zip(samples[:-1], samples[1:]):
            total += abs(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])))
        return total

    @staticmethod
    def arm_motion(samples):
        if len(samples) < 2:
            return 0.0
        total = 0.0
        for a, b in zip(samples[:-1], samples[1:]):
            qa = np.asarray(a[1], dtype=float)
            qb = np.asarray(b[1], dtype=float)
            n = min(len(qa), len(qb))
            total += float(np.linalg.norm(qb[:n] - qa[:n], ord=1))
        return total

    def final_ee_error(self):
        if not self.ee_samples:
            return None
        ee = np.asarray(self.ee_samples[-1][1:4], dtype=float)
        return float(np.linalg.norm(ee - self.target))

    def summarize(self):
        final_err = self.final_ee_error()
        min_arm_dist = self.min_arm_obstacle_distance()
        return {
            "run_name": self.run_name,
            "variant": self.variant,
            "success": bool(final_err is not None and final_err <= self.success_threshold),
            "success_threshold_m": self.success_threshold,
            "final_ee_error_m": final_err,
            "min_arm_obstacle_distance_m": min_arm_dist,
            "base_path_length_m": self.base_path_length(self.base_samples),
            "base_yaw_total_variation_rad": self.yaw_total_variation(self.base_samples),
            "arm_motion_l1_rad": self.arm_motion(self.joint_samples),
            "duration_s": self.elapsed(),
            "base_samples": len(self.base_samples),
            "joint_samples": len(self.joint_samples),
            "ee_samples": len(self.ee_samples),
            "target": self.target.tolist(),
            "output": str(self.output_path),
        }

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.elapsed() < self.duration:
            rate.sleep()
        summary = self.summarize()
        with self.output_path.open("w") as f:
            json.dump(summary, f, indent=2, sort_keys=True)
        rospy.loginfo("[charging_metrics] saved %s", self.output_path)
        rospy.loginfo("[charging_metrics] success=%s final_ee_error=%.3f min_arm_obstacle_dist=%s",
                      summary["success"],
                      summary["final_ee_error_m"] if summary["final_ee_error_m"] is not None else float("nan"),
                      "nan" if summary["min_arm_obstacle_distance_m"] is None else f"{summary['min_arm_obstacle_distance_m']:.3f}")


if __name__ == "__main__":
    rospy.init_node("charging_demo_metrics", anonymous=True)
    ChargingDemoMetrics().run()
