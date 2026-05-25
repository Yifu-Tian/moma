#!/usr/bin/env python3
import math
import os
from pathlib import Path
from datetime import datetime

import cv2
import numpy as np
import rospy
import rospkg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


class ChargingSceneRecorder:
    def __init__(self):
        self.width = int(rospy.get_param("~width", 1280))
        self.height = int(rospy.get_param("~height", 800))
        self.fps = float(rospy.get_param("~fps", 20.0))
        self.duration = float(rospy.get_param("~duration", 20.0))
        self.scenario = rospy.get_param("~scenario", "charging")
        self.success_threshold = float(rospy.get_param("~success_threshold", 0.15))
        self.show_execution_trail = bool(rospy.get_param("~show_execution_trail", False))
        self.show_planned_traj = bool(rospy.get_param("~show_planned_traj", True))

        self.x_min = float(rospy.get_param("~x_min", -4.0))
        self.x_max = float(rospy.get_param("~x_max", 4.0))
        self.y_min = float(rospy.get_param("~y_min", -2.5))
        self.y_max = float(rospy.get_param("~y_max", 2.5))

        default_output_dir = self.default_output_dir()
        self.output_dir = Path(rospy.get_param("~output_dir", str(default_output_dir)))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_path = self.output_dir / f"{self.scenario}_{stamp}.mp4"

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.writer = cv2.VideoWriter(str(self.output_path), fourcc, self.fps, (self.width, self.height))
        if not self.writer.isOpened():
            raise RuntimeError(f"failed to open video writer: {self.output_path}")

        self.map_points = None
        self.charger_markers = {}
        self.robot_markers = {}
        self.backend_markers = {}
        self.robot_trail = []
        self.ee_trail = []
        self.planned_base_path = []
        self.planned_ee_path = []
        self.start_base_pos = None
        self.latest_base_pos = None
        self.latest_ee_pos = None
        self.start_time = rospy.Time.now()
        self.frame_count = 0

        rospy.Subscriber("/map_generator/global_cloud", PointCloud2, self.map_callback, queue_size=1)
        rospy.Subscriber("/charging_demo/markers", MarkerArray, self.charger_callback, queue_size=1)
        rospy.Subscriber("/model_vis/vis_mm", MarkerArray, self.robot_callback, queue_size=1)
        rospy.Subscriber("/remani_planner_node/back_end_mm_mesh_vis", MarkerArray, self.backend_callback, queue_size=1)

        rospy.loginfo("[charging_scene_recorder] Recording to %s", self.output_path)

    @staticmethod
    def default_output_dir():
        pkg_path = Path(rospkg.RosPack().get_path("remani_planner"))
        return pkg_path.parents[1] / "output"

    def world_to_px(self, x, y):
        u = int((x - self.x_min) / (self.x_max - self.x_min) * (self.width - 1))
        v = int((self.y_max - y) / (self.y_max - self.y_min) * (self.height - 1))
        return u, v

    def map_callback(self, msg):
        pts = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            if self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max and z <= 1.2:
                pts.append((x, y, z))
        if not pts:
            return
        arr = np.asarray(pts, dtype=np.float32)
        stride = max(1, int(len(arr) / 14000))
        self.map_points = arr[::stride]

    def charger_callback(self, msg):
        self.update_markers(self.charger_markers, msg)

    def robot_callback(self, msg):
        filtered = MarkerArray()
        filtered.markers = [m for m in msg.markers if m.ns != "charging_port"]
        self.update_markers(self.robot_markers, filtered)

    def backend_callback(self, msg):
        self.update_markers(self.backend_markers, msg)
        self.update_planned_paths()

    @staticmethod
    def marker_key(marker):
        return (marker.ns, marker.id)

    def update_markers(self, store, msg):
        for marker in msg.markers:
            key = self.marker_key(marker)
            if marker.action == Marker.DELETE:
                store.pop(key, None)
            elif marker.action == Marker.DELETEALL:
                store.clear()
            else:
                store[key] = marker

    def update_planned_paths(self):
        backend = [m for m in self.backend_markers.values()
                   if m.type == Marker.MESH_RESOURCE and m.ns == "vis_mm_back_end"]
        if not backend:
            return

        base_by_step = {}
        gripper_by_step = {}
        for marker in backend:
            step = marker.id // 100
            part = marker.id % 100
            pos = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
            if part == 0:
                base_by_step[step] = pos
            elif part in (18, 19, 20):
                gripper_by_step.setdefault(step, []).append(pos)

        base_path = [base_by_step[k] for k in sorted(base_by_step.keys())]
        ee_path = []
        for k in sorted(gripper_by_step.keys()):
            pts = gripper_by_step[k]
            if not pts:
                continue
            ee_path.append((
                sum(p[0] for p in pts) / len(pts),
                sum(p[1] for p in pts) / len(pts),
                sum(p[2] for p in pts) / len(pts),
            ))

        if len(base_path) >= 2:
            self.planned_base_path = base_path
        if len(ee_path) >= 2:
            self.planned_ee_path = ee_path

    def draw_grid(self, img):
        img[:] = (248, 248, 248)
        for x in np.arange(math.ceil(self.x_min), self.x_max + 0.01, 0.5):
            u, _ = self.world_to_px(x, self.y_min)
            cv2.line(img, (u, 0), (u, self.height), (225, 225, 225), 1)
        for y in np.arange(math.ceil(self.y_min), self.y_max + 0.01, 0.5):
            _, v = self.world_to_px(self.x_min, y)
            cv2.line(img, (0, v), (self.width, v), (225, 225, 225), 1)

    def draw_map(self, img):
        if self.map_points is None:
            return
        for x, y, z in self.map_points:
            u, v = self.world_to_px(float(x), float(y))
            if 0 <= u < self.width and 0 <= v < self.height:
                color = (0, int(max(40, min(255, 120 + 120 * z))), 255)
                cv2.circle(img, (u, v), 1, color, -1)

    def draw_marker(self, img, marker, color_override=None, alpha=1.0):
        color = color_override or self.marker_color(marker)
        if marker.type == Marker.LINE_STRIP:
            pts = [self.world_to_px(p.x, p.y) for p in marker.points]
            if len(pts) >= 2:
                cv2.polylines(img, [np.asarray(pts, dtype=np.int32)], False, color, 2)
            return
        if marker.type == Marker.LINE_LIST:
            pts = [self.world_to_px(p.x, p.y) for p in marker.points]
            for a, b in zip(pts[0::2], pts[1::2]):
                cv2.line(img, a, b, color, 2)
            return
        if marker.type == Marker.POINTS:
            for p in marker.points:
                cv2.circle(img, self.world_to_px(p.x, p.y), 2, color, -1)
            return
        if marker.type == Marker.ARROW and len(marker.points) >= 2:
            a = self.world_to_px(marker.points[0].x, marker.points[0].y)
            b = self.world_to_px(marker.points[1].x, marker.points[1].y)
            cv2.arrowedLine(img, a, b, color, 2, tipLength=0.25)
            return

        x = marker.pose.position.x
        y = marker.pose.position.y
        u, v = self.world_to_px(x, y)
        if marker.type == Marker.CUBE:
            sx = max(5, int(abs(marker.scale.x) / (self.x_max - self.x_min) * self.width))
            sy = max(5, int(abs(marker.scale.y) / (self.y_max - self.y_min) * self.height))
            cv2.rectangle(img, (u - sx // 2, v - sy // 2), (u + sx // 2, v + sy // 2), color, -1)
            cv2.rectangle(img, (u - sx // 2, v - sy // 2), (u + sx // 2, v + sy // 2), (30, 30, 30), 1)
        elif marker.type == Marker.SPHERE:
            r = max(4, int(max(marker.scale.x, marker.scale.y) / (self.x_max - self.x_min) * self.width * 0.5))
            cv2.circle(img, (u, v), r, color, -1)
            cv2.circle(img, (u, v), r, (30, 30, 30), 1)
        else:
            r = 5 if marker.type == Marker.MESH_RESOURCE else 3
            cv2.circle(img, (u, v), r, color, -1)

    def draw_label(self, img, text, anchor, offset=(8, -8), color=(30, 30, 30)):
        x = int(anchor[0] + offset[0])
        y = int(anchor[1] + offset[1])
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.42
        thickness = 1
        (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
        x = max(2, min(self.width - tw - 2, x))
        y = max(th + 2, min(self.height - baseline - 2, y))
        cv2.putText(img, text, (x, y), font, scale, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)

    @staticmethod
    def draw_dashed_polyline(img, pts, color, thickness=2, dash=10, gap=7):
        if len(pts) < 2:
            return
        pts = [tuple(map(int, p)) for p in pts]
        for a, b in zip(pts[:-1], pts[1:]):
            ax, ay = a
            bx, by = b
            length = math.hypot(bx - ax, by - ay)
            if length < 1.0:
                continue
            vx = (bx - ax) / length
            vy = (by - ay) / length
            dist = 0.0
            while dist < length:
                start = dist
                end = min(length, dist + dash)
                p0 = (int(ax + vx * start), int(ay + vy * start))
                p1 = (int(ax + vx * end), int(ay + vy * end))
                cv2.line(img, p0, p1, color, thickness)
                dist += dash + gap

    def draw_charger(self, img):
        for marker in self.charger_markers.values():
            self.draw_marker(img, marker)

        charger = self.charger_markers.get(("charging_port", 0))
        target = self.charger_markers.get(("charging_port", 1))
        if charger is not None:
            p = self.world_to_px(charger.pose.position.x, charger.pose.position.y)
            self.draw_label(img, "charger", p, offset=(10, -12), color=(0, 95, 150))
        if target is not None:
            p = self.world_to_px(target.pose.position.x, target.pose.position.y)
            cv2.circle(img, p, 8, (40, 210, 40), 2)
            self.draw_label(img, "target", p, offset=(10, 20), color=(20, 120, 40))

    def target_position(self):
        target = self.charger_markers.get(("charging_port", 1))
        if target is None:
            return None
        p = target.pose.position
        return (p.x, p.y, p.z)

    def ee_target_distance(self):
        target = self.target_position()
        if target is None or self.latest_ee_pos is None:
            return None
        dx = self.latest_ee_pos[0] - target[0]
        dy = self.latest_ee_pos[1] - target[1]
        dz = self.latest_ee_pos[2] - target[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def is_success(self):
        dist = self.ee_target_distance()
        if dist is None:
            return False
        return dist <= self.success_threshold

    @staticmethod
    def marker_color(marker):
        b = int(max(0, min(255, marker.color.b * 255)))
        g = int(max(0, min(255, marker.color.g * 255)))
        r = int(max(0, min(255, marker.color.r * 255)))
        if r == 0 and g == 0 and b == 0:
            return (120, 120, 120)
        return (b, g, r)

    def draw_robot(self, img):
        robot = [m for m in self.robot_markers.values() if m.type == Marker.MESH_RESOURCE]
        if robot:
            pts = [(m.pose.position.x, m.pose.position.y, m.pose.position.z) for m in robot]
            pts.sort(key=lambda p: p[2])
            xy = [self.world_to_px(p[0], p[1]) for p in pts]
            if len(xy) >= 2:
                cv2.polylines(img, [np.asarray(xy, dtype=np.int32)], False, (65, 65, 65), 3)
            for i, p in enumerate(xy):
                radius = 4
                color = (70, 70, 70)
                if i == 0:
                    radius = 12
                    color = (95, 95, 95)
                elif i == len(xy) - 1:
                    radius = 8
                    color = (255, 80, 80)
                cv2.circle(img, p, radius, color, -1)
                cv2.circle(img, p, radius, (30, 30, 30), 1)

            base = min(pts, key=lambda p: p[2])
            ee = self.end_effector_position(robot)
            if self.start_base_pos is None:
                self.start_base_pos = base
            self.latest_base_pos = base
            self.latest_ee_pos = ee
            self.robot_trail.append((base[0], base[1]))
            self.ee_trail.append((ee[0], ee[1]))
            self.robot_trail = self.robot_trail[-600:]
            self.ee_trail = self.ee_trail[-600:]

            self.draw_label(img, "base", self.world_to_px(base[0], base[1]),
                            offset=(14, -14), color=(60, 60, 60))
            self.draw_label(img, "end-effector", self.world_to_px(ee[0], ee[1]),
                            offset=(12, -14), color=(170, 45, 45))

            if self.start_base_pos is not None:
                start_px = self.world_to_px(self.start_base_pos[0], self.start_base_pos[1])
                cv2.circle(img, start_px, 9, (40, 120, 255), 2)
                self.draw_label(img, "start", start_px, offset=(10, 18), color=(35, 80, 180))

        for marker in self.robot_markers.values():
            if marker.type != Marker.MESH_RESOURCE:
                self.draw_marker(img, marker)

    @staticmethod
    def end_effector_position(robot_markers):
        # For FastArmer with idx=0, gripper mesh ids are 18/19/20.
        # Use the gripper center when available. Falling back to the highest
        # marker keeps the recorder usable for other manipulators.
        gripper = [m for m in robot_markers if m.id in (18, 19, 20)]
        if gripper:
            x = sum(m.pose.position.x for m in gripper) / len(gripper)
            y = sum(m.pose.position.y for m in gripper) / len(gripper)
            z = sum(m.pose.position.z for m in gripper) / len(gripper)
            return (x, y, z)
        pts = [(m.pose.position.x, m.pose.position.y, m.pose.position.z) for m in robot_markers]
        return max(pts, key=lambda p: p[2])

    def draw_trails(self, img):
        if self.show_execution_trail and len(self.robot_trail) >= 2:
            pts = np.asarray([self.world_to_px(x, y) for x, y in self.robot_trail], dtype=np.int32)
            self.draw_dashed_polyline(img, pts, (125, 125, 125), 2)
        if self.show_execution_trail and len(self.ee_trail) >= 2:
            pts = np.asarray([self.world_to_px(x, y) for x, y in self.ee_trail], dtype=np.int32)
            cv2.polylines(img, [pts], False, (255, 80, 80), 3)

    def draw_planned_traj(self, img):
        if not self.show_planned_traj:
            return
        if len(self.planned_base_path) >= 2:
            pts = np.asarray([self.world_to_px(p[0], p[1]) for p in self.planned_base_path], dtype=np.int32)
            cv2.polylines(img, [pts], False, (70, 70, 70), 3)
            cv2.circle(img, tuple(pts[0]), 5, (70, 70, 70), -1)
            cv2.circle(img, tuple(pts[-1]), 6, (40, 160, 40), -1)
        if len(self.planned_ee_path) >= 2:
            pts = np.asarray([self.world_to_px(p[0], p[1]) for p in self.planned_ee_path], dtype=np.int32)
            cv2.polylines(img, [pts], False, (255, 80, 80), 3)
            cv2.circle(img, tuple(pts[-1]), 6, (255, 80, 80), -1)

    def draw_legend(self, img):
        x0 = self.width - 315
        y0 = 22
        w = 285
        h = 168
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (255, 255, 255), -1)
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (185, 185, 185), 1)
        cv2.putText(img, "Legend", (x0 + 14, y0 + 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (30, 30, 30), 2, cv2.LINE_AA)

        rows = [
            ("base", (95, 95, 95), "circle"),
            ("end-effector", (255, 80, 80), "circle"),
            ("planned base path", (70, 70, 70), "line"),
            ("planned ee path", (255, 80, 80), "line"),
            ("execution trail hidden", (125, 125, 125), "dashed"),
            ("target / charger", (40, 210, 40), "target"),
        ]
        y = y0 + 50
        for label, color, kind in rows:
            x = x0 + 18
            if kind == "circle":
                cv2.circle(img, (x + 12, y - 4), 7, color, -1)
                cv2.circle(img, (x + 12, y - 4), 7, (30, 30, 30), 1)
            elif kind == "dashed":
                self.draw_dashed_polyline(img, [(x, y - 4), (x + 34, y - 4)], color, 2, dash=8, gap=5)
            elif kind == "target":
                cv2.rectangle(img, (x, y - 11), (x + 28, y + 3), (255, 190, 20), -1)
                cv2.circle(img, (x + 34, y - 4), 7, color, 2)
            else:
                cv2.line(img, (x, y - 4), (x + 34, y - 4), color, 3)
            cv2.putText(img, label, (x0 + 68, y + 1),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (45, 45, 45), 1, cv2.LINE_AA)
            y += 20

    def draw_metrics(self, img):
        dist = self.ee_target_distance()
        if dist is None:
            text = "ee-target: waiting"
            color = (85, 85, 85)
        else:
            ok = dist <= self.success_threshold
            text = "ee-target: %.3f m   threshold: %.2f m   success: %s" % (
                dist, self.success_threshold, "YES" if ok else "NO")
            color = (20, 125, 45) if ok else (40, 40, 190)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, text, (24, 66), font, 0.56, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(img, text, (24, 66), font, 0.56, color, 1, cv2.LINE_AA)

        if self.start_base_pos is None:
            start_text = "start base: waiting"
        else:
            start_text = "start base: x=%.2f m, y=%.2f m, yaw from launch" % (
                self.start_base_pos[0], self.start_base_pos[1])
        cv2.putText(img, start_text, (24, 90), font, 0.50, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(img, start_text, (24, 90), font, 0.50, (55, 55, 55), 1, cv2.LINE_AA)

    def draw_axes(self, img):
        origin = (70, self.height - 78)
        axis_len = 70
        x_end = (origin[0] + axis_len, origin[1])
        y_end = (origin[0], origin[1] - axis_len)

        cv2.circle(img, origin, 3, (40, 40, 40), -1)
        cv2.arrowedLine(img, origin, x_end, (40, 40, 190), 2, tipLength=0.20)
        cv2.arrowedLine(img, origin, y_end, (30, 130, 40), 2, tipLength=0.20)
        cv2.putText(img, "+x", (x_end[0] + 8, x_end[1] + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (40, 40, 190), 1, cv2.LINE_AA)
        cv2.putText(img, "+y", (y_end[0] - 12, y_end[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (30, 130, 40), 1, cv2.LINE_AA)
        cv2.putText(img, "world xy", (origin[0] - 28, origin[1] + 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (70, 70, 70), 1, cv2.LINE_AA)

    def render_frame(self):
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.draw_grid(img)
        self.draw_map(img)

        self.draw_planned_traj(img)

        self.draw_charger(img)

        self.draw_trails(img)
        self.draw_robot(img)
        self.draw_legend(img)
        self.draw_metrics(img)
        self.draw_axes(img)

        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        cv2.putText(img, f"{self.scenario}  t={elapsed:04.1f}s", (24, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(img, "Top-down view: x/y plane, z is ignored for projection", (24, self.height - 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 60, 60), 1, cv2.LINE_AA)
        return img

    def run(self):
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            if elapsed >= self.duration:
                break
            self.writer.write(self.render_frame())
            self.frame_count += 1
            rate.sleep()

        self.writer.release()
        rospy.loginfo("[charging_scene_recorder] Saved %d frames to %s", self.frame_count, self.output_path)
        self.log_final_metrics()

    def log_final_metrics(self):
        target = self.target_position()
        dist = self.ee_target_distance()
        success = self.is_success()

        if self.latest_base_pos is None:
            rospy.logwarn("[charging_scene_recorder] Final base position unavailable.")
        else:
            rospy.loginfo("[charging_scene_recorder] Final base position: x=%.3f y=%.3f z=%.3f",
                          self.latest_base_pos[0], self.latest_base_pos[1], self.latest_base_pos[2])

        if self.start_base_pos is None:
            rospy.logwarn("[charging_scene_recorder] Start base position unavailable.")
        else:
            rospy.loginfo("[charging_scene_recorder] Start base position: x=%.3f y=%.3f z=%.3f",
                          self.start_base_pos[0], self.start_base_pos[1], self.start_base_pos[2])

        if self.latest_ee_pos is None:
            rospy.logwarn("[charging_scene_recorder] Final end-effector position unavailable.")
        else:
            rospy.loginfo("[charging_scene_recorder] Final end-effector position: x=%.3f y=%.3f z=%.3f",
                          self.latest_ee_pos[0], self.latest_ee_pos[1], self.latest_ee_pos[2])

        if target is None:
            rospy.logwarn("[charging_scene_recorder] Target position unavailable.")
        else:
            rospy.loginfo("[charging_scene_recorder] Target position: x=%.3f y=%.3f z=%.3f",
                          target[0], target[1], target[2])

        if dist is None:
            rospy.logwarn("[charging_scene_recorder] Final ee-target distance unavailable.")
        else:
            rospy.loginfo("[charging_scene_recorder] Final ee-target distance: %.3f m", dist)
            rospy.loginfo("[charging_scene_recorder] Success threshold: %.3f m", self.success_threshold)
            rospy.loginfo("[charging_scene_recorder] Success: %s", "true" if success else "false")


if __name__ == "__main__":
    rospy.init_node("charging_scene_recorder")
    recorder = ChargingSceneRecorder()
    recorder.run()
