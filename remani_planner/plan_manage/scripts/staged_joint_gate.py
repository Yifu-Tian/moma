#!/usr/bin/env python3
import math

import rospy
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Bool


class StagedJointGate:
    def __init__(self):
        self.manipulator_dof = int(rospy.get_param("~manipulator_dof", 6))
        self.init_joint_deg = rospy.get_param("~init_joint_deg", [0.0, 60.0, -120.0, 0.0, 0.0, 0.0])
        self.blend_duration = max(0.0, float(rospy.get_param("~blend_duration", 3.0)))
        self.publish_rate = max(1.0, float(rospy.get_param("~publish_rate", 50.0)))
        self.sync_threshold = max(0.0, float(rospy.get_param("~sync_threshold", 0.08)))
        self.sync_hold_ticks = max(1, int(rospy.get_param("~sync_hold_ticks", 5)))
        self.wait_for_sync = bool(rospy.get_param("~wait_for_sync", True))
        self.init_joint = [float(v) * math.pi / 180.0 for v in self.init_joint_deg[:self.manipulator_dof]]
        while len(self.init_joint) < self.manipulator_dof:
            self.init_joint.append(0.0)

        self.requested_enabled = bool(rospy.get_param("~start_enabled", False))
        self.forwarding = self.requested_enabled
        self.last_raw = None
        self.current_joint = list(self.init_joint)
        self.hold_joint = list(self.init_joint)
        self.blend_start_joint = list(self.init_joint)
        self.blend_start_time = None
        self.sync_count = 0
        self.raw_sub = rospy.Subscriber("~raw_joint_cmd", JointTrajectoryControllerState, self.raw_cb, queue_size=1)
        self.switch_sub = rospy.Subscriber("/arm_affordance/whole_body_enabled", Bool, self.switch_cb, queue_size=1)
        self.pub = rospy.Publisher("~joint_cmd", JointTrajectoryControllerState, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_cb)
        rospy.loginfo("[staged_joint_gate] whole-body starts enabled=%s", self.requested_enabled)

    def switch_cb(self, msg):
        if msg.data and not self.requested_enabled:
            self.forwarding = False
            self.blend_start_time = None
            self.hold_joint = list(self.current_joint)
            self.sync_count = 0
            if self.wait_for_sync:
                rospy.logwarn(
                    "[staged_joint_gate] whole-body requested; waiting for replanned arm command near current pose.",
                )
            else:
                self.forwarding = True
                self.blend_start_joint = list(self.hold_joint)
                self.blend_start_time = rospy.Time.now()
                rospy.logwarn(
                    "[staged_joint_gate] whole-body requested; blending directly to current arm command for %.2fs.",
                    self.blend_duration,
                )
        elif not msg.data:
            self.forwarding = False
            self.blend_start_time = None
            self.hold_joint = list(self.init_joint)
            self.sync_count = 0
        self.requested_enabled = bool(msg.data)

    def make_cmd(self, positions, stamp):
        msg = JointTrajectoryControllerState()
        msg.header.stamp = stamp
        msg.desired.positions = list(positions)
        msg.desired.velocities = [0.0] * self.manipulator_dof
        msg.desired.effort = [0.0] * self.manipulator_dof
        return msg

    def raw_positions(self):
        if self.last_raw is None:
            return None
        positions = list(self.last_raw.desired.positions[:self.manipulator_dof])
        if len(positions) != self.manipulator_dof:
            return None
        return positions

    def smoothstep(self, ratio):
        ratio = max(0.0, min(1.0, ratio))
        return ratio * ratio * (3.0 - 2.0 * ratio)

    def blended_positions(self, now):
        target = self.raw_positions()
        if target is None:
            return list(self.current_joint)
        if self.blend_start_time is None or self.blend_duration <= 0.0:
            return target
        elapsed = (now - self.blend_start_time).to_sec()
        alpha = self.smoothstep(elapsed / self.blend_duration)
        if alpha >= 1.0:
            self.blend_start_time = None
            return target
        return [
            start + alpha * (goal - start)
            for start, goal in zip(self.blend_start_joint, target)
        ]

    def raw_is_synchronized(self, target):
        if target is None:
            return False
        err = math.sqrt(sum((a - b) * (a - b) for a, b in zip(target, self.hold_joint)))
        return err <= self.sync_threshold

    def raw_cb(self, msg):
        self.last_raw = msg

    def timer_cb(self, _event):
        now = rospy.Time.now()
        if self.requested_enabled and not self.forwarding:
            target = self.raw_positions()
            if self.raw_is_synchronized(target):
                self.sync_count += 1
                if self.sync_count >= self.sync_hold_ticks:
                    self.forwarding = True
                    self.blend_start_joint = list(self.hold_joint)
                    self.blend_start_time = now
                    rospy.logwarn(
                        "[staged_joint_gate] synchronized with replanned arm command; blending for %.2fs.",
                        self.blend_duration,
                    )
            else:
                self.sync_count = 0
            self.current_joint = list(self.hold_joint)
        if self.requested_enabled and self.forwarding:
            self.current_joint = self.blended_positions(now)
        else:
            if not self.requested_enabled:
                self.current_joint = list(self.init_joint)
                self.blend_start_time = None
        self.pub.publish(self.make_cmd(self.current_joint, now))


if __name__ == "__main__":
    rospy.init_node("staged_joint_gate")
    StagedJointGate()
    rospy.spin()
