#!/usr/bin/env python3
import math

import rospy
from rospy.exceptions import ROSException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class MomaRobotStateBridge:
    def __init__(self):
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_yaw = 0.0
        self.arm_positions = [0.0] * 6
        self.pub = rospy.Publisher("/moma_joint_states", JointState, queue_size=1)
        rospy.Subscriber("~odometry", Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber("~joint_state", JointState, self.joint_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.publish)

    def odom_callback(self, msg):
        self.base_x = msg.pose.pose.position.x
        self.base_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.base_yaw = math.atan2(siny_cosp, cosy_cosp)

    def joint_callback(self, msg):
        if len(msg.position) >= 6:
            self.arm_positions = list(msg.position[:6])

    def publish(self, _event):
        if rospy.is_shutdown():
            return
        msg = JointState()
        msg.header = Header(stamp=rospy.Time.now())
        msg.name = [
            "virtual_joint1",
            "virtual_joint2",
            "virtual_joint3",
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        msg.position = [
            self.base_x,
            self.base_y,
            self.base_yaw,
            self.arm_positions[0],
            self.arm_positions[1],
            self.arm_positions[2],
            self.arm_positions[3],
            self.arm_positions[4],
            self.arm_positions[5],
        ]
        try:
            self.pub.publish(msg)
        except ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("moma_robot_state_bridge")
    MomaRobotStateBridge()
    rospy.spin()
