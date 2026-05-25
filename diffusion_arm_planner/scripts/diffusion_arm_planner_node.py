#!/usr/bin/env python3
import math
import time

import rospy

from remani_diffusion_msgs.srv import DiffusionArmPlan, DiffusionArmPlanResponse


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _reshape_row_major(data, rows, cols):
    if rows <= 0 or cols <= 0:
        return []
    if len(data) != rows * cols:
        return []
    matrix = []
    for r in range(rows):
        row = []
        offset = r * cols
        for c in range(cols):
            row.append(data[offset + c])
        matrix.append(row)
    return matrix


def _flatten_row_major(matrix):
    data = []
    for row in matrix:
        data.extend(row)
    return data


def _linear_traj(start_q, goal_q, horizon):
    if horizon <= 1:
        return [start_q[:]]
    traj = []
    for t in range(horizon):
        alpha = float(t) / float(horizon - 1)
        point = []
        for d in range(len(start_q)):
            point.append((1.0 - alpha) * start_q[d] + alpha * goal_q[d])
        traj.append(point)
    return traj


class DummyDiffusionArmPlannerNode(object):
    def __init__(self):
        self.service_name = rospy.get_param("~service_name", "/diffusion_arm_planner/plan")
        self.max_horizon = int(rospy.get_param("~max_horizon", 512))
        self.default_joint_limit = float(rospy.get_param("~default_joint_limit", math.pi))
        self.log_verbose = bool(rospy.get_param("~verbose", False))

        self._service = rospy.Service(self.service_name, DiffusionArmPlan, self._handle_plan)
        rospy.loginfo("[diffusion_arm_planner] service ready at %s", self.service_name)

    def _handle_plan(self, req):
        t0 = time.time()
        resp = DiffusionArmPlanResponse()

        horizon = int(req.horizon)
        dof = int(req.manip_dof)

        if horizon < 2 or horizon > self.max_horizon:
            resp.success = False
            resp.status = "invalid_horizon"
            resp.score = 0.0
            resp.infer_ms = 0.0
            resp.arm_traj_q = []
            return resp

        if dof <= 0:
            resp.success = False
            resp.status = "invalid_dof"
            resp.score = 0.0
            resp.infer_ms = 0.0
            resp.arm_traj_q = []
            return resp

        if len(req.arm_start_q) != dof or len(req.arm_goal_q) != dof:
            resp.success = False
            resp.status = "invalid_start_goal"
            resp.score = 0.0
            resp.infer_ms = 0.0
            resp.arm_traj_q = []
            return resp

        if len(req.base_traj_xyyaw) != horizon * 3:
            resp.success = False
            resp.status = "invalid_base_traj"
            resp.score = 0.0
            resp.infer_ms = 0.0
            resp.arm_traj_q = []
            return resp

        arm_seed = _reshape_row_major(req.arm_seed_q, horizon, dof)
        if arm_seed:
            traj = [row[:] for row in arm_seed]
            status = "ok_seed"
        else:
            traj = _linear_traj(list(req.arm_start_q), list(req.arm_goal_q), horizon)
            status = "ok_linear"

        # Optional dummy "guidance": smooth + simple joint clipping.
        if req.use_guidance and horizon > 2:
            alpha = _clamp(req.w_smooth, 0.0, 1.0)
            for t in range(1, horizon - 1):
                for d in range(dof):
                    smooth_target = 0.5 * (traj[t - 1][d] + traj[t + 1][d])
                    traj[t][d] = (1.0 - alpha) * traj[t][d] + alpha * smooth_target

            joint_clip = max(self.default_joint_limit, req.w_joint_limit if req.w_joint_limit > 0.0 else 0.0)
            for t in range(horizon):
                for d in range(dof):
                    traj[t][d] = _clamp(traj[t][d], -joint_clip, joint_clip)
            status += "_guided"

        # Hard enforce start/goal.
        traj[0] = list(req.arm_start_q)
        traj[horizon - 1] = list(req.arm_goal_q)

        resp.success = True
        resp.status = status
        resp.score = 1.0
        resp.infer_ms = (time.time() - t0) * 1000.0
        resp.arm_traj_q = _flatten_row_major(traj)

        if self.log_verbose:
            rospy.loginfo(
                "[diffusion_arm_planner] request horizon=%d dof=%d -> %s infer_ms=%.3f",
                horizon, dof, resp.status, resp.infer_ms
            )

        return resp


if __name__ == "__main__":
    rospy.init_node("diffusion_arm_planner")
    DummyDiffusionArmPlannerNode()
    rospy.spin()
