import numpy as np
import mujoco

class SafetyLimits:
    """
    Full safety suite for Franka Panda teleoperation.
    Implements joint limits, Cartesian workspace limits,
    velocity limits, force limits and emergency stop.
    """

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.estop = False
        self.estop_reason = ""

        # ── Joint limits (hard) ──────────────────────────────
        self.joint_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        self.joint_max = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

        # Soft limit margin
        self.joint_soft_margin = 0.1  # radians

        # ── Joint velocity limits ────────────────────────────
        self.max_joint_velocity = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])  # rad/s

        # ── Cartesian workspace limits ───────────────────────
        self.workspace_min = np.array([0.0,  -0.7, 0.1])
        self.workspace_max = np.array([0.85,  0.7, 1.2])

        # ── Force limits ─────────────────────────────────────
        self.max_external_force = 200.0   # Newtons
        self.max_external_torque = 500.0  # Nm

        # ── Velocity smoothing ───────────────────────────────
        self.prev_target_qpos = None

        print("Safety limits initialized")
        print(f"Workspace: x={self.workspace_min[0]:.2f}-{self.workspace_max[0]:.2f} "
              f"y={self.workspace_min[1]:.2f}-{self.workspace_max[1]:.2f} "
              f"z={self.workspace_min[2]:.2f}-{self.workspace_max[2]:.2f}")

    def trigger_estop(self, reason="Manual trigger"):
        """Trigger emergency stop"""
        self.estop = True
        self.estop_reason = reason
        print(f"⚠️  EMERGENCY STOP: {reason}")

    def reset_estop(self):
        """Reset emergency stop — only call when safe to do so"""
        self.estop = False
        self.estop_reason = ""
        print("Emergency stop reset")

    def is_estop(self):
        return self.estop

    def check_joint_limits(self, target_qpos):
        """
        Check and enforce joint limits with soft margins.
        Returns clamped target_qpos and whether any limit was hit.
        """
        clamped = target_qpos.copy()
        limit_hit = False

        for i in range(7):
            clamped[i] = np.clip(clamped[i], self.joint_min[i], self.joint_max[i])

            if (clamped[i] > self.joint_max[i] - self.joint_soft_margin or
                clamped[i] < self.joint_min[i] + self.joint_soft_margin):
                limit_hit = True

        return clamped, limit_hit

    def check_joint_velocity(self, target_qpos, dt=0.002):
        """
        Limit joint velocity by clamping the rate of change of target_qpos.
        Returns velocity-limited target_qpos.
        """
        if self.prev_target_qpos is None:
            self.prev_target_qpos = target_qpos.copy()
            return target_qpos

        desired_vel = (target_qpos - self.prev_target_qpos) / dt
        clamped_vel = np.clip(desired_vel, -self.max_joint_velocity, self.max_joint_velocity)
        clamped_target = self.prev_target_qpos + clamped_vel * dt
        self.prev_target_qpos = clamped_target.copy()

        return clamped_target

    def check_workspace(self, ee_pos):
        """
        Check if end-effector is within workspace limits.
        Returns True if within limits, False if violated.
        """
        if np.any(ee_pos < self.workspace_min) or np.any(ee_pos > self.workspace_max):
            return False
        return True

    def check_force_limits(self, ee_force, ee_torque):
        """
        Check if external forces exceed safe limits.
        Triggers estop if exceeded.
        Returns True if safe, False if limit exceeded.
        """
        force_mag = np.linalg.norm(ee_force)
        torque_mag = np.linalg.norm(ee_torque)

        if force_mag > self.max_external_force:
            self.trigger_estop(f"Force limit exceeded: {force_mag:.1f}N > {self.max_external_force}N")
            return False

        if torque_mag > self.max_external_torque:
            self.trigger_estop(f"Torque limit exceeded: {torque_mag:.1f}Nm > {self.max_external_torque}Nm")
            return False

        return True

    def apply(self, target_qpos, ee_body_id, ee_force=None, ee_torque=None):
        """
        Main safety check — call this every frame before applying target_qpos.
        Returns safe target_qpos and a status dict.
        """
        status = {
            "estop": False,
            "joint_limit": False,
            "workspace_violation": False,
            "force_limit": False,
        }

        # Emergency stop — hold current position
        if self.estop:
            status["estop"] = True
            return self.data.qpos[:7].copy(), status

        # Force limits
        if ee_force is not None and ee_torque is not None:
            if not self.check_force_limits(ee_force, ee_torque):
                status["force_limit"] = True
                return self.data.qpos[:7].copy(), status

        # Workspace check
        ee_pos = self.data.xpos[ee_body_id].copy()
        if not self.check_workspace(ee_pos):
            status["workspace_violation"] = True
            return self.data.qpos[:7].copy(), status

        # Joint velocity limits
        safe_qpos = self.check_joint_velocity(target_qpos)

        # Joint position limits
        safe_qpos, joint_limit_hit = self.check_joint_limits(safe_qpos)
        if joint_limit_hit:
            status["joint_limit"] = True

        return safe_qpos, status