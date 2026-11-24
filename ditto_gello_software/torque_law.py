"""The leader arm's torque law: what makes it float in the hand.

Four terms, each ported from FACTR's teleoperation controller through the ROS-free copy in
GELLO, without the gripper joint and with every term switchable:

- github.com/wuphilipp/gello_software, ``gello/factr/gravity_compensation.py`` (MIT):
  ``gravity_compensation``, ``friction_compensation``, ``null_space_regulation``,
  ``joint_limit_barrier``;
- github.com/RaindragonD/factr_teleop, ``factr_teleop/factr_teleop.py`` (Apache-2.0), which
  those are copies of; its example configuration is where the default gains come from;
- github.com/philiphan0109/factr2_next, ``teacher_arm/base_teleop.py`` (Apache-2.0): a gravity
  gain per joint.

Their licences are in ``licenses/``. Changed from the originals: no gripper joint, no ROS, no
servo driver, every term optional. The functions take and return joint-space arrays (rad,
rad/s, N m) and touch no hardware.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ditto_gello_software.model import LeaderModel


@dataclass
class Gains:
    """Defaults are FACTR's (``configs/franka_example.yaml``), tuned there for a Franka-sized
    leader. Only gravity compensation and the joint-limit barrier start switched on."""

    # Gravity compensation: share of the model's gravity torque that is applied, for all
    # joints, times a factor per joint. FACTR runs 0.85: a little under, so the arm sinks
    # slowly instead of drifting up.
    gravity_gain: float = 0.85
    # Found on the first arm built: its wrist joints need more than the model says.
    gravity_joint_gain: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 1.0, 1.0, 1.0, 1.25, 1.5, 1.0])
    )

    # Stiction dither: below this joint speed, add and subtract a share of the joint's gravity
    # torque on alternate ticks, so the gearbox never settles into static friction.
    stiction: bool = False
    stiction_speed: float = 0.9  # rad/s
    stiction_gain: float = 0.6

    # Null-space regulation: a weak spring towards a resting pose, applied only to the motion
    # that leaves the flange where it is (the elbow's swing).
    null_space: bool = False
    null_space_kp: float = 0.1  # N m/rad
    null_space_kd: float = 0.01  # N m s/rad
    null_space_target: np.ndarray = field(
        default_factory=lambda: np.radians([0.0, 20.0, 0.0, 90.0, 0.0, 70.0, 0.0])
    )

    # Joint-limit barrier: a spring and damper that act only past a limit (less the margin).
    limit_barrier: bool = True
    limit_kp: float = 1.03561658022  # N m/rad
    limit_kd: float = 0.04315069084  # N m s/rad
    limit_margin: float = 0.1  # rad
    # Never more torque than this, and none once a joint is pushed this far past the barrier
    # (until it is back inside): released from deep in, the spring would spin a light joint.
    limit_max_torque: float = 0.2  # N m
    limit_release: float = 0.35  # rad (20 degrees)


class TorqueLaw:
    def __init__(self, model: LeaderModel, gains: Gains | None = None) -> None:
        self.model = model
        self.gains = gains or Gains()
        self.n = model.nq
        self._dither = np.ones(self.n, dtype=bool)
        self._armed = np.zeros(self.n, dtype=bool)  # per joint: the barrier acts on it

    def reset_barrier(self) -> None:
        """Call when torque comes on: a joint outside its limits then (moved there with torque
        off) is not pushed back; its barrier starts once it has been brought inside."""
        self._armed[:] = False

    def joint_limit_barrier(self, q, qd) -> np.ndarray:
        g = self.gains
        upper, lower = self.model.upper - g.limit_margin, self.model.lower + g.limit_margin
        past = np.where(q > upper, q - upper, np.where(q < lower, q - lower, 0.0))
        self._armed = (self._armed | (past == 0.0)) & (np.abs(past) < g.limit_release)
        tau = np.where(self._armed & (past != 0.0), -g.limit_kp * past - g.limit_kd * qd, 0.0)
        return np.clip(tau, -g.limit_max_torque, g.limit_max_torque)

    def gravity_compensation(self, q, qd) -> np.ndarray:
        g = self.gains
        return self.model.inverse_dynamics(q, qd) * g.gravity_gain * g.gravity_joint_gain

    def friction_compensation(self, qd, tau_gravity) -> np.ndarray:
        g = self.gains
        slow = np.abs(qd) < g.stiction_speed
        tau = np.where(self._dither, 1.0, -1.0) * g.stiction_gain * np.abs(tau_gravity) * slow
        self._dither = np.where(slow, ~self._dither, self._dither)
        return tau

    def null_space_regulation(self, q, qd) -> np.ndarray:
        g = self.gains
        J = self.model.flange_jacobian(q)
        projector = np.eye(self.n) - np.linalg.pinv(J) @ J
        return projector @ (-g.null_space_kp * (q - g.null_space_target) - g.null_space_kd * qd)

    def step(self, q, qd) -> tuple[np.ndarray, dict[str, np.ndarray]]:
        """Joint torques for this tick, and the terms they are the sum of."""
        g = self.gains
        parts = {"gravity": self.gravity_compensation(q, qd)}
        if g.limit_barrier:
            parts["limit"] = self.joint_limit_barrier(q, qd)
        if g.null_space:
            parts["null_space"] = self.null_space_regulation(q, qd)
        if g.stiction:
            parts["stiction"] = self.friction_compensation(qd, parts["gravity"])
        return sum(parts.values()), parts
