"""Rigid-body model of the leader arm (pinocchio), built from its URDF.

The link masses and centres of mass in the URDF are estimates from the CAD, so expect to scale
the gravity torques (``TorqueLaw.gravity_gain``) once the real arm is on the bench.
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin

from ditto_gello_software import leader_urdf

GRAVITY = 9.81


class LeaderModel:
    """``base_rpy``: how the arm's base is turned from upright (roll, pitch, yaw in rad, fixed
    axes), for an arm mounted tilted or on a wall. As in FACTR 2's ``_set_gravity_vector``,
    this turns the gravity vector into the base frame instead of adding a joint."""

    def __init__(self, base_rpy=(0.0, 0.0, 0.0)) -> None:
        self.model = pin.buildModelFromUrdf(str(leader_urdf()))
        self.data = self.model.createData()
        self.nq = self.model.nq
        self.lower = self.model.lowerPositionLimit.copy()
        self.upper = self.model.upperPositionLimit.copy()
        self._bare_last = None  # the last link's own inertia, kept when a point mass is added
        R_world_base = pin.rpy.rpyToMatrix(*np.asarray(base_rpy, dtype=float))
        self.model.gravity.linear = R_world_base.T @ np.array([0.0, 0.0, -GRAVITY])

    def set_point_mass(self, kg: float, at_flange_m=(0.0, 0.0, 0.0)) -> None:
        """Fix a point mass to the last link, at ``at_flange_m`` in the last joint's frame
        (z is along joint 7), in place of any set before. A handle or a test weight; a
        negative mass stands for a support that lifts there. Zero removes it."""
        last = self.model.njoints - 1
        if self._bare_last is None:
            self._bare_last = self.model.inertias[last].copy()
        self.model.inertias[last] = self._bare_last + pin.Inertia(
            float(kg), np.asarray(at_flange_m, dtype=float), np.zeros((3, 3))
        )

    def inverse_dynamics(self, q, qd) -> np.ndarray:
        """Joint torques (N m) that hold the arm against gravity and its velocity-dependent
        forces, with zero acceleration. At rest this is the gravity torque."""
        return pin.rnea(self.model, self.data, q, qd, np.zeros(self.nq)).copy()

    def flange_jacobian(self, q) -> np.ndarray:
        """6 x 7 Jacobian of the last joint's frame, expressed in that frame."""
        return pin.computeJointJacobian(self.model, self.data, q, self.model.njoints - 1)

    def moving_mass(self) -> float:
        """Mass of everything beyond joint 2, kg."""
        return float(sum(self.model.inertias[i].mass for i in range(2, self.model.njoints)))
