"""Record a motion of the arm by hand, then play it back with a spring and damper per joint.

Playback commands torque, not position: each joint is pulled towards the recorded angle by

    torque = stiffness * kp * (recorded angle - angle) + sqrt(stiffness) * kd * (recorded speed - speed)

so ``stiffness`` sets how hard the arm insists on the recording and how hard it pushes back
when held (the damping follows it so that the joint stays as well damped). Unless the loop
adds gravity compensation, the springs carry the arm's weight, which shows as a steady lag
behind the recording that shrinks as the stiffness goes up.

The speed in that formula is worked out here from the change in angle between ticks. The
servos' own speed reading arrives about 20 ms late (measured on the first arm), which is a
quarter of a cycle at the 10 Hz these joints ring at: damping built on it does not damp, and
the first playback, which used it, shook itself apart.

Nothing here touches hardware: ``gravity_comp`` runs these inside its loop.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class TrackingGains:
    """``kp`` (N m/rad) and ``kd`` (N m s/rad) per joint, at stiffness 1.

    The defaults come from each joint's inertia as seen at the joint, gearbox included, fitted
    from the first arm's log (``JOINT_INERTIA``, good to perhaps a factor of two): each joint
    is given a natural frequency of ``NATURAL_HZ`` and a damping ratio of ``DAMPING_RATIO``.
    That frequency is kept low because the loop reacts some 8 ms late; stiffness 2 raises it
    by 1.4. These springs are soft against gravity (about 0.4 N m at J2), so without gravity
    compensation expect the shoulder to hang 10 to 15 degrees behind the recording."""

    kp: np.ndarray = field(default_factory=lambda: JOINT_INERTIA * (2.0 * np.pi * NATURAL_HZ) ** 2)
    kd: np.ndarray = field(
        default_factory=lambda: 2.0 * DAMPING_RATIO * JOINT_INERTIA * 2.0 * np.pi * NATURAL_HZ
    )
    stiffness: float = 1.0
    speed_cutoff_hz: float = 30.0  # low-pass on the speed worked out from the angles
    # A joint moving faster than this times the recording's own top speed, plus the margin,
    # stops the playback: the arm is running away or ringing, not following.
    speed_factor: float = 2.0
    speed_margin: float = 1.0  # rad/s
    max_error: float = 0.6  # rad: a joint this far from the recording stops the playback
    max_start: float = 1.0  # rad: playback will not start from further than this
    approach_speed: float = 0.5  # rad/s: how fast the arm is brought to the recording's start
    approach_min_s: float = 1.5
    ramp_s: float = 0.5  # the springs come on over this long


JOINT_INERTIA = np.array([1.0, 1.7, 0.9, 1.6, 0.25, 0.35, 0.1]) * 1e-3  # kg m^2, J1 to J7
NATURAL_HZ = 4.8
DAMPING_RATIO = 0.8


class Trajectory:
    """Joint angles and speeds against time, from zero."""

    def __init__(self, t, q, qd) -> None:
        self.t = np.asarray(t, dtype=float) - float(t[0])
        self.q, self.qd = np.asarray(q, dtype=float), np.asarray(qd, dtype=float)
        self.duration = float(self.t[-1])

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """Angles and speeds at time ``t``, between the two recorded samples around it."""
        k = int(np.clip(np.searchsorted(self.t, t), 1, len(self.t) - 1))
        x = float(np.clip((t - self.t[k - 1]) / (self.t[k] - self.t[k - 1]), 0.0, 1.0))
        return (self.q[k - 1] + x * (self.q[k] - self.q[k - 1]),
                self.qd[k - 1] + x * (self.qd[k] - self.qd[k - 1]))

    def save(self, path) -> None:
        np.savez_compressed(path, t=self.t, q=self.q, qd=self.qd)

    @classmethod
    def load(cls, path) -> Trajectory:
        d = np.load(path)
        return cls(d["t"], d["q"], d["qd"])


class Recorder:
    PERIOD_S = 0.01  # one sample per this long
    MAX_S = 300.0  # the longest recording

    def __init__(self) -> None:
        self._t: list[float] = []
        self._q: list[np.ndarray] = []
        self._qd: list[np.ndarray] = []

    @property
    def seconds(self) -> float:
        return self._t[-1] - self._t[0] if self._t else 0.0

    def add(self, t: float, state) -> bool:
        """Take this tick's state if a sample is due. False once the recording is full."""
        if self._t and t - self._t[-1] < self.PERIOD_S - 1e-6:
            return True
        if self.seconds >= self.MAX_S:
            return False
        self._t.append(t)
        self._q.append(state.q.copy())
        self._qd.append(state.qd.copy())
        return True

    def trajectory(self) -> Trajectory | None:
        """What was recorded, or None if it is shorter than half a second."""
        if self.seconds < 0.5:
            return None
        t, q = np.array(self._t), np.array(self._q)
        # Speeds from the recorded angles (smoothed over 50 ms), not the servos' late reading.
        qd = np.gradient(q, t, axis=0)
        kernel = np.ones(5) / 5.0
        qd = np.column_stack([np.convolve(np.pad(qd[:, j], 2, mode="edge"), kernel, mode="valid")
                              for j in range(q.shape[1])])
        return Trajectory(t, q, qd)


class Player:
    """Plays a :class:`Trajectory`: first brings the arm from where it is to the recording's
    first pose, then follows the recording. :meth:`step` returns this tick's torques."""

    def __init__(self, gains: TrackingGains) -> None:
        self.gains = gains
        self.trajectory: Trajectory | None = None
        self.done, self.fault = True, ""
        self.time = 0.0  # seconds into the recording (0 during the approach)
        self.approaching = False
        self.error = np.zeros(len(gains.kp))  # recorded angle - angle, this tick

    def start(self, t: float, q_now, trajectory: Trajectory) -> str:
        """Begin playback at time ``t``. Returns why not, or "" if it began."""
        g = self.gains
        away = trajectory.q[0] - np.asarray(q_now)
        j = int(np.argmax(np.abs(away)))
        if abs(away[j]) > g.max_start:
            return (f"J{j + 1} is {np.degrees(abs(away[j])):.0f}° from where the recording "
                    f"starts (limit {np.degrees(g.max_start):.0f}°): move the arm closer first")
        self.trajectory, self._t0, self._from = trajectory, t, np.array(q_now, dtype=float)
        self._approach_s = max(g.approach_min_s, float(np.abs(away).max()) / g.approach_speed)
        self.done, self.fault, self.time, self.approaching = False, "", 0.0, True
        self._speed_limit = (g.speed_factor * np.maximum(np.abs(trajectory.qd).max(axis=0), g.approach_speed)
                             + g.speed_margin)
        self._q_prev, self._t_prev, self._speed = np.array(q_now, dtype=float), t, np.zeros(len(away))
        self._sum_sq, self._worst, self._count = np.zeros(len(away)), np.zeros(len(away)), 0
        return ""

    def step(self, t: float, state) -> tuple[np.ndarray, np.ndarray]:
        """Torques (N m) for this tick and the angles they pull towards. Sets ``done`` when
        the recording has been played, and ``fault`` if a joint is too far from it."""
        g, traj = self.gains, self.trajectory
        step = t - self._t_prev
        if step > 0.0:  # this joint speed: change in angle, through a first-order low-pass
            raw = (state.q - self._q_prev) / step
            blend = step / (step + 1.0 / (2.0 * np.pi * g.speed_cutoff_hz))
            self._speed = self._speed + blend * (raw - self._speed)
            self._q_prev, self._t_prev = state.q.copy(), t
        dt = t - self._t0
        self.approaching = dt < self._approach_s
        if not self.approaching:
            self.time = dt - self._approach_s
            if self.time >= traj.duration:
                self.done = True
        q_ref, qd_ref = self.reference(t)
        self.error = q_ref - state.q
        if not self.approaching:
            self._sum_sq += self.error**2
            self._worst = np.maximum(self._worst, np.abs(self.error))
            self._count += 1
        j = int(np.argmax(np.abs(self.error)))
        if abs(self.error[j]) > g.max_error:
            self.fault = (f"J{j + 1} was {np.degrees(abs(self.error[j])):.0f}° from the recording "
                          f"(limit {np.degrees(g.max_error):.0f}°)")
        fast = np.abs(self._speed) > self._speed_limit
        if fast.any():
            j = int(np.argmax(np.abs(self._speed) / self._speed_limit))
            self.fault = (f"J{j + 1} moved at {abs(self._speed[j]):.1f} rad/s, more than the "
                          f"recording allows ({self._speed_limit[j]:.1f})")
        ramp = min(1.0, dt / g.ramp_s)
        tau = ramp * (g.stiffness * g.kp * self.error
                      + np.sqrt(g.stiffness) * g.kd * (qd_ref - self._speed))
        return tau, q_ref

    def reference(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """Where the arm should be at time ``t``, and how fast: the smooth move to the
        recording's first pose, then the recording."""
        traj, dt = self.trajectory, t - self._t0
        if dt < self._approach_s:
            x = max(0.0, dt / self._approach_s)
            return (self._from + (traj.q[0] - self._from) * x * x * (3.0 - 2.0 * x),
                    (traj.q[0] - self._from) * 6.0 * x * (1.0 - x) / self._approach_s)
        return traj.sample(dt - self._approach_s)

    def errors(self) -> tuple[np.ndarray, np.ndarray]:
        """Typical (root mean square) and largest error per joint over the recording so far,
        degrees."""
        count = max(self._count, 1)
        return np.degrees(np.sqrt(self._sum_sq / count)), np.degrees(self._worst)

    def report(self, how: str) -> str:
        """Markdown table of how closely the recording was followed; ``how`` names the
        controller that did it."""
        if not self._count:
            return ""
        rms, worst = self.errors()
        rows = [f"**{how}**: followed {self.time:.1f} s of {self.trajectory.duration:.1f} s.", "",
                "| J | typical error ° | largest ° |", "|---|---|---|"]
        rows += [f"| {j + 1} | {rms[j]:.1f} | {worst[j]:.1f} |" for j in range(len(rms))]
        return "\n".join(rows)
