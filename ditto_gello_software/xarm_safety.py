"""What may be sent to the xArm7: every gello joint vector passes this gate first.

    gate = XArmGate()
    ok, why = gate.engage(gello_q, xarm_q)       # once, with the xArm's measured joints
    q = gate.step(t, gello_q)                    # every sample: joints to send, or None

``gello_q`` is always gello joints (``gello_from_raw`` of what the encoders read), never
raw_gello. Joint streaming sends ``q``; pose streaming sends the TCP pose of ``q`` (the xArm7's
forward kinematics), so both go through the same checks.

The gate engages only when the gello matches the xArm7 joint for joint. After that, a sample
is passed on only if it is finite and fresh, inside the xArm7's joint limits less a margin,
and no joint has moved faster than ``max_speed`` since the last one. Anything else is a safety
stop: nothing more is passed on, the xArm7 keeps its last command (mini_manipulation_env stops
it when commands stop coming, ``command_timeout_s``), and only a new ``engage`` restarts the
stream. A joint read a turn off, a dropped servo read or a hand slipping all end the stream
instead of moving the xArm7.
"""

from __future__ import annotations

import numpy as np
import yourdfpy

from ditto_gello_software import XARM_URDF


def xarm_limits() -> tuple[np.ndarray, np.ndarray]:
    """The xArm7's joint limits (rad), from its URDF."""
    u = yourdfpy.URDF.load(str(XARM_URDF), load_meshes=False)
    lim = [u.joint_map[n].limit for n in u.actuated_joint_names]
    return np.array([x.lower for x in lim]), np.array([x.upper for x in lim])


class XArmGate:
    def __init__(self, margin: float = np.radians(2.0), max_speed: float = 2.0,
                 engage_tol: float = np.radians(2.0), stale_s: float = 0.05,
                 noise: float = np.radians(0.3)) -> None:
        """``margin`` (rad): how far inside the xArm7's joint limits every joint must stay.
        ``max_speed`` (rad/s): the fastest a joint may move from one sample to the next (the
        xArm7's own limit is 3.14). ``engage_tol`` (rad): how close each gello joint must be to
        the xArm7's to engage. ``stale_s``: the longest gap between samples. ``noise`` (rad):
        a step this small is never called too fast (encoder counts, timing jitter)."""
        lower, upper = xarm_limits()
        self.lower, self.upper = lower + margin, upper - margin
        self.max_speed, self.engage_tol, self.stale_s, self.noise = max_speed, engage_tol, stale_s, noise
        self.engaged = False
        self.stop_reason = "not engaged"
        self._q = self._t = None

    def _outside(self, q) -> str:
        if not np.all(np.isfinite(q)):
            return "a joint is not a number"
        out = (q < self.lower) | (q > self.upper)
        if out.any():
            j = int(np.argmax(out))
            return (f"J{j + 1} at {np.degrees(q[j]):.1f}° is outside the xArm7's limits "
                    f"({np.degrees(self.lower[j]):.1f}° to {np.degrees(self.upper[j]):.1f}°, with the margin)")
        return ""

    def engage(self, gello_q, xarm_q, t: float | None = None) -> tuple[bool, str]:
        """Start passing samples on, if the gello (gello joints) matches the xArm7's measured
        joints to ``engage_tol`` on every joint (as numbers: a joint a turn off does not match)
        and is inside the limits. Returns (engaged, why not)."""
        q, x = np.asarray(gello_q, float), np.asarray(xarm_q, float)
        why = self._outside(q)
        if not why:
            off = np.abs(q - x)
            if not np.all(off <= self.engage_tol):
                j = int(np.argmax(off))
                why = (f"J{j + 1} is {np.degrees(off[j]):.1f}° from the xArm7's "
                       f"(engage within {np.degrees(self.engage_tol):g}°)")
        if why:
            self.engaged, self.stop_reason = False, why
            return False, why
        self.engaged, self.stop_reason, self._q, self._t = True, "", q.copy(), t
        return True, ""

    def stop(self, why: str) -> None:
        """A safety stop: nothing is passed on until the next ``engage``."""
        self.engaged, self.stop_reason = False, why

    def step(self, t: float, gello_q) -> np.ndarray | None:
        """The joints to send to the xArm7 for this sample (gello joints, read at ``t``
        seconds), or None: not engaged, or a safety stop now (``stop_reason`` says why)."""
        if not self.engaged:
            return None
        q = np.asarray(gello_q, float)
        why = self._outside(q)
        if not why and self._t is not None:
            dt = t - self._t
            if dt > self.stale_s:
                why = f"no new sample for {dt * 1000:.0f} ms"
            elif dt > 0.0:
                step = np.abs(q - self._q)
                fast = step > self.noise + self.max_speed * dt
                if fast.any():
                    j = int(np.argmax(np.where(fast, step, 0.0)))
                    why = (f"J{j + 1} jumped {np.degrees(step[j]):.1f}° in {dt * 1000:.0f} ms "
                           f"(more than {self.max_speed:g} rad/s)")
        if why:
            self.stop(why)
            return None
        if self._t is None or t > self._t:
            self._q, self._t = q.copy(), t
        return q
