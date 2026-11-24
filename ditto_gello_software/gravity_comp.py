"""Gravity compensation on the leader arm, with the arm's pose shown on the leader and the
xArm7 in the viewer.

    python -m ditto_gello_software.gravity_comp --u2d2 /dev/ttyUSB0   # the real arm
    python -m ditto_gello_software.gravity_comp --sim                 # a simulated arm

Then open http://localhost:8080. Run it from the repository's top folder.

It starts with torque off and the page shows only gravity compensation and the start check
(below); the rest of the page appears once that is done. At the top of the page:

1. hold the arm, then press **Gravity compensation ON**: the servos go to current control and the gravity
   gain shown is applied at once (0.5 unless ``--gravity-gain`` says otherwise);
2. raise **gravity gain** until the arm stays where you leave it. If a joint pushes the wrong
   way or the arm gets heavier, press **Gravity compensation OFF**;
3. **Gravity compensation OFF**, Ctrl+C, a failed servo read or a joint moving faster than ``--max-speed``
   all switch torque off: the arm goes limp, so keep a hand near it.

The current that can be commanded is capped per servo model (``--limit-xm-mA``,
``--limit-xc-mA``).

**Record and play back.** "Record" stores the arm's motion while you move it, with gravity compensation
on. "Play back" then drives the arm along the recording with a spring and damper per joint
(``trajectory.py``) and no gravity compensation, unless its checkbox is ticked; the stiffness
slider sets how hard. Playback stops, and gravity compensation takes over, if a joint ends up
too far from the recording or moves much faster than the recording does.
Each recording is saved to ``logs/``; ``--trajectory`` loads one.

"Play back with" chooses between those springs and the servos' own position control, so the
two can be compared on the same recording: the page keeps the last tracking table of each, and
every playback's settings and errors go to a ``_playbacks.jsonl`` file in ``logs/``.

**Hold test** (under Tests). "Hold this pose" keeps the arm where it is with the servos' position control,
for pushing on it: the page shows how far each joint is pushed off, the most so far and the
extra current the servo answered with. What the encoders do not see (the arm visibly moving
while these stay near zero) is give in the printed parts and their joints to the horns. A
joint pushed 20 degrees off ends the hold.

**Start check.** The page first offers only gravity compensation and this check. With torque
on, bring the arm by hand near the start pose, ``START_POSE_DEG``, the joints the xArm7 starts
from: every joint within 10 degrees (or 5). The parts of a joint still outside that are shown
red, deeper the further off. Then see that the render matches the arm (J1, J3, J5 and J7 have
no stops, and after power-up the servos only know their angle within one turn) and that no
cable is wrapped round a joint, and press "Confirm", then "Home": the servos take the arm
exactly to the start pose, gravity compensation takes over again, and the rest of the page
appears. Throughout, J1, J3, J5 and J7 are kept within half a turn of the start pose by the
joint-limit barrier, so the next run reads them on the right turn and the cables never wind.

**raw_gello and gello.** raw_gello is the joints as the encoders give them; gello is the same
pose in the xArm7's joints (``SHOULDER_FLIP`` in ``__init__.py``). The page and the log show
both; everything else here (the start check, the barrier, recordings) works in raw_gello.

**Tuning.** "Sag test" (under Tests) talks you through measurements that show which joints get
too little or too much compensation (see ``sag_test.py``); the factors in "Gravity compensation
settings" are where the answer goes. Every run writes two files to ``logs/``: the arm's state
and torques at 100 Hz (``.npz``) and the sag test's measurements (``_sag.jsonl``).
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time

import numpy as np

from ditto_gello_software import SHOULDER_FLIP, START_RAW_RAD, gello_from_raw, raw_from_gello
from ditto_gello_software.chain import POSITION_P_GAIN, LeaderChain, SimChain, latency_timer_ms
from ditto_gello_software.model import LeaderModel
from ditto_gello_software.sag_test import LOG_DIR, SagTest, last_table, summary_table
from ditto_gello_software.torque_law import Gains, TorqueLaw
from ditto_gello_software.trajectory import Player, Recorder, TrackingGains, Trajectory
from ditto_gello_software.view import TEMPERATURE_BUTTON, servo_report, temperature_table
from ditto_gello_software.ditto_hand import show_live
from ditto_gello_software.viewer import Viewer

POSE_HZ = 30.0
TEXT_HZ = 5.0
LOG_HZ = 100.0
MAX_FAILED_READS = 5  # in a row, before torque is switched off
MAX_MASS_G = 500.0  # largest test mass the page accepts on the flange
# Seven values each. raw_gello: the joints as the encoders give them; gello: the same pose in the
# xArm7's joints (see SHOULDER_FLIP). Everything else is in raw_gello joints.
LOG_COLUMNS = ("raw_gello", "qd", "tau", "tau_cmd", "current_mA", "joint_gain", "q_ref", "gello")
GRAVITY, RECORD, REPLAY, HOLD = "gravity compensation", "recording", "playing back", "holding a pose"
HOME = "going to the start pose"
MODES = (GRAVITY, RECORD, REPLAY, HOLD, HOME)  # the log's "mode" column is the index into this
HOLD_RELEASE = np.radians(20.0)  # a joint pushed this far off the held pose ends the hold
HOLD_SETTLE_TICKS = 300  # the arm is left this long to settle under its own weight first
# Playback starts by fading gravity compensation out over this long while the springs come on.
# Dropping it at once lets the arm fall until the springs have stretched enough to catch it.
GRAVITY_FADE_S = 1.0
# The two ways a recording can be played back.
SPRINGS, SERVO = "springs (torque control)", "servo position control"
# The start pose in raw_gello rad: the xArm7 starts there too (starting_joint_pos in
# mini_manipulation_env's configs/rig/right_xarm_ditto.yaml).
START_Q = START_RAW_RAD
START_TOL_DEG = (10.0, 5.0)  # how close every joint must be to go to the start pose; the first is the default
WOUND_WARN_DEG = 150.0  # a full-turn joint this far from the start pose is nearing its half turn
OFF_RED_DEG = 60.0  # a joint this far beyond the tolerance is shown fully red
ARROW_DEG = 36.0  # the start table shows one more arrow per this many degrees off (five at most)
# Going to the start pose: a smooth move over this long at least, slower than this speed,
# stopped if a joint falls this far behind it (a hand holding the arm back). It is there when
# every joint is this close, at most HOME_GRACE_S after the move ends.
HOME_MIN_S, HOME_SPEED, HOME_LAG = 1.5, 0.2, np.radians(5.0)
HOME_DONE, HOME_GRACE_S = np.radians(2.0), 1.0
# The most the page lets a current cap be set to, mA. Under each model's own Current Limit
# register as read from the first arm (2690 mA on its J2, 910 mA on the XC330s).
CAP_MAX_MA = {"XM430-W210": 2600.0, "XC330-T288": 900.0}


class GravityComp:
    """The loop: read the joints, compute the torques, command them. All servo traffic happens
    in :meth:`run`'s thread; the page's buttons only leave requests for it."""

    def __init__(self, chain: LeaderChain, law: TorqueLaw, rate_hz: float, limit_mA,
                 max_speed: float) -> None:
        self.chain, self.law = chain, law
        self.dt = 1.0 / rate_hz
        self.limit_mA = np.asarray(limit_mA, dtype=float)
        self.max_speed = max_speed
        self.message = "gravity compensation off"
        self.running = True
        self._want: bool | None = None  # a request from the page: True on, False off
        self._want_temperatures = False
        self.temperatures = ""  # the page's answer to the last temperature request
        self._failed_in_a_row = 0
        self.lock = threading.Lock()
        self.state = None  # latest State
        self.tau = np.zeros(law.n)  # latest torques asked for
        self.tau_cmd = np.zeros(law.n)  # and commanded, after the current cap
        self.hz = self.worst_ms = 0.0
        self.failed_reads = 0
        self.mass_g = self.mass_mm = 0.0
        self.sag = SagTest(law)
        self.mode = GRAVITY
        self.tracking = TrackingGains()
        self.player = Player(self.tracking)
        self.recorder = Recorder()
        self.trajectory: Trajectory | None = None
        self.recordings = 0  # how many recordings have been finished (the page saves each)
        self.trajectory_message = "Nothing recorded yet."
        self.replay_controller = SPRINGS  # what the next playback uses
        self.replay_reports: dict[str, str] = {}  # the last playback's table, per controller
        self.position_p = np.array([float(POSITION_P_GAIN[s.name]) for s in chain.servos])
        self.position_d = np.zeros(law.n)
        self.position_i = np.zeros(law.n)
        self.q_ref = np.full(law.n, np.nan)  # where playback wants the arm; NaN outside it
        self.lookahead_ms = 0.0  # position playback sends the pose this far ahead in the recording
        self.playbacks: list[dict] = []  # settings and result of each playback, for the log
        self.hold_q = np.zeros(law.n)  # the held pose, and what pushing on it has shown
        self.hold_mA0 = np.zeros(law.n)
        self.hold_sag = np.zeros(law.n)  # how far it settled from the held pose on its own
        self.hold_most = np.zeros(law.n)
        self.hold_mA_at_most = np.zeros(law.n)
        self._hold_n = 0
        self._playing_with = SPRINGS
        self.replay_gravity = False  # keep gravity compensation on during playback
        self.start_tol = np.radians(START_TOL_DEG[0])  # from the page
        self.started = False  # the arm went to the start pose: recording, playback and holds are allowed
        self._home_from = np.zeros(law.n)
        self._home_t0 = self._home_s = 0.0
        self._want_mode: str | None = None
        self.log: list[np.ndarray] = []  # one row per 1 / LOG_HZ, see _log_row
        self._log_every = max(1, round(rate_hz / LOG_HZ))
        self._ticks = 0
        self._t0 = time.monotonic()

    def request(self, on: bool) -> None:
        self._want = on

    def request_temperatures(self) -> None:
        self._want_temperatures = True

    def request_mode(self, mode: str) -> None:
        """From the page: RECORD or REPLAY to start one, GRAVITY to stop either."""
        self._want_mode = mode

    def _to_gravity(self, why: str = "") -> None:
        """End a recording or a playback; gravity compensation takes over if torque is on."""
        if self.mode == RECORD:
            recorded = self.recorder.trajectory()
            if recorded is None:
                why = why or "Recording too short (under half a second): not kept."
            else:
                self.trajectory, self.recordings = recorded, self.recordings + 1
                why = why or f"Recorded {recorded.duration:.1f} s."
        elif self.mode == REPLAY:
            how = self._playing_with
            if how == SPRINGS:
                how += (f", stiffness {self.tracking.stiffness:.2f}, "
                        + ("with" if self.replay_gravity else "no") + " gravity compensation")
            report = self.player.report(how)
            if report:
                self.replay_reports[self._playing_with] = report
                typical, largest = self.player.errors()
                servo = self._playing_with == SERVO
                self.playbacks.append({
                    "time": time.strftime("%H:%M:%S"), "controller": self._playing_with,
                    "seconds": round(self.player.time, 2), "ended": why or "finished",
                    "typical_deg": typical.round(2).tolist(), "largest_deg": largest.round(2).tolist(),
                    "cap_mA": self.limit_mA.tolist(),
                    **({"p": self.position_p.tolist(), "d": self.position_d.tolist(),
                        "i": self.position_i.tolist(), "lookahead_ms": self.lookahead_ms} if servo else
                       {"stiffness": self.tracking.stiffness, "gravity": self.replay_gravity,
                        "kp": self.tracking.kp.round(3).tolist(), "kd": self.tracking.kd.round(4).tolist()}),
                })
        elif self.mode == HOLD:
            why = why or "Hold ended."
        elif self.mode == HOME:
            why = why or "Going to the start pose stopped."
        self.mode = GRAVITY
        if why:
            self.trajectory_message = why
            print(f"[gravity_comp] {why}")
        if self.chain.position_mode and self.chain.torque_enabled:  # back to torque control
            try:
                self.chain.torque_on(self.limit_mA)
                self.law.reset_barrier()
            except Exception as e:
                self._off(f"torque off: could not go back to torque control ({e})")

    def _change_mode(self, want: str, t: float, state) -> None:
        if want == GRAVITY:
            self._to_gravity("Playback stopped." if self.mode == REPLAY else "")
        elif self.mode != GRAVITY:
            self.trajectory_message = f"Stop {self.mode} first."
        elif not self.started and want != HOME:
            self.trajectory_message = "Confirm the start pose first."
        elif not self.chain.torque_enabled:
            self.trajectory_message = "Press **Gravity compensation ON** first (hold the arm)."
        elif want == HOME:
            off = np.abs(state.q - START_Q)
            if (off > self.start_tol).any():
                self.trajectory_message = (f"Not going to the start pose: J{int(np.argmax(off)) + 1} is "
                                           f"{np.degrees(off.max()):.0f}° from it.")
            else:
                try:
                    self._home_from = self.chain.position_on(self.limit_mA, self.position_p,
                                                             self.position_d, self.position_i)
                except Exception as e:
                    self._off(f"torque off: could not switch to position control ({e})")
                    return
                self._home_t0, self._home_s = t, max(HOME_MIN_S, float(off.max()) / HOME_SPEED)
                self.mode, self.trajectory_message = HOME, "Going to the start pose."
        elif want == RECORD:
            self.recorder = Recorder()
            self.mode, self.trajectory_message = RECORD, "Recording: move the arm."
        elif want == HOLD:
            try:
                q = self.chain.position_on(self.limit_mA, self.position_p, self.position_d, self.position_i)
            except Exception as e:
                self._off(f"torque off: could not switch to position control ({e})")
                return
            self._hold(q, "Holding this pose: let it settle, then push on the arm.")
        elif self.trajectory is None:
            self.trajectory_message = "Nothing recorded yet."
        else:
            refused = self.player.start(t, state.q, self.trajectory)
            if refused:
                self.trajectory_message = f"Not playing back: {refused}."
            else:
                self._playing_with = self.replay_controller
                if self._playing_with == SERVO:
                    try:
                        self.chain.position_on(self.limit_mA, self.position_p, self.position_d,
                                               self.position_i)
                    except Exception as e:
                        self._off(f"torque off: could not switch to position control ({e})")
                        return
                self.mode, self._replay_t0 = REPLAY, t
                self.trajectory_message = f"Playing back with {self._playing_with}."

    def _hold(self, q, message: str) -> None:
        """Hold pose ``q`` with the servos' position control (already on) and measure pushes."""
        self.hold_q = np.array(q, dtype=float)
        self.hold_mA0, self.hold_most = np.zeros(self.law.n), np.zeros(self.law.n)
        self.hold_mA_at_most, self._hold_n = np.zeros(self.law.n), 0
        self.hold_sag = np.zeros(self.law.n)
        self.mode, self.trajectory_message = HOLD, message

    def set_mass(self, grams: float, mm: float) -> None:
        """A point mass on the flange, ``mm`` along joint 7's axis, in the model (and on the
        simulated arm, if that is what is connected)."""
        self.mass_g, self.mass_mm = float(grams), float(mm)
        at = (0.0, 0.0, self.mass_mm / 1000.0)
        self.law.model.set_point_mass(self.mass_g / 1000.0, at)
        if hasattr(self.chain, "set_point_mass"):
            self.chain.set_point_mass(self.mass_g / 1000.0, at)

    def context(self) -> dict:
        g = self.law.gains
        return {"gravity_gain": float(g.gravity_gain),
                "joint_gain": [float(x) for x in g.gravity_joint_gain],
                "mass_g": self.mass_g, "mass_mm": self.mass_mm}

    def _off(self, why: str) -> None:
        if self.chain.torque_enabled:
            try:
                self.chain.torque_off()
            except Exception as e:  # the bus may be the thing that failed
                why += f" (and switching torque off failed: {e})"
        self.tau = np.zeros(self.law.n)
        self.tau_cmd = np.zeros(self.law.n)
        self.message = why
        print(f"[gravity_comp] {why}")
        if self.mode != GRAVITY:
            self._to_gravity("Playback stopped: torque went off." if self.mode == REPLAY else "")

    def _tick(self) -> None:
        want, self._want = self._want, None
        if want is False:
            self._off("gravity compensation off")
        if self._want_temperatures:
            self._want_temperatures = False
            if self.chain.torque_enabled:  # the controller is running: do not load the bus
                self.temperatures = ("**The controller is on.** Press **Gravity compensation OFF** (hold the "
                                     f"arm), then press **{TEMPERATURE_BUTTON}** again.")
            else:
                self.temperatures = temperature_table(self.chain)
        try:
            state = self.chain.read()
            self._failed_in_a_row = 0
        except RuntimeError as e:
            self.failed_reads += 1
            self._failed_in_a_row += 1
            if self._failed_in_a_row >= MAX_FAILED_READS and self.chain.torque_enabled:
                self._off(f"torque off: {MAX_FAILED_READS} servo reads failed in a row ({e})")
            elif self.mode == REPLAY and self.chain.position_mode:
                # The servos follow the positions they are sent: keep sending them on time
                # even without a reading, or the arm stops and then jumps to catch up.
                try:
                    self.chain.write_position(self.player.reference(
                        time.monotonic() - self._t0 + self.lookahead_ms / 1000.0)[0])
                except Exception as e2:
                    self._off(f"torque off: writing to the servos failed ({e2})")
            return
        if want is True and not self.chain.torque_enabled:
            try:
                self.chain.torque_on(self.limit_mA)
                self.law.reset_barrier()
                self.message = "gravity compensation on"
                print("[gravity_comp] gravity compensation on")
            except Exception as e:
                self._off(f"torque off: could not switch it on ({e})")
        t = state.t - self._t0
        want_mode, self._want_mode = self._want_mode, None
        if want_mode is not None:
            self._change_mode(want_mode, t, state)
        tau = tau_cmd = np.zeros(self.law.n)
        q_ref = np.full(self.law.n, np.nan)
        if self.chain.torque_enabled:
            fast = np.abs(state.qd) > self.max_speed
            if fast.any():
                j = int(np.argmax(np.abs(state.qd)))
                self._off(f"torque off: J{j + 1} moved at {abs(state.qd[j]):.1f} rad/s "
                          f"(limit {self.max_speed:.1f})")
            else:
                if self.mode == HOME:  # a smooth move (minimum jerk) to exactly the start pose
                    x = min(1.0, (t - self._home_t0) / self._home_s)
                    q_ref = self._home_from + (START_Q - self._home_from) * x ** 3 * (10.0 - 15.0 * x + 6.0 * x ** 2)
                    lag = np.abs(state.q - q_ref)
                    late = t - self._home_t0 > self._home_s + HOME_GRACE_S
                    if lag.max() > HOME_LAG or late:
                        self._to_gravity(f"Going to the start pose stopped: J{int(np.argmax(lag)) + 1} "
                                         f"was held {np.degrees(lag.max()):.1f}° off.")
                    elif x >= 1.0 and lag.max() < HOME_DONE:
                        self.started = True
                        print("[gravity_comp] start check passed: the arm is at the start pose")
                        self._to_gravity("At the start pose: gravity compensation is back on.")
                if self.mode == HOLD:  # the servos hold the pose; watch how far it is pushed
                    q_ref, off = self.hold_q, state.q - self.hold_q
                    self._hold_n += 1
                    last = self._hold_n - (HOLD_SETTLE_TICKS - 50)
                    if 0 < last <= 50:  # where it settled, and the current that holds it there
                        self.hold_sag += (off - self.hold_sag) / last
                        self.hold_mA0 += (state.current_mA - self.hold_mA0) / last
                    elif last > 50:  # from here on, pushes are measured from where it settled
                        pushed = off - self.hold_sag
                        further = np.abs(pushed) > self.hold_most
                        self.hold_most = np.where(further, np.abs(pushed), self.hold_most)
                        self.hold_mA_at_most = np.where(further, state.current_mA - self.hold_mA0,
                                                        self.hold_mA_at_most)
                    j = int(np.argmax(np.abs(off)))
                    if abs(off[j]) > HOLD_RELEASE:
                        self._to_gravity(f"Hold released: J{j + 1} was pushed "
                                         f"{np.degrees(abs(off[j])):.0f}° off the pose.")
                if self.mode == REPLAY:  # the springs alone: no gravity compensation
                    tau, q_ref = self.player.step(t, state)
                    if self.chain.position_mode:  # the servos do the tracking: no torques from here
                        tau = np.zeros(self.law.n)
                    else:
                        fade = 1.0 if self.replay_gravity else 1.0 - (t - self._replay_t0) / GRAVITY_FADE_S
                        if fade > 0.0:
                            tau = tau + fade * self.law.gravity_compensation(state.q, state.qd)
                        if self.law.gains.limit_barrier:
                            tau = tau + self.law.joint_limit_barrier(state.q, state.qd)
                    if self.player.fault:
                        self._to_gravity(f"Playback stopped: {self.player.fault}.")
                    elif self.player.done:
                        self._to_gravity("Playback finished.")
                if self.mode not in (REPLAY, HOLD, HOME):
                    tau, _ = self.law.step(state.q, state.qd)
                    if self.mode == RECORD and not self.recorder.add(t, state):
                        self._to_gravity(f"Recording stopped at its {Recorder.MAX_S:.0f} s limit.")
                try:
                    if self.chain.position_mode:
                        ahead = self.lookahead_ms / 1000.0
                        self.chain.write_position(self.player.reference(t + ahead)[0]
                                                  if self.mode == REPLAY and ahead else q_ref)
                        tau_cmd = np.zeros(self.law.n)
                    elif self.chain.torque_enabled:
                        tau_cmd = self.chain.write_torque(tau)
                except Exception as e:
                    self._off(f"torque off: writing to the servos failed ({e})")
                    tau = tau_cmd = np.zeros(self.law.n)
        self.sag.update(t, state, tau, tau_cmd, self.chain.torque_enabled, self.context())
        self._ticks += 1
        if self._ticks % self._log_every == 0:
            self._log_row(t, state, tau, tau_cmd, q_ref)
        with self.lock:
            self.state, self.tau, self.tau_cmd, self.q_ref = state, tau, tau_cmd, q_ref

    def _log_row(self, t, state, tau, tau_cmd, q_ref) -> None:
        g = self.law.gains
        self.log.append(np.concatenate([
            [t, float(self.chain.torque_enabled), g.gravity_gain, self.mass_g, self.mass_mm,
             float(self.sag.phase == "measure"), float(MODES.index(self.mode)), self.tracking.stiffness,
             float(self.chain.position_mode), float(self.replay_gravity),
             float(self.failed_reads), float(self.chain.read_retries)],
            state.q, state.qd, tau, tau_cmd, state.current_mA, g.gravity_joint_gain, q_ref,
            gello_from_raw(state.q),
        ]))

    def save_log(self, path) -> None:
        rows = np.array(list(self.log))
        if not len(rows):
            return
        n = self.law.n
        arrays = {"t": rows[:, 0], "torque_on": rows[:, 1], "gravity_gain": rows[:, 2],
                  "mass_g": rows[:, 3], "mass_mm": rows[:, 4], "sag_measuring": rows[:, 5],
                  "mode": rows[:, 6], "stiffness": rows[:, 7], "position_mode": rows[:, 8],
                  "replay_gravity": rows[:, 9], "failed_reads": rows[:, 10], "read_retries": rows[:, 11],
                  "limit_mA": self.limit_mA, "position_p": self.position_p,
                  "position_d": self.position_d, "position_i": self.position_i,
                  "tracking_kp": self.tracking.kp, "tracking_kd": self.tracking.kd,
                  "shoulder_flip": SHOULDER_FLIP}
        for k, name in enumerate(LOG_COLUMNS):
            arrays[name] = rows[:, 12 + k * n: 12 + (k + 1) * n]
        np.savez_compressed(path, **arrays)

    def run(self) -> None:
        next_tick = time.perf_counter()
        count, worst, t_rate = 0, 0.0, next_tick
        try:
            while self.running:
                t0 = time.perf_counter()
                self._tick()
                now = time.perf_counter()
                worst = max(worst, now - t0)
                count += 1
                if now - t_rate >= 1.0:
                    self.hz, self.worst_ms = count / (now - t_rate), 1000.0 * worst
                    count, worst, t_rate = 0, 0.0, now
                next_tick += self.dt
                if next_tick < now:  # fell behind: do not try to catch up
                    next_tick = now
                time.sleep(next_tick - now)
        finally:
            self._off("torque off: stopped")
            self.chain.close()


def start_table(q, tol: float) -> str:
    """Per joint, which way to turn it to reach the start pose: ✓ within ``tol`` (rad), else ↑
    (increase the angle) or ↓ (decrease it), one arrow per ARROW_DEG off, up to five."""
    move = np.degrees(START_Q - q)
    rows = ["| J | move |", "|---|---|"]
    for k, m in enumerate(move):
        n = min(5, 1 + int(abs(m) // ARROW_DEG))
        rows.append(f"| J{k + 1} | {'✓' if abs(m) <= np.degrees(tol) else ('↑' if m > 0 else '↓') * n} |")
    return "\n".join(rows)


def panel(viewer: Viewer, loop: GravityComp, stem: str) -> None:
    """The page's controls and readout, refreshed from a thread of their own. At first it
    offers only gravity compensation and the start check; the rest appears once the arm has
    gone to the start pose."""
    gui, gains, chain = viewer.server.gui, loop.law.gains, loop.chain
    families = {name: [j for j, s in enumerate(chain.servos) if s.name == name]
                for name in dict.fromkeys(s.name for s in chain.servos)}
    later = []  # hidden until the start check is done

    def section(title: str, expand: bool = False):
        f = gui.add_folder(title, expand_by_default=expand)
        later.append(f)
        return f

    with viewer.panel:
        steps_md = gui.add_markdown("")
        on_btn = gui.add_button("Gravity compensation ON")
        off_btn = gui.add_button("Gravity compensation OFF (arm goes limp)")
        gain = gui.add_slider("gravity gain", min=0.0, max=1.2, step=0.01,
                              initial_value=float(gains.gravity_gain))
        held = gui.add_checkbox("a hand holds the simulated arm", True) if isinstance(chain, SimChain) else None
        status = gui.add_markdown("")
        # The start check: only until the arm has gone to the start pose.
        start_md = gui.add_markdown("")
        start_tol = gui.add_dropdown("tolerance", options=[f"±{d:g}°" for d in START_TOL_DEG])
        confirm_btn = gui.add_button("Confirm: render matches the arm")
        go_btn = gui.add_button("Home: servos move the arm to the start pose")
        go_btn.visible = False
        first = (start_md, start_tol, confirm_btn, go_btn)
        page = {"confirmed": False}  # Confirm pressed: Home is offered
        release_top = gui.add_button("Release (gravity compensation)")
        sag_prompt = gui.add_markdown("")
        later += [release_top, sag_prompt]
        with section("Start pose"):
            home_btn = gui.add_button("Go to the start pose")
            home_md = gui.add_markdown("")
        with section("Record and play back", expand=True):
            rec_btn = gui.add_button("Record")
            play_btn = gui.add_button("Play back")
            stop_btn = gui.add_button("Stop")
            controller = gui.add_dropdown("play back with", options=[SPRINGS, SERVO],
                                          initial_value=loop.replay_controller)
            stiffness = gui.add_slider("spring stiffness", min=0.1, max=2.0, step=0.05,
                                       initial_value=float(loop.tracking.stiffness))
            replay_gravity = gui.add_checkbox("gravity compensation during playback", loop.replay_gravity)
            traj_md = gui.add_markdown("")
            with gui.add_folder("Playback settings", expand_by_default=False):
                gui.add_markdown("Springs, at stiffness 1: kp N m/rad, kd N m s/rad.")
                kps = [gui.add_number(f"J{j + 1} kp", float(loop.tracking.kp[j]), min=0.0, max=20.0, step=0.1)
                       for j in range(loop.law.n)]
                kds = [gui.add_number(f"J{j + 1} kd", float(loop.tracking.kd[j]), min=0.0, max=1.0, step=0.005)
                       for j in range(loop.law.n)]
                gui.add_markdown("Servo position control: the servos' gains, register units.")
                p_gains = {name: gui.add_number(f"P, {name}", float(loop.position_p[js[0]]), min=0.0, max=16000.0, step=50.0)
                           for name, js in families.items()}
                d_gains = {name: gui.add_number(f"D, {name}", float(loop.position_d[js[0]]), min=0.0, max=16000.0, step=50.0)
                           for name, js in families.items()}
                i_gains = {name: gui.add_number(f"I, {name}", float(loop.position_i[js[0]]), min=0.0, max=16000.0, step=10.0)
                           for name, js in families.items()}
                lookahead = gui.add_number("look ahead (ms)", loop.lookahead_ms, min=0.0, max=60.0, step=1.0)
                gui.add_markdown("Current caps, mA (all modes; from the next Gravity compensation ON or playback).")
                caps = {name: gui.add_number(name, float(loop.limit_mA[js[0]]), min=50.0, max=CAP_MAX_MA[name], step=50.0)
                        for name, js in families.items()}
        with section("Gravity compensation settings"):
            mass_g = gui.add_number("mass on the flange (g)", loop.mass_g, min=0.0, max=MAX_MASS_G, step=1.0)
            mass_mm = gui.add_number("its centre beyond the flange (mm)", loop.mass_mm, min=0.0, max=300.0, step=1.0)
            gui.add_markdown("Per joint: gravity gain x factor.")
            factors = [gui.add_number(f"J{j + 1} factor", float(gains.gravity_joint_gain[j]), min=0.0, max=3.0, step=0.01)
                       for j in range(loop.law.n)]
            barrier = gui.add_checkbox("joint-limit barrier", gains.limit_barrier)
            stiction = gui.add_checkbox("stiction dither", gains.stiction)
            null_space = gui.add_checkbox("null-space spring to 'ready'", gains.null_space)
        with section("Tests"):
            with gui.add_folder("Hold test", expand_by_default=False):
                gui.add_markdown("Servo position control holds the pose; push on the arm.")
                hold_btn = gui.add_button("Hold this pose")
                release_btn = gui.add_button("Release")
                hold_md = gui.add_markdown("")
            with gui.add_folder("Sag test", expand_by_default=False):
                sag_start = gui.add_button("Start sag test")
                sag_stop = gui.add_button("Stop sag test")
                sag_md = gui.add_markdown("Follow the line under Release.")
        with section("Diagnostics"):
            detail = gui.add_markdown("")
            readout = gui.add_markdown("")
            temp_btn = gui.add_button(TEMPERATURE_BUTTON)
            temp_md = gui.add_markdown("Works with torque off only.")
    for h in later:
        h.visible = False
    viewer.hide_pose_controls()
    viewer.show_controls(False)

    def set_factors(_=None) -> None:
        gains.gravity_joint_gain = np.array([float(f.value) for f in factors])

    def set_tracking(_=None) -> None:
        loop.tracking.kp = np.array([float(x.value) for x in kps])
        loop.tracking.kd = np.array([float(x.value) for x in kds])
        loop.tracking.stiffness = float(stiffness.value)

    def set_position_gains(_=None) -> None:
        for name, js in families.items():
            loop.position_p[js] = float(p_gains[name].value)
            loop.position_d[js] = float(d_gains[name].value)
            loop.position_i[js] = float(i_gains[name].value)
            loop.limit_mA[js] = float(np.clip(caps[name].value, 50.0, CAP_MAX_MA[name]))

    start_tol.on_update(lambda _: setattr(loop, "start_tol", np.radians(
        START_TOL_DEG[start_tol.options.index(start_tol.value)])))
    confirm_btn.on_click(lambda _: page.update(confirmed=True))
    go_btn.on_click(lambda _: loop.request_mode(HOME))
    home_btn.on_click(lambda _: loop.request_mode(HOME))
    release_top.on_click(lambda _: loop.request_mode(GRAVITY))
    controller.on_update(lambda _: setattr(loop, "replay_controller", controller.value))
    lookahead.on_update(lambda _: setattr(loop, "lookahead_ms", float(lookahead.value)))
    hold_btn.on_click(lambda _: loop.request_mode(HOLD))
    release_btn.on_click(lambda _: loop.request_mode(GRAVITY))
    for x in (*p_gains.values(), *d_gains.values(), *i_gains.values(), *caps.values()):
        x.on_update(set_position_gains)
    rec_btn.on_click(lambda _: loop.request_mode(RECORD))
    play_btn.on_click(lambda _: loop.request_mode(REPLAY))
    stop_btn.on_click(lambda _: loop.request_mode(GRAVITY))
    replay_gravity.on_update(lambda _: setattr(loop, "replay_gravity", bool(replay_gravity.value)))
    for x in (stiffness, *kps, *kds):
        x.on_update(set_tracking)

    on_btn.on_click(lambda _: loop.request(True))
    off_btn.on_click(lambda _: loop.request(False))
    gain.on_update(lambda _: setattr(gains, "gravity_gain", float(gain.value)))
    mass_g.on_update(lambda _: loop.set_mass(np.clip(mass_g.value, 0.0, MAX_MASS_G), mass_mm.value))
    mass_mm.on_update(lambda _: loop.set_mass(np.clip(mass_g.value, 0.0, MAX_MASS_G), mass_mm.value))
    for f in factors:
        f.on_update(set_factors)
    temp_btn.on_click(lambda _: loop.request_temperatures())
    sag_start.on_click(lambda _: loop.sag.start())
    sag_stop.on_click(lambda _: loop.sag.stop())
    barrier.on_update(lambda _: setattr(gains, "limit_barrier", bool(barrier.value)))
    stiction.on_update(lambda _: setattr(gains, "stiction", bool(stiction.value)))
    null_space.on_update(lambda _: setattr(gains, "null_space", bool(null_space.value)))
    if held is not None:
        held.on_update(lambda _: setattr(chain, "held", bool(held.value)))
    viewer.set_external(True)

    def refresh() -> None:
        next_text, written, spoken, saved, played, shown = 0.0, 0, "", 0, 0, False
        mA_to_Nm = 1.0 / (chain._mA_per_unit * chain._units_per_Nm)
        full_turn = chain.upper - chain.lower > 2.0 * np.pi
        while loop.running:
            time.sleep(1.0 / POSE_HZ)
            with loop.lock:
                state, tau, tau_cmd, q_ref = loop.state, loop.tau, loop.tau_cmd, loop.q_ref
            if state is None:
                continue
            viewer.set_joints(state.q)
            color = "green" if chain.torque_enabled else None  # None: the default blue
            if on_btn.color != color:
                on_btn.color = color
            viewer.set_desired(q_ref if np.isfinite(q_ref).all() else None if loop.started else START_Q)
            sag = loop.sag
            if sag.prompt != sag_prompt.content:
                sag_prompt.content = sag.prompt
            if sag.phase != spoken:  # say each step once in the terminal, with a beep
                spoken = sag.phase
                if sag.prompt:
                    print("\a[sag test] " + sag.prompt.replace("**", "").replace("  \n", " "), flush=True)
            if saved < loop.recordings and loop.trajectory is not None:  # keep each recording
                saved = loop.recordings
                path = f"{stem}_trajectory_{saved}.npz"
                loop.trajectory.save(path)
                print(f"[gravity_comp] saved {path}", flush=True)
            while played < len(loop.playbacks):  # each playback's settings and result
                with open(f"{stem}_playbacks.jsonl", "a") as f:
                    f.write(json.dumps(loop.playbacks[played]) + "\n")
                played += 1
            if loop.temperatures and loop.temperatures != temp_md.content:
                temp_md.content = loop.temperatures
            if held is not None and held.value != chain.held:
                held.value = chain.held
            while written < len(sag.results):  # new measurements: to the file and the page
                with open(f"{stem}_sag.jsonl", "a") as f:
                    f.write(json.dumps(sag.results[written]) + "\n")
                written += 1
                sag_md.content = (
                    f"**Measurement {written}** (ratio = gravity gain x factor)\n\n"
                    + last_table(sag.results[-1])
                    + f"\n\n**All {written} so far**\n\n"
                    + summary_table(sag.results, gains.gravity_gain)
                )
            now = time.monotonic()
            if now < next_text:
                continue
            next_text = now + 1.0 / TEXT_HZ
            viewer.show_on_sliders(state.q)
            off, tol = np.abs(state.q - START_Q), loop.start_tol
            close = bool((off <= tol).all())
            ready = close and chain.torque_enabled and loop.mode == GRAVITY
            for btn, disabled in ((confirm_btn, not ready), (go_btn, not ready), (home_btn, not ready),
                                  (release_top, loop.mode not in (HOME, HOLD))):
                if btn.disabled != disabled:
                    btn.disabled = disabled
            if not loop.started:  # Confirm first, then Home
                for btn, show in ((confirm_btn, not page["confirmed"]), (go_btn, page["confirmed"])):
                    if btn.visible != show:
                        btn.visible = show
            if loop.started and not shown:  # the start check is done: the rest of the page
                shown = True
                for h in first:
                    h.visible = False
                for h in later:
                    h.visible = True
                viewer.show_controls(True)
            # Red, deeper the further off, on the parts of each joint still outside the tolerance.
            viewer.tint_joints(None if loop.started else
                               np.where(off > tol, 0.5 + 0.5 * (off - tol) / np.radians(OFF_RED_DEG), 0.0))
            if loop.mode == HOME:
                steps = "**Going to the start pose.**"
            elif loop.started:
                steps = ""
            elif not chain.torque_enabled:
                steps = "**1.** Hold the arm. Press **Gravity compensation ON**."
            elif not close:
                steps = "**2.** Turn each joint the way its arrows point (↑ increase, ↓ decrease) until all are ✓."
            elif not page["confirmed"]:
                steps = "**3.** Render matches the arm? No cable wrapped? Press **Confirm**."
            else:
                steps = "**4.** Press **Home**: the servos move the arm the last few degrees."
            if loop.mode != HOME and "start pose" in loop.trajectory_message:  # how going there ended
                steps += ("  \n" if steps else "") + loop.trajectory_message
            wound = full_turn & (np.degrees(off) > WOUND_WARN_DEG)
            if wound.any():
                j = int(np.argmax(np.where(wound, off, 0.0)))
                steps += f"  \n**J{j + 1} is {np.degrees(off[j]):.0f}° from the start pose: turn it back.**"
            if steps != steps_md.content:
                steps_md.content = steps
            table = start_table(state.q, tol)
            box = home_md if loop.started else start_md
            if table != box.content:
                box.content = table
            text = f"**{loop.mode.capitalize()}.** {loop.trajectory_message}"
            if loop.mode == RECORD:
                text += f"  \n{loop.recorder.seconds:.1f} s so far."
            elif loop.mode == REPLAY:
                p, worst = loop.player, int(np.argmax(np.abs(loop.player.error)))
                text += ("  \nMoving to the start of the recording." if p.approaching else
                         f"  \n{p.time:.1f} s of {p.trajectory.duration:.1f} s.")
                text += f" Furthest off: J{worst + 1}, {np.degrees(abs(p.error[worst])):.1f}°."
            elif loop.trajectory is not None:
                text += f"  \nReady to play: {loop.trajectory.duration:.1f} s."
            for report in loop.replay_reports.values():  # the last playback with each controller
                text += "\n\n" + report
            if text != traj_md.content:
                traj_md.content = text
            if loop.mode == HOLD or loop.hold_most.any():
                holding = loop.mode == HOLD
                pushed = (np.degrees(state.q - loop.hold_q - loop.hold_sag) if holding
                          else np.zeros(loop.law.n))
                rows = [("**Holding.** " if holding else "**Last hold.** ")
                        + "Sag: how far it settled under its own weight. Then, measured from there: "
                        "pushed off now, the most so far, the extra current at that point, and the "
                        "stiffness those two give.", "",
                        "| J | sag ° | now ° | most ° | extra mA | N m/rad |", "|---|---|---|---|---|---|"]
                for k in range(loop.law.n):
                    most, extra = np.degrees(loop.hold_most[k]), loop.hold_mA_at_most[k]
                    stiff = (f"{abs(extra) * mA_to_Nm[k] / loop.hold_most[k]:.1f}"
                             if most > 0.5 and abs(extra) > 5 else "")
                    rows.append(f"| {k + 1} | {np.degrees(loop.hold_sag[k]):+.2f} | {pushed[k]:+.2f} "
                                f"| {most:.2f} | {extra:+.0f} | {stiff} |")
                hold_md.content = "\n".join(rows)
            status.content = f"**{loop.message}** · loop {loop.hz:.0f} Hz"
            detail.content = (f"`{chain.port}`, slowest tick {loop.worst_ms:.1f} ms, failed reads "
                              f"{loop.failed_reads}, repeated {chain.read_retries}")
            limit_Nm = loop.limit_mA / (chain._mA_per_unit * chain._units_per_Nm)
            gello = np.degrees(gello_from_raw(state.q))
            rows = ["| J | raw_gello ° | gello ° | asked N m | sent N m | of cap | measured mA |",
                    "|---|---|---|---|---|---|---|"]
            for k in range(loop.law.n):
                capped = " **capped**" if abs(tau[k] - tau_cmd[k]) > 0.02 * limit_Nm[k] else ""
                rows.append(
                    f"| {k + 1} | {np.degrees(state.q[k]):.1f} | {gello[k]:.1f} | {tau[k]:+.3f} | {tau_cmd[k]:+.3f} "
                    f"| {100 * abs(tau_cmd[k]) / limit_Nm[k]:.0f} %{capped} | {state.current_mA[k]:+.0f} |"
                )
            readout.content = "\n".join(rows)

    threading.Thread(target=refresh, daemon=True).start()


def main() -> None:
    ap = argparse.ArgumentParser(description="Gravity compensation on the Ditto GELLO leader arm.")
    ap.add_argument("--port", type=int, default=8080, help="port of the web page")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--u2d2", metavar="PORT", help="serial port of the real arm's U2D2, e.g. /dev/ttyUSB0")
    src.add_argument("--sim", action="store_true", help="a simulated arm instead of the real one")
    ap.add_argument("--rate", type=float, default=500.0, help="loop rate, Hz")
    ap.add_argument("--hands", action="store_true",
                    help="show the DITTO hands: the leader on the gello, the follower on the xArm7")
    ap.add_argument("--hand-u2d2", metavar="PORT", default="/dev/ttyUSB1",
                    help="with --hands: the DITTO leader hand's U2D2, read only (default /dev/ttyUSB1)")
    ap.add_argument("--gravity-gain", type=float, default=0.5,
                    help="share of the model's gravity torque applied when torque comes on")
    ap.add_argument("--joint-gain", type=float, nargs=7, default=None, metavar="F",
                    help="factor on the gravity gain per joint, J1 to J7 (default: 1 1 1 1 1.25 1.5 1)")
    ap.add_argument("--trajectory", metavar="FILE", help="a saved recording to have ready for playback")
    # The flange carries only the hand mount's latch base (ditto_j7_adapter: 4.2 cm3 of PLA,
    # 5.2 g solid, plus four M2 screws), its centre 8 mm out along J7. The DITTO hand that clips
    # onto it is held by the user, not compensated.
    ap.add_argument("--mass-g", type=float, default=6.0, help="mass fixed to the flange, g")
    ap.add_argument("--mass-mm", type=float, default=8.0,
                    help="its centre's distance beyond the flange, along joint 7's axis, mm")
    ap.add_argument("--limit-xm-mA", type=float, default=1000.0, help="current cap of the XM430 servos")
    ap.add_argument("--limit-xc-mA", type=float, default=500.0, help="current cap of the XC330 servos")
    ap.add_argument("--max-speed", type=float, default=6.0,
                    help="a joint faster than this (rad/s) switches torque off")
    ap.add_argument("--base-rpy", type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=("R", "P", "Y"),
                    help="how the base is turned from upright, degrees")
    ap.add_argument("--sim-mass-scale", type=float, default=1.0,
                    help="with --sim: the simulated arm's mass over the model's")
    args = ap.parse_args()
    if not 0.0 <= args.mass_g <= MAX_MASS_G:
        ap.error(f"--mass-g must be between 0 and {MAX_MASS_G:.0f}")

    base_rpy = np.radians(args.base_rpy)
    if args.sim:
        chain = SimChain(dt=1.0 / args.rate, mass_scale=args.sim_mass_scale, base_rpy=base_rpy)
    else:
        chain = LeaderChain(args.u2d2)
    try:
        chain.connect()
    finally:
        if chain.models:
            print(f"Servos on {chain.port}:\n{servo_report(chain)}")
    mismatches = chain.model_mismatches()
    if mismatches:
        chain.close()
        raise SystemExit("Not driving this arm: " + "; ".join(mismatches))
    latency = latency_timer_ms(chain.port)
    if latency is not None and latency > 1:
        # Every servo round trip then takes that long: the loop runs at a fraction of its rate
        # and a switch to position control leaves the arm limp for half a second.
        chain.close()
        raise SystemExit(f"Not driving this arm: the latency timer of {chain.port} is {latency} ms (1 ms "
                         "needed). Set it with\n  sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/"
                         f"{os.path.basename(os.path.realpath(chain.port))}/latency_timer'\nor make it "
                         "permanent with the udev rule in the README.")

    model = LeaderModel(base_rpy)
    # The model's limits are the xArm7's, so gello joints: the barrier needs them in raw_gello
    # joints (the shoulder flip mirrors J2's). The joints that turn a full circle each way (no
    # stops) are kept within half a turn of the start pose: their cables cannot wind, and the
    # next start reads them on the right turn.
    full_turn = model.upper - model.lower > 2.0 * np.pi
    a, b = raw_from_gello(model.lower), raw_from_gello(model.upper)
    model.lower = np.where(full_turn, START_Q - np.pi, np.minimum(a, b))
    model.upper = np.where(full_turn, START_Q + np.pi, np.maximum(a, b))
    gains = Gains(gravity_gain=args.gravity_gain)
    if args.joint_gain is not None:
        gains.gravity_joint_gain = np.array(args.joint_gain)
    law = TorqueLaw(model, gains)
    limit_mA = [args.limit_xm_mA if s.name.startswith("XM") else args.limit_xc_mA for s in chain.servos]
    loop = GravityComp(chain, law, args.rate, limit_mA, args.max_speed)
    loop.set_mass(args.mass_g, args.mass_mm)
    if args.trajectory:
        loop.trajectory = Trajectory.load(args.trajectory)
        loop.trajectory_message = f"Loaded {args.trajectory}."

    LOG_DIR.mkdir(exist_ok=True)
    stem = str(LOG_DIR / f"gravity_comp_{time.strftime('%Y%m%d_%H%M%S')}")
    log_path, sag_path = f"{stem}.npz", f"{stem}_sag.jsonl"

    viewer = Viewer(args.port, panel="Ditto GELLO", hands=args.hands)
    if args.hands:
        show_live(viewer, args.hand_u2d2)
    panel(viewer, loop, stem)
    print(f"Viewer running at http://localhost:{args.port}  (Ctrl+C to stop; torque is off until "
          f"you press Gravity compensation ON in the page)\nLogging to {log_path}", flush=True)

    def stop(*_):
        loop.running = False

    # Python hands the interpreter from thread to thread every 5 ms by default, and a servo
    # read asks for it back many times, so each redraw of the page (30 a second) can hold a
    # tick up for several of those. Measured on the first arm with a browser on the page:
    # 32 ticks a second over 4 ms at 0.5 ms, 6 at 0.02 ms. A late tick leaves the servos on an
    # old goal during playback, which is felt as roughness.
    sys.setswitchinterval(0.00002)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        loop.run()
    finally:
        loop.save_log(log_path)
        print(f"Saved {log_path}" + (f"\nSag test: {sag_path}  "
              "(python -m ditto_gello_software.sag_test prints the report)" if loop.sag.results else ""))


if __name__ == "__main__":
    main()
