"""Sag test: find which joints get too little or too much gravity compensation.

The test runs inside ``gravity_comp`` (the page's "Sag test" folder) and talks you through one
measurement after another:

1. MOVE the arm to a pose and hold it still;
2. it counts down, then you LET GO;
3. it watches the arm for a few seconds, then you CATCH it.

A joint that moves the way gravity pulls it gets too little compensation at that pose; one
that moves against gravity gets too much; one that stays is within what friction hides. A
joint that moves a little while another falls far is marked "unclear" and not counted: fix
the joint that fell, then measure again. Each
measurement is one line of a ``..._sag.jsonl`` file in ``logs/``. Over several poses and gain
settings the lines bracket, for every joint, the share of the model's gravity torque that
balances it:

    python -m ditto_gello_software.sag_test            # report on the newest log
    python -m ditto_gello_software.sag_test logs/gravity_comp_20261118_161800_sag.jsonl

"Ratio" below is gravity gain x the joint's own factor: the share of the model's gravity
torque that was being applied to that joint.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

LOG_DIR = Path(__file__).resolve().parents[1] / "logs"

IDLE, MOVE, COUNTDOWN, MEASURE, RESULT = "idle", "move", "countdown", "measure", "result"


class SagTest:
    """The test's steps, advanced once per control tick by :meth:`update`. It only reads the
    arm's state and writes ``prompt`` and ``results``; it commands nothing."""

    STILL_SPEED = 0.06  # rad/s: slower than this on every joint is "still"
    MOVING_SPEED = 0.3  # rad/s: faster than this on any joint is "being moved"
    STILL_S = 1.0  # how long the arm must be still before the countdown starts
    COUNTDOWN_S = 3.0
    MEASURE_S = 3.0
    MOVED_DEG = 1.0  # a joint that drifts less than this "stayed"
    ABORT_DEG = 15.0  # a joint that drifts this far ends the measurement early
    MIN_LOAD = 0.02  # N m: below this gravity torque a joint says nothing at that pose
    SWING_DEG = 10.0  # another joint drifting this far, and SWING_RATIO times further than
    SWING_RATIO = 3.0  # this one, makes this one's verdict "unclear"

    def __init__(self, law) -> None:
        self.law = law
        self.phase = IDLE
        self.prompt = ""
        self.results: list[dict] = []
        self._t_phase = self._t_still = 0.0
        self._armed = True  # the arm has been moved since the last measurement
        self._held: list[np.ndarray] = []

    def start(self) -> None:
        self._armed = True
        self._go(MOVE, 0.0)

    def stop(self) -> None:
        self.phase, self.prompt = IDLE, ""

    def _go(self, phase: str, t: float) -> None:
        self.phase, self._t_phase, self._t_still = phase, t, t

    def update(self, t: float, state, tau, tau_cmd, torque_on: bool, context: dict) -> None:
        """``t``: seconds; ``tau`` and ``tau_cmd``: torques asked for and sent this tick;
        ``context``: what to store with a measurement (gains, mass)."""
        if self.phase == IDLE:
            return
        speed = float(np.abs(state.qd).max())
        if not torque_on and self.phase != MEASURE:
            self.prompt = "Sag test: press **Gravity compensation ON** first (hold the arm)."
            self._go(MOVE, t)
            return

        if self.phase in (MOVE, RESULT):
            if speed > self.MOVING_SPEED:
                self._armed = True
                if self.phase == RESULT:
                    self._go(MOVE, t)
            if self.phase == MOVE:
                self.prompt = "**MOVE** the arm to a new pose, then hold it still."
            if speed > self.STILL_SPEED:
                self._t_still = t
            elif self._armed and t - self._t_still >= self.STILL_S:
                self._held = []
                self._go(COUNTDOWN, t)

        elif self.phase == COUNTDOWN:
            left = self.COUNTDOWN_S - (t - self._t_phase)
            self.prompt = f"**HOLD STILL**: let go in {max(1, int(np.ceil(left)))}"
            if speed > self.MOVING_SPEED:
                self._go(MOVE, t)
            elif left <= 0.3:
                self._held.append(state.q.copy())
            if left <= 0.0 and self._held:
                self._q0 = np.mean(self._held, axis=0)
                self._load = self.law.model.inverse_dynamics(self._q0, np.zeros(self.law.n))
                self._context = dict(context)
                self._peak_speed, self._capped = 0.0, np.zeros(self.law.n, dtype=bool)
                self._last: list[np.ndarray] = []
                self._go(MEASURE, t)

        elif self.phase == MEASURE:
            left = self.MEASURE_S - (t - self._t_phase)
            self.prompt = f"**LET GO, hands off**: measuring, {max(0.0, left):.0f} s left"
            self._peak_speed = max(self._peak_speed, speed)
            # more than the rounding to whole current units (at most 0.0014 N m) explains
            self._capped |= np.abs(tau - tau_cmd) > 0.005 + 0.02 * np.abs(tau)
            self._last = (self._last + [state.q.copy()])[-50:]
            drift = np.degrees(state.q - self._q0)
            ended_early = bool(np.abs(drift).max() > self.ABORT_DEG) or not torque_on
            if left <= 0.0 or ended_early:
                self._finish(t, ended_early)

    def _finish(self, t: float, ended_early: bool) -> None:
        drift = np.degrees(np.mean(self._last[-10:], axis=0) - self._q0)
        ratio = self._context["gravity_gain"] * np.asarray(self._context["joint_gain"])
        verdict = []
        for j in range(self.law.n):
            if abs(self._load[j]) < self.MIN_LOAD:
                verdict.append("no load")
            elif abs(drift[j]) < self.MOVED_DEG:
                verdict.append("stayed")
            else:  # the load is the torque that holds the joint: gravity pulls the other way
                verdict.append("sagged" if drift[j] * self._load[j] < 0 else "rose")
        # A joint that falls far swings the others with it, so a small movement beside a
        # large one says little about the small one's own balance.
        for j in range(self.law.n):
            others = np.abs(np.delete(drift, j)).max()
            if (verdict[j] in ("sagged", "rose") and others > self.SWING_DEG
                    and others > self.SWING_RATIO * abs(drift[j])):
                verdict[j] = "unclear"
        result = {
            "t": round(t, 3),
            **self._context,
            "ratio": ratio.round(4).tolist(),
            "pose_deg": np.degrees(self._q0).round(2).tolist(),
            "load_Nm": self._load.round(4).tolist(),
            "drift_deg": drift.round(2).tolist(),
            "verdict": verdict,
            "capped": self._capped.tolist(),
            "peak_speed": round(self._peak_speed, 3),
            "seconds": round(t - self._t_phase, 2),
            "ended_early": ended_early,
        }
        self.results.append(result)
        moved = [f"J{j + 1} {v} {abs(drift[j]):.0f}°" for j, v in enumerate(verdict)
                 if v in ("sagged", "rose")]
        self.prompt = ("**CATCH the arm.** " + (", ".join(moved) if moved else "Nothing moved.")
                       + "  \nMove it to another pose for the next measurement.")
        self._armed = False
        self._go(RESULT, t)


def summarise(results: list[dict], n: int = 7) -> list[dict]:
    """Per joint, what the measurements say about the ratio that balances it: it is above every
    ratio the joint sagged at and below every ratio it rose at. Capped joints are left out
    (they were getting less than the ratio says)."""
    rows = []
    for j in range(n):
        seen = {"sagged": [], "rose": [], "stayed": []}
        for r in results:
            v = r["verdict"][j]
            if v in seen and not r["capped"][j]:
                seen[v].append(r["ratio"][j])
        low = max(seen["sagged"]) if seen["sagged"] else None
        high = min(seen["rose"]) if seen["rose"] else None
        if not any(seen.values()):
            advice, target = "no gravity load in the poses tried", None
        elif low is not None and high is not None and low < high:
            target = 0.5 * (low + high)
            advice = f"between {low:.2f} and {high:.2f}"
        elif low is not None and high is not None:
            target = 0.5 * (low + high)
            advice = (f"mixed: sagged at {low:.2f}, rose at {high:.2f} "
                      "(friction, or the model is off differently in different poses)")
        elif low is not None:
            above = [x for x in seen["stayed"] if x > low]
            target = float(np.median(above)) if above else None
            advice = f"above {low:.2f}: raise it" + (f"; stayed at {min(above):.2f}" if above else "")
        elif high is not None:
            below = [x for x in seen["stayed"] if x < high]
            target = float(np.median(below)) if below else None
            advice = f"below {high:.2f}: lower it" + (f"; stayed at {max(below):.2f}" if below else "")
        else:
            target = float(np.median(seen["stayed"]))
            advice = (f"stayed at {min(seen['stayed']):.2f} to {max(seen['stayed']):.2f}: "
                      "within friction")
        rows.append({"joint": j + 1, "sagged": len(seen["sagged"]), "rose": len(seen["rose"]),
                     "stayed": len(seen["stayed"]), "advice": advice, "target": target})
    return rows


def summary_table(results: list[dict], gravity_gain: float | None = None) -> str:
    """Markdown table of :func:`summarise`. With ``gravity_gain`` it adds the joint factor that
    gives each joint its suggested ratio at that gain."""
    head = "| J | sagged | rose | stayed | balancing ratio |"
    rule = "|---|---|---|---|---|"
    if gravity_gain:
        head, rule = head + " factor to try |", rule + "---|"
    lines = [head, rule]
    for row in summarise(results):
        line = (f"| {row['joint']} | {row['sagged']} | {row['rose']} | {row['stayed']} "
                f"| {row['advice']} |")
        if gravity_gain:
            line += (f" {row['target'] / gravity_gain:.2f} |" if row["target"] is not None else " |")
        lines.append(line)
    return "\n".join(lines)


def last_table(result: dict) -> str:
    lines = ["| J | ratio | load N m | drift ° | |", "|---|---|---|---|---|"]
    for j, v in enumerate(result["verdict"]):
        note = v + (" (capped)" if result["capped"][j] else "")
        lines.append(f"| {j + 1} | {result['ratio'][j]:.2f} | {result['load_Nm'][j]:+.3f} "
                     f"| {result['drift_deg'][j]:+.1f} | {note} |")
    return "\n".join(lines)


def load(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def newest_log() -> Path | None:
    logs = sorted(LOG_DIR.glob("*_sag.jsonl"))
    return logs[-1] if logs else None


def main() -> None:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else newest_log()
    if path is None or not path.exists():
        raise SystemExit(f"No sag-test log found ({path or LOG_DIR}).")
    results = load(path)
    print(f"{path}: {len(results)} measurement(s)\n")
    for k, r in enumerate(results):
        print(f"#{k + 1}  gravity gain {r['gravity_gain']:.2f}, mass {r['mass_g']:.0f} g at "
              f"{r['mass_mm']:.0f} mm, pose {r['pose_deg']}"
              + ("  (ended early)" if r["ended_early"] else ""))
        print(last_table(r) + "\n")
    if results:
        print("All measurements, per joint:\n")
        print(summary_table(results, results[-1]["gravity_gain"]))
        print(f"\n(\"factor to try\" is for gravity gain {results[-1]['gravity_gain']:.2f}.)")


if __name__ == "__main__":
    main()
