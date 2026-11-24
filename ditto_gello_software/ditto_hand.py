"""The DITTO leader hand, read only: its finger angles, live, for the viewer.

The hand has its own U2D2 (``/dev/ttyUSB1`` by default), servos 121 to 123 (index 1 to 3) and
111 to 114 (thumb 1 to 4) at 4 Mbps. Nothing is written to them: their positions are read and
turned into the hand model's joint angles the way roamlab/ditto does it for the 7dof_f02 hand
config (``hand_interfaces``: zero at 2048 counts, direction 1, no transmission), which are the
angles of ``ditto_leader_2f.urdf``. The follower hand is shown at those angles plus 7dof_f02's
leader offsets (the follower is held that far ahead of the leader in teleop, OFFSETS.md).
"""

from __future__ import annotations

import threading
import time

import numpy as np

FINGERS = ("index1", "index2", "index3", "thumb1", "thumb2", "thumb3", "thumb4")
LEADER_IDS = (121, 122, 123, 111, 112, 113, 114)  # in FINGERS order
FOLLOWER_OFFSETS = np.radians([0.0, 15.0, 5.0, 0.0, 0.0, 0.0, 0.0])  # 7dof_f02 leader.follower_offsets_deg
HAND_HOME = np.radians([5.0, 50.0, -5.0, -5.0, -55.0, 0.0, -35.0])  # the rig's follower home (no hand read)
ZERO_COUNTS, RAD_PER_COUNT = 2048, 2.0 * np.pi / 4096


class LeaderHand:
    """Reads the leader hand in a thread of its own; ``q`` is the latest finger angles (rad,
    FINGERS order) and ``error`` the last read failure ('' when reads work)."""

    def __init__(self, port: str = "/dev/ttyUSB1", baudrate: int = 4_000_000, rate_hz: float = 30.0) -> None:
        from dynamixel_u2d2 import U2D2Interface

        self.port = port
        self._u2d2 = U2D2Interface(port, baudrate, motor_ids=list(LEADER_IDS))
        self.q = self._read()  # raises if the hand does not answer
        self.error = ""
        self._dt = 1.0 / rate_hz
        self._running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _read(self) -> np.ndarray:
        pos, _, _ = self._u2d2.sync_read_state()
        if any(p is None for p in pos):
            raise RuntimeError(f"no data from servo ID {[i for i, p in zip(LEADER_IDS, pos) if p is None]}")
        return (np.array(pos, dtype=float) - ZERO_COUNTS) * RAD_PER_COUNT

    def _run(self) -> None:
        while self._running:
            try:
                self.q, self.error = self._read(), ""
            except Exception as e:  # a missed read: keep the last angles, say why
                self.error = str(e)
            time.sleep(self._dt)

    def close(self) -> None:
        self._running = False
        time.sleep(2 * self._dt)
        self._u2d2.close()


def show_live(viewer, port: str = "/dev/ttyUSB1") -> LeaderHand | None:
    """Keep ``viewer``'s DITTO hands on the leader hand's angles, read on ``port``. If the hand
    does not answer, they stay at the rig's home and the page says why; returns None then."""
    try:
        hand = LeaderHand(port)
    except Exception as e:
        print(f"Leader hand on {port} not read ({e}); the hands are shown at the rig's home.", flush=True)
        viewer.set_fingers(HAND_HOME, f"Leader hand on `{port}`: **not read** ({e}). Shown at the rig's home.")
        return None
    print(f"Leader hand on {port}: live", flush=True)

    def run() -> None:
        while hand._running:
            d = np.degrees(hand.q)
            viewer.set_fingers(hand.q, (
                f"Live from the leader hand on `{port}`"
                + (f", **last read failed** ({hand.error})" if hand.error else "") + ".  \n"
                + f"index {d[0]:.0f}, {d[1]:.0f}, {d[2]:.0f}°; thumb {d[3]:.0f}, {d[4]:.0f}, {d[5]:.0f}, {d[6]:.0f}°"))
            time.sleep(1.0 / 30.0)

    threading.Thread(target=run, daemon=True).start()
    return hand
