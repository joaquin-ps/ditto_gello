"""Viewer for the leader arm and the xArm7, with the option of showing the real arm.

    python -m ditto_gello_software.view                       # sliders only
    python -m ditto_gello_software.view --u2d2 /dev/ttyUSB0   # the real arm sets the pose
    python -m ditto_gello_software.view --fake                # the same, without an arm

Then open http://localhost:8080. Run it from the repository's top folder.

With ``--u2d2`` the servos (IDs 1 to 7 at 4 Mbps) are only read: torque stays as it is, so move
the arm by hand. Use it to check an assembled arm against the design:

- every ID answers and is the servo model the arm has at that joint;
- moving joint N by hand moves joint N on the screen, the same way;
- at the Zero pose every servo reads 2048 (the "from zero" column reads 0).
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from ditto_gello_software.chain import (
    MODEL_NAMES,
    RAD_PER_COUNT,
    FakeChain,
    LeaderChain,
    latency_timer_ms,
)
from ditto_gello_software.ditto_hand import show_live
from ditto_gello_software.viewer import Viewer

POSE_HZ = 30.0  # how often the arms on the screen are moved
TEXT_HZ = 5.0  # how often the table and the sliders are rewritten

# A servo this hot can deform the PLA bolted to it. PLA softens at its glass transition, about
# 55 to 60 C, and creeps under load a little below that. The servo reports the temperature
# inside its case, which is hotter than the surface the print touches, so 50 C leaves margin.
PLA_WARN_C = 50.0
PLA_NEAR_C = PLA_WARN_C - 5.0
TEMPERATURE_BUTTON = "Check servo temperatures"


def servo_report(chain: LeaderChain) -> str:
    lines = []
    for i, servo in zip(chain.ids, chain.servos):
        got = chain.models.get(i)
        name = "no answer" if got is None else MODEL_NAMES.get(got, f"model {got}")
        ok = "" if got == servo.model_number else f"   <-- the arm has a {servo.name} here"
        lines.append(f"  ID {i} (J{i}): {name}{ok}")
    return "\n".join(lines)


def temperature_table(chain: LeaderChain) -> str:
    """Read every servo's temperature and its shutdown limit, as a table for the page. Call it
    with torque off, from the thread that owns the port."""
    try:
        temp, limit = chain.temperatures()
    except Exception as e:
        return f"**Could not read the temperatures** ({e})."
    rows = [f"Read at {time.strftime('%H:%M:%S')}. PLA warning at {PLA_WARN_C:.0f} °C.", "",
            "| J | servo | °C | servo limit | below limit | PLA |", "|---|---|---|---|---|---|"]
    for k, servo in enumerate(chain.servos):
        if temp[k] >= PLA_WARN_C:
            pla = f"**too hot: {temp[k] - PLA_WARN_C:.0f} °C over, let it cool**"
        elif temp[k] >= PLA_NEAR_C:
            pla = f"close: {PLA_WARN_C - temp[k]:.0f} °C left"
        else:
            pla = f"ok, {PLA_WARN_C - temp[k]:.0f} °C left"
        rows.append(f"| {k + 1} | {servo.name} | {temp[k]:.0f} | {limit[k]:.0f} "
                    f"| {limit[k] - temp[k]:.0f} °C ({100 * temp[k] / limit[k]:.0f} % of it) | {pla} |")
    hottest = int(np.argmax(temp))
    if temp[hottest] >= PLA_WARN_C:
        rows.insert(0, f"**J{hottest + 1} is at {temp[hottest]:.0f} °C: hot enough to deform PLA.**  ")
    return "\n".join(rows)


def table(chain: LeaderChain, state) -> str:
    rows = ["| J | servo | counts | from zero | degrees |", "|---|---|---|---|---|"]
    outside = chain.outside_limits(state.q)
    for k, i in enumerate(chain.ids):
        got = chain.models.get(i)
        name = MODEL_NAMES.get(got, f"model {got}")
        if got != chain.servos[k].model_number:
            name = f"**{name}, not {chain.servos[k].name}**"
        from_zero = state.q[k] / RAD_PER_COUNT  # counts, through sign, zero and turn count
        deg = f"{np.degrees(state.q[k]):.1f}" + (" **outside limits**" if outside[k] else "")
        rows.append(f"| {i} | {name} | {state.raw[k]:.0f} | {from_zero:+.0f} | {deg} |")
    return "\n".join(rows)


def main() -> None:
    ap = argparse.ArgumentParser(description="Viewer for the Ditto GELLO leader arm and the xArm7.")
    ap.add_argument("--port", type=int, default=8080, help="port of the web page")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--u2d2", metavar="PORT", help="serial port of the real arm's U2D2, e.g. /dev/ttyUSB0")
    src.add_argument("--fake", action="store_true", help="a pretend arm moving through the poses")
    ap.add_argument("--rate", type=float, default=100.0, help="how often the servos are read, Hz")
    ap.add_argument("--hands", action="store_true",
                    help="show the DITTO hands: the leader on the gello, the follower on the xArm7")
    ap.add_argument("--hand-u2d2", metavar="PORT", default="/dev/ttyUSB1",
                    help="with --hands: the DITTO leader hand's U2D2, read only (default /dev/ttyUSB1)")
    args = ap.parse_args()

    chain = None
    if args.fake:
        chain = FakeChain()
    elif args.u2d2:
        chain = LeaderChain(args.u2d2)
    if chain is not None:
        try:
            chain.connect()
        except Exception:
            if chain.models:
                print(f"Servos on {chain.port}:\n{servo_report(chain)}")
            raise
        print(f"Servos on {chain.port}:\n{servo_report(chain)}")
        latency = latency_timer_ms(chain.port)
        if latency is not None and latency > 1:
            print(f"Note: the latency timer of {chain.port} is {latency} ms; "
                  "`dynamixel-port --latency-timer 1` sets it to 1 ms for faster reads.")

    viewer = Viewer(args.port, panel="Real arm" if chain else None, hands=args.hands)
    if args.hands:
        show_live(viewer, args.hand_u2d2)
    print(f"Viewer running at http://localhost:{args.port}  (Ctrl+C to stop)")
    if chain is None:
        while True:
            time.sleep(1.0)

    with viewer.panel:
        follow = viewer.server.gui.add_checkbox("the arm sets the pose", True)
        status = viewer.server.gui.add_markdown("")
        readout = viewer.server.gui.add_markdown("")
        temp_btn = viewer.server.gui.add_button(TEMPERATURE_BUTTON)
        temp_md = viewer.server.gui.add_markdown("")
    want_temperatures = []  # the button leaves a request; the loop below owns the port
    temp_btn.on_click(lambda _: want_temperatures.append(True))
    follow.on_update(lambda _: viewer.set_external(follow.value))
    viewer.set_external(True)

    period = 1.0 / args.rate
    reads = errors = 0
    last_error = ""
    state = None
    next_pose = next_text = rate_t0 = time.monotonic()
    rate_reads, hz = 0, 0.0
    try:
        while True:
            tick = time.monotonic()
            if want_temperatures:  # this program never switches torque on, so it may read now
                want_temperatures.clear()
                temp_md.content = temperature_table(chain)
            try:
                state = chain.read()
                reads += 1
            except RuntimeError as e:
                errors += 1
                last_error = str(e)
            now = time.monotonic()
            if state is not None and follow.value and now >= next_pose:
                viewer.set_joints(state.q)
                next_pose = now + 1.0 / POSE_HZ
            if now >= next_text:
                hz = (reads - rate_reads) / (now - rate_t0) if now > rate_t0 else hz
                rate_reads, rate_t0 = reads, now
                stale = state is None or now - state.t > 0.5
                status.content = (
                    f"`{chain.port}`: "
                    + ("**no data from the servos**" if stale else f"reading at {hz:.0f} Hz")
                    + f"  \nfailed reads: {errors}" + (f" (last: {last_error})" if errors else "")
                    + f", repeated: {chain.read_retries}"
                )
                if state is not None:
                    readout.content = table(chain, state)
                    if follow.value:
                        viewer.show_on_sliders(state.q)
                next_text = now + 1.0 / TEXT_HZ
            time.sleep(max(0.0, period - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        pass
    finally:
        chain.close()


if __name__ == "__main__":
    main()
