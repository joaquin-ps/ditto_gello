"""Check the servo bus: how often a read of all servos fails, and whose answer goes missing.

    python -m ditto_gello_software.bus_check --u2d2 /dev/ttyUSB0
    python -m ditto_gello_software.bus_check --u2d2 /dev/ttyUSB0 --pairs   # about 2 minutes more

Read-only: no register is written and torque is left as it is. Stop every other program that
uses the port first.

In a read of several servos, each answers once it has heard the answer of the one before it in
the list, so one servo that misses its neighbour's answer silences every servo after it. This
tool sends the same read the control loop does, looks at the raw bytes that come back, and
reports which answers arrived. ``--pairs`` tries every servo directly after every other one,
which shows which two servos do not hear each other.
"""

from __future__ import annotations

import argparse
import time
from collections import Counter

import numpy as np
from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead

from ditto_gello_software.chain import MODEL_NAMES, LeaderChain, latency_timer_ms

REGISTERS = {  # name: (address, bytes); the same on XM430 and XC330
    "firmware": (6, 1), "return delay": (9, 1), "drive mode": (10, 1), "operating mode": (11, 1),
    "torque on": (64, 1), "hardware error": (70, 1), "volts x10": (144, 2), "deg C": (146, 1),
}
ADDR_STATE, LEN_STATE = 126, 10  # present current, velocity, position
HEADER = bytes([0xFF, 0xFF, 0xFD, 0x00])
LISTEN_S = 0.006  # a complete answer from seven servos takes about 1 ms


def settings(u2d2, ids) -> None:
    ph, port = u2d2._packetHandler, u2d2._portHandler
    print(f"{'ID':>3} {'model':<11}" + "".join(f"{name:>16}" for name in REGISTERS))
    for i in ids:
        row = []
        for address, size in REGISTERS.values():
            read = ph.read1ByteTxRx if size == 1 else ph.read2ByteTxRx
            value, result, _ = read(port, i, address)
            row.append(str(value) if result == COMM_SUCCESS else "?")
        model, result, _ = ph.read2ByteTxRx(port, i, 0)
        name = MODEL_NAMES.get(model, str(model)) if result == COMM_SUCCESS else "?"
        print(f"{i:>3} {name:<11}" + "".join(f"{v:>16}" for v in row))


def intact_ids(ph, stream: bytes) -> set[int]:
    """IDs of the complete status packets with a good checksum in ``stream``. A packet's length
    comes from its own length field: an answer whose data happens to contain the header's
    bytes is sent one byte longer (the protocol's byte stuffing), which is still a good packet."""
    ids, at = set(), stream.find(HEADER)
    while at >= 0:
        end = at + 7 + (stream[at + 5] + (stream[at + 6] << 8) if at + 7 <= len(stream) else 1 << 20)
        if end <= len(stream) and ph.updateCRC(0, list(stream[at:end - 2]), end - at - 2) == \
                stream[end - 2] + (stream[end - 1] << 8):
            ids.add(stream[at + 4])
            at = stream.find(HEADER, end)
        else:
            at = stream.find(HEADER, at + 1)
    return ids


def read_raw(u2d2, order, reads: int, rate_hz: float | None = None) -> tuple[int, Counter, float]:
    """Send ``reads`` sync reads listing the servos in ``order``. Returns how many came back
    incomplete, a count of which servos' answers were missing or damaged in those, and the
    median time for a complete answer (ms)."""
    ph, port = u2d2._packetHandler, u2d2._portHandler
    group = GroupSyncRead(port, ph, ADDR_STATE, LEN_STATE)
    for i in order:
        group.addParam(i)
    bad, missing, took = 0, Counter(), []
    for _ in range(reads):
        t0 = time.perf_counter()
        port.ser.reset_input_buffer()
        group.txPacket()
        port.is_using = False
        buf, intact = bytearray(), set()
        while len(intact) < len(order) and time.perf_counter() - t0 < LISTEN_S:
            got = port.readPort(64)
            if got:
                buf += bytes(got)
                intact = intact_ids(ph, bytes(buf))
        if len(intact) == len(order):
            took.append(time.perf_counter() - t0)
        else:
            bad += 1
            missing[tuple(i for i in order if i not in intact)] += 1
        if rate_hz:
            time.sleep(max(0.0, 1.0 / rate_hz - (time.perf_counter() - t0)))
    return bad, missing, 1000.0 * float(np.median(took)) if took else float("nan")


def order_test(u2d2, order, reads: int, rate_hz: float | None, label: str) -> None:
    bad, missing, ms = read_raw(u2d2, order, reads, rate_hz)
    pace = f"at {rate_hz:.0f} Hz" if rate_hz else "back to back"
    print(f"\n{label}, order {list(order)}, {reads} reads {pace}: "
          f"{bad} incomplete ({100.0 * bad / reads:.2f} %), a complete one takes {ms:.2f} ms")
    for gone, k in missing.most_common():
        print(f"  {k} x no intact answer from ID {list(gone)}")


def sdk_test(u2d2, reads: int, rate_hz: float) -> None:
    """The read the control loop makes, through the SDK's own parser, without the repeat."""
    codes, slowest = Counter(), 0.0
    log, u2d2._log = u2d2._log, lambda msg: None
    for _ in range(reads):
        t0 = time.perf_counter()
        try:
            u2d2.sync_read_state()
        except RuntimeError as e:
            codes[str(e)] += 1
        slowest = max(slowest, time.perf_counter() - t0)
        time.sleep(max(0.0, 1.0 / rate_hz - (time.perf_counter() - t0)))
    u2d2._log = log
    failed = sum(codes.values())
    print(f"\nas the control loop reads, {reads} reads at "
          f"{rate_hz:.0f} Hz: {failed} failed ({100.0 * failed / reads:.2f} %), "
          f"slowest read {1000.0 * slowest:.1f} ms" + (f"  {dict(codes)}" if codes else ""))


def pair_test(u2d2, ids, reads: int) -> None:
    print(f"\nevery servo answering directly after every other one, {reads} reads each "
          "(incomplete reads; rows: first to answer, columns: second):")
    print("      " + "".join(f"{b:>6}" for b in ids))
    for a in ids:
        cells = []
        for b in ids:
            cells.append("     -" if a == b else f"{read_raw(u2d2, (a, b), reads)[0]:>6}")
        print(f"  {a:>2}: " + "".join(cells), flush=True)


def main() -> None:
    ap = argparse.ArgumentParser(description="Check the leader arm's servo bus (read-only).")
    ap.add_argument("--u2d2", metavar="PORT", required=True, help="serial port, e.g. /dev/ttyUSB0")
    ap.add_argument("--reads", type=int, default=4000, help="reads per test")
    ap.add_argument("--pairs", action="store_true", help="also test every pair of servos")
    args = ap.parse_args()

    chain = LeaderChain(args.u2d2)
    chain.connect()
    u2d2 = chain._u2d2
    print(f"{args.u2d2}: latency timer {latency_timer_ms(args.u2d2)} ms, {chain.baudrate} bps\n")
    try:
        settings(u2d2, chain.ids)
        sdk_test(u2d2, args.reads, 500.0)
        order_test(u2d2, chain.ids, args.reads, 500.0, "raw bytes")
        order_test(u2d2, chain.ids, args.reads, None, "raw bytes")
        if args.pairs:
            pair_test(u2d2, chain.ids, max(500, args.reads // 2))
    finally:
        chain.close()


if __name__ == "__main__":
    main()
