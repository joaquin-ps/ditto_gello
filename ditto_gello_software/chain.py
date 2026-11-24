"""The leader's seven servos on one U2D2, through our ``dynamixel_u2d2`` library.

Joint ``i`` is servo ID ``i`` (1 to 7, from the base). Everything that leaves this module is in
joint units: radians, radians per second, newton-metres. A joint's angle is zero when its servo
reads ``ZERO_COUNTS`` (2048, the middle of the range), which is the position the build guide
bolts each link on at.

Reading never changes a servo: :meth:`LeaderChain.connect` and :meth:`LeaderChain.read` only
read registers. Only :meth:`LeaderChain.torque_on` and :meth:`LeaderChain.write_torque` write.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass

import numpy as np
import yourdfpy

from ditto_gello_software import NUM_JOINTS, START_RAW_RAD, leader_urdf

COUNTS_PER_REV = 4096
ZERO_COUNTS = 2048
RAD_PER_COUNT = 2.0 * np.pi / COUNTS_PER_REV
RAD_S_PER_UNIT = 0.229 * 2.0 * np.pi / 60.0  # Present Velocity: 0.229 rev/min per unit
BAUDRATE = 4_000_000

ADDR_TEMPERATURE_LIMIT, ADDR_PRESENT_TEMPERATURE = 31, 146  # one byte each, degrees C

# Position gains the servos power up with (read from the first arm), in register units.
POSITION_P_GAIN = {"XM430-W210": 800, "XC330-T288": 1200}


@dataclass(frozen=True)
class Servo:
    name: str
    model_number: int
    mA_per_unit: float  # Goal Current and Present Current register unit
    # Goal Current units per newton-metre. FACTR's values (factr_teleop dynamixel/driver.py,
    # TORQUE_TO_CURRENT_MAPPING); not yet checked on our servos.
    units_per_Nm: float


XM430 = Servo("XM430-W210", 1030, 2.69, 1000.0 / 2.69)
XC330 = Servo("XC330-T288", 1220, 1.0, 1158.73)
SERVOS = (XC330, XM430, XC330, XM430, XC330, XC330, XC330)  # J1 to J7 (ditto_gello_hardware/README.md, "Servo setup")
MODEL_NAMES = {1030: "XM430-W210", 1220: "XC330-T288", 1210: "XC330-T181", 1020: "XM430-W350"}


@dataclass
class State:
    t: float  # time.monotonic() of the read
    q: np.ndarray  # joint angles, rad
    qd: np.ndarray  # joint velocities, rad/s
    current_mA: np.ndarray  # present current, signed with the joint
    raw: np.ndarray  # Present Position, counts


def latency_timer_ms(port: str) -> int | None:
    """The FTDI latency timer of a ``/dev/ttyUSB*`` port, or None if it cannot be read. Above
    1 ms it caps how fast the servos can be read."""
    name = os.path.basename(os.path.realpath(port))
    try:
        with open(f"/sys/bus/usb-serial/devices/{name}/latency_timer") as f:
            return int(f.read())
    except (OSError, ValueError):
        return None


def joint_limits() -> tuple[np.ndarray, np.ndarray]:
    """The leader's joint limits (rad), from its URDF. They are the xArm7's."""
    u = yourdfpy.URDF.load(str(leader_urdf()), load_meshes=False)
    lim = [u.joint_map[n].limit for n in u.actuated_joint_names]
    return np.array([x.lower for x in lim]), np.array([x.upper for x in lim])


def probe(port: str, ids=range(1, NUM_JOINTS + 1), baudrate: int = BAUDRATE) -> dict[int, int | None]:
    """Ask each ID for its model number: ``{id: model number, or None if it did not answer}``."""
    from dynamixel_u2d2 import U2D2Interface

    u2d2 = U2D2Interface(port, baudrate)
    u2d2._log = lambda msg: None  # a servo that does not answer is a result here, not an error
    found: dict[int, int | None] = {}
    try:
        for i in ids:
            try:
                found[i] = int(u2d2.get_model_number(i))
            except RuntimeError:
                found[i] = None
    finally:
        u2d2.close()
    return found


class LeaderChain:
    """Read the leader's joints, and (once :meth:`torque_on` is called) drive them with torque.

    ``sign`` and ``zero_counts`` are per joint; the design has every sign +1 and every zero at
    2048, so they are here for an arm that was put together differently.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = BAUDRATE,
        sign=None,
        zero_counts=None,
    ) -> None:
        self.port, self.baudrate = port, baudrate
        self.ids = list(range(1, NUM_JOINTS + 1))
        self.read_retries = 0  # reads that failed once and worked when repeated
        self._retry_said = 0.0
        self.servos = SERVOS
        self.sign = np.ones(NUM_JOINTS) if sign is None else np.array(sign, dtype=float)
        self.zero_counts = (np.full(NUM_JOINTS, float(ZERO_COUNTS)) if zero_counts is None
                            else np.array(zero_counts, dtype=float))
        self.lower, self.upper = joint_limits()
        # Where each joint is assumed to be when its turn count is unknown (a servo forgets it
        # at power-up and reports one turn, 0 to 4095): the middle of its range, or, for the
        # joints that turn a full circle each way, the start pose (raw_gello). Such a joint is
        # read right if it is within 180 degrees of that pose when the program starts. J7 sits
        # near -167 there, where a centre at zero would misread it by a turn.
        full_turn = self.upper - self.lower > 2.0 * np.pi
        self.centre = np.where(full_turn, START_RAW_RAD, 0.5 * (self.lower + self.upper))
        self.models: dict[int, int | None] = {}
        self.torque_enabled = False
        self.position_mode = False  # the servos are holding positions, not taking torques
        self.limit_units = np.zeros(NUM_JOINTS)
        self._units_per_Nm = np.array([s.units_per_Nm for s in self.servos])
        self._mA_per_unit = np.array([s.mA_per_unit for s in self.servos])
        self._turns: np.ndarray | None = None
        self._last_q: np.ndarray | None = None
        self._u2d2 = None

    # ---- connection ---------------------------------------------------------------------------
    def _open(self):
        import dynamixel_sdk.port_handler as sdk_port
        from dynamixel_u2d2 import U2D2Interface

        # How long the SDK waits for an answer is built from this constant, which assumes the
        # USB adapter's default 16 ms latency timer: 34 ms, or 17 ticks of the loop with the
        # torques frozen. With the timer at 1 ms an answer is back in 1.5 ms, so 10 ms is
        # plenty, and still longer than the pauses Python's own threads cause.
        latency = latency_timer_ms(self.port)
        if latency is not None:
            sdk_port.LATENCY_TIMER = max(4, latency)
        self.models = probe(self.port, self.ids, self.baudrate)
        missing = [i for i, m in self.models.items() if m is None]
        if missing:
            raise ConnectionError(
                f"no answer from servo ID {missing} on {self.port} at {self.baudrate} bps "
                f"(answered: {[i for i, m in self.models.items() if m is not None] or 'none'}). "
                "Check power, the cable chain, and each servo's ID and baud rate "
                "(dynamixel-scan finds servos at any baud rate)."
            )
        return U2D2Interface(self.port, self.baudrate, motor_ids=self.ids)

    def connect(self) -> None:
        self._u2d2 = self._open()
        self.read()

    def model_mismatches(self) -> list[str]:
        """Servos whose model is not the one the arm has at that joint."""
        out = []
        for i, servo in zip(self.ids, self.servos):
            got = self.models.get(i)
            if got is not None and got != servo.model_number:
                out.append(f"ID {i} is a {MODEL_NAMES.get(got, f'model {got}')}, "
                           f"the arm has a {servo.name} at J{i}")
        return out

    def close(self) -> None:
        if self._u2d2 is None:
            return
        try:
            if self.torque_enabled:
                self.torque_off()
        finally:
            self._u2d2.close()
            self._u2d2 = None

    # ---- reading ------------------------------------------------------------------------------
    def read(self) -> State:
        """One sync read of all seven servos, repeated once if it fails. RuntimeError if the
        repeat fails too, or a servo's data is missing from the read."""
        try:
            pos, vel, cur = self._u2d2.sync_read_state()
        except RuntimeError as e:
            stamp = time.strftime("%H:%M:%S")
            try:
                pos, vel, cur = self._u2d2.sync_read_state()
            except RuntimeError:
                print(f"[chain {stamp}] servo read failed twice in a row ({e})", flush=True)
                raise
            self.read_retries += 1
            if time.monotonic() - self._retry_said > 1.0:  # at most one line a second
                self._retry_said = time.monotonic()
                print(f"[chain {stamp}] servo read failed ({e}); the repeat worked "
                      f"({self.read_retries} so far)", flush=True)
        if any(v is None for v in (*pos, *vel, *cur)):
            bad = [i for i, p in zip(self.ids, pos) if p is None]
            raise RuntimeError(f"sync read returned no data for servo ID {bad}")
        raw = np.array(pos, dtype=float)
        q = self.sign * (raw - self.zero_counts) * RAD_PER_COUNT
        if self._turns is None:
            # First read, or the servos' own turn count may have changed: take the turn that
            # keeps each joint where it was, or nearest its centre if this is the first read.
            near = self.centre if self._last_q is None else self._last_q
            self._turns = np.round((near - q) / (2.0 * np.pi))
        q = q + 2.0 * np.pi * self._turns
        self._last_q = q
        return State(
            t=time.monotonic(),
            q=q,
            qd=self.sign * np.array(vel, dtype=float) * RAD_S_PER_UNIT,
            current_mA=self.sign * np.array(cur, dtype=float) * self._mA_per_unit,
            raw=raw,
        )

    def outside_limits(self, q) -> np.ndarray:
        return (q < self.lower) | (q > self.upper)

    def _read_byte(self, address: int) -> np.ndarray:
        """One one-byte register from every servo, in joint order (one bulk read, repeated
        once if it fails)."""
        params = [(i, address, 1) for i in self.ids]
        got = self._u2d2.bulk_read(params) or self._u2d2.bulk_read(params)
        if not got:
            raise RuntimeError(f"bulk read of register {address} failed")
        return np.array([got[(i, address)][0] for i in self.ids], dtype=float)

    def temperatures(self) -> tuple[np.ndarray, np.ndarray]:
        """Each servo's internal temperature and its shutdown limit (degrees C, J1 to J7)."""
        return self._read_byte(ADDR_PRESENT_TEMPERATURE), self._read_byte(ADDR_TEMPERATURE_LIMIT)

    # ---- torque -------------------------------------------------------------------------------
    def torque_on(self, limit_mA) -> None:
        """Put every servo in current control with torque enabled, commanding zero. ``limit_mA``
        (one value or one per joint) caps what :meth:`write_torque` may ask of each servo."""
        mismatches = self.model_mismatches()
        if mismatches:
            raise RuntimeError("refusing to drive the arm: " + "; ".join(mismatches))
        self.limit_units = np.broadcast_to(np.asarray(limit_mA, float), (NUM_JOINTS,)) / self._mA_per_unit
        for i in self.ids:  # the operating mode can only change with torque off
            self._u2d2.disable_torque(i)
            self._u2d2.set_motor_mode(i, "current")
        self._turns = None  # a mode change may reset a servo's turn count
        self.read()
        self._u2d2.sync_write_currents([0] * NUM_JOINTS)
        for i in self.ids:
            self._u2d2.enable_torque(i)
        self.torque_enabled, self.position_mode = True, False

    def position_on(self, limit_mA, p_gain, d_gain, i_gain=None) -> np.ndarray:
        """Put every servo in its own position control, holding where it is (the joint angles
        it returns), with its current
        capped at ``limit_mA``. ``p_gain``, ``d_gain`` and ``i_gain`` (zero if not given) are
        the servos' Position P, D and I gains, in register units, one per joint; the motion
        profile is set to zero, so a servo goes straight for each position it is sent. Torque
        is off for about 30 ms while the mode changes."""
        i_gain = np.zeros(NUM_JOINTS) if i_gain is None else i_gain
        mismatches = self.model_mismatches()
        if mismatches:
            raise RuntimeError("refusing to drive the arm: " + "; ".join(mismatches))
        self.limit_units = np.broadcast_to(np.asarray(limit_mA, float), (NUM_JOINTS,)) / self._mA_per_unit
        for i, p, d, ki in zip(self.ids, p_gain, d_gain, i_gain):  # writable with torque on
            self._u2d2.set_position_p_gain(i, int(p))
            self._u2d2.set_position_i_gain(i, int(ki))
            self._u2d2.set_position_d_gain(i, int(d))
            self._u2d2.set_velocity_limit(i, 0)  # Profile Velocity: 0 is no profile
        for i in self.ids:
            self._u2d2.disable_torque(i)
            # The servos' own position control with the current capped by Goal Current. Unlike
            # plain position control it counts turns, which J4 (to 225 degrees) and the joints
            # that turn a full circle need.
            self._u2d2.set_motor_mode(i, "current_based_position")
        self._turns = None  # a mode change may reset a servo's turn count
        here = self.read().q
        self.position_mode = True
        self._u2d2.sync_write_currents(np.round(self.limit_units).astype(int).tolist())
        self.write_position(here)
        for i in self.ids:
            self._u2d2.enable_torque(i)
        self.write_position(here)  # again, with the cap: enabling torque may reset both
        self._u2d2.sync_write_currents(np.round(self.limit_units).astype(int).tolist())
        self.torque_enabled = True
        return here

    def write_position(self, q) -> None:
        """Command joint angles (rad), in position mode."""
        if not self.position_mode:
            raise RuntimeError("write_position before position_on")
        counts = self.zero_counts + self.sign * (np.asarray(q, float) - 2.0 * np.pi * self._turns) / RAD_PER_COUNT
        self._u2d2.sync_write_positions([int(c) for c in np.round(counts)])

    def write_torque(self, tau) -> np.ndarray:
        """Command joint torques (N m). Returns the torques actually commanded, after the
        current cap and rounding to the register's unit."""
        if not self.torque_enabled or self.position_mode:
            raise RuntimeError("write_torque needs torque_on (and not position mode)")
        units = np.clip(self.sign * np.asarray(tau, float) * self._units_per_Nm,
                        -self.limit_units, self.limit_units)
        units = np.round(units).astype(int)
        self._u2d2.sync_write_currents(units.tolist())
        return self.sign * units / self._units_per_Nm

    def torque_off(self) -> None:
        """Zero current, then torque off, on every servo. The arm goes limp: hold it."""
        self.torque_enabled = False
        try:
            if not self.position_mode:  # there the register is the current cap, not a command
                self._u2d2.sync_write_currents([0] * NUM_JOINTS)
        except Exception:
            pass  # still switch torque off below
        self.position_mode = False
        for i in self.ids:
            self._u2d2.disable_torque(i)


class FakeChain(LeaderChain):
    """No hardware: ``dynamixel_u2d2``'s fake interface, its servos moved through the viewer's
    poses, so that the path from servo counts to the screen can be tried without an arm."""

    PERIOD_S = 4.0

    def __init__(self, poses_deg=None, **kw) -> None:
        super().__init__("fake", **kw)
        from ditto_gello_software.viewer import POSES

        self._poses = np.radians(list((poses_deg or POSES).values()))
        self._t0 = time.monotonic()
        self._power_on: np.ndarray | None = None  # counts the fake servos lost at "power-up"

    def _open(self):
        from dynamixel_u2d2 import FakeU2D2Interface

        self.models = {i: s.model_number for i, s in zip(self.ids, self.servos)}
        return FakeU2D2Interface(motor_ids=self.ids)

    def temperatures(self) -> tuple[np.ndarray, np.ndarray]:
        """Made-up values: the fake servos have no temperature."""
        limit = np.array([80.0 if s.name.startswith("XM") else 70.0 for s in self.servos])
        return np.array([31.0, 36.0, 33.0, 52.0, 30.0, 47.0, 29.0]), limit

    def _scripted(self, t: float) -> np.ndarray:
        k, x = divmod(t / self.PERIOD_S, 1.0)
        a, b = self._poses[int(k) % len(self._poses)], self._poses[(int(k) + 1) % len(self._poses)]
        s = np.clip(x / 0.6, 0.0, 1.0)  # move for 60 % of the period, then rest
        return a + (b - a) * s * s * (3.0 - 2.0 * s)

    def read(self) -> State:
        if not self.torque_enabled:
            q = self._scripted(time.monotonic() - self._t0)
            counts = np.round(self.zero_counts + self.sign * q / RAD_PER_COUNT)
            if self._power_on is None:
                # A real servo reports 0 to 4095 after power-up, whatever turn its joint is
                # on, and counts on from there.
                self._power_on = counts % COUNTS_PER_REV - counts
            for i, c in zip(self.ids, counts + self._power_on):
                self._u2d2.set_motor_state(i, position=int(c))
        return super().read()


class SimChain(FakeChain):
    """No hardware, with dynamics: the arm falls, and is held, as the torques commanded to it
    say. For trying the torque law and the loop around it before they drive a real arm.

    Each :meth:`read` moves the simulation on by ``dt`` under the last commanded torques (zero
    with torque off), then reports the result through the fake servos, so positions come back
    in whole counts and torques go out in whole current units as on the real arm.

    It is a stand-in, not a model of the servos: gearbox inertia and friction are round
    numbers, the same for every joint. ``mass_scale`` makes the simulated arm heavier or
    lighter than the model the torque law uses. Two things are taken from the first real
    arm's logs, because a controller that ignores them works here and fails there: the joints
    are light (about 1e-3 kg m^2 at J1 to J4), and the speed the servos report is about 20 ms
    old.
    """

    ARMATURE = 3e-4  # gearbox inertia seen at the joint, kg m^2
    SERVO_STIFFNESS = 15.0  # N m/rad of the pretend position loop at P gain 800
    SPEED_LAG_S = 0.02  # how old the reported speed is
    VISCOUS = 0.01  # N m s/rad
    COULOMB = 0.01  # N m

    def __init__(self, dt: float = 0.002, start_deg=None,
                 mass_scale: float = 1.0, base_rpy=(0.0, 0.0, 0.0), **kw) -> None:
        super().__init__(**kw)
        import pinocchio as pin

        from ditto_gello_software.model import LeaderModel
        from ditto_gello_software.viewer import POSES

        self._pin = pin
        self._m = LeaderModel(base_rpy=base_rpy)
        for inertia in self._m.model.inertias:
            inertia.mass *= mass_scale  # the centre of mass stays where it is
        self._m.data = self._m.model.createData()
        self.dt = dt
        self._q = np.radians(POSES["ready"] if start_deg is None else start_deg).astype(float)
        self._qd = np.zeros(NUM_JOINTS)
        self._tau = np.zeros(NUM_JOINTS)
        self.held = True  # a "hand" holds the arm still; torque coming on lets go of it
        self._past_qd = [np.zeros(NUM_JOINTS)] * (1 + round(self.SPEED_LAG_S / dt))

    def set_point_mass(self, kg: float, at_flange_m=(0.0, 0.0, 0.0)) -> None:
        """Hang a point mass on the simulated arm's flange (see ``LeaderModel.set_point_mass``).
        Not scaled by ``mass_scale``."""
        self._m.set_point_mass(kg, at_flange_m)

    def torque_on(self, limit_mA) -> None:
        super().torque_on(limit_mA)
        self.held = False

    def write_torque(self, tau) -> np.ndarray:
        self._tau = super().write_torque(tau)
        return self._tau

    def position_on(self, limit_mA, p_gain, d_gain, i_gain=None) -> np.ndarray:
        here = super().position_on(limit_mA, p_gain, d_gain, i_gain)
        self._goal = here.copy()
        # A stand-in for the servo's position loop: a stiff, well-damped spring to the goal,
        # scaled by the P gain, with the torque capped as the current is.
        self._servo_kp = self.SERVO_STIFFNESS * np.asarray(p_gain, float) / 800.0
        self._servo_kd = 2.0 * np.sqrt(self._servo_kp * self.ARMATURE * 4.0)
        return here

    def write_position(self, q) -> None:
        super().write_position(q)
        self._goal = np.array(q, dtype=float)

    def torque_off(self) -> None:
        super().torque_off()
        self._tau = np.zeros(NUM_JOINTS)

    def _advance(self) -> None:
        pin, m, dt = self._pin, self._m, self.dt
        if self.position_mode:
            cap = self.limit_units / self._units_per_Nm
            self._tau = np.clip(self._servo_kp * (self._goal - self._q) - self._servo_kd * self._qd, -cap, cap)
        M = pin.crba(m.model, m.data, self._q)
        M = np.triu(M) + np.triu(M, 1).T + np.diag(np.full(NUM_JOINTS, self.ARMATURE))
        bias = pin.nonLinearEffects(m.model, m.data, self._q, self._qd)  # gravity and Coriolis
        friction = self.COULOMB * np.tanh(self._qd / 0.05)
        # Viscous friction taken implicitly, so that light joints stay stable at this step.
        self._qd = np.linalg.solve(M + dt * self.VISCOUS * np.eye(NUM_JOINTS),
                                   M @ self._qd + dt * (self._tau - bias - friction))
        self._q = self._q + dt * self._qd
        hit = (self._q < self.lower) | (self._q > self.upper)  # hard stops
        self._q = np.clip(self._q, self.lower, self.upper)
        self._qd[hit] = 0.0

    def read(self) -> State:
        if self.held:
            self._qd = np.zeros(NUM_JOINTS)
        else:
            self._advance()
        counts = np.round(self.zero_counts + self.sign * self._q / RAD_PER_COUNT)
        if self._power_on is None:
            self._power_on = counts % COUNTS_PER_REV - counts
        self._past_qd = self._past_qd[1:] + [self._qd.copy()]
        for i, c, v in zip(self.ids, counts + self._power_on, self.sign * self._past_qd[0] / RAD_S_PER_UNIT):
            self._u2d2.set_motor_state(i, position=int(c), velocity=int(round(v)))
        return LeaderChain.read(self)
