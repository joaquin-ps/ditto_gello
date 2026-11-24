# ditto_gello

Software for the Ditto GELLO: the 1:2 leader arm for the xArm7, as built (the `2xm` variant,
XM430 servos at J2 and J4, link 5 option A). `ditto_gello_hardware/` is a copy of the hardware
package: URDFs, meshes and print files.

```bash
conda activate ditto_gello
python -m ditto_gello_software.gravity_comp --u2d2 /dev/ttyUSB0   # the arm, http://localhost:8080
python -m ditto_gello_software.gravity_comp --sim                 # a simulated arm
python -m ditto_gello_software.view --u2d2 /dev/ttyUSB0           # read only: watch the arm
python -m ditto_gello_software.bus_check --u2d2 /dev/ttyUSB0      # read only: servo bus health
python -m ditto_gello_software.view --u2d2 /dev/ttyUSB0 --hands   # ... with the DITTO hands, see below
```

Run them from this folder, one at a time: only one program can use the U2D2.

## USB latency timer: 1 ms

The U2D2's USB adapter holds each servo answer for its latency timer before passing it on: 16 ms
by default, reset whenever it is plugged in. At 16 ms the control loop runs at about 60 Hz
instead of 500 Hz, and a switch to the servos' position control leaves the arm limp for about
half a second. `gravity_comp` refuses to drive the arm unless it is 1 ms. Make it so at every
plug-in with a udev rule (once per computer):

```bash
echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVERS=="ftdi_sio", ATTR{latency_timer}="1"' \
  | sudo tee /etc/udev/rules.d/99-ftdi-latency.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # 1
```

## Start-up

`gravity_comp` opens with gravity compensation and the start check only:

1. Hold the arm, press **Gravity compensation ON**.
2. Turn each joint the way its arrows point (↑ increase the angle, ↓ decrease it; more arrows,
   further to go) until every joint shows ✓ and no part of the render is red (within 10°,
   or 5°).
3. Check that the render matches the arm and no cable is wrapped round a joint, then press
   **Confirm**.
4. Press **Home**: the servos take the arm exactly to the start pose, gravity compensation takes
   over again, and the rest of the page appears.

The start pose is the xArm7's reset pose in mini_manipulation_env (`START_POSE_DEG`).

## The DITTO hands

`--hands` (on `view` and `gravity_comp`) shows the DITTO leader hand (`ditto_leader_2f`, orange)
on the gello's flange and the follower hand (`ditto_follower_7dof`, grey) on the xArm7's. The
finger angles come live from the leader hand's own U2D2 (`--hand-u2d2`, default `/dev/ttyUSB1`),
read only, converted as roamlab/ditto does for the 7dof_f02 hand config. The follower is shown
at the same angles plus 7dof_f02's leader offsets (index 2 +15°, index 3 +5°).

The leader hand sits on its mount (`ditto_gello_hardware/urdf/ditto_mount`, from the hardware
repo's `leader/cad/ditto_mount.py`): the latch base (blue) on servo 7's horn and the bracket
(green) on the hand. The latch base is screwed on turned −90° about J7 from the CAD's placement
(the horn's four holes allow it). That is the one way the leader's finger axes are parallel to
the follower's on the xArm7 at equal joints, and at the start pose the hand then slides into
the latch forwards (+x, away from the arm's base).

## raw_gello and gello

- **raw_gello**: the joint angles as the gello's encoders give them. Gravity compensation, the
  joint-limit barrier, the start check and recordings use these.
- **gello**: the same pose in the xArm7's joints. This is what is compared with the xArm7 and
  sent to it.

They differ by the shoulder flip (`SHOULDER_FLIP` in `ditto_gello_software/__init__.py`):
gello = (J1 − 180°, −J2, J3 − 180°, J4, J5, J6, J7) of raw_gello. J1, J2 and J3 meet at one
point on both arms, so the two describe the same arm from the elbow out. The flip keeps the
gello's J1 and J3 near zero, where its cables are routed, while the xArm7 uses its reset joints
(J1 −177°, J3 −163°). Set `SHOULDER_FLIP = False` to make the two the same. Logs keep both.

## Joint values that are safe to send to the xArm7

Send only what `XArmGate` (`ditto_gello_software/xarm_safety.py`) passes. Both joint streaming
and pose streaming go through it: pose streaming sends the TCP pose of the joints it passes.

1. **gello joints, never raw_gello.**
2. **After the start check.** The arm has been taken to the start pose (the xArm7's reset
   joints).
3. **After engaging.** Every gello joint is within 2° of the xArm7's measured joint, compared
   as numbers: a joint that reads a whole turn off is the same pose but is refused.
4. **Inside the xArm7's joint limits, less 2°:**

   | Joint | xArm7 limits | Sent only within |
   |---|---|---|
   | J1, J3, J5, J7 | −360° to 360° | −358° to 358° |
   | J2 | −118° to 120° | −116° to 118° |
   | J4 | −11° to 225° | −9° to 223° |
   | J6 | −97° to 180° | −95° to 178° |

5. **No joint faster than 2 rad/s** from one sample to the next (the xArm7's limit is
   3.14 rad/s), apart from a 0.3° allowance for encoder noise. A jump (a joint read a turn off,
   a bad servo read) is far over this.
6. **Fresh and finite.** No more than 50 ms between samples, and no missing values.

Anything else is a **safety stop**. The gate passes nothing more, the xArm7 keeps its last
command, and mini_manipulation_env stops it when commands stop coming (`command_timeout_s`,
0.5 s). Only a new engage, with the gello matching the xArm7 again, restarts the stream.

The 2° margins, 2 rad/s and 50 ms are first values, not tuned on the rig yet. mini_manipulation_env
still applies its own checks downstream (collision guard, workspace bounds, speed limits). The
gate is the first check, not the only one.

Nothing streams to the xArm7 yet: the gate is in place for when it does.
