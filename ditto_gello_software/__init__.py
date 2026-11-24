"""Software for the Ditto GELLO leader arm.

The arm's URDFs and meshes are read from ``ditto_gello_hardware/`` beside this package.
"""

from pathlib import Path

import numpy as np

HARDWARE = Path(__file__).resolve().parents[1] / "ditto_gello_hardware"
URDF_DIR = HARDWARE / "urdf"
XARM_URDF = URDF_DIR / "xarm7" / "xarm7.urdf"
HORN_ZERO = URDF_DIR / "horn_zero.json"

NUM_JOINTS = 7

# Where a session starts, gello joints in degrees, J1 to J7: the xArm7's reset joints in
# mini_manipulation_env (configs/rig/right_xarm_ditto.yaml starting_joint_pos, rad: -3.0900,
# 0.6156, -2.8425, 1.1096, 0.2122, 1.6909, -2.9118). The TCP is then at (251.4, 155.3, 521.2) mm,
# roll/pitch/yaw (177.7, 0.1, -174.5) deg.
START_POSE_DEG = (-177.044, 35.2713, -162.8633, 63.5754, 12.1582, 96.8814, -166.8339)

# raw_gello: the joints as the gello's encoders give them. gello: the same pose in the xArm7's
# joints, what is compared with and sent to it. They differ only by the shoulder flip: J1, J2
# and J3 meet at one point (on both arms), so (J1 + 180, -J2, J3 + 180) is the same arm from the
# elbow out. The flip puts the gello's J1 and J3 near zero, where its cables are routed, at
# the start pose above (J1 -177, J3 -163). Set False to make raw_gello and gello the same.
SHOULDER_FLIP = True
_SIGN = np.array([1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0]) if SHOULDER_FLIP else np.ones(7)
_OFFSET = np.radians([-180.0, 0.0, -180.0, 0.0, 0.0, 0.0, 0.0]) if SHOULDER_FLIP else np.zeros(7)


def gello_from_raw(raw_gello):
    """raw_gello joints (rad) to gello joints (rad)."""
    return _SIGN * np.asarray(raw_gello, dtype=float) + _OFFSET


def raw_from_gello(gello):
    """gello joints (rad) to raw_gello joints (rad)."""
    return _SIGN * (np.asarray(gello, dtype=float) - _OFFSET)


START_RAW_RAD = raw_from_gello(np.radians(START_POSE_DEG))  # the start pose in raw_gello joints
# A candidate start pose for mme, not used yet: the joints nearest all zeros that put the TCP at
# (250, 155, 520) mm, (180, 0, 180) deg, 1.9 mm and 6 deg from the one above. J7 stays near -142
# because the hand is mounted turned on the flange.
NEAR_ZERO_POSE_DEG = (46.5, -33.3, -9.3, 63.5, -5.1, 96.5, -141.9)


def leader_urdf() -> Path:
    """The gello's URDF: the arm as built, the 2xm variant (XM430 at J2 and J4) with link 5
    option A (slotted tube)."""
    return URDF_DIR / "ditto_leader_2xm.urdf"
