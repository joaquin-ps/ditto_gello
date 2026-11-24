# Ditto hand models

`ditto_follower_7dof/` and `ditto_leader_7dof/` are copied unchanged from
[roamlab/ditto](https://github.com/roamlab/ditto/tree/reverse_teleop), `assets/urdf/`, branch `reverse_teleop`
(commit `4e4a251`, 2026-11-09). Do not edit them here: copy them again when they change there.

`ditto_gello_hardware/view.py --hands` shows them: the follower hand on the xArm7's flange and the leader hand on
the Ditto GELLO's flange.

| Model | Root link | How it is mounted |
|---|---|---|
| `ditto_follower_7dof` | `hand_mount` | `hand_mount` is the xArm7's `link7` (the bare flange). Its own fixed joint `hand_mount` -> `palm` is the mount: xyz (0, 0, 10) mm, rpy (0, 90, 90) deg. |
| `ditto_leader_7dof` | `hand_piece` | No mount in the model. `view.py` places `hand_piece` on the leader's `link7` (`LEADER_HAND_RPY`, `LEADER_MOUNTS`). |

## New leader models: `ditto_leader_2f/`, `ditto_leader_3f/`

Built by `scripts/build_ditto_leader_urdfs.py` from the SolidWorks export
`ditto_3f_leader_urdf_4dof_thumb.zip` (sw2urdf, `DITTO_cad_3f_v3/ditto_leader_urdf_assembly`, 2026-11-19):
`2f` is the index and the thumb, `3f` adds the middle finger (`middle1`-`middle3`, turning the same way as
`index1`-`index3`). Do not edit them by hand: rebuild them.

They are drop-in replacements for `ditto_leader_7dof`: root `hand_piece` in the same frame, the same joint
names, zeros and positive directions, and its joint limits. Checked over random poses with the same joint
values: every joint axis matches to 0.001 mm / 0.001 deg, and the fingertip parts (index3, thumb4) to 0.002 deg.
The export itself differs in three ways, all fixed by the build: its joints turn the other way on index1-3
and thumb1, and its thumb1 and thumb4 zeros are off by +0.341 and -0.246 deg. The zero is where each part's
screw holes sit on its XC330 horn's holes (along the case's width and length at servo zero); on the export's
meshes every other joint is on that to 0.0003 deg, and the old URDF had thumb1 and thumb4 right.
Each joint frame is also slid along its axis onto the old URDF's (no change to the kinematics), so
joint frame origins match too: ditto's viewers line the hands up at the index1 child frame's origin.

New frames: `tcp` (where the follower hand sits on a robot arm's TCP; a bare frame, at (-42.15, 68.51, 6.22) mm,
rpy (90, 0, 0) deg in `hand_piece`), and `*_fingertip`, `*_fingerpad` (separate links with their own meshes,
fixed to the finger's last link). ditto's viewers colour only a joint's own link past a limit, so these stay
grey there. The old marker links
(`index_sphere`, `thumb_sphere`, `trakstar_sphere`) are not in them.

`scripts/compare_leader_urdfs_viser.py` shows old and new side by side or overlaid on the same sliders.
