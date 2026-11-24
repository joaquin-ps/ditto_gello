"""Viewer for the Ditto GELLO leader arm (viser).

    pip install -r requirements.txt
    python view.py                  # then open http://localhost:8080
    python view.py --hands          # ... with the Ditto hands on: the follower on the xArm7, the leader on the GELLO

The leader and the xArm7 it drives move on the same joint sliders. Every joint frame of the
leader is the xArm7's joint frame at half scale, so equal joint angles give equal poses.

The hands are the two models of roamlab/ditto (urdf/ditto_hands). The follower hand's root link,
hand_mount, is the xArm7's flange (link7): its own fixed joint hand_mount -> palm holds the mount.
The leader hand's root link is hand_piece; where it sits on the leader's flange is LEADER_MOUNTS below.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import trimesh
import viser
import viser.transforms as vtf
import yourdfpy
from viser.extras import ViserUrdf

URDF_DIR = Path(__file__).resolve().parent / "urdf"
XARM_URDF = URDF_DIR / "xarm7" / "xarm7.urdf"
HORN_ZERO = URDF_DIR / "horn_zero.json"     # horn screw holes at each servo's zero, and the holes of the part bolted on
VARIANTS = {"2 x XM430 (J2, J4)": "2xm", "3 x XM430 (J2, J3, J4)": "3xm"}
LINK5 = {"A: slotted tube": "a", "B: X core": "b"}     # two forearm options, both kept until tested
XARM_MODES = ("beside the leader, full size", "over the leader, half scale", "hidden")
XARM_BESIDE = (0.0, 0.55, 0.0)      # where the full-size xArm7 stands, m

# ---- the Ditto hands (left out when urdf/ditto_hands is missing) -------------------------------------------------
HANDS_DIR = URDF_DIR / "ditto_hands"
FOLLOWER_HAND = HANDS_DIR / "ditto_follower_7dof" / "ditto_follower_7dof.urdf"      # root link hand_mount = the xArm7's link7
LEADER_HAND = HANDS_DIR / "ditto_leader_2f" / "ditto_leader_2f.urdf"                # root link hand_piece (as ditto uses since 2026-11-19)
FINGERS = ("index1", "index2", "index3", "thumb1", "thumb2", "thumb3", "thumb4")    # the same angles drive both hands
HAND_HOME = (5, 50, -5, -5, -55, 0, -35)                                            # degrees: the rig's follower home
# Leader link7 (flange) -> hand_piece. LEADER_HAND_RPY makes the leader hand's finger axes parallel to those of the
# follower hand as it is mounted on the xArm7. With LEADER_HAND_XYZ as well, the leader's index1-3 and thumb1 axes
# lie on the follower's (within 0.1 mm): the leader hand then sits on its flange as the follower sits on the xArm7's.
LEADER_HAND_RPY = (-np.pi / 2, 0.0, -np.pi / 2)
LEADER_HAND_XYZ = np.array([-0.00632, -0.04216, 0.07860])
PINCH = np.array([-0.1030, -0.0035, 0.1316])    # halfway between the follower's fingertip points at HAND_HOME, in the flange frame
LEADER_MOUNTS = {       # the offset is the open choice; the rotation is the same in all of them
    "as the follower: same offset from the flange": LEADER_HAND_XYZ,                           # touches link 6: the hand reaches 8 mm behind the flange face
    "as the follower, 10 mm further out": LEADER_HAND_XYZ + np.array([0.0, 0.0, 0.010]),       # 5 mm clear of link 6 at every joint 7 angle
    "half offset: pinch point at half scale": LEADER_HAND_XYZ - PINCH / 2,                     # for comparison: the hand ends up inside the wrist
}
FOLLOWER_HAND_COLOR, LEADER_HAND_COLOR = (0.58, 0.58, 0.63), (0.95, 0.55, 0.15)

POSES = {                           # joint angles in degrees
    "ready": [0, 20, 0, 90, 0, 70, 0],
    "rig start (mme)": [-177.0, 35.3, -162.9, 63.6, 12.2, 96.9, -166.8],   # mini_manipulation_env right_xarm_ditto starting_joint_pos

    "reach forward": [0, 60, 0, 60, 0, 30, 0],
    "arm straight up": [0, 0, 0, 180, 0, 0, 0],
    "elbow out": [30, 30, 60, 100, -40, 60, 20],
    "shoulder limit +120": [0, 120, 0, 150, 0, 0, 0],
    "elbow limit 225": [0, -20, 0, 225, 0, 0, 0],
    "wrist limit -97": [0, 20, 0, 90, 0, -97, 0],
    "xArm7 zero (folded)": [0, 0, 0, 0, 0, 0, 0],
}

NOTES = """**Ditto GELLO: 1:2 leader arm for the xArm7**

Blue: printed links. Grey: base (from FACTR). Black: servos.
The leader ends at servo 7's horn; no handle is included.
Link 5 (the forearm) has two options, A and B: pick one below.

**Zero** puts every joint at 0, the xArm7's zero. Each servo is meant
to read zero there too. Red pins: a horn's top, right, bottom and left
screw holes with its servo at zero; the white one is the top hole.
Yellow pins: the holes of the part bolted to that horn; the blue one
is where the part's top-hole mark is. At Zero each yellow pin sits
inside a red one and each blue pin inside a white one.

The xArm7 follows the same sliders. Joint frames are drawn on both
(red x, green y, blue z).
"""

HAND_NOTES = """Grey hand: the Ditto follower on the xArm7's flange. Orange hand: the
Ditto leader on the GELLO's flange. Both follow the finger sliders.
Red ball: the pinch point, the same point of each hand. Frames: the
follower's `palm` and the leader's `hand_piece`.
"""


def leader_urdf(variant: str, link5: str = "a") -> Path:
    return URDF_DIR / f"ditto_leader_{variant}{'' if link5 == 'a' else '_link5' + link5}.urdf"


def _pose(T):
    return dict(wxyz=vtf.SO3.from_matrix(T[:3, :3]).wxyz, position=T[:3, 3])


def _rpy(rpy):
    return vtf.SO3.from_rpy_radians(*rpy).as_matrix()


def launch(port: int = 8080, variant: str = "2xm", pose_deg=POSES["ready"], xarm_mode: str = XARM_MODES[0], link5: str = "a",
           hands: bool = False) -> viser.ViserServer:
    """Start the viewer and return the server (it keeps serving for as long as the process lives)."""
    server = viser.ViserServer(port=port)
    server.scene.set_up_direction("+z")
    server.scene.add_grid("/grid", width=2.0, height=2.0, cell_size=0.1, position=(0.2, 0.25, 0.0))
    server.scene.configure_environment_map(hdri=None)       # light the scene ourselves: the default map is downloaded,
    server.scene.add_light_hemisphere("/lights/sky", ground_color=(190, 195, 205), intensity=1.6)   # so it is missing offline
    server.initial_camera.position = (0.98, -0.38, 0.52)
    server.initial_camera.look_at = (0.2, 0.27, 0.3)

    # ---- the xArm7: full size beside the leader, and a half-scale ghost on top of it --------
    xarm = yourdfpy.URDF.load(str(XARM_URDF), load_meshes=False)                     # kinematics only: limits, joint frames
    joints = xarm.actuated_joint_names
    full_root = server.scene.add_frame("/xarm7", show_axes=False, position=XARM_BESIDE)
    xarm_full = ViserUrdf(server, yourdfpy.URDF.load(str(XARM_URDF)), root_node_name="/xarm7/robot",
                          mesh_color_override=(0.90, 0.91, 0.93, 0.72))                  # see-through, so the joint frames inside it show
    ghost_root = server.scene.add_frame("/xarm7_half_scale", show_axes=False)
    xarm_ghost = ViserUrdf(server, yourdfpy.URDF.load(str(XARM_URDF)), scale=0.5, root_node_name="/xarm7_half_scale/robot",
                           mesh_color_override=(0.25, 0.75, 0.95, 0.18))
    server.scene.add_label("/xarm7/name", "xArm7", position=(0.0, 0.0, -0.03))
    xarm_frames = []
    for i in range(len(joints)):
        fr = server.scene.add_frame(f"/xarm7/J{i + 1}", axes_length=0.10, axes_radius=0.004, origin_radius=0.008)   # twice the leader's
        server.scene.add_label(f"/xarm7/J{i + 1}/label", f"J{i + 1}", position=(0.0, 0.0, 0.03 + 0.03 * (i % 2)))
        xarm_frames.append(fr)

    # ---- the Ditto hands: the follower on the xArm7's flange, the leader on the GELLO's ------
    hand = None
    if FOLLOWER_HAND.exists() and LEADER_HAND.exists():
        hand = {"follower_root": server.scene.add_frame("/xarm7/hand", show_axes=False),       # = link7 = hand_mount
                "leader_root": server.scene.add_frame("/leader_hand", axes_length=0.04, axes_radius=0.002, origin_radius=0.004)}   # = hand_piece
        for key, path, root, color in (("follower", FOLLOWER_HAND, "/xarm7/hand/urdf", FOLLOWER_HAND_COLOR),
                                       ("leader", LEADER_HAND, "/leader_hand/urdf", LEADER_HAND_COLOR)):
            u = yourdfpy.URDF.load(str(path))
            hand[key] = (ViserUrdf(server, u, root_node_name=root, mesh_color_override=color), list(u.actuated_joint_names))
            if key == "follower":
                hand["limits"] = [u.joint_map[n].limit for n in FINGERS]
                T = np.asarray(u.joint_map["hand_mount_to_palm"].origin)
                hand["frames"] = [server.scene.add_frame("/xarm7/hand/palm", axes_length=0.04, axes_radius=0.002, origin_radius=0.004, **_pose(T)),
                                  server.scene.add_label("/xarm7/hand/palm/label", "palm", position=(0.0, 0.0, 0.015)),
                                  server.scene.add_label("/leader_hand/label", "hand_piece", position=(0.0, 0.0, 0.015))]
        server.scene.add_icosphere("/xarm7/hand/pinch", radius=0.006, color=(230, 40, 40), position=PINCH)
        server.scene.add_icosphere("/leader_hand/pinch", radius=0.006, color=(230, 40, 40),        # the same point of the leader hand
                                   position=_rpy(LEADER_HAND_RPY).T @ (PINCH - LEADER_HAND_XYZ))

    # ---- the leader, rebuilt whenever the variant changes ---------------------------------
    state = {"urdf": None, "links": {}, "prints": [], "hardware": [], "frames": [], "root": None, "marks": [], "pins": []}
    meshes = {}
    horn_zero = json.loads(HORN_ZERO.read_text()) if HORN_ZERO.exists() else {}

    def load_leader(v: str, opt: str):
        if state["root"] is not None:
            state["root"].remove()
        path = leader_urdf(v, opt)
        u = yourdfpy.URDF.load(str(path), load_meshes=False)
        colors = {m.name: tuple(float(c) for c in m.color.rgba[:3]) for m in u.robot.materials if m.color is not None}
        state.update(urdf=u, links={}, prints=[], hardware=[], frames=[], root=server.scene.add_frame("/leader", show_axes=False),
                     marks=horn_zero.get(v if opt == "a" else f"{v}_link5{opt}", []), pins=[])
        for link in u.robot.links:
            state["links"][link.name] = server.scene.add_frame(f"/leader/{link.name}", show_axes=False)
            for k, vis in enumerate(link.visuals):
                fn = path.parent / vis.geometry.mesh.filename
                if fn not in meshes:
                    m = trimesh.load(fn)
                    m.apply_scale(vis.geometry.mesh.scale)
                    meshes[fn] = m
                m = meshes[fn]
                T = np.eye(4) if vis.origin is None else np.asarray(vis.origin)
                h = server.scene.add_mesh_simple(f"/leader/{link.name}/visual_{k}", m.vertices, m.faces, flat_shading=True,
                                                 color=colors[vis.material.name], **_pose(T))
                state["prints" if vis.material.name.startswith("print") else "hardware"].append(h)
        for i, n in enumerate(u.actuated_joint_names):
            child = u.joint_map[n].child
            state["frames"].append(server.scene.add_frame(f"/leader/{child}/J{i + 1}", axes_length=0.05, axes_radius=0.002, origin_radius=0.004))
            state["frames"].append(server.scene.add_label(f"/leader/{child}/J{i + 1}/label", f"J{i + 1}", position=(0.0, 0.0, 0.015 + 0.015 * (i % 2))))
        state["frames"].append(server.scene.add_label("/leader/name", "Ditto GELLO leader (1:2)", position=(0.0, 0.0, -0.03)))
        for m in state["marks"]:                             # pins along the joint axis, out of the horn face through the part
            for key, link, axis, length, color, width in (("motor_zero", m["parent"], m["axis_parent"], 0.012, (230, 40, 40), 9.0),
                                                          ("link_holes", m["child"], m["axis_child"], 0.020, (255, 220, 0), 3.5)):
                p0 = np.array(m[key], dtype=np.float32)
                seg = np.stack([p0, p0 + length * np.array(axis, dtype=np.float32)], axis=1)
                state["pins"].append(server.scene.add_line_segments(f"/leader/{link}/J{m['joint']}_{key}", seg, color, line_width=width))
            for key, p0, link, axis, length, color, width in (("motor_top", m["motor_zero"][0], m["parent"], m["axis_parent"], 0.016, (255, 255, 255), 11.0),
                                                              ("link_top", m["link_top"], m["child"], m["axis_child"], 0.026, (40, 110, 255), 4.5)):
                p0 = np.array(p0, dtype=np.float32)                # the top hole of the horn, and the part's mark for it
                seg = np.stack([p0, p0 + length * np.array(axis, dtype=np.float32)])[None]
                state["pins"].append(server.scene.add_line_segments(f"/leader/{link}/J{m['joint']}_{key}", seg, color, line_width=width))

    # ---- controls -----------------------------------------------------------------------------
    server.gui.add_markdown(NOTES)
    zero_btn = server.gui.add_button("Zero")
    zero_md = server.gui.add_markdown("")
    variant_dd = server.gui.add_dropdown("leader", options=list(VARIANTS), initial_value=next(k for k, v in VARIANTS.items() if v == variant))
    link5_dd = server.gui.add_dropdown("link 5", options=list(LINK5), initial_value=next(k for k, v in LINK5.items() if v == link5))
    xarm_dd = server.gui.add_dropdown("xArm7", options=list(XARM_MODES), initial_value=xarm_mode)
    sliders = []
    with server.gui.add_folder("Joints (degrees)"):
        for n in joints:
            lim = xarm.joint_map[n].limit
            lo, hi = (-180.0, 180.0) if lim.upper - lim.lower > 6.28 else (float(np.degrees(lim.lower)), float(np.degrees(lim.upper)))
            sliders.append(server.gui.add_slider(n, min=round(lo, 1), max=round(hi, 1), step=0.1, initial_value=0.0))
    with server.gui.add_folder("Poses"):
        buttons = {name: server.gui.add_button(name) for name in POSES}
    if hand:
        with server.gui.add_folder("Ditto hands"):
            server.gui.add_markdown(HAND_NOTES)
            cb_hands = server.gui.add_checkbox("show the hands", hands)
            mount_dd = server.gui.add_dropdown("leader hand mount", options=list(LEADER_MOUNTS))
            shift = [server.gui.add_slider(f"shift {a} (mm)", min=-100.0, max=100.0, step=0.5, initial_value=0.0) for a in "xyz"]
            mount_md = server.gui.add_markdown("")
            with server.gui.add_folder("Fingers (degrees)"):
                fingers = [server.gui.add_slider(n, min=round(float(np.degrees(lim.lower)), 1), max=round(float(np.degrees(lim.upper)), 1), step=0.5,
                                                 initial_value=float(v)) for n, lim, v in zip(FINGERS, hand["limits"], HAND_HOME)]
                hand_home_btn = server.gui.add_button("hand home")
    with server.gui.add_folder("Show"):
        cb_frames = server.gui.add_checkbox("joint frames", True)
        cb_prints = server.gui.add_checkbox("printed parts", True)
        cb_hw = server.gui.add_checkbox("servos, bearing, electronics", True)
        cb_pins = server.gui.add_checkbox("horn-hole pins (zero check)", True)

    def apply(_=None):
        q = np.radians([s.value for s in sliders])
        u = state["urdf"]
        u.update_cfg(q)
        for name, fr in state["links"].items():
            T = u.get_transform(name, u.base_link)
            fr.wxyz, fr.position = vtf.SO3.from_matrix(T[:3, :3]).wxyz, T[:3, 3]
        xarm_full.update_cfg(q)
        xarm_ghost.update_cfg(q)
        xarm.update_cfg(q)
        for n, fr in zip(joints, xarm_frames):
            T = xarm.get_transform(xarm.joint_map[n].child, xarm.base_link)
            fr.wxyz, fr.position = vtf.SO3.from_matrix(T[:3, :3]).wxyz, T[:3, 3]
        if hand:
            T7 = xarm.get_transform("link7", xarm.base_link)                  # the xArm7's flange = the follower's hand_mount
            hand["follower_root"].wxyz, hand["follower_root"].position = vtf.SO3.from_matrix(T7[:3, :3]).wxyz, T7[:3, 3]
            M = np.eye(4)                                                     # the leader's flange -> hand_piece
            M[:3, :3], M[:3, 3] = _rpy(LEADER_HAND_RPY), LEADER_MOUNTS[mount_dd.value] + np.array([s.value for s in shift]) / 1000.0
            T = u.get_transform("link7", u.base_link) @ M
            hand["leader_root"].wxyz, hand["leader_root"].position = vtf.SO3.from_matrix(T[:3, :3]).wxyz, T[:3, 3]
            angles = dict(zip(FINGERS, np.radians([s.value for s in fingers])))
            for key in ("follower", "leader"):
                model, names = hand[key]
                model.update_cfg(np.array([angles[n] for n in names]))
            pinch = PINCH + M[:3, 3] - LEADER_HAND_XYZ                        # the leader's pinch point in its flange frame
            mm = lambda v: ", ".join(f"{1000 * x:.1f}" for x in v)
            mount_md.content = (f"**Leader:** `link7` → `hand_piece` xyz ({mm(M[:3, 3])}) mm, rpy ({', '.join(f'{np.degrees(a):.0f}' for a in LEADER_HAND_RPY)})°  \n"
                                "**Follower:** `link7` = `hand_mount` → `palm` xyz (0.0, 0.0, 10.0) mm, rpy (0, 90, 90)°  \n"
                                f"**Pinch point from the flange:** follower ({mm(PINCH)}) mm, leader ({mm(pinch)}) mm")
        # zero check, from the model as posed: how far each part's horn holes are from the horn's holes at servo zero
        rows, ok = [], True
        for m in state["marks"]:
            world = lambda link, pts: np.array(pts) @ u.get_transform(link, u.base_link)[:3, :3].T + u.get_transform(link, u.base_link)[:3, 3]
            a, b = world(m["child"], m["link_holes"]), world(m["parent"], m["motor_zero"])
            d = 1000.0 * max(np.linalg.norm(b - p, axis=1).min() for p in a) if len(a) else float("nan")
            dt = 1000.0 * float(np.linalg.norm(world(m["child"], [m["link_top"]])[0] - b[0]))      # the part's top mark against the horn's top hole
            ok = ok and d < 0.05 and dt < 0.05
            rows.append(f"J{m['joint']} {d:.1f} / {dt:.1f}")
        if rows:
            zero_md.content = (("**Every part is on its horn's top, right, bottom and left holes, top mark on the top hole.**" if ok
                                else "**Not at the servos' zero** (move the joints to 0).")
                               + "  \nOffset in mm, holes / top mark: " + ", ".join(rows))

    def show(_=None):
        beside = xarm_dd.value == XARM_MODES[0]
        full_root.visible = beside
        ghost_root.visible = xarm_dd.value == XARM_MODES[1]
        for fr in xarm_frames:
            fr.visible = beside and cb_frames.value
        for h in state["frames"]:
            h.visible = cb_frames.value
        for h in state["prints"]:
            h.visible = cb_prints.value
        for h in state["hardware"]:
            h.visible = cb_hw.value
        for h in state["pins"]:
            h.visible = cb_pins.value
        if hand:
            hand["follower_root"].visible = hand["leader_root"].visible = cb_hands.value      # the follower also hides with the full-size xArm7
            hand["leader_root"].show_axes = cb_frames.value
            for h in hand["frames"]:
                h.visible = cb_frames.value

    def set_pose(q_deg):
        for s, v in zip(sliders, q_deg):
            s.value = float(np.clip(v, s.min, s.max))
        apply()

    def change_variant(_=None):
        load_leader(VARIANTS[variant_dd.value], LINK5[link5_dd.value])
        show()
        apply()

    for s in sliders:
        s.on_update(apply)
    for name, b in buttons.items():
        b.on_click(lambda _, q=POSES[name]: set_pose(q))
    for h in (xarm_dd, cb_frames, cb_prints, cb_hw, cb_pins) + ((cb_hands,) if hand else ()):
        h.on_update(show)
    if hand:
        for h in [mount_dd] + shift + fingers:
            h.on_update(apply)

        @hand_home_btn.on_click
        def _(_):
            for s, v in zip(fingers, HAND_HOME):
                s.value = float(v)
            apply()
    zero_btn.on_click(lambda _: set_pose([0.0] * 7))
    variant_dd.on_update(change_variant)
    link5_dd.on_update(change_variant)

    load_leader(variant, link5)
    show()
    set_pose(pose_deg)
    return server


def main():
    ap = argparse.ArgumentParser(description="Viewer for the Ditto GELLO leader arm and the xArm7.")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--variant", choices=list(VARIANTS.values()), default="2xm", help="leader variant shown at start")
    ap.add_argument("--link5", choices=list(LINK5.values()), default="a", help="forearm (link 5) option shown at start")
    ap.add_argument("--hands", action="store_true", help="start with the Ditto hands shown (follower on the xArm7, leader on the GELLO)")
    ap.add_argument("--pose", choices=list(POSES), default="ready", help="joint pose shown at start")
    args = ap.parse_args()
    launch(args.port, args.variant, POSES[args.pose], link5=args.link5, hands=args.hands)
    print(f"Ditto GELLO viewer running at http://localhost:{args.port}  (Ctrl+C to stop)")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
