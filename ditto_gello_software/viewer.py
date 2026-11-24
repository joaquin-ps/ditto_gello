"""The viser scene: the leader arm and the xArm7 it drives, at the same joint angles.

This is the scene of ``ditto_gello_hardware/view.py`` (same arms, controls and zero check), as
a class whose pose can also be set from outside with :meth:`Viewer.set_joints`. That is how the
real arm's joint angles are shown: see ``view.py`` in this package.
"""

from __future__ import annotations

import json

import numpy as np
import trimesh
import viser
import viser.transforms as vtf
import yourdfpy
from viser.extras import ViserUrdf

from ditto_gello_software import (HORN_ZERO, NEAR_ZERO_POSE_DEG, START_RAW_RAD, URDF_DIR, XARM_URDF,
                                  gello_from_raw, leader_urdf, raw_from_gello)
from ditto_gello_software.ditto_hand import FINGERS, FOLLOWER_OFFSETS, HAND_HOME

XARM_MODES = ("beside the leader, full size", "over the leader, half scale", "hidden")
XARM_BESIDE = (0.0, 0.55, 0.0)  # where the full-size xArm7 stands, m
DESIRED_COLOR, DESIRED_OPACITY = (70, 150, 255), 0.35  # the faded blue "desired" leader
OFF_COLOR = np.array([230, 30, 30])  # what tint_joints turns a joint's parts towards
PRINT_COLOR = (0.68, 0.69, 0.72)  # the printed links, grey (the URDF's blue hides tint_joints' red)
# The DITTO hands (``hands``): the follower on the xArm7's flange, the leader on the gello's.
HANDS_DIR = URDF_DIR / "ditto_hands"
FOLLOWER_HAND = HANDS_DIR / "ditto_follower_7dof" / "ditto_follower_7dof.urdf"  # root hand_mount = the xArm7's link7
LEADER_HAND = HANDS_DIR / "ditto_leader_2f" / "ditto_leader_2f.urdf"  # root hand_piece
FOLLOWER_HAND_COLOR, LEADER_HAND_COLOR = (0.45, 0.46, 0.50), (0.95, 0.55, 0.15)
# The hand mount (hardware leader/cad/ditto_mount.py): the latch base on servo 7's horn, the
# bracket on the hand, both drawn in the hand's tcp frame (mm). The CAD puts the tcp frame on the
# gello's link7 axes, MOUNT_TCP_Z out along z; the latch base is screwed on turned MOUNT_TURN_DEG
# from that about J7 (the horn's four holes allow it). Then the leader hand's fingers are parallel
# to the follower's on the xArm7 at equal joints, and at the start pose the hand slides into the
# latch forwards (+x, away from the arm's base).
MOUNT_DIR = URDF_DIR / "ditto_mount"
MOUNT_PARTS = {"ditto_j7_adapter": (60, 95, 160), "ditto_hand_bracket": (70, 160, 120)}  # latch base, bracket
MOUNT_TCP_Z = 0.021  # m (ditto_mount.py TCP_OFFSET)
MOUNT_TURN_DEG = -90.0

def _raw_deg(q_rad) -> list:
    """raw_gello degrees, each within -180 to 180 (the sliders' range)."""
    return [round((float(d) + 180.0) % 360.0 - 180.0, 2) for d in np.degrees(q_rad)]


POSES = {  # raw_gello joint angles in degrees (the gello's own; the xArm7 shown gets gello_from_raw)
    "start (mme rig)": _raw_deg(START_RAW_RAD),
    "near-zero start (candidate for mme)": _raw_deg(raw_from_gello(np.radians(NEAR_ZERO_POSE_DEG))),
    "ready": [0, 20, 0, 90, 0, 70, 0],
    "reach forward": [0, 60, 0, 60, 0, 30, 0],
    "arm straight up": [0, 0, 0, 180, 0, 0, 0],
    "elbow out": [30, 30, 60, 100, -40, 60, 20],
    "shoulder limit +120": [0, 120, 0, 150, 0, 0, 0],
    "elbow limit 225": [0, -20, 0, 225, 0, 0, 0],
    "wrist limit -97": [0, 20, 0, 90, 0, -97, 0],
    "xArm7 zero (folded)": [0, 0, 0, 0, 0, 0, 0],
}

NOTES = """**Ditto GELLO: 1:2 leader arm for the xArm7**

Light grey: printed links. Dark grey: base (from FACTR). Black: servos.

**Zero** puts every joint at 0, the xArm7's zero. Each servo is meant
to read zero there too. Red pins: a horn's top, right, bottom and left
screw holes with its servo at zero; the white one is the top hole.
Yellow pins: the holes of the part bolted to that horn; the blue one
is where the part's top-hole mark is. At Zero each yellow pin sits
inside a red one and each blue pin inside a white one.

The xArm7 is at the same joint angles. Joint frames are drawn on both
(red x, green y, blue z).
"""


def _turn_z(deg: float) -> np.ndarray:
    """A turn about z, 4 x 4."""
    T = np.eye(4)
    T[:3, :3] = vtf.SO3.from_z_radians(np.radians(deg)).as_matrix()
    return T


def _pose(T):
    return dict(wxyz=vtf.SO3.from_matrix(T[:3, :3]).wxyz, position=T[:3, 3])


class Viewer:
    """The scene and its controls.

    The sliders set the pose unless :meth:`set_external` has handed it to the caller, who then
    calls :meth:`set_joints` with joint angles in radians. ``panel`` names a folder created
    above the sliders for the caller's own controls (``self.panel``; None without it).
    """

    def __init__(
        self,
        port: int = 8080,
        pose_deg=POSES["ready"],
        xarm_mode: str = XARM_MODES[0],
        panel: str | None = None,
        hands: bool = False,
    ) -> None:
        self.server = server = viser.ViserServer(port=port)
        server.scene.set_up_direction("+z")
        server.scene.add_grid("/grid", width=2.0, height=2.0, cell_size=0.1, position=(0.2, 0.25, 0.0))
        # Light the scene ourselves: the default environment map is downloaded, so it is
        # missing offline.
        server.scene.configure_environment_map(hdri=None)
        server.scene.add_light_hemisphere("/lights/sky", ground_color=(190, 195, 205), intensity=1.6)
        server.initial_camera.position = (0.98, -0.38, 0.52)
        server.initial_camera.look_at = (0.2, 0.27, 0.3)

        # The xArm7: full size beside the leader, and a half-scale ghost on top of it.
        self._xarm = yourdfpy.URDF.load(str(XARM_URDF), load_meshes=False)  # limits, joint frames
        self.joint_names = list(self._xarm.actuated_joint_names)
        self._full_root = server.scene.add_frame("/xarm7", show_axes=False, position=XARM_BESIDE)
        self._xarm_full = ViserUrdf(
            server,
            yourdfpy.URDF.load(str(XARM_URDF)),
            root_node_name="/xarm7/robot",
            mesh_color_override=(0.90, 0.91, 0.93, 0.72),  # see-through: its joint frames show
        )
        self._ghost_root = server.scene.add_frame("/xarm7_half_scale", show_axes=False)
        self._xarm_ghost = ViserUrdf(
            server,
            yourdfpy.URDF.load(str(XARM_URDF)),
            scale=0.5,
            root_node_name="/xarm7_half_scale/robot",
            mesh_color_override=(0.25, 0.75, 0.95, 0.18),
        )
        server.scene.add_label("/xarm7/name", "xArm7", position=(0.0, 0.0, -0.03))
        self._xarm_frames = []
        for i in range(len(self.joint_names)):
            fr = server.scene.add_frame(
                f"/xarm7/J{i + 1}", axes_length=0.10, axes_radius=0.004, origin_radius=0.008
            )
            server.scene.add_label(
                f"/xarm7/J{i + 1}/label", f"J{i + 1}", position=(0.0, 0.0, 0.03 + 0.03 * (i % 2))
            )
            self._xarm_frames.append(fr)

        # The leader.
        self._leader = {"urdf": None, "links": {}, "prints": [], "hardware": [], "frames": [],
                        "root": None, "marks": [], "pins": [], "tint": []}
        self._meshes: dict = {}
        self._desired = {"urdf": None, "links": {}, "root": None}  # the faded blue copy
        self._horn_zero = json.loads(HORN_ZERO.read_text()) if HORN_ZERO.exists() else {}
        self._q = np.zeros(len(self.joint_names))
        self._external = False
        self._syncing = False  # True while this class writes the sliders itself

        # Controls.
        gui = server.gui
        notes = gui.add_markdown(NOTES)
        self.panel = gui.add_folder(panel) if panel else None
        self._zero_btn = gui.add_button("Zero")
        self._zero_md = gui.add_markdown("")
        self._xarm_dd = gui.add_dropdown("xArm7", options=list(XARM_MODES), initial_value=xarm_mode)
        self._sliders = []
        joints_folder = gui.add_folder("Joints (degrees)")
        with joints_folder:
            for n in self.joint_names:
                lim = self._xarm.joint_map[n].limit
                wide = lim.upper - lim.lower > 6.28
                lo, hi = (-180.0, 180.0) if wide else (np.degrees(lim.lower), np.degrees(lim.upper))
                self._sliders.append(
                    gui.add_slider(n, min=round(float(lo), 1), max=round(float(hi), 1), step=0.5,
                                   initial_value=0.0)
                )
        poses_folder = gui.add_folder("Poses")
        with poses_folder:
            self._pose_btns = {name: gui.add_button(name) for name in POSES}
        show_folder = gui.add_folder("Show")
        with show_folder:
            self._cb_frames = gui.add_checkbox("joint frames", True)
            self._cb_prints = gui.add_checkbox("printed parts", True)
            self._cb_hw = gui.add_checkbox("servos, bearing, electronics", True)
            self._cb_pins = gui.add_checkbox("horn-hole pins (zero check)", True)

        self._hand_md = hands_folder = None
        if hands:
            hands_folder = gui.add_folder("DITTO hands")
            with hands_folder:
                gui.add_markdown("Orange: the leader hand on the gello, on its mount (blue: latch base, "
                                 "green: bracket). Grey: the follower on the xArm7.")
                self._hand_md = gui.add_markdown("")

        # Everything but the caller's panel: what to show (show_controls), and what sets the pose
        # by hand (hide_pose_controls, for a caller that always sets it).
        self._controls = [self._xarm_dd, show_folder] + ([hands_folder] if hands else [])
        self._pose_controls = [notes, self._zero_btn, self._zero_md, joints_folder, poses_folder]
        for s in self._sliders:
            s.on_update(self._on_slider)
        for name, b in self._pose_btns.items():
            b.on_click(lambda _, q=POSES[name]: self.set_pose_deg(q))
        for h in (self._xarm_dd, self._cb_frames, self._cb_prints, self._cb_hw, self._cb_pins):
            h.on_update(self._show)
        self._zero_btn.on_click(lambda _: self.set_pose_deg([0.0] * len(self._sliders)))

        self._load_leader()
        self._hands = self._load_hands() if hands else None
        self._show()
        self.set_pose_deg(pose_deg)

    # ---- pose ---------------------------------------------------------------------------------
    def set_external(self, on: bool) -> None:
        """On: the caller sets the pose with :meth:`set_joints`; sliders and pose buttons are
        greyed out and only show it. Off: the sliders set the pose again, from where it is."""
        self._external = on
        for h in (*self._sliders, *self._pose_btns.values(), self._zero_btn):
            h.disabled = on
        if not on:
            self.show_on_sliders(self._q)
            self._on_slider()

    def set_fingers(self, leader_q, status: str | None = None) -> None:
        """Pose the DITTO hands (if shown): the leader's finger angles (rad, ``FINGERS`` order),
        the follower at those plus the 7dof_f02 leader offsets. ``status`` goes under the mount
        choice."""
        if self._hands is None:
            return
        lq = np.asarray(leader_q, dtype=float)
        for key, q in (("leader", lq), ("follower", lq + FOLLOWER_OFFSETS)):
            model, names = self._hands[key]
            angle = dict(zip(FINGERS, q))
            model.update_cfg(np.array([angle[n] for n in names]))
        if status is not None and status != self._hand_md.content:
            self._hand_md.content = status

    def show_controls(self, on: bool) -> None:
        """Show or hide the viewer's display controls (xArm7 view, what to show)."""
        for h in self._controls:
            h.visible = on

    def hide_pose_controls(self) -> None:
        """Hide the sliders, poses, Zero and notes, for a caller that always sets the pose."""
        for h in self._pose_controls:
            h.visible = False

    def set_joints(self, q) -> None:
        """Pose both arms at raw_gello joint angles ``q`` (rad); the xArm7 gets the same pose in
        its own joints (``gello_from_raw``). The sliders are left alone: call
        :meth:`show_on_sliders` for that, less often."""
        self._q = np.array(q, dtype=float)
        self._apply()

    def show_on_sliders(self, q) -> None:
        """Write ``q`` (rad) to the sliders without moving the arms. A slider cannot show an
        angle outside its range, so it stops at its end there."""
        self._syncing = True
        try:
            for s, v in zip(self._sliders, np.degrees(q)):
                v = round(float(np.clip(v, s.min, s.max)) * 2.0) / 2.0  # the slider's 0.5 step
                if v != s.value:
                    s.value = v
        finally:
            self._syncing = False

    def set_pose_deg(self, q_deg) -> None:
        """Move the sliders, and with them both arms, to ``q_deg`` (degrees)."""
        self.show_on_sliders(np.radians(q_deg))
        self._on_slider()

    def _on_slider(self, _=None) -> None:
        if self._syncing or self._external:
            return
        self._q = np.radians([s.value for s in self._sliders])
        self._apply()

    def _apply(self) -> None:
        q, u = self._q, self._leader["urdf"]
        u.update_cfg(q)
        for name, fr in self._leader["links"].items():
            T = u.get_transform(name, u.base_link)
            fr.wxyz, fr.position = vtf.SO3.from_matrix(T[:3, :3]).wxyz, T[:3, 3]
        xq = gello_from_raw(q)  # the xArm7 in the same pose, in its own joints
        self._xarm_full.update_cfg(xq)
        self._xarm_ghost.update_cfg(xq)
        self._xarm.update_cfg(xq)
        for n, fr in zip(self.joint_names, self._xarm_frames):
            T = self._xarm.get_transform(self._xarm.joint_map[n].child, self._xarm.base_link)
            fr.wxyz, fr.position = vtf.SO3.from_matrix(T[:3, :3]).wxyz, T[:3, 3]
        if self._hands is not None:
            h = self._hands
            for root, T in (("follower_root", self._xarm.get_transform("link7", self._xarm.base_link)),
                            ("leader_root", u.get_transform("link7", u.base_link) @ h["mount"])):
                h[root].wxyz, h[root].position = vtf.SO3.from_matrix(T[:3, :3]).wxyz, T[:3, 3]
        self._zero_check()

    def _zero_check(self) -> None:
        """From the model as posed: how far each part's horn holes are from the horn's holes
        at servo zero."""
        u, rows, ok = self._leader["urdf"], [], True
        for m in self._leader["marks"]:
            def world(link, pts):
                T = u.get_transform(link, u.base_link)
                return np.array(pts) @ T[:3, :3].T + T[:3, 3]

            a, b = world(m["child"], m["link_holes"]), world(m["parent"], m["motor_zero"])
            d = 1000.0 * max(np.linalg.norm(b - p, axis=1).min() for p in a) if len(a) else float("nan")
            # the part's top mark against the horn's top hole
            dt = 1000.0 * float(np.linalg.norm(world(m["child"], [m["link_top"]])[0] - b[0]))
            ok = ok and d < 0.05 and dt < 0.05
            rows.append(f"J{m['joint']} {d:.1f} / {dt:.1f}")
        if rows:
            head = ("**Every part is on its horn's top, right, bottom and left holes, top mark on "
                    "the top hole.**" if ok else "**Not at the servos' zero** (move the joints to 0).")
            text = head + "  \nOffset in mm, holes / top mark: " + ", ".join(rows)
            if text != self._zero_md.content:
                self._zero_md.content = text

    # ---- scene --------------------------------------------------------------------------------
    def _load_leader(self) -> None:
        server, st = self.server, self._leader
        path = leader_urdf()
        u = yourdfpy.URDF.load(str(path), load_meshes=False)
        colors = {m.name: tuple(float(c) for c in m.color.rgba[:3])
                  for m in u.robot.materials if m.color is not None}
        colors["print_new"] = PRINT_COLOR
        st.update(urdf=u, links={}, prints=[], hardware=[], frames=[],
                  root=server.scene.add_frame("/leader", show_axes=False),
                  marks=self._horn_zero.get("2xm", []),
                  pins=[], tint=[])
        moved_by = {}  # link: index of the nearest joint above it that moves it (None: the base)
        for link in u.robot.links:
            joint = next((jt for jt in u.robot.joints if jt.child == link.name), None)
            while joint is not None and joint.name not in u.actuated_joint_names:
                joint = next((jt for jt in u.robot.joints if jt.child == joint.parent), None)
            moved_by[link.name] = None if joint is None else u.actuated_joint_names.index(joint.name)
        for link in u.robot.links:
            st["links"][link.name] = server.scene.add_frame(f"/leader/{link.name}", show_axes=False)
            for k, vis in enumerate(link.visuals):
                fn = path.parent / vis.geometry.mesh.filename
                if fn not in self._meshes:
                    m = trimesh.load(fn)
                    m.apply_scale(vis.geometry.mesh.scale)
                    self._meshes[fn] = m
                m = self._meshes[fn]
                T = np.eye(4) if vis.origin is None else np.asarray(vis.origin)
                h = server.scene.add_mesh_simple(
                    f"/leader/{link.name}/visual_{k}", m.vertices, m.faces, flat_shading=True,
                    color=colors[vis.material.name], **_pose(T),
                )
                st["prints" if vis.material.name.startswith("print") else "hardware"].append(h)
                if moved_by[link.name] is not None:
                    st["tint"].append((h, 255.0 * np.array(colors[vis.material.name]), moved_by[link.name]))
        self._load_desired(path)
        for i, n in enumerate(u.actuated_joint_names):
            child = u.joint_map[n].child
            if not u.link_map[child].visuals:  # nothing to turn red (J7: no handle): a ball instead
                st["tint"].append((server.scene.add_icosphere(f"/leader/{child}/off", radius=0.012,
                                                              color=OFF_COLOR, visible=False), None, i))
            st["frames"].append(server.scene.add_frame(
                f"/leader/{child}/J{i + 1}", axes_length=0.05, axes_radius=0.002, origin_radius=0.004))
            st["frames"].append(server.scene.add_label(
                f"/leader/{child}/J{i + 1}/label", f"J{i + 1}",
                position=(0.0, 0.0, 0.015 + 0.015 * (i % 2))))
        st["frames"].append(server.scene.add_label(
            "/leader/name", "Ditto GELLO leader (1:2)", position=(0.0, 0.0, -0.03)))
        for m in st["marks"]:  # pins along the joint axis, out of the horn face through the part
            for key, link, axis, length, color, width in (
                ("motor_zero", m["parent"], m["axis_parent"], 0.012, (230, 40, 40), 9.0),
                ("link_holes", m["child"], m["axis_child"], 0.020, (255, 220, 0), 3.5),
            ):
                p0 = np.array(m[key], dtype=np.float32)
                seg = np.stack([p0, p0 + length * np.array(axis, dtype=np.float32)], axis=1)
                st["pins"].append(server.scene.add_line_segments(
                    f"/leader/{link}/J{m['joint']}_{key}", seg, color, line_width=width))
            for key, p0, link, axis, length, color, width in (  # the horn's top hole, the part's mark
                ("motor_top", m["motor_zero"][0], m["parent"], m["axis_parent"], 0.016, (255, 255, 255), 11.0),
                ("link_top", m["link_top"], m["child"], m["axis_child"], 0.026, (40, 110, 255), 4.5),
            ):
                p0 = np.array(p0, dtype=np.float32)
                seg = np.stack([p0, p0 + length * np.array(axis, dtype=np.float32)])[None]
                st["pins"].append(server.scene.add_line_segments(
                    f"/leader/{link}/J{m['joint']}_{key}", seg, color, line_width=width))

    def _load_hands(self) -> dict:
        """The two DITTO hand models, the leader's place on the gello's link7, and its mount."""
        server = self.server
        h = {"follower_root": server.scene.add_frame("/xarm7/hand", show_axes=False),  # = the xArm7's link7
             "leader_root": server.scene.add_frame("/leader_hand", show_axes=False)}  # = hand_piece
        for key, path, root, color in (("follower", FOLLOWER_HAND, "/xarm7/hand/urdf", FOLLOWER_HAND_COLOR),
                                       ("leader", LEADER_HAND, "/leader_hand/urdf", LEADER_HAND_COLOR)):
            u = yourdfpy.URDF.load(str(path))
            h[key] = (ViserUrdf(server, u, root_node_name=root, mesh_color_override=color), list(u.actuated_joint_names))
            if key == "leader":
                tcp = np.eye(4)  # link7 -> the hand's tcp frame
                tcp[2, 3] = MOUNT_TCP_Z
                hp_tcp = u.get_transform("tcp", "hand_piece")  # the tcp frame in hand_piece's
                h["mount"] = _turn_z(MOUNT_TURN_DEG) @ tcp @ np.linalg.inv(hp_tcp)  # link7 -> hand_piece
                for name, color in MOUNT_PARTS.items():  # the mount's parts, in the tcp frame
                    path = MOUNT_DIR / f"{name}.stl"
                    if path.exists():
                        m = trimesh.load(path)
                        server.scene.add_mesh_simple(f"/leader_hand/{name}", m.vertices / 1000.0, m.faces,
                                                     color=color, flat_shading=True, **_pose(hp_tcp))
        self._hands = h
        self.set_fingers(HAND_HOME)
        return h

    def _load_desired(self, path) -> None:
        """A see-through blue copy of the leader, hidden until :meth:`set_desired` poses it."""
        server, d = self.server, self._desired
        if d["root"] is not None:
            d["root"].remove()
        u = yourdfpy.URDF.load(str(path), load_meshes=False)
        d.update(urdf=u, links={}, root=server.scene.add_frame("/leader_desired", show_axes=False, visible=False))
        for link in u.robot.links:
            d["links"][link.name] = server.scene.add_frame(f"/leader_desired/{link.name}", show_axes=False)
            for k, vis in enumerate(link.visuals):
                m = self._meshes[path.parent / vis.geometry.mesh.filename]  # loaded for the leader
                T = np.eye(4) if vis.origin is None else np.asarray(vis.origin)
                server.scene.add_mesh_simple(
                    f"/leader_desired/{link.name}/visual_{k}", m.vertices, m.faces, flat_shading=True,
                    color=DESIRED_COLOR, opacity=DESIRED_OPACITY, **_pose(T),
                )

    def set_desired(self, q) -> None:
        """Show the faded blue leader at joint angles ``q`` (rad): where the arm is being
        asked to be, beside where it is. ``None`` hides it."""
        d = self._desired
        if q is None:
            if d["root"].visible:
                d["root"].visible = False
            return
        u = d["urdf"]
        u.update_cfg(np.asarray(q, dtype=float))
        for name, fr in d["links"].items():
            T = u.get_transform(name, u.base_link)
            fr.wxyz, fr.position = vtf.SO3.from_matrix(T[:3, :3]).wxyz, T[:3, 3]
        if not d["root"].visible:
            d["root"].visible = True

    def tint_joints(self, amount) -> None:
        """Turn the parts each joint moves (up to the next joint) red by ``amount``, one value
        per joint from 0 (their own colour) to 1 (red). ``None`` is all zeros."""
        amount = np.zeros(len(self.joint_names)) if amount is None else np.clip(amount, 0.0, 1.0)
        for h, own, j in self._leader["tint"]:
            if own is None:  # the ball: shown only while the joint is off
                if h.visible != (amount[j] > 0.0):
                    h.visible = bool(amount[j] > 0.0)
                own = np.full(3, 255.0)
            color = tuple(int(c) for c in np.round(own + amount[j] * (OFF_COLOR - own)))
            if tuple(h.color) != color:
                h.color = color

    def _show(self, _=None) -> None:
        beside = self._xarm_dd.value == XARM_MODES[0]
        self._full_root.visible = beside
        self._ghost_root.visible = self._xarm_dd.value == XARM_MODES[1]
        for fr in self._xarm_frames:
            fr.visible = beside and self._cb_frames.value
        for key, cb in (("frames", self._cb_frames), ("prints", self._cb_prints),
                        ("hardware", self._cb_hw), ("pins", self._cb_pins)):
            for h in self._leader[key]:
                h.visible = cb.value

