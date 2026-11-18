"""Viewer for the Ditto GELLO leader arm (viser).

    pip install -r requirements.txt
    python view.py                  # then open http://localhost:8080

The leader and the xArm7 it drives move on the same joint sliders. Every joint frame of the
leader is the xArm7's joint frame at half scale, so equal joint angles give equal poses.
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

POSES = {                           # joint angles in degrees
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


def leader_urdf(variant: str, link5: str = "a") -> Path:
    return URDF_DIR / f"ditto_leader_{variant}{'' if link5 == 'a' else '_link5' + link5}.urdf"


def _pose(T):
    return dict(wxyz=vtf.SO3.from_matrix(T[:3, :3]).wxyz, position=T[:3, 3])


def launch(port: int = 8080, variant: str = "2xm", pose_deg=POSES["ready"], xarm_mode: str = XARM_MODES[0], link5: str = "a") -> viser.ViserServer:
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
            sliders.append(server.gui.add_slider(n, min=round(lo, 1), max=round(hi, 1), step=0.5, initial_value=0.0))
    with server.gui.add_folder("Poses"):
        buttons = {name: server.gui.add_button(name) for name in POSES}
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
    for h in (xarm_dd, cb_frames, cb_prints, cb_hw, cb_pins):
        h.on_update(show)
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
    args = ap.parse_args()
    launch(args.port, args.variant, link5=args.link5)
    print(f"Ditto GELLO viewer running at http://localhost:{args.port}  (Ctrl+C to stop)")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
