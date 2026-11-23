"""Render a rescue_pipeline.py run with MuJoCo's own renderer: no browser, no ffmpeg.

  MUJOCO_GL=egl python3 experiments/righting/render_mj.py RUN_DIR [--every 2] [--size 480x360]

Reads RUN_DIR/scene.xml and the right.json / left.json frame lists. For each frame it sets both robots'
chassis poses, head, lift (with the wrist and, for the real body, the lower links following it) and
wheels, and the cube's pose, then draws both trials side by side. Writes RUN_DIR/rescue.webp (animated)
and RUN_DIR/keyframes.png (six moments), each labelled with the time and the patient's tilt from upright.
With the real body the visual meshes are drawn and the collision pieces hidden.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw

JOINTS = ("head", "lift", "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")
MAX_FRAMES = 5000
# Drawing only: a key light from above the camera side, with shadows, and a soft fill from the other side.
# They go into a copy of the scene that is only rendered (scene-render.xml); the run itself is not changed.
RENDER_LIGHTS = (
    '<light name="render_key" type="directional" dir="0.35 0.45 -0.82" diffuse="0.5 0.5 0.49" '
    'specular="0.5 0.5 0.5" castshadow="true"/>'
    '<light name="render_fill" type="directional" dir="-0.6 -0.3 -0.74" diffuse="0.16 0.16 0.18" '
    'specular="0.1 0.1 0.1" castshadow="false"/>'
)


def lit_scene(run_dir: Path) -> Path:
    """The run's scene with the drawing lights added, written next to it."""
    text = (run_dir / "scene.xml").read_text()
    assert text.count("<worldbody>") == 1, "scene needs exactly one worldbody"
    out = run_dir / "scene-render.xml"
    out.write_text(text.replace("<worldbody>", "<worldbody>" + RENDER_LIGHTS, 1))
    return out


def set_robot(model, data, prefix, pose):
    assert len(pose) == 7 + len(JOINTS), "pose is position, quaternion and six joints"
    data.joint(prefix + "root").qpos[:7] = pose[:7]
    for name, value in zip(JOINTS, pose[7:]):
        data.joint(prefix + name + "_joint").qpos[0] = value
    lift = pose[7 + JOINTS.index("lift")]
    data.joint(prefix + "wrist_joint").qpos[0] = -lift
    if mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, prefix + "lift_lower_joint") >= 0:
        data.joint(prefix + "lift_lower_joint").qpos[0] = lift


def tilt_deg(quat):
    """Angle between a body's +z and world +z, from its quaternion (w, x, y, z)."""
    w, x, y, z = quat
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


def render_side(model, data, renderer, camera, option, frame):
    set_robot(model, data, "", frame["poses"][0])
    set_robot(model, data, "patient_", frame["poses"][1])
    if "cube" in frame and mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "cube_root") >= 0:
        data.joint("cube_root").qpos[:7] = frame["cube"]
    mj.mj_forward(model, data)
    renderer.update_scene(data, camera=camera, scene_option=option)
    return Image.fromarray(renderer.render())


def label(img, text):
    draw = ImageDraw.Draw(img)
    draw.rectangle([0, 0, img.width, 18], fill=(255, 255, 255))
    draw.text((4, 3), text, fill=(0, 0, 0))
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--every", type=int, default=2, help="draw every Nth frame (frames are 30 Hz)")
    ap.add_argument("--size", default="480x360")
    a = ap.parse_args()
    width, height = (int(n) for n in a.size.split("x"))
    run = json.loads((a.run_dir / "run.json").read_text())
    sides = {name: json.loads((a.run_dir / f"{name}.json").read_text()) for name in ("right", "left")}
    assert len(sides["right"]) == len(sides["left"]) and 0 < len(sides["right"]) <= MAX_FRAMES, "paired frames"
    model = mj.MjModel.from_xml_path(str(lit_scene(a.run_dir)))
    model.vis.global_.offwidth, model.vis.global_.offheight = max(width, 640), max(height, 480)
    model.vis.quality.shadowsize = 8192  # a coarse shadow map speckles across the wide table
    data = mj.MjData(model)
    renderer = mj.Renderer(model, height, width)
    camera = mj.MjvCamera()
    camera.lookat[:] = (-0.04, 0.0, 0.02)
    camera.distance, camera.azimuth, camera.elevation = 0.36, 215.0, -32.0
    model.vis.headlight.ambient[:] = (0.22, 0.22, 0.22)
    model.vis.headlight.diffuse[:] = (0.25, 0.25, 0.25)
    model.vis.headlight.specular[:] = (0.2, 0.2, 0.2)
    option = mj.MjvOption()
    real_body = bool(run.get("body"))
    option.geomgroup[1] = 0 if real_body else 1  # the real body is drawn by its visual meshes (group 2)
    option.geomgroup[3] = 0
    title = ("real body" if real_body else "parametric body") + (
        ", corrected dimensions" if run.get("overrides") else ", table dimensions")
    frames, keys = [], []
    n = len(sides["right"])
    drawn = list(range(0, n, max(1, a.every)))
    if drawn[-1] != n - 1:
        drawn.append(n - 1)  # always draw the last frame
    # Six key moments, each the drawn frame nearest an even fraction of the run (one frame per moment).
    key_at = {min(drawn, key=lambda i: abs(i - f * (n - 1))) for f in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)}
    for i in drawn:
        panels = []
        for name in ("right", "left"):
            fr = sides[name][i]
            img = render_side(model, data, renderer, camera, option, fr)
            tilt = tilt_deg(fr["poses"][1][3:7])
            panels.append(label(img, f"{title} | patient on its {name} side | t {fr['t']:5.2f} s | tilt {tilt:5.1f} deg"))
        sheet = Image.new("RGB", (2 * width + 6, height), "white")
        sheet.paste(panels[0], (0, 0))
        sheet.paste(panels[1], (width + 6, 0))
        frames.append(sheet)
        if i in key_at:
            keys.append(sheet)
    frames[0].save(a.run_dir / "rescue.webp", save_all=True, append_images=frames[1:],
                   duration=int(1000 * a.every / 30), loop=0, quality=70)
    grid = Image.new("RGB", (frames[0].width, frames[0].height * len(keys)), "white")
    for k, img in enumerate(keys):
        grid.paste(img, (0, k * img.height))
    grid.save(a.run_dir / "keyframes.png")
    print(f"{len(frames)} frames -> {a.run_dir / 'rescue.webp'}; {len(keys)} key frames -> {a.run_dir / 'keyframes.png'}")


if __name__ == "__main__":
    sys.exit(main())
