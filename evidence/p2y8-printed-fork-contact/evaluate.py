"""Cross fork geometry with independently measured initial cube heading."""

import xml.etree.ElementTree as ET
import csv
import gzip
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import mujoco as mj
import numpy as np

here = Path(__file__).resolve().parent
repo = Path(sys.argv[1]).resolve()
base = repo / "evidence/p2y8-cube-contact-replay"
sys.path.insert(0, str(base))
from replay import start_pose, rotation, K

cal = json.loads((repo / "web/camera_calibration_vector2.json").read_text())
K[:2] = [[cal["fx_px"], 0, cal["cx_px"]], [0, cal["fy_px"], cal["cy_px"]]]
picks = json.loads(
    (repo / "evidence/p2y8-paired-cube-push/endpoint-picks.json").read_text()
)
states = np.genfromtxt(base / "inputs.csv", delimiter=",", names=True)
observed = np.genfromtxt(
    repo / "evidence/p2y8-paired-cube-push/comparison.csv", names=True, delimiter=","
)
yaws = np.genfromtxt(here / "overhead-yaw.csv", names=True, delimiter=",")


def run(key, observer=None, *, extra_reach_mm=0, wrist_pitch_deg=0, wrist_roll_deg=0, observer_stride=5, rails=False):
    trial, geometry, heading, drive, depth, lateral, height = key[:7]
    xml = ET.parse(
        base / "fixture.xml" if geometry == "boxes" else Path(sys.argv[2]).resolve()
    )
    if len(key) > 7:
        from copy import deepcopy

        world = xml.find("worldbody")
        world.find("geom[@name='table']").set("conaffinity", "11")
        cube_body = world.find("body[@name='cube']")
        for geom in cube_body.findall("geom"):
            floor_copy = deepcopy(geom)
            floor_copy.attrib.update(
                name="floor_" + geom.get("name"), contype="8", conaffinity="0"
            )
            cube_body.append(floor_copy)
            coefficient, torsion, rolling = geom.get("friction").split()
            geom.set("friction", f"{float(coefficient) * key[7]} {torsion} {rolling}")
    if drive == "weld":
        world = xml.find("worldbody")
        fork = world.find("body[@name='fork']")
        fork.attrib.pop("mocap")
        ET.SubElement(fork, "freejoint", name="wrist_motion")
        ET.SubElement(
            fork, "inertial", mass=".006", pos="0 0 -.010", diaginertia="2e-6 1e-6 2e-6"
        )
        ET.SubElement(world, "body", name="target", mocap="true")
        equality = ET.SubElement(xml.getroot(), "equality")
        ET.SubElement(equality, "weld", body1="fork", body2="target", solref=".002 1")
    if rails:
        sys.path.insert(0,str(repo))
        from replay_links import add_links, rail_paths
        add_links(xml,repo)
    m = mj.MjModel.from_xml_string(ET.tostring(xml.getroot(), encoding="unicode"))
    d = mj.MjData(m)
    pick = next(p for p in picks if p["trial"] == trial and p["camera"] == "on")
    center, local = start_pose(pick)
    yaw0 = np.deg2rad(pick["state"]["yaw"])
    angles = yaws[yaws["trial"] == trial]["degrees"]
    yaw = yaw0 + local if heading == "rgb" else np.deg2rad(angles[0])
    center = rotation(yaw0) @ (center + [depth, lateral]) + pick["state"]["xy"]
    cube = d.joint("cube_root").qpos
    cube[:] = [*center * 0.001, 0.022, np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    r = states[states["trial"] == trial]
    t = np.arange(0, r["sec"][-1], m.opt.timestep)
    x, y, h, l = (
        np.interp(t, r["sec"], r[k]) for k in ("x_mm", "y_mm", "yaw_deg", "lift_mm")
    )
    h = np.deg2rad(h)
    reach = extra_reach_mm - 41 + np.sqrt(66**2 - (l - 45) ** 2)
    path = (
        np.column_stack((x + reach * np.cos(h), y + reach * np.sin(h), l + height))
        * 0.001
    )
    cy, sy = np.cos(h / 2), np.sin(h / 2)
    pitch_half = np.deg2rad(wrist_pitch_deg) / 2
    cp, sp = np.cos(pitch_half), np.sin(pitch_half)
    # Compose chassis yaw with pitch around the local wrist axis.
    cr, sr = np.cos(np.deg2rad(wrist_roll_deg)/2), np.sin(np.deg2rad(wrist_roll_deg)/2)
    # Local roll about forward is applied before wrist pitch and chassis yaw.
    q = np.column_stack((cy*cp*cr + sy*sp*sr, cy*cp*sr - sy*sp*cr,
                         cy*sp*cr + sy*cp*sr, sy*cp*cr - cy*sp*sr))
    if drive == "weld":
        d.joint("wrist_motion").qpos[:] = [*path[0], *q[0]]
    if rails:
        rail_xyz,rail_q=rail_paths(repo,x,y,h,l)
        ids=[m.body(f"rail{i}").mocapid[0] for i in range(2)]
    trace = []
    for j, pos in enumerate(path):
        d.mocap_pos[0] = pos
        d.mocap_quat[0] = q[j]
        if rails:
            d.mocap_pos[ids]=rail_xyz[j]
            d.mocap_quat[ids]=rail_q[j]
        mj.mj_step(m, d)
        if observer is not None and j % observer_stride == 0:
            observer(m, d, t[j] - 1.5, l[j])
        if j % 20 == 0 or j == len(path) - 1:
            w, a, b, c = cube[3:]
            angle = np.rad2deg(
                np.arctan2(2 * (w * c + a * b), 1 - 2 * (b * b + c * c)) - yaw
            )
            trace.append(
                [
                    t[j],
                    *(cube[:2] * 1000 - center),
                    cube[2] * 1000 - 22,
                    (angle + 180) % 360 - 180,
                    np.rad2deg(np.arccos(np.clip(1 - 2 * (a * a + b * b), -1, 1))),
                ]
            )
    end = trace[-1]
    err = np.linalg.norm(
        np.array(end[1:3])
        - [observed["over_forward"][trial - 1], observed["over_left"][trial - 1]]
    )
    return [
        *key,
        np.rad2deg(yaw),
        *end[1:3],
        end[4],
        angles[1] - angles[0],
        err,
        max(row[3] for row in trace),
        max(row[5] for row in trace),
    ], trace


if __name__ == "__main__":
    jobs = [
        (i, g, h, d, 0, 0, 0)
        for i in range(1, 6)
        for g in ("boxes", "stl")
        for h in ("rgb", "overhead")
        for d in ("mocap", "weld")
    ]
    jobs += [
        (i, "stl", "overhead", "weld", 0, 0, z)
        for i in range(1, 6)
        for z in (-3, -1.5, 1.5)
    ]
    with (
        ProcessPoolExecutor(4) as pool,
        (here / "scores.csv").open("w") as f,
        gzip.open(here / "trajectories.csv.gz", "wt") as tf,
    ):
        out = csv.writer(f, lineterminator="\n")
        tracks = csv.writer(tf, lineterminator="\n")
        out.writerow(
            [
                "trial",
                "fork",
                "heading",
                "drive",
                "depth_offset",
                "lateral_offset",
                "height_offset",
                "initial_yaw_deg",
                "forward",
                "left",
                "yaw_change_deg",
                "observed_yaw_change_deg",
                "vector_error_mm",
                "peak_rise_mm",
                "peak_tilt_deg",
            ]
        )
        tracks.writerow(
            [
                "trial",
                "fork",
                "heading",
                "drive",
                "depth_offset",
                "lateral_offset",
                "height_offset",
                "time",
                "forward_mm",
                "left_mm",
                "rise_mm",
                "yaw_change_deg",
                "tilt_deg",
            ]
        )
        for row, trace in pool.map(run, jobs):
            out.writerow(row)
            tracks.writerows([*row[:7], *x] for x in trace)
