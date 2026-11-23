"""Prescribe the fork trajectory from SDK poses and replay against a free canonical cube."""

import csv
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
import matplotlib.pyplot as plt

p = Path(__file__).resolve().parent
assets = p.parents[1] / "web/public/robot/vector2"
v = json.loads((assets / "model.json").read_text())["values"]
source = ET.parse(assets / "vector-free-shelf.xml").getroot()
scene = ET.Element("mujoco")
scene.append(source.find("option"))
world = ET.SubElement(scene, "worldbody")
ET.SubElement(world, "geom", type="plane", size="1 1 .01", contype="0", conaffinity="1")
fork = ET.SubElement(world, "body", name="fork", mocap="true")
for geom in source.findall(".//body[@name='lift_fork']/geom"):
    fork.append(geom)
world.append(source.find(".//body[@name='cube']"))
ET.SubElement(
    ET.SubElement(scene, "sensor"), "accelerometer", name="cube_accel", site="cube_chip"
)
model = mj.MjModel.from_xml_string(ET.tostring(scene, encoding="unicode"))
dt = model.opt.timestep


def load(name):
    return np.genfromtxt(
        p / name, delimiter=",", names=True, dtype=None, encoding="utf8"
    )


path = load("path.csv.gz")
observations = load("observations.csv")
physical = load("cube.csv.gz")
fig, axes = plt.subplots(1, 3, figsize=(12, 4), sharey=True, layout="constrained")
summary = []
for ax, name in zip(axes, ["W33", "W34", "W35"]):
    robot = path[path["trial"] == name]
    times = np.arange(-3, 0.105, dt)

    def interp(key):
        arr = robot[key]
        if key == "yaw_rad":
            arr = np.unwrap(arr)
        return np.interp(times, robot["t_s"], arr)

    x, y, yaw, pitch, h = map(
        interp, ["x_mm", "y_mm", "yaw_rad", "pitch_rad", "lift_mm"]
    )
    # Same rear-axle pivot convention as the existing W4 fork replay.
    px = -v["wheel_base_mm"]
    pz = v["wheel_rear_radius_mm"]
    shoulder = v["lift_base_position_mm"]
    length = v["lift_arm_length_mm"]
    h = np.clip(h, v["lift_height_min_mm"], v["lift_height_max_mm"])
    reach = shoulder[0] + np.sqrt(length**2 - (h - shoulder[2]) ** 2)
    along = px + (reach - px) * np.cos(pitch) - (h - pz) * np.sin(pitch)
    height = pz + (reach - px) * np.sin(pitch) + (h - pz) * np.cos(pitch)
    pos = np.c_[x + along * np.cos(yaw), y + along * np.sin(yaw), height] / 1000
    a, b = np.cos(yaw / 2), np.sin(yaw / 2)
    c, s = np.cos(pitch / 2), np.sin(pitch / 2)
    quat = np.c_[a * c, b * s, -a * s, b * c]
    measured = physical[physical["trial"] == name]
    mt = measured["packet_s"]
    g = np.stack([measured[k] for k in "xyz"], axis=1) / 8192
    dev = np.linalg.norm(g - np.median(g[(mt >= -7) & (mt < -5)], axis=0), axis=1)
    use = (mt >= -0.8) & (mt <= 0.1)
    ax.scatter(mt[use], dev[use], color="black", s=6, label="Physical cube")
    for obs in observations[observations["trial"] == name]:
        data = mj.MjData(model)
        cy = obs["yaw_rad"]
        data.qpos[:7] = [
            obs["x_mm"] / 1000,
            obs["y_mm"] / 1000,
            0.0227,
            np.cos(cy / 2),
            0,
            0,
            np.sin(cy / 2),
        ]
        data.mocap_pos[0] = pos[0]
        data.mocap_quat[0] = quat[0]
        for _ in range(250):
            mj.mj_step(model, data)
        forces = []
        first = None
        forkid = model.body("fork").id
        cubeid = model.body("cube").id
        for i, t in enumerate(times):
            data.mocap_pos[0] = pos[i]
            data.mocap_quat[0] = quat[i]
            mj.mj_step(model, data)
            if first is None:
                for contact in data.contact:
                    if (
                        set(model.geom_bodyid[contact.geom]) == {forkid, cubeid}
                        and contact.dist <= 0
                    ):
                        first = t
                        break
            forces.append(data.sensor("cube_accel").data.copy() / 9.81)
        arg = dict(dt=dt, start=times[0], forces=np.asarray(forces).tolist())
        packets = json.loads(
            subprocess.check_output(
                ["node", str(p / "mount_sensor.mjs")], input=json.dumps(arg), text=True
            )
        )
        arr = np.array(packets)
        st = arr[:, 0]
        sd = np.linalg.norm(
            arr[:, 1:4] - np.median(arr[(st >= -2.9) & (st < -2.1), 1:4], axis=0),
            axis=1,
        )
        use2 = (st >= -0.8) & (st <= 0.1)
        ax.plot(st[use2], sd[use2], alpha=0.8, lw=1, label=obs["observation"] + " pose")
        summary.append(
            [
                name,
                obs["observation"],
                first,
                dev[use].max(),
                sd[use2].max(),
                int(measured["tap"][use][-1]) - int(measured["tap"][mt < -0.8][-1]),
                int(arr[use2, 4][-1] - arr[st < -0.8, 4][-1]),
            ]
        )
    ax.set_title(name)
    ax.set_xlabel("Seconds from detected stand")
    ax.legend(fontsize=8)
axes[0].set_ylabel("Change from resting acceleration (g)")
fig.savefig(p / "mount-replay.png", dpi=160)
with (p / "mount-scores.csv").open("w") as f:
    w = csv.writer(f, lineterminator="\n")
    w.writerow(
        "trial observation first_fork_contact_s physical_peak_g sim_peak_g physical_tap_delta sim_tap_delta".split()
    )
    w.writerows(summary)
