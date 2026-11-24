"""Prescribe robot trajectory from SDK poses and replay against a free canonical cube."""

from __future__ import annotations

import argparse
import copy
import csv
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
from typing import Any

import mujoco as mj
import numpy as np

HERE = Path(__file__).resolve().parent
ASSETS = HERE.parents[1] / "web" / "public" / "robot" / "vector2"
TRIALS = ("W33", "W34", "W35")
DEFAULT_DRIVE = "freejoint"
DEFAULT_CONTACT = "fork"
SENSOR_PHASES = 5


def _load_structured(name: str) -> np.ndarray:
    """Load CSV or gzipped CSV structured array from evidence directory."""
    path = HERE / name
    assert path.is_file(), f"missing data file: {path}"
    arr = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf8")
    assert len(arr) > 0, f"empty data file: {path}"
    return arr


def _load_model_config() -> tuple[dict[str, Any], dict[str, Any], ET.Element]:
    """Load Vector 2 model parameters, derived inertia, and shelf XML root."""
    doc = json.loads((ASSETS / "model.json").read_text(encoding="utf8"))
    source = ET.parse(ASSETS / "vector-free-shelf.xml").getroot()
    v, derived = doc["values"], doc["derived"]
    assert "total_mass_g" in v and "chassis_inertia" in derived, "incomplete model.json"
    assert source.find(".//body[@name='cube']") is not None, "cube body missing in shelf XML"
    return v, derived, source


def _attach_drive(
    body: ET.Element, name: str, drive: str, mass_kg: float, inertia: list[float]
) -> None:
    """Configure body as mocap or mass-bearing freejoint rigid body."""
    assert drive in ("freejoint", "mocap"), f"unknown drive mode: {drive}"
    assert mass_kg > 0.0 and len(inertia) == 3, "invalid robot inertial properties"
    if drive == "mocap":
        body.set("mocap", "true")
        return
    ET.SubElement(body, "freejoint", name=f"{name}_root")
    diag = f"{inertia[0]:.9g} {inertia[1]:.9g} {inertia[2]:.9g}"
    ET.SubElement(body, "inertial", pos="0 0 0", mass=f"{mass_kg:.9g}", diaginertia=diag)


def _clone_geom(geom: ET.Element, parent: ET.Element, offset_m: tuple[float, float, float]) -> None:
    """Append geom element into parent body shifted by offset_m."""
    assert geom.tag == "geom" and len(offset_m) == 3, "invalid geom clone inputs"
    copy = ET.SubElement(parent, "geom", dict(geom.attrib))
    ox, oy, oz = offset_m
    if ox == 0.0 and oy == 0.0 and oz == 0.0:
        return
    if "pos" in copy.attrib:
        px, py, pz = (float(c) for c in copy.attrib["pos"].split())
        copy.set("pos", f"{px + ox:.9g} {py + oy:.9g} {pz + oz:.9g}")
    elif "fromto" in copy.attrib:
        c = [float(x) for x in copy.attrib["fromto"].split()]
        shifted = (c[0] + ox, c[1] + oy, c[2] + oz, c[3] + ox, c[4] + oy, c[5] + oz)
        copy.set("fromto", " ".join(f"{x:.9g}" for x in shifted))


def _add_chassis_body(
    world: ET.Element, source: ET.Element, drive: str, mass_kg: float, inertia: list[float]
) -> None:
    """Add canonical chassis and wheels body aligned at front axle frame."""
    src_ch = source.find(".//body[@name='chassis']")
    assert src_ch is not None and drive in ("freejoint", "mocap"), "missing chassis in source XML"
    chassis = ET.SubElement(world, "body", name="robot_chassis")
    _attach_drive(chassis, "robot_chassis", drive, mass_kg, inertia)
    for g in src_ch.findall("geom"):
        _clone_geom(g, chassis, (0.0, 0.0, 0.0))
    for wname in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
        wb = src_ch.find(f"body[@name='{wname}']")
        assert wb is not None, f"missing wheel body {wname}"
        off = tuple(float(c) for c in wb.attrib["pos"].split())
        for g in wb.findall("geom"):
            _clone_geom(g, chassis, (off[0], off[1], off[2]))


def build_model(
    source: ET.Element,
    v: dict[str, Any],
    derived: dict[str, Any],
    drive: str = DEFAULT_DRIVE,
    contact: str = DEFAULT_CONTACT,
) -> mj.MjModel:
    """Construct MuJoCo model with fork (and optional chassis/wheels) plus free cube."""
    assert drive in ("freejoint", "mocap"), f"bad drive: {drive}"
    assert contact in ("fork", "full"), f"bad contact: {contact}"
    mass_kg = float(v["total_mass_g"]) * 0.001
    inertia = list(derived["chassis_inertia"])
    scene = ET.Element("mujoco")
    scene.append(copy.deepcopy(source.find("option")))
    world = ET.SubElement(scene, "worldbody")
    ET.SubElement(world, "geom", type="plane", size="1 1 .01", contype="0", conaffinity="1")
    fork = ET.SubElement(world, "body", name="fork")
    _attach_drive(fork, "fork", drive, mass_kg, inertia)
    for geom in source.findall(".//body[@name='lift_fork']/geom"):
        fork.append(copy.deepcopy(geom))
    if contact == "full":
        _add_chassis_body(world, source, drive, mass_kg, inertia)
    world.append(copy.deepcopy(source.find(".//body[@name='cube']")))
    ET.SubElement(ET.SubElement(scene, "sensor"), "accelerometer", name="cube_accel", site="cube_chip")
    return mj.MjModel.from_xml_string(ET.tostring(scene, encoding="unicode"))


def compute_trajectory(robot: np.ndarray, v: dict[str, Any], dt: float) -> dict[str, np.ndarray]:
    """Interpolate SDK robot poses into world fork, chassis, and quaternion traces."""
    assert len(robot) > 1 and dt > 0.0, "invalid trajectory inputs"
    times = np.arange(-3.0, 0.105, dt)

    def interp(key: str) -> np.ndarray:
        arr = robot[key]
        return np.interp(times, robot["t_s"], np.unwrap(arr) if key == "yaw_rad" else arr)

    x, y, yaw, pitch, h = map(interp, ("x_mm", "y_mm", "yaw_rad", "pitch_rad", "lift_mm"))
    px, pz = -v["wheel_base_mm"], v["wheel_rear_radius_mm"]
    shoulder, length = v["lift_base_position_mm"], v["lift_arm_length_mm"]
    h = np.clip(h, v["lift_height_min_mm"], v["lift_height_max_mm"])
    reach = shoulder[0] + np.sqrt(length**2 - (h - shoulder[2]) ** 2)
    cp, sp, cy, sy = np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    fork_along = px + (reach - px) * cp - (h - pz) * sp
    fork_h = pz + (reach - px) * sp + (h - pz) * cp
    ch_along = px * (1.0 - cp) + pz * sp
    ch_h = pz - px * sp - pz * cp
    a, b, c, s = np.cos(yaw / 2), np.sin(yaw / 2), np.cos(pitch / 2), np.sin(pitch / 2)
    quat = np.c_[a * c, b * s, -a * s, b * c]
    assert np.all(np.isfinite(quat)) and len(times) == len(quat), "bad trajectory interpolation"
    return {
        "times": times,
        "fork_pos": np.c_[x + fork_along * cy, y + fork_along * sy, fork_h] / 1000.0,
        "chassis_pos": np.c_[x + ch_along * cy, y + ch_along * sy, ch_h] / 1000.0,
        "quat": quat,
    }


def freejoint_qvel_for_body(
    model: mj.MjModel, qadr: int, dadr: int, pose0: np.ndarray, pose1: np.ndarray, dt: float
) -> np.ndarray:
    """Compute exact MuJoCo freejoint qvel [world_v, body_w] between consecutive 7-DOF poses."""
    assert dt > 0.0 and len(pose0) == 7 and len(pose1) == 7, "invalid pose inputs"
    q0, q1 = model.qpos0.copy(), model.qpos0.copy()
    qvel = np.zeros(model.nv)
    q0[qadr : qadr + 7] = pose0
    q1[qadr : qadr + 7] = pose1
    mj.mj_differentiatePos(model, qvel, dt, q0, q1)
    out = qvel[dadr : dadr + 6].copy()
    assert np.all(np.isfinite(out)), "non-finite differentiated freejoint qvel"
    return out


def _driven_targets(model: mj.MjModel, traj: dict[str, np.ndarray], drive: str) -> list[tuple[Any, ...]]:
    """Collect address metadata for driven robot bodies in model."""
    assert drive in ("freejoint", "mocap"), f"bad drive: {drive}"
    targets = []
    for bname, pkey in (("fork", "fork_pos"), ("robot_chassis", "chassis_pos")):
        bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, bname)
        if bid < 0:
            continue
        jid = int(model.body_jntadr[bid])
        mid = int(model.body_mocapid[bid])
        qadr = int(model.jnt_qposadr[jid]) if jid >= 0 else -1
        dadr = int(model.jnt_dofadr[jid]) if jid >= 0 else -1
        targets.append((bid, qadr, dadr, mid, traj[pkey], traj["quat"]))
    assert len(targets) >= 1, "no driven robot body found"
    return targets


def _apply_drive(
    model: mj.MjModel,
    data: mj.MjData,
    drive: str,
    targets: list[tuple[Any, ...]],
    bufs: tuple[np.ndarray, np.ndarray, np.ndarray],
    idx: int,
    nxt: int,
    dt: float,
) -> None:
    """Set mocap or velocity-driven freejoint pose/velocity at step idx."""
    q0_buf, q1_buf, qvel_buf = bufs
    assert idx >= 0 and nxt >= idx and dt > 0.0, "invalid step indices"
    if drive == "mocap":
        for _, _, _, mid, pos_arr, quat_arr in targets:
            data.mocap_pos[mid] = pos_arr[idx]
            data.mocap_quat[mid] = quat_arr[idx]
        return
    np.copyto(q0_buf, data.qpos)
    np.copyto(q1_buf, data.qpos)
    for _, qadr, _, _, pos_arr, quat_arr in targets:
        q0_buf[qadr : qadr + 3] = pos_arr[idx]
        q0_buf[qadr + 3 : qadr + 7] = quat_arr[idx]
        q1_buf[qadr : qadr + 3] = pos_arr[nxt]
        q1_buf[qadr + 3 : qadr + 7] = quat_arr[nxt]
    mj.mj_differentiatePos(model, qvel_buf, dt, q0_buf, q1_buf)
    for _, qadr, dadr, _, pos_arr, quat_arr in targets:
        data.qpos[qadr : qadr + 3] = pos_arr[idx]
        data.qpos[qadr + 3 : qadr + 7] = quat_arr[idx]
        data.qvel[dadr : dadr + 6] = qvel_buf[dadr : dadr + 6] if nxt > idx else 0.0


def _first_fork_contact(
    model: mj.MjModel, data: mj.MjData, fork_id: int, cube_id: int, current: float | None, t_s: float
) -> float | None:
    """Return earliest timestamp when fork and cube touch with dist <= 0."""
    assert fork_id >= 0 and cube_id >= 0, "invalid body IDs"
    if current is not None:
        return current
    for contact in data.contact:
        if set(model.geom_bodyid[contact.geom]) == {fork_id, cube_id} and contact.dist <= 0.0:
            return float(t_s)
    return None


def simulate_trial(
    model: mj.MjModel,
    obs: np.void,
    traj: dict[str, np.ndarray],
    drive: str = DEFAULT_DRIVE,
) -> dict[str, Any]:
    """Run MuJoCo replay for one visual observation and return raw accel and contact telemetry."""
    data = mj.MjData(model)
    dt = float(model.opt.timestep)
    times = traj["times"]
    cube_id, fork_id = model.body("cube").id, model.body("fork").id
    cq = int(model.jnt_qposadr[int(model.body_jntadr[cube_id])])
    cy = float(obs["yaw_rad"])
    data.qpos[cq : cq + 7] = [obs["x_mm"] / 1000.0, obs["y_mm"] / 1000.0, 0.0227,
                              np.cos(cy / 2.0), 0.0, 0.0, np.sin(cy / 2.0)]
    targets = _driven_targets(model, traj, drive)
    bufs = (model.qpos0.copy(), model.qpos0.copy(), np.zeros(model.nv))
    for _ in range(250):
        _apply_drive(model, data, drive, targets, bufs, 0, 0, dt)
        mj.mj_step(model, data)

    forces, first, max_lin, max_ang = [], None, 0.0, 0.0
    for i, t in enumerate(times):
        _apply_drive(model, data, drive, targets, bufs, i, min(i + 1, len(times) - 1), dt)
        mj.mj_step(model, data)
        first = _first_fork_contact(model, data, fork_id, cube_id, first, t)
        max_ang = max(max_ang, float(np.linalg.norm(data.cvel[fork_id][0:3])))
        max_lin = max(max_lin, float(np.linalg.norm(data.cvel[fork_id][3:6])))
        forces.append((data.sensor("cube_accel").data.copy() / 9.81).tolist())

    f_arr = np.asarray(forces)
    base = np.median(f_arr[(times >= -2.9) & (times < -2.1)], axis=0)
    raw_dev = np.linalg.norm(f_arr - base, axis=1)
    win = (times >= -0.8) & (times <= 0.1)
    assert len(forces) == len(times) and np.any(win), "empty simulation window"
    return {
        "dt": dt, "times": times, "forces": forces, "first_fork_contact_s": first,
        "sim_raw_peak_g": float(raw_dev[win].max()),
        "max_cvel_lin": max_lin, "max_cvel_ang": max_ang,
    }


def _score_one_phase(dt: float, start_s: float, forces: list[list[float]]) -> dict[str, Any]:
    """Run Node CubeSensor on one phase-aligned force stream and return peak and tap count."""
    assert dt > 0.0 and len(forces) > 10, "insufficient sensor samples"
    arg = {"dt": dt, "start": start_s, "forces": forces}
    out = subprocess.check_output(["node", str(HERE / "mount_sensor.mjs")],
                                  input=json.dumps(arg), text=True, cwd=str(HERE))
    arr = np.array(json.loads(out))
    st = arr[:, 0]
    sd = np.linalg.norm(arr[:, 1:4] - np.median(arr[(st >= -2.9) & (st < -2.1), 1:4], axis=0), axis=1)
    use = (st >= -0.8) & (st <= 0.1)
    assert np.any(use), "empty packet scoring window"
    return {
        "st": st[use], "sd": sd[use], "peak_g": float(sd[use].max()),
        "tap_delta": int(arr[use, 4][-1] - arr[st < -0.8, 4][-1]),
    }


def score_sensor_phases(dt: float, start_s: float, forces: list[list[float]]) -> dict[str, Any]:
    """Evaluate CubeSensor across all five 2 ms subsampling phases within a 10 ms tick."""
    frame_steps = int(round(0.002 / dt))
    assert frame_steps >= 1 and abs(frame_steps * dt - 0.002) < 1e-9, "dt must divide 2 ms"
    phases = [
        _score_one_phase(dt, start_s + k * frame_steps * dt, forces[k * frame_steps :])
        for k in range(SENSOR_PHASES)
    ]
    peaks = [ph["peak_g"] for ph in phases]
    assert len(phases) == SENSOR_PHASES, "unexpected phase count"
    return {
        "st": phases[0]["st"], "sd": phases[0]["sd"],
        "sim_peak_g": peaks[0], "sim_tap_delta": phases[0]["tap_delta"],
        "sim_phase_min_g": min(peaks), "sim_phase_max_g": max(peaks),
        "sim_phase_taps_nonzero": sum(1 for ph in phases if ph["tap_delta"] > 0),
    }


def _physical_series(physical: np.ndarray, name: str) -> dict[str, Any]:
    """Compute baseline-subtracted physical cube accel and tap count in [-0.8, 0.1]."""
    measured = physical[physical["trial"] == name]
    assert len(measured) > 0, f"missing physical samples for {name}"
    mt = measured["packet_s"]
    g = np.stack([measured[k] for k in "xyz"], axis=1) / 8192.0
    dev = np.linalg.norm(g - np.median(g[(mt >= -7.0) & (mt < -5.0)], axis=0), axis=1)
    use = (mt >= -0.8) & (mt <= 0.1)
    assert np.any(use) and np.any(mt < -0.8), f"empty mount window for {name}"
    return {
        "mt": mt[use], "dev": dev[use], "peak_g": float(dev[use].max()),
        "tap_delta": int(measured["tap"][use][-1]) - int(measured["tap"][mt < -0.8][-1]),
    }


def save_plot(panels: list[dict[str, Any]]) -> None:
    """Render W33-W35 comparison plot cleanly."""
    import matplotlib.pyplot as plt  # noqa: PLC0415

    assert len(panels) == 3, "expected 3 trial panels"
    fig, axes = plt.subplots(1, 3, figsize=(12, 4), sharey=True, layout="constrained")
    for ax, panel in zip(axes, panels):
        ax.scatter(panel["mt"], panel["dev"], color="black", s=6, label="Physical cube")
        for label, st, sd in panel["curves"]:
            ax.plot(st, sd, alpha=0.8, lw=1.1, label=label)
        ax.set_title(panel["trial"])
        ax.set_xlabel("Seconds from detected stand")
        ax.legend(fontsize=8)
    axes[0].set_ylabel("Change from resting acceleration (g)")
    fig.savefig(HERE / "mount-replay.png", dpi=160)
    plt.close(fig)
    assert (HERE / "mount-replay.png").is_file(), "plot was not written"


def run_all(drive: str = DEFAULT_DRIVE, contact: str = DEFAULT_CONTACT, write: bool = True, plot: bool = True) -> list[list[Any]]:
    """Run W33-W35 replay suite, optionally writing mount-scores.csv and mount-replay.png."""
    assert drive in ("freejoint", "mocap") and contact in ("fork", "full"), "bad replay mode"
    v, derived, source = _load_model_config()
    model = build_model(source, v, derived, drive=drive, contact=contact)
    path_arr, obs_arr, phys_arr = _load_structured("path.csv.gz"), _load_structured("observations.csv"), _load_structured("cube.csv.gz")
    summary, panels = [], []
    for name in TRIALS:
        traj = compute_trajectory(path_arr[path_arr["trial"] == name], v, float(model.opt.timestep))
        phys = _physical_series(phys_arr, name)
        curves = []
        for obs in obs_arr[obs_arr["trial"] == name]:
            sim = simulate_trial(model, obs, traj, drive=drive)
            pkt = score_sensor_phases(sim["dt"], float(sim["times"][0]), sim["forces"])
            curves.append((f"{obs['observation']} pose [{pkt['sim_phase_min_g']:.2f}..{pkt['sim_phase_max_g']:.2f} g]", pkt["st"], pkt["sd"]))
            summary.append([
                name, obs["observation"], sim["first_fork_contact_s"], phys["peak_g"],
                pkt["sim_peak_g"], phys["tap_delta"], pkt["sim_tap_delta"], sim["sim_raw_peak_g"],
                pkt["sim_phase_min_g"], pkt["sim_phase_max_g"], pkt["sim_phase_taps_nonzero"],
            ])
        panels.append({"trial": name, "mt": phys["mt"], "dev": phys["dev"], "curves": curves})
    if plot:
        save_plot(panels)
    if write:
        cols = ("trial observation first_fork_contact_s physical_peak_g sim_peak_g "
                "physical_tap_delta sim_tap_delta sim_raw_peak_g sim_phase_min_g "
                "sim_phase_max_g sim_phase_taps_nonzero").split()
        with (HERE / "mount-scores.csv").open("w", newline="", encoding="utf8") as f:
            w = csv.writer(f, lineterminator="\n")
            w.writerow(cols)
            w.writerows(summary)
    assert len(summary) == 6, "expected 6 trial/observation score rows"
    return summary


def main(argv: list[str] | None = None) -> int:
    """CLI entry point for mount replay."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--drive", choices=("freejoint", "mocap"), default=DEFAULT_DRIVE)
    ap.add_argument("--contact", choices=("fork", "full"), default=DEFAULT_CONTACT)
    ap.add_argument("--no-plot", action="store_true", help="skip writing mount-replay.png")
    ap.add_argument("--no-write", action="store_true", help="skip writing mount-scores.csv")
    args = ap.parse_args(argv)
    assert args.drive in ("freejoint", "mocap") and args.contact in ("fork", "full")
    rows = run_all(drive=args.drive, contact=args.contact, write=not args.no_write, plot=not args.no_plot)
    if args.no_write:
        print(json.dumps(rows, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
