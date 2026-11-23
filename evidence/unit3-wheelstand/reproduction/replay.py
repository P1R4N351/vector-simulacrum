"""Replay one physical wheelstand (W4 or W8) through the full model: four onset cases, one model variant.

Derived from W4/free_chassis.py --firmware; W8 starts from the camera-registered cube position
(W8/camera-pose.csv), W4 from its pre-command SDK sighting. Model switches:
  --driven-tips      parameter tip_supports_driven = 1 (tip supports geared to their tread)
  --lift servo:CAP   the model's position servo with a lift torque cap of CAP mN m
  --lift motor:S:W   the public lift PID (liftController.cpp, hardware gains) driving a linear DC motor
                     with stall torque S mN m at the joint and no-load speed W rad/s at full power
  --override k=v     any parameter (JSON value), e.g. com_z_mm=18
  --cube-forward-mm  move the starting cube along the robot heading (sensitivity checks)
Prints the physical reference, then one JSON object per case.
"""
import argparse
import csv
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco as mj
import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))
from simulacrum import mjcf, params  # noqa: E402
from simulacrum.cube import CubeAccelerometer  # noqa: E402
from simulacrum.drive import TreadController  # noqa: E402

EVID = REPO / "evidence/unit3-wheelstand"
END_S = 5.8           # replay end, seconds after the command (W4 clock)
SETTLE_S = 0.20
LIFT_KP, LIFT_KD, LIFT_KI, LIFT_SUM_MAX = 3.0, 3000.0, 0.1, 5.0   # liftController.cpp:83-86
CONTROL_DT = 0.005
DUMP = []
CUBE_SHIFT_MM = 0.0


def build(v, fixture):
    tree = ET.fromstring(mjcf.build_mjcf(v, (), cube="free"))
    fx = ET.parse(fixture)
    asset = ET.SubElement(tree, "asset")
    asset.extend(fx.findall(".//asset/mesh"))
    fork = tree.find(".//body[@name='lift_fork']")
    attrs = {k: x for k, x in fork.find("geom").attrib.items() if k not in ("name", "type", "size", "pos", "mass")}
    for geom in fork.findall("geom"):
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
    pieces = fx.findall(".//body[@name='fork']/geom[@type='mesh']")
    assert len(pieces) == 58
    for piece in pieces:
        ET.SubElement(fork, "geom", dict(attrs, type="mesh", mesh=piece.get("mesh"), name=piece.get("name"), mass="0"))
    return mj.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))


class LiftMotor:
    """liftController.cpp's PID (hardware gains) on the lift angle, driving a linear DC motor at the joint."""

    def __init__(self, stall_mNm, noload_radps):
        assert stall_mNm > 0 and noload_radps > 0
        self.stall, self.noload = stall_mNm * 1e-3, noload_radps
        self.prev_err, self.err_sum, self.power = 0.0, 0.0, 0.0

    def update(self, desired, angle):
        err = desired - angle
        power = LIFT_KP * err + LIFT_KD * (err - self.prev_err) * CONTROL_DT + LIFT_KI * self.err_sum
        self.err_sum = float(np.clip(self.err_sum + err, -LIFT_SUM_MAX, LIFT_SUM_MAX))
        self.prev_err = err
        self.power = float(np.clip(power, -1.0, 1.0))

    def torque(self, speed):
        return self.stall * self.power - self.stall / self.noload * speed


def lowering_goal(v, elapsed, start_angle):
    goal = mjcf.lift_angle_of_height(v, 32)
    distance = max(0, start_angle - goal)
    vmax = min(10.0, np.sqrt(200.0 * distance))
    if vmax == 0:
        return goal
    ramp = vmax / 200.0
    cruise = max(0.0, distance / vmax - ramp)
    end = 2 * ramp + cruise
    if elapsed < ramp:
        return start_angle - 100.0 * elapsed ** 2
    if elapsed < ramp + cruise:
        return start_angle - 0.5 * vmax * ramp - vmax * (elapsed - ramp)
    return goal + 100.0 * max(0.0, end - elapsed) ** 2


class Trial:
    def __init__(self, name):
        self.name = name
        self.dir = EVID / name
        self.approach = np.genfromtxt(self.dir / "approach.csv", names=True, delimiter=",")
        self.state = np.genfromtxt(self.dir / "state.csv.gz", names=True, delimiter=",")
        self.cube = np.genfromtxt(self.dir / "cube.csv.gz", names=True, delimiter=",")
        with (self.dir / "actions.csv").open() as f:
            self.zero = next(float(r["sec"]) for r in csv.DictReader(f) if r["action"] == "pop_a_wheelie")
        target = json.loads((self.dir / "initial-cube.json").read_text())
        w, x, y, z = [target["q" + str(i)] for i in range(4)]
        self.cube_yaw = float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        a = self.approach
        if name == "W4":
            self.early, self.late = 3.950686, 4.015283
        else:
            drop = np.flatnonzero((a["lift_mm"][1:] - a["lift_mm"][:-1]) < -10)[0] + 1
            self.early, self.late = [float(s) for s in a["s"][drop - 2:drop]]
            target = self.camera_xy(target)
        self.cube_xy = np.array([target["x"], target["y"]])
        if CUBE_SHIFT_MM:
            yaw = np.interp(self.late, a["s"], a["yaw"])
            self.cube_xy = self.cube_xy + CUBE_SHIFT_MM * np.array([np.cos(yaw), np.sin(yaw)])
        self.shift = self.late - 4.015283

    def camera_xy(self, target):
        with (self.dir / "camera-pose.csv").open() as f:
            ref = next(r for r in csv.DictReader(f) if r["case"] == "geometry" and
                       [float(r[k]) for k in ("focal_px", "lens_height_mm", "fork_height_mm")] == [498.4, 65.5, 93.0])
        x, y, yaw = [np.interp(3.065, self.approach["s"], self.approach[k]) for k in ("x_mm", "y_mm", "yaw")]
        fwd, left = float(ref["forward_mm"]), float(ref["left_mm"])
        return dict(target, x=x + fwd * np.cos(yaw) - left * np.sin(yaw), y=y + fwd * np.sin(yaw) + left * np.cos(yaw))

    def obs(self, t, k):
        if k in self.approach.dtype.names:
            return float(np.interp(t, self.approach["s"], self.approach[k]))
        return float(np.interp(t + self.zero, self.state["sec"], self.state[k]))

    def physical(self):
        s = self.state
        t = s["sec"] - self.zero
        win = (t > 3.0 + self.shift) & (t < 5.5 + self.shift)
        pitch = np.degrees(s["pitch"][win])
        i = int(np.argmax(pitch))
        c = self.cube
        ct = c["sec"] - self.zero
        cw = (ct >= self.late - 0.3) & (ct <= self.late + 1.2)
        g = np.linalg.norm(np.stack([c[k] for k in "xyz"]), axis=0)[cw] / 8192
        taps = c["tap"][cw]
        tw = t[win]
        hold = (tw >= self.late + 0.4) & (tw <= self.late + 0.8)
        return dict(peak_pitch_deg=float(pitch[i]), t_peak_s=float(tw[i] - self.late),
                    hold_pitch_deg=float(np.median(pitch[hold])),
                    peak_g=float(g.max()), taps=int(taps.max() - taps.min()))


class _Run:
    """One replay: the model state, its controllers and the recorded trace. Split from run() by phase."""

    def __init__(self, m, v, tr, start, onset, lift_spec, seed):
        assert start < onset < END_S + tr.shift, "release, onset and end out of order"
        kind, *nums = lift_spec.split(":")
        assert kind in ("servo", "motor") and len(nums) == (1 if kind == "servo" else 2), "bad --lift"
        self.m, self.v, self.tr, self.start, self.onset = m, v, tr, start, onset
        self.d = d = mj.MjData(m)
        self.dt = m.opt.timestep
        self.motor = None
        act = m.actuator("lift")
        if kind == "servo":
            act.forcerange[:] = [-float(nums[0]) * 1e-3, float(nums[0]) * 1e-3]
        else:
            self.motor = LiftMotor(float(nums[0]), float(nums[1]))
            act.gainprm[0] = 0.0
            act.biasprm[:3] = 0.0
            act.forcerange[:] = [-1.0, 1.0]
        self.cube_id = m.body("cube").id
        cube = self.cube = m.joint("cube_root").qposadr[0]
        self.yaw = yaw = tr.obs(start, "yaw")
        d.qpos[:2] = np.array([tr.obs(start, "x_mm"), tr.obs(start, "y_mm")]) * 1e-3
        d.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        d.qpos[cube:cube + 2] = tr.cube_xy * 1e-3
        d.qpos[cube + 3:cube + 7] = [np.cos(tr.cube_yaw / 2), 0, 0, np.sin(tr.cube_yaw / 2)]
        self.lift = lift = mjcf.lift_angle_of_height(v, tr.obs(start, "lift_mm"))
        for name, val in [("lift", lift), ("wrist", -lift), ("head", tr.obs(start, "head"))]:
            d.joint(name + "_joint").qpos[0] = val
        d.ctrl[m.actuator("head").id] = tr.obs(start, "head")
        d.ctrl[act.id] = lift
        self.lj = m.joint("lift_joint")
        self.chassis = m.jnt_bodyid[m.joint("root").id]
        mj.mj_forward(m, d)
        self.overlap = max([-c.dist * 1000 for c in d.contact if self.cube_id in m.geom_bodyid[c.geom] and c.dist < 0
                            and 0 not in m.geom_bodyid[c.geom]], default=0.0)

    def lift_force(self, goal):
        if self.motor is None:
            return
        self.motor.update(goal, float(self.d.qpos[self.lj.qposadr[0]]))
        self.motor_torque()

    def motor_torque(self):
        self.d.qfrc_applied[self.lj.dofadr[0]] = self.motor.torque(float(self.d.qvel[self.lj.dofadr[0]]))

    def settle(self, seed):
        m, v, tr, d, start = self.m, self.v, self.tr, self.d, self.start
        for i in range(round(SETTLE_S / self.dt)):
            if i % round(CONTROL_DT / self.dt) == 0:
                self.lift_force(self.lift)
            mj.mj_step(m, d)
        self.center = d.qpos[self.cube:self.cube + 2].copy()
        speed = np.mean([tr.obs(start, "left"), tr.obs(start, "right")])
        d.qvel[:2] = 1e-3 * speed * np.array([np.cos(self.yaw), np.sin(self.yaw)])
        for axle, radius in [("f", v["wheel_front_radius_mm"]), ("r", v["wheel_rear_radius_mm"])]:
            for side, key in [("l", "left"), ("r", "right")]:
                d.joint("wheel_" + axle + side + "_joint").qvel[0] = tr.obs(start, key) / radius
        self.controllers = [TreadController(v), TreadController(v)]
        self.sensor = CubeAccelerometer(v, self.dt * 4, seed=seed)
        self.trace = []
        self.drive_ended = False
        self.lowering_start = mjcf.lift_angle_of_height(v, tr.obs(self.onset, "lift_mm"))

    def control(self, t):
        """One firmware control tick: tread speed loops and the lift and head targets."""
        m, v, tr, d, onset = self.m, self.v, self.tr, self.d, self.onset
        lowering = t >= onset
        pitch_rad = np.arcsin(np.clip(d.xmat[self.chassis].reshape(3, 3)[2, 0], -1, 1))
        if lowering:
            self.drive_ended |= pitch_rad > 1.2 or t - onset > 0.5
        for ctl, side in zip(self.controllers, ["left", "right"]):
            measured = d.joint("wheel_f" + side[0] + "_joint").qvel[0] * v["wheel_front_radius_mm"]
            requested = tr.obs(t, side) if t < 4.84 + tr.shift else 0
            if lowering:
                requested = 0 if self.drive_ended else 150
            d.actuator(side + "_wheel").ctrl[0] = ctl.update(requested, measured) * 3.92 / v["motor_reference_voltage_v"]
        goal = (lowering_goal(v, t - onset, self.lowering_start) if lowering
                else mjcf.lift_angle_of_height(v, tr.obs(t, "lift_mm")))
        d.actuator("lift").ctrl[0] = goal
        self.lift_force(goal)
        d.actuator("head").ctrl[0] = tr.obs(t, "head")

    def dump(self, t):
        m, d, tr = self.m, self.d, self.tr
        names = sorted({tuple(sorted((m.geom(c.geom[0]).name[:14], m.geom(c.geom[1]).name[:14]))) for c in d.contact})
        rc = d.xmat[self.chassis].reshape(3, 3)
        print(json.dumps(dict(t=round(t - tr.late, 3), pitch=round(float(np.degrees(np.arctan2(rc[2, 0], rc[2, 2]))), 1),
                              rate=round(float(np.degrees(d.qvel[4])), 0),
                              wrist_rel_cube_mm=[round(float(x), 1) for x in
                                                 1000 * (d.xpos[m.body("lift_fork").id] - d.xpos[self.cube_id])],
                              lift_mm=round(mjcf.lift_height_of_angle(self.v, float(d.qpos[self.lj.qposadr[0]])), 1),
                              contacts=names)), file=sys.stderr)

    def sample(self, i, t):
        """Every fourth physics step: the cube sensor and one trace row."""
        m, v, d, cube_id = self.m, self.v, self.d, self.cube_id
        self.sensor.step((i + 1) * self.dt, d.sensor("cube_accel").data / (v["gravity_mmps2"] * 1e-3))
        rot = d.xmat[cube_id].reshape(3, 3)
        top = d.xpos[cube_id, :2] + 0.022 * rot[:2, 2]
        rc = d.xmat[self.chassis].reshape(3, 3)
        pitch = np.degrees(np.arctan2(rc[2, 0], rc[2, 2]))
        self.trace.append([t, pitch, np.linalg.norm(top - self.center) * 1000,
                           np.degrees(np.arctan2(rot[1, 0], rot[0, 0]) - self.tr.cube_yaw),
                           mjcf.lift_height_of_angle(v, float(d.qpos[self.lj.qposadr[0]])),
                           d.qvel[m.joint("wheel_fl_joint").dofadr[0]] * v["wheel_front_radius_mm"],
                           1000 * d.xpos[cube_id, 0], 1000 * d.xpos[cube_id, 2], 1000 * d.xpos[self.chassis, 0]])


def run(m, v, tr, start, onset, lift_spec, seed):
    """Replay from release time `start`, wheelstand command at `onset`; returns (trace, packets, taps, overlap)."""
    r = _Run(m, v, tr, start, onset, lift_spec, seed)
    r.settle(seed)
    ticks = round(1e-3 * v["control_period_ms"] / r.dt)
    assert ticks >= 1, "control period shorter than the physics step"
    for i, t in enumerate(np.arange(start, END_S + tr.shift, r.dt)):
        if i % ticks == 0:
            r.control(t)
        elif r.motor is not None:
            r.motor_torque()
        mj.mj_step(m, r.d)
        if DUMP and any(abs(t - (tr.late + q)) < r.dt / 2 for q in DUMP):
            r.dump(t)
        if (i + 1) % 4:
            continue
        r.sample(i, t)
    pk = np.array([[start + tt, np.linalg.norm(a) / 8192] for tt, p in r.sensor.messages
                   for a in np.frombuffer(p[2:], "<i2").reshape(3, 3)])
    return np.array(r.trace), pk, len(r.sensor.taps.events), r.overlap


def score(tr, tr_a, pk, taps, onset):
    t, pitch = tr_a[:, 0], tr_a[:, 1]
    i = int(np.argmax(pitch))
    s = tr.state
    ts = s["sec"] - tr.zero
    win = (ts >= tr.late - 0.1) & (ts <= tr.late + 1.0)
    sim_at = np.interp(ts[win], t, pitch)
    rmse = float(np.sqrt(np.mean((sim_at - np.degrees(s["pitch"][win])) ** 2)))
    settled = tr_a[t > END_S - 0.3 + tr.shift]
    wpk = pk[(pk[:, 0] >= tr.late - 0.3) & (pk[:, 0] <= tr.late + 1.2)]
    hold = (t >= tr.late + 0.4) & (t <= tr.late + 0.8)
    s1 = (ts >= tr.late - 0.05) & (ts <= tr.late + 0.25)
    s1_pitch = float(np.sqrt(np.mean((np.interp(ts[s1], t, pitch) - np.degrees(s["pitch"][s1])) ** 2)))
    s1_lift = float(np.sqrt(np.mean((np.interp(ts[s1], t, tr_a[:, 4]) - s["lift"][s1]) ** 2)))
    return dict(stage1_pitch_rmse_deg=s1_pitch, stage1_lift_rmse_mm=s1_lift,peak_pitch_deg=float(pitch[i]), t_peak_s=float(t[i] - tr.late), pitch_rmse_deg=rmse,
                hold_pitch_deg=float(np.median(pitch[hold])),
                travel_mm=float(np.median(settled[:, 2])), yaw_deg=float(np.median(settled[:, 3])),
                peak_g=float(wpk[:, 1].max()) if len(wpk) else 0.0, taps=taps, onset_s=onset)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fixture")
    ap.add_argument("--trial", default="W4", choices=["W4", "W8"])
    ap.add_argument("--driven-tips", action="store_true")
    ap.add_argument("--lift", default="servo:30")
    ap.add_argument("--seed", type=int, default=17)
    ap.add_argument("--override", action="append", default=[], help="name=value parameter override")
    ap.add_argument("--cases", default="all", choices=["all", "one"])
    ap.add_argument("--trace", help="write the first case's trace to this .npz")
    ap.add_argument("--cube-forward-mm", type=float, default=0.0, help="move the initial cube along the robot heading")
    ap.add_argument("--dump", default="", help="comma-separated times after the late onset: print contacts")
    a = ap.parse_args()
    DUMP.extend(float(x) for x in a.dump.split(",") if x)
    global CUBE_SHIFT_MM
    CUBE_SHIFT_MM = a.cube_forward_mm
    overrides = {k: json.loads(x) for k, x in (o.split("=", 1) for o in a.override)}
    if a.driven_tips:
        overrides["tip_supports_driven"] = 1
    v = params.values(params.load(model="vector_2"), overrides)
    m = build(v, a.fixture)
    m.opt.timestep = v["sim_timestep_ms"] * 1e-3 / 4
    tr = Trial(a.trial)
    print(json.dumps(dict(trial=a.trial, physical=tr.physical(), lift=a.lift, driven_tips=a.driven_tips,
                          overrides=overrides)), flush=True)
    cases = [(s + tr.shift, o) for s in (3.5, 3.8) for o in (tr.early, tr.late)]
    if a.cases == "one":
        cases = cases[1:2]
    for k, (start, onset) in enumerate(cases):
        tr_a, pk, taps, overlap = run(m, v, tr, start, onset, a.lift, a.seed)
        out = score(tr, tr_a, pk, taps, onset)
        out.update(release_s=start, initial_overlap_mm=overlap)
        print(json.dumps(out), flush=True)
        if a.trace and k == 0:
            np.savez(a.trace, trace=tr_a, packets=pk)


if __name__ == "__main__":
    main()
