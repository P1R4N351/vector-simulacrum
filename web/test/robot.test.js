// The browser robot layer against runs of the Python VectorSim..
//
// Same MJCF, same parameter values (sensor noise zeroed on both sides), same
// commands at the same ticks. MuJoCo here is the WebAssembly build; there it
// was the native one, version recorded in the fixture.
//
// Measured 2026-11-18 (mujoco 4.0.1 both sides): on the table the two agree
// to 1e-8 mm; in the run that stops at the edge to 1.2e-3 mm. Once the robot
// tips off the table the tumble is chaotic and the two part company, so the
// falling runs are compared up to the edge and then on outcome only.
import assert from "node:assert/strict";
import { test } from "node:test";
import { CONTROL_DEFAULT, VectorSim, liftAngleOfHeight, liftHeightOfAngle } from "../src/sim/robot.js";
import { pyRound } from "../src/sim/util.js";
import { mujoco, readJson, readText, sha256 } from "./helpers.js";

const fixture = readJson("test/fixtures/robot.json");
const model = readJson("public/robot/model.json");
const xml = readText("public/robot/vector.xml");
const METHOD = {
  set_wheel_motors: "setWheelMotors", command_head_angle: "commandHeadAngle",
  command_lift_height: "commandLiftHeight", stop_all_motors: "stopAllMotors", drive_straight: "driveStraight",
  turn_in_place: "turnInPlace", set_head_angle: "setHeadAngle", set_lift_height: "setLiftHeight",
};
// Tolerances per numeric column (tick, x, y, z, heading, wl, wr, power l, power r, head, lift).
const TOLERANCE = [0, 0.01, 0.01, 0.01, 1e-3, 1, 1, 1e-3, 1e-3, 1e-6, 1e-6];
const FIRST_LATE_NUMBER = 17; // after the flags: battery volts, two tread heats, then the range sensor's reading and flags
const NOISE_KEYS = ["accel_noise_rms_mg", "gyro_noise_rms_dps", "tof_range_std_fraction", "tof_weak_distance_sd_mm"];

function row(sim) {
  const truth = sim.simTruth();
  const [x, y, z] = sim.poseMm();
  const rep = sim.reportedNow();
  return [sim.tickCount, x, y, z, sim.headingTotalRad(), rep.wl, rep.wr, truth.motor_power[0],
    truth.motor_power[1], rep.head_deg, rep.lift_mm, truth.cliff_flags, sim.hasControl, sim.reactions,
    truth.fallen, sim.hazards.pickedUp, sim.hazards.falling, sim.batteryVolts, sim.treadHeat[0], sim.treadHeat[1],
    sim.state.prox_distance_mm, sim.state.prox_signal_quality, sim.state.prox_found_object,
    sim.state.prox_unobstructed, sim.state.prox_is_lift_in_fov];
}

async function makeSim(level, overrides = {}, headingDeg = 0) {
  const values = { ...fixture.values, ...overrides };
  return new VectorSim(await mujoco(), { xml, values, derived: model.derived, controlLevel: level, headingDeg });
}

async function replay(run) {
  const sim = await makeSim(run.level, run.overrides, run.heading_deg ?? 0);
  const rows = [row(sim)];
  let at = 0;
  for (let tick = 0; tick < run.ticks; tick++) {
    while (at < run.script.length && run.script[at][0] === tick) {
      const [, name, ...args] = run.script[at];
      assert.equal(sim[METHOD[name]](...args), true, `${name} accepted at tick ${tick}`);
      at += 1;
    }
    sim.tick(1);
    if (sim.tickCount % fixture.sample_ticks === 0) rows.push(row(sim));
  }
  sim.dispose();
  return rows;
}

function compareRows(mine, theirs, count, label) {
  assert.ok(count > 10, `${label}: too few rows to compare (${count})`);
  for (let i = 0; i < count; i++) {
    for (let c = 0; c < TOLERANCE.length; c++) {
      const gap = Math.abs(mine[i][c] - theirs[i][c]);
      assert.ok(gap <= TOLERANCE[c], `${label} row ${i} ${fixture.columns[c]}: ${mine[i][c]} against ${theirs[i][c]}`);
    }
    for (let c = TOLERANCE.length; c < FIRST_LATE_NUMBER; c++) {
      assert.deepEqual(mine[i][c], theirs[i][c], `${label} row ${i} ${fixture.columns[c]}`);
    }
    for (let c = FIRST_LATE_NUMBER; c < fixture.columns.length; c++) {
      assert.ok(Math.abs(mine[i][c] - theirs[i][c]) <= 1e-6, `${label} row ${i} ${fixture.columns[c]}: ${mine[i][c]} against ${theirs[i][c]}`);
    }
  }
}

test("the exported model is the one the fixtures were made with", () => {
  assert.equal(sha256(xml), model.source.xml_sha256, "vector.xml differs from the hash in model.json");
  assert.equal(fixture.mujoco_version, "4.0.1", "fixture made with another MuJoCo; re-measure the tolerances");
  for (const [key, value] of Object.entries(model.values)) {
    if (!NOISE_KEYS.includes(key)) assert.deepEqual(fixture.values[key], value, `fixture value ${key}`);
  }
  for (const key of NOISE_KEYS) assert.equal(fixture.values[key], 0, `fixture ${key} is zero`);
});

test("driving, turning, head and lift match the Python on the table", async () => {
  const run = fixture.runs.drive_and_turn;
  const rows = await replay(run);
  assert.equal(rows.length, run.rows.length);
  compareRows(rows, run.rows, run.rows.length, "drive_and_turn");
});

test("at the default level the robot stops at the edge and backs off, as the Python does", async () => {
  const run = fixture.runs.edge_default;
  const rows = await replay(run);
  assert.equal(rows.length, run.rows.length);
  compareRows(rows, run.rows, run.rows.length, "edge_default");
  const last = rows.at(-1);
  assert.equal(last[13], 1, "one cliff reaction");
  assert.equal(last[12], false, "control was taken away");
  assert.equal(last[14], false, "the robot is still on the table");
});

test("a 300 mm/s spin is not clamped to the path-following limit, and battery and heat match the Python", async () => {
  const run = fixture.runs.fast_spin;
  const rows = await replay(run);
  assert.equal(rows.length, run.rows.length);
  compareRows(rows, run.rows, run.rows.length, "fast_spin");
  const fastest = Math.max(...rows.map((r) => Math.abs(r[5])));
  assert.ok(fastest > model.values.wheel_speed_max_mmps, `measured tread speed reached ${fastest} mm/s`);
  assert.ok(rows.at(-1)[18] > 0.05 && rows.at(-1)[19] > 0.05, "the treads warmed");
});

test("a weak, sagging battery slows the treads as it does in the Python", async () => {
  const run = fixture.low_battery;
  const rows = await replay(run);
  assert.equal(rows.length, run.rows.length);
  compareRows(rows, run.rows, run.rows.length, "low_battery");
  const lowest = Math.min(...rows.map((r) => r[17]));
  assert.ok(lowest < 3.4 && lowest > 3.0, `battery sagged to ${lowest} V`);
});

test("the range sensor finds the block, loses it behind the lift, and reads as the Python does", async () => {
  const run = fixture.runs.look_at_block;
  const rows = await replay(run);
  assert.equal(rows.length, run.rows.length);
  compareRows(rows, run.rows, run.rows.length, "look_at_block");
  assert.ok(rows.some((r) => r[22]) && rows.some((r) => r[24]) && rows.some((r) => r[23]), "found, lift in view and clear all occur");
  const found = rows.filter((r) => r[22]);
  assert.ok(found.every((r) => r[20] > 120 && r[20] < 200), "the block is read at about its distance");
});

test("at the override level the robot drives off the table, as the Python does", async () => {
  const run = fixture.runs.edge_override;
  const rows = await replay(run);
  const edge = run.rows.findIndex((r) => r[11].some(Boolean));
  assert.ok(edge > 10, "the Python run reaches the edge");
  compareRows(rows, run.rows, edge, "edge_override before the edge");
  assert.equal(rows.at(-1)[13], 0, "no cliff reaction");
  assert.equal(rows.at(-1)[14], true, "the robot fell");
  assert.equal(run.rows.at(-1)[14], true, "the Python robot fell too");
});

// One run of an action list on a fresh robot, the start nudged along x by nudgeM metres.
async function runActions(run, nudgeM) {
  const sim = await makeSim(run.level);
  sim.data.qpos[0] += nudgeM;
  const steps = [];
  for (const step of run.steps) {
    const [name, ...args] = step.call;
    const started = sim[METHOD[name]](...args);
    for (let guard = 0; sim.actionRunning && guard < 20000; guard++) sim.stepAction();
    assert.equal(sim.actionRunning, false, `${name} ended`);
    steps.push({ started, ...sim.lastAction, clock: sim.tickCount, pose: sim.poseMm() });
  }
  sim.dispose();
  return steps;
}

const apart = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

// The encoder counts whole edges and the firmware keeps speed in whole mm/s, so the model has
// points where a difference in the last digit changes what happens next. A long sequence can
// pass through one and then differ by millimetres and ticks, here and in the Python alike
// (measured 2026-11-18: a 1 nm nudge at the start moves the sixth action's end by 1.5 mm and
// 11 ticks). So the port is held to the Python as closely as it holds to itself when nudged by
// a nanometre or two: exactly where the sequence is insensitive, loosely only where it is not.
test("blocking actions give the Python's results, as closely as the model repeats itself", async () => {
  for (const run of fixture.actions) {
    const mine = await runActions(run, 0);
    const nudged = [];
    for (const nudgeM of [1e-9, -1e-9, 2e-9, -2e-9]) nudged.push(await runActions(run, nudgeM));
    let exactSteps = 0;
    run.steps.forEach((step, i) => {
      const got = mine[i];
      const label = `${run.level} ${step.call[0]}(${step.call.slice(1).join(", ")})`;
      assert.equal(got.result, step.result, `${label} result`);
      assert.equal(got.started, step.result !== "no_control", `${label} accepted`);
      if (step.result === "aborted_off_ground") return; // the tumble is chaotic: outcome only
      const poseSpread = Math.max(...nudged.map((n) => apart(n[i].pose, got.pose)));
      const tickSpread = Math.max(...nudged.map((n) => Math.abs(n[i].clock - got.clock)));
      const poseTol = Math.max(0.01, 3 * poseSpread);
      const tickTol = 3 * tickSpread;
      if (tickTol === 0 && poseTol === 0.01) exactSteps += 1;
      assert.ok(Math.abs(got.clock - step.tick) <= tickTol, `${label} clock ${got.clock} against ${step.tick} (allowed ${tickTol})`);
      assert.ok(Math.abs(got.ticks - step.ticks) <= tickTol, `${label} ticks ${got.ticks} against ${step.ticks} (allowed ${tickTol})`);
      assert.ok(apart(got.pose, step.pose_mm) <= poseTol, `${label} pose off by ${apart(got.pose, step.pose_mm)} mm (allowed ${poseTol})`);
      assert.ok(Math.abs(got.moved_mm - step.moved_mm) <= poseTol, `${label} moved ${got.moved_mm} against ${step.moved_mm}`);
    });
    assert.ok(exactSteps >= 1, `${run.level}: at least one action is compared exactly (${exactSteps})`);
  }
});

test("reset puts the robot back exactly where a fresh one starts", async () => {
  const fresh = await makeSim(CONTROL_DEFAULT);
  const start = row(fresh);
  fresh.setWheelMotors(90, 40);
  fresh.tick(300);
  assert.ok(Math.hypot(fresh.poseMm()[0], fresh.poseMm()[1]) > 20, "the robot moved before the reset");
  fresh.reset();
  assert.deepEqual(row(fresh), start);
  fresh.dispose();
});

test("a wheel command is refused while an action runs or control is gone", async () => {
  const sim = await makeSim(CONTROL_DEFAULT);
  assert.equal(sim.driveStraight(40, 50), true);
  assert.equal(sim.setWheelMotors(50, 50), false, "refused during an action");
  sim.tick(2000);
  assert.equal(sim.actionRunning, false);
  assert.equal(sim.lastAction.result, "completed");
  sim.setWheelMotors(100, 100);
  sim.tick(1500);
  assert.equal(sim.hasControl, false, "the edge took control away");
  assert.equal(sim.setWheelMotors(50, 50), false, "refused without control");
  assert.equal(sim.requestControl(), true);
  assert.equal(sim.setWheelMotors(-30, -30), true, "accepted once control is back");
  sim.dispose();
});

test("only the front wheels have motors; the rear wheels are turned by the track", async () => {
  const mj = await mujoco();
  const sim = await makeSim(CONTROL_DEFAULT);
  const jointOf = (name) => mj.mj_name2id(sim.model, mj.mjtObj.mjOBJ_JOINT.value, name);
  const driven = [0, 1, 2, 3].map((actuator) => sim.model.actuator_trnid[2 * actuator]);
  assert.ok(driven.includes(jointOf("wheel_fl_joint")) && driven.includes(jointOf("wheel_fr_joint")), "front wheels are driven");
  assert.ok(!driven.includes(jointOf("wheel_rl_joint")) && !driven.includes(jointOf("wheel_rr_joint")), "rear wheels have no motor");
  sim.setWheelMotors(80, 40);
  sim.tick(400);
  const [fl, fr, rl, rr] = sim.wheelSpinRadps();
  const gear = model.derived.rear_gear;
  assert.ok(fl > 4 && fr > 2 && fl > 1.5 * fr, `front wheels turn at their own speeds: ${fl}, ${fr}`);
  assert.ok(Math.abs(rl / fl - gear) < 0.02 * gear, `rear left follows the front by the radius ratio: ${rl / fl} against ${gear}`);
  assert.ok(Math.abs(rr / fr - gear) < 0.02 * gear, `rear right follows the front by the radius ratio: ${rr / fr} against ${gear}`);
  const angles = sim.wheelAnglesRad();
  assert.ok(Math.abs(angles[2] / angles[0] - gear) < 0.02 * gear, "rear left angle follows too");
  assert.equal(sim.simTruth().wheel_angles_rad.length, 4);
  sim.dispose();
});

test("head and lift targets are readable and the actions reach them", async () => {
  const sim = await makeSim(CONTROL_DEFAULT);
  const v = model.values;
  assert.ok(Math.abs(sim.liftTargetMm - v.lift_height_min_mm) < 1e-9 && sim.headTargetRad === 0);
  assert.equal(sim.setHeadAngle(v.head_angle_max_deg), true);
  for (let guard = 0; sim.actionRunning && guard < 5000; guard++) sim.stepAction();
  assert.equal(sim.lastAction.result, "completed");
  assert.ok(Math.abs((sim.headAngleRad() * 180) / Math.PI - v.head_angle_max_deg) <= v.head_angle_tol_deg, "head at its top");
  assert.equal(sim.setLiftHeight(v.lift_height_max_mm), true);
  for (let guard = 0; sim.actionRunning && guard < 5000; guard++) sim.stepAction();
  assert.equal(sim.lastAction.result, "completed");
  assert.ok(sim.liftHeightMm() > v.lift_height_max_mm - 2, `lift near its top: ${sim.liftHeightMm()}`);
  assert.ok(Math.abs(sim.liftTargetMm - v.lift_height_max_mm) < 1e-6);
  sim.dispose();
});

test("lift duration applies to one action", async () => {
  const sim = await makeSim(CONTROL_DEFAULT);
  for (const [height, duration, lo, hi] of [[92, 1.4, 1.2, 2], [32, 0, 0, 1]]) {
    assert.ok(sim.setLiftHeight(height, null, duration));
    let ticks = 0;
    while (sim.actionRunning && ticks * sim.dt < 3) {
      sim.stepAction();
      ticks++;
    }
    assert.equal(sim.lastAction.result, "completed");
    assert.ok(ticks * sim.dt > lo && ticks * sim.dt < hi);
    sim.tick(Math.round(0.5 / sim.dt));
  }
  sim.dispose();
});

test("lift height and angle convert both ways, and rounding is Python's", () => {
  const v = model.values;
  for (const height of [v.lift_height_min_mm, 50, 70, v.lift_height_max_mm]) {
    assert.ok(Math.abs(liftHeightOfAngle(v, liftAngleOfHeight(v, height)) - height) < 1e-9, `height ${height}`);
  }
  assert.ok(Math.abs(liftAngleOfHeight(v, v.lift_height_min_mm) - model.derived.lift_angle_min) < 1e-12);
  assert.deepEqual([0.5, 1.5, 2.5, -0.5, -1.5, 2.4999, 2.5001].map(pyRound), [0, 2, 2, 0, -2, 2, 3]);
});

test("42 mm contact trial keeps both stock axle pairs in place", async () => {
  const runtime = await mujoco();
  const trial = readText("public/robot/vector-contact42.xml");
  assert.equal(sha256(trial), model.source.trial_xml_sha256);
  const compiled = runtime.MjModel.from_xml_string(trial);
  try {
    const x = (name) => {
      const id = runtime.mj_name2id(compiled, runtime.mjtObj.mjOBJ_BODY.value, name);
      assert.ok(id >= 0, name);
      return 1000 * compiled.body_pos[3 * id];
    };
    for (const side of ["l", "r"]) {
      assert.equal(x("wheel_f" + side), 0);
      assert.equal(x("wheel_r" + side), -model.values.wheel_base_mm);
      assert.equal(x("patch_f" + side), -3);
      assert.equal(x("patch_r" + side), -45);
    }
  } finally {
    compiled.delete();
  }
});

for (const scene of ["vector-free", "vector-contact42-free"]) {
  test(`${scene} detects, pushes and resets the free cube`, async () => {
    const runtime = await mujoco();
    const source = readText(`public/robot/${scene}.xml`);
    assert.equal(sha256(source), model.source[scene + "_sha256"]);
    const sim = new VectorSim(runtime, { xml: source, values: model.values, derived: model.derived });
    try {
      const id = runtime.mj_name2id(sim.model, runtime.mjtObj.mjOBJ_BODY.value, "cube");
      assert.ok(id > 0);
      const x = () => sim.data.xpos[3 * id];
      const start = x();
      assert.ok(sim.state.prox_found_object);
      assert.ok(sim.state.prox_distance_mm > 45 && sim.state.prox_distance_mm < 90);
      assert.ok(sim.setWheelMotors(40, 40));
      sim.tick(Math.round(2.3 / sim.dt));
      assert.ok(x() > start + 0.008);
      sim.reset();
      assert.ok(Math.abs(x() - start) < 1e-6);
    } finally {
      sim.dispose();
    }
  });
}
