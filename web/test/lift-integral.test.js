// The lift's integral term in the browser's robot, as simulacrum/robot.py and tests/test_lift_integral.py:
// the firmware's angleErrorSum_ (LIFTC:832-892) moves the lift servo's target by Ki / Kp x the sum.
import assert from "node:assert/strict";
import { test } from "node:test";
import { CONTROL_DEFAULT, VectorSim, liftErrorSumStep, liftIntegralOffsetRad } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "./helpers.js";

const model = readJson("public/robot/model.json");
const v = model.values;
const DEG = Math.PI / 180;
const tol = v.lift_angle_tol_deg * DEG;
const cap = v.lift_error_sum_max;
const [kp, ki] = v.lift_power_gains;

test("the offset is Ki / Kp of the sum", () => {
  assert.ok(Math.abs(liftIntegralOffsetRad(v, 3) - (ki / kp) * 3) < 1e-12);
  assert.ok(Math.abs(liftIntegralOffsetRad(v, -3) + (ki / kp) * 3) < 1e-12);
});

test("off target, or while the target moves, the error goes on the sum, clipped both ways", () => {
  assert.ok(Math.abs(liftErrorSumStep(v, 0.3, 2 * tol, true) - (0.3 + 2 * tol)) < 1e-12);
  assert.ok(Math.abs(liftErrorSumStep(v, 0.3, -2 * tol, true) - (0.3 - 2 * tol)) < 1e-12);
  assert.ok(Math.abs(liftErrorSumStep(v, 0.3, 0.5 * tol, false) - (0.3 + 0.5 * tol)) < 1e-12);
  assert.equal(liftErrorSumStep(v, cap - 0.01, 1, true), cap);
  assert.equal(liftErrorSumStep(v, -cap + 0.01, -1, true), -cap);
});

test("holding its target the lift bleeds the sum only while its power is above the in-position limit", () => {
  const small = (0.5 * v.lift_power_in_position_max) / ki;
  assert.equal(liftErrorSumStep(v, small, 0, true), small);
  assert.equal(liftErrorSumStep(v, -small, 0, true), -small);
  const large = (2 * v.lift_power_in_position_max) / ki;
  assert.ok(Math.abs(liftErrorSumStep(v, large, 0, true) - (large - v.lift_error_sum_decay)) < 1e-12);
  assert.ok(Math.abs(liftErrorSumStep(v, -large, 0, true) - (-large + v.lift_error_sum_decay)) < 1e-12);
});

test("the exported lift servo may aim past either end by the largest offset; the joint may not", () => {
  const reach = liftIntegralOffsetRad(v, cap);
  for (const scene of ["vector", "vector-free-dock"]) {
    const xml = readText(`public/robot/${scene}.xml`);
    const servo = xml.match(/<position name="lift"[^>]*ctrlrange="([^"]+)"/);
    const joint = xml.match(/<joint name="lift_joint"[^>]*range="([^"]+)"/);
    assert.ok(servo && joint, `${scene}: lift servo or joint missing`);
    const [lo, hi] = servo[1].split(" ").map(Number);
    const [jlo, jhi] = joint[1].split(" ").map(Number);
    assert.ok(Math.abs(lo - (model.derived.lift_angle_min - reach)) < 1e-5, `${scene}: ctrlrange low ${lo}`);
    assert.ok(Math.abs(hi - (model.derived.lift_angle_max + reach)) < 1e-5, `${scene}: ctrlrange high ${hi}`);
    assert.ok(Math.abs(jlo - model.derived.lift_angle_min) < 1e-5 && Math.abs(jhi - model.derived.lift_angle_max) < 1e-5);
  }
});

test("a new target reaching an in-position lift clears the sum; one reaching a moving lift keeps it", async () => {
  const mj = await mujoco();
  const sim = new VectorSim(mj, {
    xml: readText("public/robot/vector.xml"), values: v, derived: model.derived, controlLevel: CONTROL_DEFAULT,
    noiseSeed: 1, liftMm: v.lift_height_max_mm,
  });
  try {
    sim.tick(Math.round(0.5 / sim.dt));
    assert.ok(sim._liftInPos, "the lift must start in position");
    sim._liftErrorSum = 2;
    assert.ok(sim.commandLiftHeight(v.lift_height_max_mm));
    assert.equal(sim._liftErrorSum, 2, "the same target again changes nothing");
    assert.ok(sim.commandLiftHeight(v.lift_height_min_mm));
    assert.equal(sim._liftErrorSum, 0);
    sim._liftErrorSum = 2;
    assert.ok(sim.commandLiftHeight(60));
    assert.equal(sim._liftErrorSum, 2);
  } finally {
    sim.dispose();
  }
});
