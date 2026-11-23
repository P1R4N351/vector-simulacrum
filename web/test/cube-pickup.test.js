// The browser's robot picks up the light cube, as evidence/cube-pickup/trial.py does in Python..
//
// The robot starts square to the cube's near face with its lift at the bottom, drives in until the
// fork's hooks are in the cube's edge notches, raises the lift to the carry height and backs away.
// With the barbed hooks and the cube's lift pockets the cube comes up with the fork and stays on it;
// before them the fork only pushed the cube along the table.
import assert from "node:assert/strict";
import { test } from "node:test";
import { CONTROL_DEFAULT, VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "./helpers.js";

const MM = 0.001;
const GAP_MM = 15;
const PUSH_MM = 7;
const SPEED_MMPS = 20;
const BACK_MM = 40;
const RISE_MM = 40; // of the lift's 60 mm travel
const model = readJson("public/robot/model.json");

function bodyId(mj, sim, name) {
  const id = mj.mj_name2id(sim.model, mj.mjtObj.mjOBJ_BODY.value, name);
  assert.ok(id >= 0, `the model has no ${name}`);
  return id;
}

// World x (mm) of the frontmost point of any contact geom on the fork.
function forkFrontMm(mj, sim) {
  const { model: m, data: d } = sim;
  const fork = bodyId(mj, sim, "lift_fork");
  let front = -Infinity;
  for (let g = 0; g < m.ngeom; g++) {
    if (m.geom_bodyid[g] !== fork || m.geom_contype[g] === 0) continue;
    let reach = 0;
    for (let k = 0; k < 3; k++) reach += Math.abs(d.geom_xmat[9 * g + k]) * m.geom_size[3 * g + k];
    front = Math.max(front, d.geom_xpos[3 * g] + reach);
  }
  assert.ok(Number.isFinite(front), "the fork has no contact geoms");
  return front / MM;
}

function cubeZmm(mj, sim) {
  return sim.data.xpos[3 * bodyId(mj, sim, "cube") + 2] / MM;
}

function runAction(sim, started) {
  assert.ok(started, "the action was refused");
  for (let guard = 0; sim.actionRunning && guard < 20000; guard++) sim.stepAction();
  assert.ok(!sim.actionRunning, "the action did not finish");
  return sim.lastAction;
}

function makeSim(mj, xml, startXyMm) {
  return new VectorSim(mj, {
    xml, values: model.values, derived: model.derived, controlLevel: CONTROL_DEFAULT, noiseSeed: 1,
    startXyMm, liftMm: model.values.lift_height_min_mm,
  });
}

// As the keyboard does it (game.js makeDriver): treads held at the held-key speed from the default
// start pose until the fork is past the cube's face, released, then the lift key held.
test("vector-free-dock: driven in on held keys, the lift key picks the cube up", async () => {
  const mj = await mujoco();
  const xml = readText("public/robot/vector-free-dock.xml");
  const sim = makeSim(mj, xml, [0, 0]);
  try {
    const cube = bodyId(mj, sim, "cube");
    const rest = cubeZmm(mj, sim);
    const travel = sim.data.xpos[3 * cube] / MM - 0.5 * model.values.cube_outer_mm - forkFrontMm(mj, sim) + PUSH_MM;
    const x0 = sim.poseMm()[0];
    assert.ok(sim.setWheelMotors(100, 100));
    for (let i = 0; i < 2000 && sim.poseMm()[0] - x0 < travel; i++) sim.tick(1);
    assert.ok(sim.poseMm()[0] - x0 >= travel, "the robot never reached the cube");
    sim.setWheelMotors(0, 0);
    sim.tick(Math.round(0.4 / sim.dt));
    assert.ok(sim.commandLiftHeight(model.values.lift_height_max_mm));
    sim.tick(Math.round(1.5 / sim.dt));
    const held = cubeZmm(mj, sim) - rest;
    assert.ok(held > RISE_MM, `the cube rose ${held.toFixed(1)} mm`);
  } finally {
    sim.dispose();
  }
});

for (const scene of ["vector-free", "vector-free-dock"]) {
  test(`${scene}: the robot docks with the cube, lifts it and carries it away`, async () => {
    const mj = await mujoco();
    const xml = readText(`public/robot/${scene}.xml`);
    const probe = makeSim(mj, xml, [0, 0]);
    const face = probe.data.xpos[3 * bodyId(mj, probe, "cube")] / MM - 0.5 * model.values.cube_outer_mm;
    const startX = face - GAP_MM - (forkFrontMm(mj, probe) - probe.poseMm()[0]);
    probe.dispose();
    for (const offset of [-1, 0, 1]) {
      const sim = makeSim(mj, xml, [startX, offset]);
      try {
        const rest = cubeZmm(mj, sim);
        assert.equal(runAction(sim, sim.driveStraight(GAP_MM + PUSH_MM, SPEED_MMPS)).result, "completed");
        sim.tick(Math.round(0.3 / sim.dt));
        runAction(sim, sim.setLiftHeight(model.values.lift_height_max_mm));
        sim.tick(Math.round(1.5 / sim.dt));
        const held = cubeZmm(mj, sim) - rest;
        runAction(sim, sim.driveStraight(-BACK_MM, SPEED_MMPS));
        sim.tick(Math.round(0.5 / sim.dt));
        const carried = cubeZmm(mj, sim) - rest;
        assert.ok(held > RISE_MM, `offset ${offset} mm: the cube rose ${held.toFixed(1)} mm`);
        assert.ok(carried > RISE_MM, `offset ${offset} mm: after backing away the cube is ${carried.toFixed(1)} mm up`);
      } finally {
        sim.dispose();
      }
    }
  });
}
