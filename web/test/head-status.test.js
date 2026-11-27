import assert from "node:assert/strict";
import test from "node:test";
import { VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "./helpers.js";

test("head status clears on motion and latches through a settled repeat", async () => {
  const { values, derived } = readJson("public/robot/vector2/model.json");
  const sim = new VectorSim(await mujoco(), { values, derived, xml: readText("public/robot/vector2/vector-free.xml") });
  try {
    assert.equal(sim.state.head_in_pos, true);
    sim.commandHeadAngle(30);
    sim.tick(sim._servoDelay);
    assert.equal(sim._headInPos, true);
    sim.tick(1);
    assert.equal(sim._headInPos, false);
    sim.tick(sim._stateTicks);
    assert.equal(sim.state.head_in_pos, false);
    sim.tick(Math.round(1.5 / sim.dt));
    assert.equal(sim.state.head_in_pos, true);
    sim.commandHeadAngle(30);
    sim.tick(sim._stateTicks);
    assert.equal(sim.state.head_in_pos, true);
  } finally {
    sim.dispose();
  }
});

test("head action waits for its own target to settle, including a queued reversal", async () => {
  const { values, derived } = readJson("public/robot/model.json");
  const sim = new VectorSim(await mujoco(), { values, derived, xml: readText("public/robot/vector.xml") });
  try {
    sim.commandHeadAngle(25);
    sim.tick(1);
    assert.ok(sim.setHeadAngle(0));
    sim.tick(1);
    assert.ok(sim.actionRunning, "the initial in-position flag belongs to the old command");
    for (let i = 0; sim.actionRunning && i < 400; i++) sim.tick(1);
    assert.equal(sim.lastAction?.result, "completed");
    assert.ok(sim._headInPos);
    assert.ok(sim.setHeadAngle(22));
    for (let i = 0; i < 400 && Math.abs(sim.headAngleRad() * 180 / Math.PI - 22) > 2; i++) sim.tick(1);
    assert.ok(sim.actionRunning, "entering angle tolerance precedes motor settling");
    for (let i = 0; sim.actionRunning && i < 400; i++) sim.tick(1);
    assert.equal(sim.lastAction?.result, "completed");
    assert.ok(sim._headInPos);
  } finally {
    sim.dispose();
  }
});
