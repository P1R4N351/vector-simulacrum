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
