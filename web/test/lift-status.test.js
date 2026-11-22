import assert from "node:assert/strict";
import test from "node:test";
import { VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "./helpers.js";

test("lift-in-position clears on a raise and stays set on a settled repeat", async () => {
  const { values, derived } = readJson("public/robot/vector2/model.json");
  const sim = new VectorSim(await mujoco(), { values, derived, xml: readText("public/robot/vector2/vector-free.xml") });
  try {
    assert.equal(sim.state.lift_in_pos, true);
    sim.commandLiftHeight(62, 0.7);
    sim.tick(sim._stateTicks);
    assert.equal(sim.state.lift_in_pos, false);
    sim.tick(Math.round(1.5 / sim.dt));
    assert.equal(sim.state.lift_in_pos, true);
    sim.commandLiftHeight(62);
    sim.tick(sim._stateTicks);
    assert.equal(sim.state.lift_in_pos, true);
    sim.stopAllMotors();
    sim.tick(sim._stateTicks);
    assert.equal(sim.state.lift_in_pos, false);
    sim.tick(Math.round(0.5 / sim.dt));
    assert.equal(sim.state.lift_in_pos, true);
  } finally {
    sim.dispose();
  }
});
