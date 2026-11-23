import assert from "node:assert/strict";
import test from "node:test";
import { VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "./helpers.js";

test("docked range holds the acquisition while SDK lift flags continue, then resumes at body rate", async () => {
  const { values, derived } = readJson("public/robot/vector2/model.json");
  const sim = new VectorSim(await mujoco(), { values, derived, xml: readText("public/robot/vector2/vector-free.xml"),
    bodyRangeClock: true, chargerContact: true });
  try {
    const initial = sim.state.prox_distance_mm;
    sim.commandLiftHeight(45);
    sim.tick(150);
    assert.equal(sim.state.prox_is_lift_in_fov, true);
    assert.equal(sim.state.prox_distance_mm, initial);
    const changes = [];
    let sample = sim._rangeSample;
    for (let i = 150; i < 620; i++) {
      sim.tick(1);
      if (sample !== sim._rangeSample) changes.push(sim.tickCount);
      sample = sim._rangeSample;
    }
    assert.deepEqual(changes, [205, 410, 615]);
    assert.equal(sim.state.prox_distance_mm, 30);
    sim.chargerContact = false;
    sim.tick(6);
    assert.equal(sim._rangeSample, sample);
    sim.tick(2);
    assert.notEqual(sim._rangeSample, sample);
    sim.chargerContact = true;
    sim.setWheelMotors(-100, 100);
    const driven = [];
    for (let i = 0; i < 110; i++) {
      const before = sim._rangeSample;
      sim.tick(1);
      if (sim._rangeSample !== before) driven.push(sim.tickCount);
    }
    assert.ok(driven.length >= 8);
    assert.ok(driven.slice(-6).every((t, i, tail) => i === 0 || [6, 7].includes(t - tail[i-1])));
    sim.reset();
    assert.equal(sim.bodyRangeClock, true);
    assert.equal(sim._rangeBodyPhaseUs, 0);
  } finally {
    sim.dispose();
  }
});

test("shelf range acquisition stays slow under WireOS head and lift power until a tread is driven", async () => {
  const { values, derived, shelf_outline_mm } = readJson("public/robot/vector2/model.json");
  const sim = new VectorSim(await mujoco(), {
    values, derived, shelfOutline: shelf_outline_mm, startDocked: true,
    xml: readText("public/robot/vector2/vector-free-shelf.xml"),
  });
  try {
    assert.ok(sim.chargerContact);
    const start = sim.rawRangeSample().count;
    for (let i = 0; i < 204; i++) sim.tickWireBody([0, 0, 0.2, 0.2]);
    assert.equal(sim.rawRangeSample().count, start);
    sim.tickWireBody([0, 0, 0.2, 0.2]);
    assert.equal(sim.rawRangeSample().count, start + 1);
    for (let i = 0; i < 7; i++) sim.tickWireBody([0, 0.2, 0, 0]);
    assert.equal(sim.rawRangeSample().count, start + 2);
    sim.reset({ bodyRangeClock: false });
    assert.equal(sim.bodyRangeClock, false);
  } finally {
    sim.dispose();
  }
});
