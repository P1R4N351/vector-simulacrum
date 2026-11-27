import assert from "node:assert/strict";
import {test} from "node:test";
import {VectorSim} from "../src/sim/robot.js";
import {mujoco,readJson,readText} from "./helpers.js";

test("calm SDK states hold both IMU channels until a supervisor update", async () => {
  const {values,derived}=readJson("public/robot/model.json");
  const sim=new VectorSim(await mujoco(),{values,derived,xml:readText("public/robot/vector-free.xml"),calmMode:true});
  const sample=()=>[...sim.state.accel_mmps2,...sim.state.gyro_radps];
  try {
    const initial=sample();
    sim.tick(48);
    assert.equal(sim.state.time_s,.24);
    assert.deepEqual(sample(),initial);
    sim.tick(12);
    assert.notDeepEqual(sample(),initial);
    const held=sample();
    sim.tick(12);
    assert.deepEqual(sample(),held);
    sim.calmMode=false;
    sim.tick(12);
    assert.notDeepEqual(sample(),held);
    sim.reset({calmMode:true});
    const reset=sample();
    sim.tick(12);
    assert.deepEqual(sample(),reset);
  } finally {sim.dispose();}
});
