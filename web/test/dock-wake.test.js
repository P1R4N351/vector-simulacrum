import assert from "node:assert/strict";
import test from "node:test";
import { VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "./helpers.js";

test("docked calm head track starts once on grant and honors stop and reset", async () => {
  const {values, derived} = readJson("public/robot/model.json");
  const frames = [[0, 8], [200, 18], [500, 28]].map(([triggerTime_ms,angle_deg]) =>
    ({triggerTime_ms,angle_deg,durationTime_ms:100}));
  const sim = new VectorSim(await mujoco(), {xml:readText("public/robot/vector-contact42-free-dock.xml"),
    values, derived, startDocked:true, calmMode:true, wakeHeadTrack:frames});
  try {
    assert.equal(sim.state.has_control, false);
    assert.equal(sim.commandHeadAngle(35), false);
    assert.equal(sim.requestControl(), true);
    sim.tick(Math.round(.15/sim.dt));
    assert.ok(Math.abs(sim.headCommandRad*180/Math.PI-8)<.1);
    sim.requestControl();
    sim.tick(Math.round(.2/sim.dt));
    assert.ok(Math.abs(sim.headCommandRad*180/Math.PI-18)<.1);
    assert.ok(sim.stopAllMotors());
    const stopped = sim.headCommandRad;
    sim.tick(Math.round(.6/sim.dt));
    assert.equal(sim.headCommandRad,stopped);
    sim.reset({calmMode:true,startDocked:true});
    assert.equal(sim.state.has_control,false);
    sim.requestControl();
    sim.commandHeadAngle(0);
    sim.tick(Math.round(.6/sim.dt));
    assert.equal(sim.headCommandRad,0);
  } finally { sim.dispose(); }
});
