// Input is printed by score.py. Replay received wheel speeds as motor setpoints.
import { readFileSync } from "node:fs";
import { VectorSim, CONTROL_OVERRIDE } from "../../web/src/sim/robot.js";
import { mujoco, readJson, readText } from "../../web/test/helpers.js";
const {trials, outwardMm} = JSON.parse(readFileSync(0, "utf8"));
const mj = await mujoco();
const profile = readJson("public/robot/vector2/model.json");
const xml = readText("public/robot/vector2/vector-free-shelf.xml");
class ShelfReplay extends VectorSim {
  _place(xy, yaw) {
    super._place(xy, yaw);
    // Move from the nominal seat toward the lip before settling on the ramp.
    const d = this.data.xmat.subarray(9 * this._dockBody);
    this.data.qpos[0] -= 0.001 * outwardMm * d[0];
    this.data.qpos[1] -= 0.001 * outwardMm * d[3];
    this.mujoco.mj_forward(this.model, this.data);
  }
}
const out = {};
for (const [trial, samples] of Object.entries(trials)) {
  const sim = new ShelfReplay(mj, {xml, ...profile, controlLevel: CONTROL_OVERRIDE, startDocked: true,
    headDeg: samples[0][3] * 180 / Math.PI, liftMm: samples[0][4]});
  const start = sim._treadTravelMm();
  const initialX = sim.data.xpos[3 * sim._chassisBody];
  let j = 0, lastHead = samples[0][3], lastLift = samples[0][4];
  const trace = [];
  for (let i = 0; i * sim.dt <= samples.at(-1)[0]; i++) {
    const t = i * sim.dt;
    while (j + 1 < samples.length && samples[j + 1][0] <= t) j++;
    const [, l, r, head, lift] = samples[j];
    if (head !== lastHead) sim.commandHeadAngle(head * 180 / Math.PI);
    if (lift !== lastLift) sim.commandLiftHeight(lift);
    lastHead = head;
    lastLift = lift;
    sim.setWheelMotors(l, r);
    sim.tick(1);
    const travel = sim._treadTravelMm().reduce((sum, x, k) => sum + (x - start[k]) / 2, 0);
    trace.push([t + sim.dt, travel, Number(sim.chargerContact), 1000 * (sim.data.xpos[3 * sim._chassisBody] - initialX), sim._pitch()]);
  }
  out[trial] = trace;
  sim.dispose();
}
console.log(JSON.stringify(out));
