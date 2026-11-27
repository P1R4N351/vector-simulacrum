// Replay each signed shaft-speed report as a target for the standard wheel controller.
import {readFileSync} from "node:fs";
import {mujoco, readJson, readText} from "../../web/test/helpers.js";
import {VectorSim} from "../../web/src/sim/robot.js";
import {rearEnvelope} from "../unit3-cube-range/rear_envelope.mjs";

const source = new URL("../hw-rec-20261119/unit2-native-replay-input.csv", import.meta.url);
const [header, ...lines] = readFileSync(source, "utf8").trim().split(/\r?\n/);
const columns = header.split(",");
const turns = new Map();
for (const line of lines) {
  const [name, ...values] = line.split(",");
  const sample = Object.fromEntries(values.map((value, i) => [columns[i + 1], Number(value)]));
  if (!turns.has(name)) turns.set(name, []);
  turns.get(name).push(sample);
}
const mj = await mujoco();
const model = readJson("public/robot/vector2/model.json");
const scene = readText("public/robot/vector2/vector-free-dock.xml");
console.log("turn,variant,ms,gyro_dps,left_mmps,right_mmps");
for (const [name, samples] of turns) {
  for (const variant of ["canonical", "driven_rear_envelope", "canonical_no_queue", "driven_rear_envelope_no_queue"]) {
    const sim = new VectorSim(mj, {xml: rearEnvelope(scene, variant.replace("_no_queue", "")), values: model.values,
      derived: model.derived, startXyMm: [0, 140], headingDeg: 0,
      headDeg: samples[0].head_rad * 180 / Math.PI, liftMm: 32.7, noiseSeed: 1});
    try {
      sim.controlLevel = "override";
      if (variant.endsWith("_no_queue")) sim._wheelDelay = 0;
      let k = 0;
      for (let tick = 0; ; tick++) {
        const ms = samples[0].ms_from_last_edge + tick * sim.dt * 1000;
        if (ms > samples.at(-1).ms_from_last_edge) break;
        while (k+1 < samples.length && samples[k+1].ms_from_last_edge <= ms) k++;
        sim.setWheelMotors(samples[k].left_mmps, samples[k].right_mmps);
        const state = sim.reportedNow();
        console.log([name,variant,ms,state.gyro_z_dps,state.wl,state.wr].join(","));
        sim.tick(1);
      }
    } finally { sim.dispose(); }
  }
}
