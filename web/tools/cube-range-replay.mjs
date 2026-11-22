// Read robot-relative cube poses from browser_range.py and run the exported Vector 2 ToF.
import { readFileSync } from "node:fs";
import { VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "../test/helpers.js";

const { values, derived } = readJson("public/robot/vector2/model.json");
const xml = readText("public/robot/vector2/vector-free.xml");
const runtime = await mujoco();
const runs = JSON.parse(readFileSync(0, "utf8"));
const result = [];
for (const run of runs) {
  const sim = new VectorSim(runtime, { xml, values, derived, noiseSeed: run.trial });
  try {
    const joint = runtime.mj_name2id(sim.model, runtime.mjtObj.mjOBJ_JOINT.value, "cube_root");
    const q = sim.model.jnt_qposadr[joint];
    const positions = sim.data.qpos;
    const chassisHeight = positions[2];
    positions.set([0, 0, 0, 1, 0, 0, 0], 0);
    // Retain the settled height of the chassis origin above the shelf.
    positions[2] = chassisHeight;
    const rows = [];
    for (const [x, y, yaw, lift] of run.poses) {
      positions.set([x / 1000, y / 1000, .022, Math.cos(yaw/2), 0, 0, Math.sin(yaw/2)], q);
      runtime.mj_forward(sim.model, sim.data);
      const hits = sim._castTof();
      const reading = sim._proxSensor.read(hits, lift, 0);
      rows.push([reading.distance_mm, Number(reading.found_object)]);
    }
    result.push(rows);
  } finally {
    sim.dispose();
  }
}
console.log(JSON.stringify(result));
