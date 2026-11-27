import { readFileSync, writeFileSync } from "node:fs";
import { CONTROL_OVERRIDE, VectorSim } from "../../web/src/sim/robot.js";
import { mujoco, readJson, readText } from "../../web/test/helpers.js";

const rows = readFileSync(new URL(process.argv[2] ?? "unit2.csv", import.meta.url), "utf8").trim().split(/\r?\n/).slice(1).map(line => line.split(","));
const first = rows.find(row => row[1] === "state");
const commands = rows.filter(row => row[1] === "command");
const {values, derived} = readJson("public/robot/model.json");
const sim = new VectorSim(await mujoco(), {
  values, derived, xml: readText("public/robot/vector-contact42-free-dock.xml"),
  startDocked: true, controlLevel: CONTROL_OVERRIDE, noiseSeed: 42,
  headDeg: Number(first[4]), liftMm: Number(first[6]),
});
const output = ["t_s,head_deg,gyro_y_rad_s,charger"];
try {
  let cue = 0;
  for (let i = 0; ; i++) {
    const time = Number(first[0]) + i * sim.dt;
    if (time > Number(commands.at(-1)[0]) + 1) break;
    if (cue < commands.length && time >= Number(commands[cue][0])) {
      const action = sim.setHeadAngle(Number(commands[cue][3]));
      if (!action || action.done && !action.ok) throw Error("Head command refused at " + time);
      cue++;
    }
    const state = sim.state;
    output.push([time, state.head_angle_rad * 180 / Math.PI, state.gyro_radps[1], +sim.chargerContact].join(","));
    sim.tick(1);
  }
  writeFileSync(new URL(process.argv[3] ?? "canonical.csv", import.meta.url), output.join("\n") + "\n");
} finally { sim.dispose(); }
