import { readFileSync, writeFileSync } from "node:fs";
import { CONTROL_OVERRIDE, VectorSim } from "../../web/src/sim/robot.js";
import { mujoco, readJson, readText } from "../../web/test/helpers.js";

const file = name => new URL(name, import.meta.url);
const rows = readFileSync(file("states.csv"), "utf8").trim().split(/\r?\n/).slice(1).map(row => row.split(","));
const states = rows.filter(row => row[1] === "state");
const trials = rows.filter(row => row[1] === "trial");
const { values, derived } = readJson("public/robot/model.json");
const xml = readText("public/robot/vector-contact42-free-dock.xml");
const module = await mujoco();
const release = process.argv.includes("--release");
const output = ["trial,from_deg,s,real_deg,sim_deg,real_gyro_y,sim_gyro_y,sim_initial_deg"];

for (const [i, trial] of trials.entries()) {
  const cue = Number(trial[0]);
  const initial = Number(states.filter(row => Number(row[0]) < cue).at(-1)[4]);
  const sim = new VectorSim(module, { values, derived, xml, controlLevel: CONTROL_OVERRIDE,
    startDocked: true, headDeg: initial, noiseSeed: 42 });
  try {
    sim.tick(Math.round(0.6 / sim.dt));
    const initialReport = sim.reportedNow().head_deg;
    const action = sim.setHeadAngle(Number(trial[3]));
    if (!action || (action.done && !action.ok)) throw Error("Head command rejected");
    let tick = 0;
    let released = false;
    for (const row of states) {
      const time = Number(row[0]) - cue;
      if (time < 0 || time >= 0.85) continue;
      const next = Math.floor(time / sim.dt);
      while (tick < next) {
        sim.tick(1);
        tick++;
        if (release && !released && sim._headInPos && Math.abs(sim._acceptedHeadTarget * 180 / Math.PI - Number(trial[3])) < 1e-6) {
          released = true;
          const id = sim._act.head;
          sim.model.actuator_gainprm[id * 10] = 0;
          sim.model.actuator_biasprm.fill(0, id * 10, id * 10 + 10);
        }
      }

      if (!sim.chargerContact) throw Error("Lost simulated dock");
      const state = sim.state;
      output.push([i+1,trial[2],time,row[4],state.head_angle_rad*180/Math.PI,row[5],state.gyro_radps[1],initialReport].join(","));
    }
  } finally { sim.dispose(); }
}
writeFileSync(file(release ? "released.csv" : "replayed.csv"),output.join("\n")+"\n");
