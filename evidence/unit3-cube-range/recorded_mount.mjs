// Replay measured wheel-speed profiles from dock-relative SDK starting poses.
import { readFileSync } from "node:fs";
import { VectorSim } from "../../web/src/sim/robot.js";
import { mujoco, readJson, readText } from "../../web/test/helpers.js";
import { rearEnvelope } from "./rear_envelope.mjs";

const trials = JSON.parse(readFileSync(0, "utf8"));
const mj = await mujoco();
const model = readJson("public/robot/vector2/model.json");
const dock = readText("public/robot/vector2/vector-free-dock.xml");
console.log("session,variant,t_s,out_mm,lat_mm,yaw_deg,left_mmps,right_mmps,on");
const extraModel = process.argv[2] ? readFileSync(process.argv[2], "utf8") : null;
for (const trial of trials) {
  for (const variant of extraModel ? ["supplied"] : ["canonical", "driven_rear_envelope", "driven_mean_only"]) {
    const sim = new VectorSim(mj, {xml: extraModel ?? rearEnvelope(dock, variant === "canonical" ? variant : "driven_rear_envelope"), values: model.values,
      derived: model.derived, startXyMm: [trial.out - 120, trial.lat], headingDeg: trial.yaw,
      headDeg: trial.head, liftMm: trial.lift, noiseSeed: 1});
    try {
      sim.controlLevel = "override";
      const samples = trial.wheels;
      let index = 0;
      for (let i = 0; i * sim.dt <= -samples[0][0] + 0.5; i++) {
        const t = samples[0][0] + i * sim.dt;
        while (index + 1 < samples.length && samples[index + 1][0] <= t) index++;
        const [, left, right] = samples[index];
        if (variant === "driven_mean_only") sim.setWheelMotors((left + right) / 2, (left + right) / 2);
        else sim.setWheelMotors(left, right);
        const [x, y] = sim.poseMm();
        const { wl, wr } = sim.reportedNow();
        console.log([trial.name, variant, t, x + 120, y, sim._yaw() * 180 / Math.PI,
          wl, wr, +sim.chargerContact].join(","));
        sim.tick(1);
      }
    } finally { sim.dispose(); }
  }
}
