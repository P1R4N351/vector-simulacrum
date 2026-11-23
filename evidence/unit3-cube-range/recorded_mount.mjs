import { pinOffsets } from "./pin_offsets.mjs";
// Replay measured wheel-speed profiles from dock-relative SDK starting poses.
import { readFileSync } from "node:fs";
import { VectorSim } from "../../web/src/sim/robot.js";
import { mujoco, readJson, readText } from "../../web/test/helpers.js";
import { rearEnvelope } from "./rear_envelope.mjs";

const trials = JSON.parse(readFileSync(0, "utf8"));
const mj = await mujoco();
const model = readJson("public/robot/vector2/model.json");
const dock = readText("public/robot/vector2/vector-free-dock.xml");
const tracePitch = process.env.VECTOR_TRACE_DOCK_PITCH === "1";
const traceShafts = process.env.VECTOR_TRACE_SHAFTS === "1";
const tracePins = process.env.VECTOR_TRACE_DOCK_PINS === "1";
console.log("session,variant,t_s,out_mm,lat_mm,yaw_deg,left_mmps,right_mmps,on" + (tracePins ? ",lx,ly,lz,rx,ry,rz" : "") + (traceShafts ? ",shaft_l,shaft_r" : "") + (tracePitch ? ",pitch_deg" : ""));
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
      let shaftTargets;
      if (trial.shaft_replay) {
        // Supplied diagnostic XML uses velocity actuators with controls in rad/s.
        if (!extraModel) throw new Error("shaft replay requires a supplied velocity-actuator model");
        sim._driveTreads = () => {
          sim.data.ctrl[sim._act.left] = shaftTargets[0] / sim.v.wheel_front_radius_mm;
          sim.data.ctrl[sim._act.right] = shaftTargets[1] / sim.v.wheel_front_radius_mm;
        };
      }
      for (let i = 0; i * sim.dt <= (trial.duration_s ?? (-samples[0][0] + 0.5)); i++) {
        const t = samples[0][0] + i * sim.dt;
        while (index + 1 < samples.length && samples[index + 1][0] <= t) index++;
        let [, left, right] = samples[index];
        if (trial.interpolate && index + 1 < samples.length) {
          const next = samples[index+1];
          const mix = (t - samples[index][0]) / (next[0] - samples[index][0]);
          left += mix * (next[1] - left);
          right += mix * (next[2] - right);
        }
        shaftTargets = [left, right];
        if (variant === "driven_mean_only") sim.setWheelMotors((left + right) / 2, (left + right) / 2);
        else sim.setWheelMotors(left, right);
        const [x, y] = sim.poseMm();
        const { wl, wr } = sim.reportedNow();
        console.log([trial.name, variant, t, x + 120, y, sim._yaw() * 180 / Math.PI,
          wl, wr, +sim.chargerContact, ...(tracePins ? pinOffsets(sim).flat() : []), ...(traceShafts ? sim._wheelSpeeds() : []), ...(tracePitch ? [sim._pitch() * 180 / Math.PI] : [])].join(","));
        sim.tick(1);
      }
    } finally { sim.dispose(); }
  }
}
