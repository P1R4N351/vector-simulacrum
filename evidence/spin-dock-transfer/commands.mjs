// Replay timed SDK commands through the web engine's normal motor entry point.
import {readFileSync} from "node:fs";
import {mujoco, readJson, readText} from "../../web/test/helpers.js";
import {VectorSim} from "../../web/src/sim/robot.js";
const mj = await mujoco();
const model = readJson("public/robot/vector2/model.json");
const xml = readText("public/robot/vector2/vector-free-dock.xml");
console.log("spin,t,wl,wr,gyro");
for (const trial of JSON.parse(readFileSync(0, "utf8"))) {
  const sim = new VectorSim(mj, {xml, values:{...model.values, battery_rest_voltage_v:trial.volts ?? model.values.battery_rest_voltage_v}, derived:model.derived,
    startXyMm:[0,140], headingDeg:0, headDeg:trial.head, liftMm:trial.lift, noiseSeed:1});
  try {
    sim.controlLevel = "override";
    let started = false, stopped = false;
    for (let i=0; i*sim.dt <= trial.duration+1.5; i++) {
      const t = i*sim.dt-0.6;
      if (!started && t >= 0) {
        sim.setWheelMotors(trial.left,trial.right);
        started = true;
      }
      if (!stopped && t >= trial.duration) {
        sim.setWheelMotors(0,0);
        stopped = true;
      }
      const state = sim.reportedNow();
      console.log([trial.spin,t,state.wl,state.wr,state.gyro_z_dps].join(","));
      sim.tick(1);
    }
  } finally { sim.dispose(); }
}
