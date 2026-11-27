// Replay each signed shaft-speed report as a target for the standard wheel controller.
import {readFileSync,writeFileSync} from "node:fs";
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
const solverRows = ["spin,ms,raw_gx,raw_gy,raw_gz,head_rad"];
const axesTest = process.argv.includes("--axes");
const travelTest = process.argv.includes("--travel");
const variants = axesTest ? ["canonical", "shaft_count_position"] : travelTest ? ["canonical", "shaft_reported", "shaft_counts", "shaft_count_position"] :
  ["canonical", "driven_rear_envelope", "canonical_no_queue", "driven_rear_envelope_no_queue"];
// Factory encoder conversion shared with replay_native_stops.py.
const countMm = 0.96 * 29 * 0.25 * Math.PI / 172.3;
const shaftScene = scene.replace(/<general name="(left|right)_wheel" joint="([^"]+)" gainprm="([^"]+)"[^>]*\/>/g,
  (_, side, joint, cap) => `<velocity name="${side}_wheel" joint="${joint}" kv="0.8" forcelimited="true" forcerange="-${cap} ${cap}"/>`);
if (travelTest && !shaftScene.includes('<velocity name="right_wheel"')) throw Error("Missing shaft actuator");
console.log("turn,variant,ms,gyro_dps,left_mmps,right_mmps" + (travelTest ? ",shaft_left_mm,shaft_right_mm" : "") + (axesTest ? ",gx,gy,head_deg,raw_gx,raw_gy,raw_gz" : ""));
for (const [name, samples] of turns) {
  for (const variant of variants) {
    const shaft = variant.startsWith("shaft_");
    const sim = new VectorSim(mj, {xml: shaft ? shaftScene : rearEnvelope(scene, variant.replace("_no_queue", "")), values: model.values,
      derived: model.derived, startXyMm: [0, 140], headingDeg: 0,
      headDeg: samples[0].head_rad * 180 / Math.PI, liftMm: (travelTest || axesTest) ? 30.64501190185547 : 32.7, noiseSeed: 1});
    try {
      sim.controlLevel = "override";
      if (variant.endsWith("_no_queue")) sim._wheelDelay = 0;
      if (axesTest && variant === "canonical") {
        let n = 0;
        const update = sim._imuFilter.update.bind(sim._imuFilter);
        sim._imuFilter.update = raw => {
          update(raw);
          const ms = samples[0].ms_from_last_edge + ++n * sim.model.opt.timestep * 1000;
          solverRows.push([name, ms, ...raw.slice(3).map(v => v*180/Math.PI), sim.headAngleRad()].join(","));
        };
      }
      let speeds;
      if (shaft) sim._driveTreads = () => {
        sim.data.ctrl[sim._act.left] = speeds[0] / sim.v.wheel_front_radius_mm;
        sim.data.ctrl[sim._act.right] = speeds[1] / sim.v.wheel_front_radius_mm;
      };
      let k = 0;
      for (let tick = 0; ; tick++) {
        const ms = samples[0].ms_from_last_edge + tick * sim.dt * 1000;
        if (ms > samples.at(-1).ms_from_last_edge) break;
        while (k+1 < samples.length && samples[k+1].ms_from_last_edge <= ms) k++;
        const current = samples[k];
        speeds = [current.left_mmps, current.right_mmps];
        if (variant.startsWith("shaft_count") && k + 1 < samples.length) {
          const next = samples[k+1];
          const scale = 1000 * countMm / (next.ms_from_last_edge - current.ms_from_last_edge);
          speeds = [(next.left_count-current.left_count)*scale, (current.right_count-next.right_count)*scale];
          if (variant === "shaft_count_position") {
            // Offline position tracking. Correct accumulated error over 50 ms on either side.
            const elapsed = (ms-current.ms_from_last_edge)/1000;
            const measured = [(current.left_count-samples[0].left_count)*countMm,
                              (samples[0].right_count-current.right_count)*countMm];
            const actual = sim._treadTravelMm();
            speeds = speeds.map((speed,i) => speed+(measured[i]+elapsed*speed-actual[i])/0.05);
          }
        }
        sim.setWheelMotors(...speeds);
        const state = sim.reportedNow();
        console.log([name,variant,ms,state.gyro_z_dps,state.wl,state.wr,...(travelTest ? sim._treadTravelMm() : []),
          ...(axesTest ? [...sim._imu()[1].slice(0,2).map(v=>v*180/Math.PI), state.head_deg,
            ...sim.data.sensordata.slice(sim._gyroAdr,sim._gyroAdr+3).map(v=>v*180/Math.PI)] : [])].join(","));
        sim.tick(1);
      }
    } finally { sim.dispose(); }
  }
}

if (axesTest) writeFileSync(new URL("native-solver-trace.csv",import.meta.url), solverRows.join("\n"));
