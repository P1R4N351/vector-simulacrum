import { readFileSync, writeFileSync } from "node:fs";
import { VectorSim, CONTROL_OVERRIDE } from "../../../web/src/sim/robot.js";
import { mujoco, readJson, readText } from "../../../web/test/helpers.js";
import { load } from "../blind-grants/motor-driver.mjs";

const frames = JSON.parse(readFileSync(process.argv[2], "utf8"));
if (frames.length !== 19) throw Error("Expected 19 firmware head targets");
const mj = await mujoco();
const { values, derived } = readJson("public/robot/model.json");
const xml = readText("public/robot/vector-contact42-free-dock.xml");
const paths = [
  ...["pilot","session-1","session-2"].map(n => "../../docked-head-20261126/grants/"+n),
  ...["calm-cross-1","calm-cross-2","blind-grant-1","blind-grant-2"].map(n => "../blind-grants/"+n),
  ...[1,2,3].map(i => "../wake-raise/wake-raise-"+i),
];
const traces = ["run,mode,t_s,real_deg,real_gyro,sim_deg,sim_gyro"];
for (const path of paths) {
  const tr = load(path);
  for (const mode of ["no-clip","untimed","timed"]) {
    const sim = new VectorSim(mj, { values,derived,xml,controlLevel:CONTROL_OVERRIDE,startDocked:true,
      noiseSeed:42,headDeg:tr.start.head,liftMm:tr.start.lift });
    const timeline = [];
    try {
      let issued = false, waiting = false, next = 0;
      for (let i=0; i*sim.dt<=8.01; i++) {
        const t = i*sim.dt;
        if (!issued && tr.cue !== null && t >= tr.cue) {
          sim._headAccel = values.head_accel_default_radps2;
          if (!sim.setHeadAngle(22)) throw Error("SDK raise refused");
          issued = waiting = true;
        }
        if (waiting && !sim.actionRunning && sim.state.head_in_pos) waiting = false;
        while (next < frames.length && t >= frames[next].triggerTime_ms/1000) {
          const f = frames[next++];
          if (mode === "no-clip" || !(tr.start.flags & 1024) || waiting || sim.actionRunning) continue;
          if (!sim.commandHeadAngle(f.angle_deg)) throw Error("Animation refused");
          if (mode === "timed") {
            const span = Math.abs(sim._headTarget-sim.headCommandRad);
            const duration = f.durationTime_ms/1000;
            if (span > 0) {
              const peak = span/(.9*duration);
              sim._headSpeed = Math.min(values.head_speed_max_radps,peak);
              sim._headAccel = Math.min(10000,peak/(.1*duration));
            }
          }
        }
        const state = sim.state;
        if (!sim.chargerContact) throw Error("Lost platform");
        timeline.push([state.head_angle_rad*180/Math.PI,state.gyro_radps[1]]);
        sim.tick();
      }
      for (const r of tr.data.filter(r=>r.t>=0 && r.t<8))
        traces.push([path.split("/").at(-1),mode,r.t,r.head,r.gyro,...timeline[Math.floor(r.t/sim.dt)]].join(","));
    } finally { sim.dispose(); }
  }
}
writeFileSync(new URL("motor-traces.csv",import.meta.url),traces.join("\n")+"\n");
