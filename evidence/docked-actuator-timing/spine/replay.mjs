import {readFileSync,writeFileSync} from "node:fs";
import {CONTROL_OVERRIDE,VectorSim} from "../../../web/src/sim/robot.js";
import {makeSyntheticSysconFlash,mujoco,readJson,readText} from "../../../web/test/helpers.js";

const rows=["unit,trial,t,physical_head_deg,physical_gyro_y,sim_head_deg,sim_gyro_y,legacy_counts,native_counts"];
for (const [unit,file] of [[1,"repeat/unit1.jsonl"],[2,"unit2.jsonl"],[3,"repeat/unit3.jsonl"]]) {
  const events=readFileSync(new URL("../"+file,import.meta.url),"utf8").trim().split("\n").map(s=>JSON.parse(s));
  const states=events.filter(e=>e.k==="s");
  const prefix=unit===3?"public/robot/vector2/":"public/robot/";
  const {values,derived}=readJson(prefix+"model.json");
  let trial=0;
  for (const cue of events.filter(e=>e.k==="c" && e.name==="head")) {
    const first=states.filter(s=>s.t<=cue.t).at(-1);
    if (!first || Math.abs(first.head*180/Math.PI-cue.deg)<55) continue;
    const sample=states.filter(s=>s.t>=cue.t && s.t<cue.t+1.1);
    if (!sample.some(s=>Math.abs(s.head*180/Math.PI-cue.deg)<4)) continue;
    const sim=new VectorSim(await mujoco(),{values,derived,xml:readText(prefix+"vector-contact42-free-dock.xml"),startDocked:true,controlLevel:CONTROL_OVERRIDE,headDeg:first.head*180/Math.PI,liftMm:first.lift,noiseSeed:42});
    try {
      sim.attachFirmware({sysconBytes:makeSyntheticSysconFlash()});
      sim.tick(60);
      sim.setHeadAngle(cue.deg);
      let ticks=0;
      for (const s of sample) {
        const t=s.t-cue.t;
        const target=Math.floor(t/sim.dt);
        sim.tick(target-ticks); ticks=target;
        const frame=sim.lastSpineFrame;
        const count=new DataView(frame.buffer,frame.byteOffset,frame.byteLength).getInt32(52,true);
        rows.push([unit,trial,t,s.head*180/Math.PI,s.gyro[1],sim.state.head_angle_rad*180/Math.PI,sim.state.gyro_radps[1],Math.round(sim.state.head_angle_rad*180/Math.PI),count].join(","));
      }
    } finally {sim.dispose();}
    trial++;
  }
}
writeFileSync(new URL("replay.csv",import.meta.url),rows.join("\n")+"\n");
