import { readFileSync, writeFileSync } from "node:fs";
import { loadDockWakeTracks } from "../../../web/src/game/wake_clip.js";
import { CONTROL_OVERRIDE, VectorSim } from "../../../web/src/sim/robot.js";
import { mujoco, readJson, readText } from "../../../web/test/helpers.js";

const { head, lift } = await loadDockWakeTracks();
const mj = await mujoco();
const { values, derived } = readJson("public/robot/model.json");
const xml = readText("public/robot/vector-contact42-free-dock.xml");
const trials = [
  ...["pilot","session-1","session-2"].map(n => "../../docked-head-20261126/grants/"+n),
  ...["calm-cross-1","calm-cross-2"].map(n => "../blind-grants/"+n),
  ...[1,2,3].map(i => "../wake-raise/wake-raise-"+i),
];
const output = ["run,s,real_lift_mm,sim_lift_mm,real_left_mmps,sim_left_mmps,real_right_mmps,sim_right_mmps,real_contact,sim_contact,sim_head_deg,sim_gyro_y"];
for (const path of trials) {
  const rows = readFileSync(new URL(path+".csv",import.meta.url),"utf8").trim().split(/\r?\n/).slice(1).map(s=>s.split(","));
  const grant = Number(rows.find(r=>r[1]==="granted")[0]);
  const first = rows.filter(r=>r[1]==="state" && Number(r[0])<grant).at(-1);
  const physical = rows.filter(r=>r[1]==="state" && Number(r[0])>=grant && Number(r[0])<grant+8);
  const command = rows.find(r=>r[1]==="raise");
  const cue = command ? Number(command[0])-grant : Infinity;
  const sim = new VectorSim(mj,{values,derived,xml,controlLevel:CONTROL_OVERRIDE,startDocked:true,noiseSeed:42,
    headDeg:Number(first[2]),liftMm:Number(first[4]),calmMode:Boolean(Number(first[7])&1024),
    wakeHeadTrack:head,wakeLiftFrame:lift});
  try {
    if (!sim.requestControl()) throw Error("SDK grant refused");
    let issued=false;
    for (const row of physical) {
      const t = Number(row[0])-grant;
      while (sim.tickCount < Math.floor(t/sim.dt)) {
        if (!issued && sim.tickCount*sim.dt >= cue) {
          if (!sim.setHeadAngle(22)) throw Error("SDK raise refused");
          issued=true;
        }
        sim.tick();
      }
      const s=sim.state;
      output.push([path.split("/").at(-1),t,row[4],s.lift_height_mm,row[5],s.left_wheel_speed_mmps,
        row[6],s.right_wheel_speed_mmps,+Boolean(Number(row[7])&4096),+sim.chargerContact,s.head_angle_rad*180/Math.PI,s.gyro_radps[1]].join(","));
    }
  } finally { sim.dispose(); }
}
writeFileSync(new URL("wake-joints.csv",import.meta.url),output.join("\n")+"\n");
