import { readFileSync } from "node:fs";
import { mujoco, readJson } from "../../web/test/helpers.js";
import { VectorSim } from "../../web/src/sim/robot.js";

const mj = await mujoco();
const {values, derived, shelf_outline_mm} = readJson("public/robot/vector2/model.json");
const runs = JSON.parse(readFileSync(0, "utf8"));
const scene = readFileSync(process.argv[2], "utf8");
const clip = JSON.parse(readFileSync(process.argv[3], "utf8")).straight_03;
const heads = clip.filter(f => f.Name === "HeadAngleKeyFrame").sort((a,b) => a.triggerTime_ms-b.triggerTime_ms);
const lifts = clip.filter(f => f.Name === "LiftHeightKeyFrame").sort((a,b) => a.triggerTime_ms-b.triggerTime_ms);
const shaft = scene.replace(/<general name="(left|right)_wheel" joint="([^"]+)" gainprm="([^"]+)"[^>]*\/>/g,
  (_, side, joint, cap) => `<velocity name="${side}_wheel" joint="${joint}" kv="0.8" forcelimited="true" forcerange="-${cap} ${cap}"/>`);
if (!shaft.includes('<velocity name="right_wheel"')) throw Error("Shaft actuator missing");

console.log("trial,mode,t,gyro,left,right,on,x_mm,y_mm");
const modes = ["command", "command-linear", "command-immediate-linear", "hold", "linear"];
const selected = process.argv[4] ? [process.argv[4]] : modes;
if (!selected.every(m => modes.includes(m))) throw Error("Unknown replay mode");
for (const [trial, rows] of Object.entries(runs)) {
  for (const mode of selected) {
    const sim = new VectorSim(mj, {values, derived, xml:mode.startsWith("command")?scene:shaft,
      shelfOutline:shelf_outline_mm, startDocked:true, headDeg:rows[0][3]*180/Math.PI,
      liftMm:rows[0][4], noiseSeed:1});
    try {
      if (mode==="command-immediate-linear") sim._wheelDelay = 0;
      let idx=0, head=0, lift=0, left=0, right=0;
      if (!mode.startsWith("command")) sim._driveTreads = () => {
        sim.data.ctrl[sim._act.left] = left / values.wheel_front_radius_mm;
        sim.data.ctrl[sim._act.right] = right / values.wheel_front_radius_mm;
      };
      for (let tick=0; tick*sim.dt<=rows.at(-1)[0]+sim.dt; tick++) {
        const t=tick*sim.dt, clipMs=(t-.4)*1000;
        while (head<heads.length && heads[head].triggerTime_ms<=clipMs) sim.commandHeadAngle(heads[head++].angle_deg);
        while (lift<lifts.length && lifts[lift].triggerTime_ms<=clipMs) {
          const f=lifts[lift++]; sim.commandLiftHeight(f.height_mm, f.durationTime_ms/1000);
        }
        while (idx+1<rows.length && rows[idx+1][0]<=t) idx++;
        const a=rows[idx], b=rows[idx+1];
        const fraction=mode.endsWith("linear") && b ? Math.max(0,(t-a[0])/(b[0]-a[0])) : 0;
        left=a[1]+fraction*((b?.[1]??a[1])-a[1]);
        right=a[2]+fraction*((b?.[2]??a[2])-a[2]);
        sim.setWheelMotors(left,right);
        const s=sim.reportedNow(), [x,y]=sim.poseMm();
        console.log([trial,mode,t,s.gyro_z_dps,s.wl,s.wr,+sim.chargerContact,x,y].join(","));
        sim.tick(1);
      }
    } finally { sim.dispose(); }
  }
}
