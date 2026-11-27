import {readFileSync,writeFileSync} from "node:fs";
import {CONTROL_OVERRIDE,VectorSim} from "../../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../../web/test/helpers.js";

const rows=readFileSync(new URL("active.csv",import.meta.url),"utf8").trim().split(/\r?\n/).slice(1).map(line=>line.split(","));
const states=rows.filter(r=>r[1]==="state"), commands=rows.filter(r=>r[1]!=="state");
const {values,derived}=readJson("public/robot/model.json");
const sim=new VectorSim(await mujoco(),{values,derived,xml:readText("public/robot/vector-contact42-free-dock.xml"),
  startDocked:true,controlLevel:CONTROL_OVERRIDE,noiseSeed:42,headDeg:+states[0][4],liftMm:+states[0][5]});
const output=["s,head_deg,lift_mm,gy,dock,pub_head,pub_lift,pub_gy"];
try {
  let next=0;
  for(let tick=0;;tick++){
    const time=+states[0][0]+tick*sim.dt;
    if(time>+commands.at(-1)[0]+1.4)break;
    if(next<commands.length && time>=+commands[next][0]){
      const [stamp,event,,target]=commands[next++];
      let action;
      if(event==="head")action=sim.setHeadAngle(+target);
      else{
        const firstHead=commands.slice(next).find(c=>c[1]==="head");
        const held=states.filter(r=>+r[0]>=+firstHead[0] && +r[0]<+firstHead[0]+.7).map(r=>+r[5]).sort((a,b)=>a-b);
        action=sim.setLiftHeight(held[Math.floor(held.length/2)]);
      }
      if(!action || (action.done && !action.ok))throw Error("Rejected "+event+" at "+stamp);
    }
    const s=sim.state;
    const now=sim.reportedNow();
    output.push([time,now.head_deg,now.lift_mm,sim._imu()[1][1],+sim.chargerContact,s.head_angle_rad*180/Math.PI,s.lift_height_mm,s.gyro_radps[1]].join(","));
    sim.tick(1);
  }
  writeFileSync(new URL("posture-replay.csv",import.meta.url),output.join("\n")+"\n");
}finally{sim.dispose();}
