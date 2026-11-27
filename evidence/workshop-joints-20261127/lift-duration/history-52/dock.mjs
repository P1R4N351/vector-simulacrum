import {readFileSync,writeFileSync} from "node:fs";
import {VectorSim,CONTROL_OVERRIDE} from "../../../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../../../web/test/helpers.js";

const raw=readFileSync(new URL("recording.csv",import.meta.url),"utf8").trim().split(/\r?\n/).slice(1).map(r=>r.split(","));
const states=raw.filter(r=>r[1]==="state");
const commands=raw.filter(r=>r[1]==="command");
const {values,derived}=readJson("public/robot/model.json");
const engine=await mujoco();
const mode=process.argv[2];
if(!["idle","sequence"].includes(mode))throw Error("Choose idle or sequence");
const first=states[0], origin=Number(first[0]);
const sim=new VectorSim(engine,{values,derived,xml:readText("public/robot/vector-contact42-free-dock.xml"),
  startDocked:true,controlLevel:CONTROL_OVERRIDE,headDeg:Number(first[5]),liftMm:Number(first[4]),noiseSeed:42});
try {
  let next=0;
  const out=["s,x_mm,y_mm,z_mm,yaw_rad,on_platform,on_dock"];
  for(const row of states){
    const receipt=Number(row[0]);
    while(sim.tickCount<Math.floor((receipt-origin)/sim.dt)){
      if(mode==="sequence" && next<commands.length && Number(commands[next][0])<=origin+sim.tickCount*sim.dt){
        const c=commands[next++];
        if(!sim.setLiftHeight(Number(c[2]),null,Number(c[3])))throw Error("Rejected lift move");
      }
      sim.tick();
    }
    out.push([receipt,...sim.poseMm(),sim._yaw(),+sim._onDockPlatform(),+sim.chargerContact].join(","));
  }
  writeFileSync(new URL("dock-"+mode+".csv",import.meta.url),out.join("\n")+"\n");
} finally {sim.dispose();}
