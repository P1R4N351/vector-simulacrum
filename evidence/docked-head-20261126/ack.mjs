import {readFileSync,writeFileSync} from "node:fs";
import {VectorSim,CONTROL_OVERRIDE} from "../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";

const {values,derived}=readJson("public/robot/model.json");
const xml=readText("public/robot/vector-contact42-free-dock.xml");
const mj=await mujoco();
const output=["session,cue,target_deg,pre_deg,physical_s,physical_pos_s,sim_angle_s,sim_pos_s,sim_action_s"];
for(const name of ["unit2","holds/holds","holds/check"]){
  const rows=readFileSync(new URL(name+".csv",import.meta.url),"utf8").trim().split(/\r?\n/).slice(1).map(line=>line.split(","));
  for(const [i,cue] of rows.entries()){
    if(cue[1]!=="command")continue;
    const start=rows.slice(0,i).findLast(r=>r[1]==="state");
    const tail=rows.slice(i+1);
    const response=tail.find(r=>r[1]==="return"&&r[2]===cue[2]);
    if(!response || Number(response[4])!==0)continue;
    const target=Number(cue[3]);
    const sim=new VectorSim(mj,{values,derived,xml,controlLevel:CONTROL_OVERRIDE,startDocked:true,
      noiseSeed:42,headDeg:Number(start[4]),liftMm:Number(start[6])});
    try{
      if(!sim.setHeadAngle(target))throw Error("refused");
      let angle=null,pos=null,done=null;
      for(let j=0;j<400;j++){
        if(angle===null && Math.abs(sim.headAngleRad()*180/Math.PI-target)<=values.head_angle_tol_deg)angle=j*sim.dt;
        if(pos===null && sim._acceptedHeadTarget===sim._headTarget && sim._headInPos)pos=j*sim.dt;
        if(!sim.actionRunning && done===null)done=j*sim.dt;
        if(pos!==null && done!==null)break;
        sim.tick();
      }
      const end=tail.findIndex(r=>r[1]==="command");
      const active=tail.slice(0,end<0?tail.length:end).filter(r=>r[1]==="state");
      const flag=active.find(r=>Math.abs(Number(r[4])-target)<=values.head_angle_tol_deg && (Number(r[10])&512));
      output.push([name,cue[2],target,start[4],Number(response[0])-Number(cue[0]),flag?Number(flag[0])-Number(cue[0]):"",angle,pos,done].join(","));
    }finally{sim.dispose();}
  }
}
writeFileSync(new URL("ack-scores.csv",import.meta.url),output.join("\n")+"\n");
