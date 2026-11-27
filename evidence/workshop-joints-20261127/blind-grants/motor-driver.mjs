import {readFileSync} from "node:fs";
import {VectorSim,CONTROL_OVERRIDE} from "../../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../../web/test/helpers.js";

const {values,derived}=readJson("public/robot/model.json");
const xml=readText("public/robot/vector-contact42-free-dock.xml");
const mj=await mujoco();
export function load(path){
  const lines=readFileSync(new URL(path+".csv",import.meta.url),"utf8").trim().split(/\r?\n/).slice(1).map(line=>line.split(","));
  const grant=Number(lines.find(r=>r[1]==="granted")[0]);
  const data=lines.filter(r=>r[1]==="state").map(r=>({t:Number(r[0])-grant, head:Number(r[2]),gyro:Number(r[3]),lift:Number(r[4]),flags:Number(r[7])}));
  const cue=lines.find(r=>r[1]==="raise");
  const start=data.filter(r=>r.t<0).at(-1);
  return {data,start,cue:cue?Number(cue[0])-grant:null};
}
function displacement(tr,t){
  const i=tr.data.findIndex(r=>r.t>=t);
  if(i<1) return tr.data.at(-1).head-tr.start.head;
  const a=tr.data[i-1],b=tr.data[i];
  return a.head+(b.head-a.head)*(t-a.t)/(b.t-a.t)-tr.start.head;
}
export const train=["pilot","session-1"].map(name=>load("../../docked-head-20261126/grants/"+name));
export function run(tr,lead,gate,stream=true,handoff={}){
  const sim=new VectorSim(mj,{values,derived,xml,controlLevel:CONTROL_OVERRIDE,startDocked:true,noiseSeed:42,headDeg:tr.start.head,liftMm:tr.start.lift});
  try {
    const output=[];
    let issued=false;
    let finished=null;
    let resumed=false;
    const wake=gate && Boolean(tr.start.flags&1024);
    for(let i=0;i*sim.dt<=8.05;i++){
      const t=i*sim.dt;
      if(issued && finished===null && !sim.actionRunning){
        finished=t;
        handoff.actionEnd=t;
      }
      if(!issued && tr.cue!==null && t>=tr.cue){
        if(!sim.setHeadAngle(22))throw Error("raise refused");
        issued=true;
      } else if(wake && t<=5.16 && !sim.actionRunning &&
                (!issued || resumed || (t >= finished+(handoff.wait ?? 0) && (!handoff.inPosition || sim._headInPos)))){
        if(issued && !resumed){
          handoff.resume=t;
          resumed=true;
        }
        const d=train.reduce((s,tr)=>s+displacement(tr,Math.min(5.16,t+lead)),0)/train.length;
        const deg=Math.max(values.head_angle_min_deg,Math.min(values.head_angle_max_deg,tr.start.head+d));
        if(stream)sim._setHeadTarget(deg*Math.PI/180);
        else if(!sim.commandHeadAngle(deg))throw Error("motor target refused");
      }
      const state=sim.state;
      if(!sim.chargerContact)throw Error("dock contact lost");
      output.push([t,state.head_angle_rad*180/Math.PI,state.gyro_radps[1]]);
      sim.tick();
    }
    return tr.data.filter(r=>r.t>=0&&r.t<8).map(r=>{
      const s=output[Math.floor(r.t/sim.dt)];
      return [r.t,r.head,r.gyro,s[1],s[2]];
    });
  } finally {sim.dispose();}
}
