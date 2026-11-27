import {readFileSync,writeFileSync} from "node:fs";
import {VectorSim,CONTROL_OVERRIDE} from "../../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../../web/test/helpers.js";

const {values,derived}=readJson("public/robot/model.json");
const xml=readText("public/robot/vector-contact42-free-dock.xml");
const mj=await mujoco();
function load(path){
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
const train=["pilot","session-1"].map(name=>load("../../docked-head-20261126/grants/"+name));
const paths=["../../docked-head-20261126/grants/session-2",...["blind-grant-1","blind-grant-2","calm-cross-1","calm-cross-2"].map(n=>"./"+n),...[1,2,3].map(n=>"../wake-raise/wake-raise-"+n)];
function run(tr,lead,gate,stream=true){
  const sim=new VectorSim(mj,{values,derived,xml,controlLevel:CONTROL_OVERRIDE,startDocked:true,noiseSeed:42,headDeg:tr.start.head,liftMm:tr.start.lift});
  try {
    const output=[];
    let issued=false;
    const wake=gate && Boolean(tr.start.flags&1024);
    for(let i=0;i*sim.dt<=8.05;i++){
      const t=i*sim.dt;
      if(!issued && tr.cue!==null && t>=tr.cue){
        if(!sim.setHeadAngle(22))throw Error("raise refused");
        issued=true;
      } else if(wake && t<=5.16 && !sim.actionRunning){
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
function rms(rows,idx){return Math.sqrt(rows.reduce((s,r)=>s+(r[idx]-r[idx+2])**2,0)/rows.length);}
const candidates=[0,.05,.10,.15,.20,.25,.30];
const fits=candidates.map(lead=>({lead,score:train.reduce((s,tr)=>s+rms(run(tr,lead,true),1),0)/train.length}));
const best=fits.reduce((a,b)=>a.score<=b.score?a:b).lead;
const lines=["run,mode,lead_s,t_s,real_deg,real_gyro,sim_deg,sim_gyro"];
for(const path of paths){
  const tr=load(path),name=path.split("/").at(-1);
  for(const [mode,lead,gate] of [["baseline",0,false],["sdk-stream",0,true],["motor",0,true],["compensated",best,true]]){
    for(const r of run(tr,lead,gate,mode!=="sdk-stream"))lines.push([name,mode,lead,...r].join(","));
  }
}
writeFileSync(new URL("motor-traces.csv",import.meta.url),lines.join("\n")+"\n");
writeFileSync(new URL("motor-fit.csv",import.meta.url),"lead_s,mean_training_angle_rmse_deg\n"+fits.map(r=>[r.lead,r.score].join(",")).join("\n")+"\n");
