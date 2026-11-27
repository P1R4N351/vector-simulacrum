import {writeFileSync} from "node:fs";
import {load,run} from "./motor-driver.mjs";

const trials=[1,2,3].map(i=>load("../wake-raise/wake-raise-"+i));
function reversing(rows,field,cue){
  const i=rows.findIndex((r,i)=>r[0]>cue+.5 && r[field]>.30 && rows[i+1]?.[field]>.30);
  return i>0 ? (rows[i][0]+rows[i-1][0])/2 : null;
}
const original=run(trials[0],.20,true);
const wait=reversing(original,2,trials[0].cue)-reversing(original,4,trials[0].cue);
if(!(wait>0 && wait<.6))throw Error("Reversal offset unexpected");
const traces=["run,mode,t_s,real_deg,real_gyro,sim_deg,sim_gyro"];
const metrics=["run,mode,delay_s,action_end_s,resume_s,real_reversal_s,sim_reversal_s,full_angle,full_gyro,handoff_angle,handoff_gyro"];
for(const [i,tr] of trials.entries()){
  for(const [name,settings] of [["immediate",{}],["in-position",{inPosition:true}],["first-run-delay",{wait}]]){
    const rows=run(tr,.20,true,true,settings);
    for(const row of rows)traces.push([i+1,name,...row].join(","));
    const errors=[];
    for(const group of [rows,rows.filter(r=>r[0]>=tr.cue+.9 && r[0]<tr.cue+1.5)]){
      for(const k of [1,2])errors.push(Math.sqrt(group.reduce((s,r)=>s+(r[k]-r[k+2])**2,0)/group.length));
    }
    metrics.push([i+1,name,settings.wait??0,settings.actionEnd,settings.resume??"",reversing(rows,2,tr.cue),reversing(rows,4,tr.cue),...errors].join(","));
  }
}
for(const [name,lines] of [["traces",traces],["scores",metrics]])
  writeFileSync(new URL("handoff-"+name+".csv",import.meta.url),lines.join("\n")+"\n");
