import {writeFileSync} from "node:fs";
import {load,run,train} from "./motor-driver.mjs";
const paths=["../../docked-head-20261126/grants/session-2",...["blind-grant-1","blind-grant-2","calm-cross-1","calm-cross-2"].map(n=>"./"+n),...[1,2,3].map(n=>"../wake-raise/wake-raise-"+n)];
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
