// Exercise the public docking routine and retain its final reverse trajectory.
import { rearEnvelope } from "./rear_envelope.mjs";
import {VectorSim} from "../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";
const mj=await mujoco();
const root=process.argv[2]==="gen1"?"public/robot":"public/robot/vector2";
const model=readJson(root+"/model.json");
console.log("start,first_contact_rel_s,left_mmps,right_mmps");
for(const [label,options] of [["direct",{startDocked:true}],["diagonal",{startXyMm:[120,80],headingDeg:45}]]) {
  const sim=new VectorSim(mj,{...options,xml:rearEnvelope(readText(root+"/vector-free-dock.xml"),process.argv[3]??"canonical"),
    values:model.values,derived:model.derived,camera:model.camera,noiseSeed:1});
  try {
    if(options.startDocked) {
      sim.driveOffCharger();
      for(let i=0;i<2000&&sim.actionRunning;i++) sim.tick(1);
    }
    sim.dockWithCharger();
    const samples=[]; let lastContact=null,previous=sim.chargerContact;
    for(let i=0;i<16000&&sim.actionRunning;i++){
      sim.tick(1);
      if(sim.chargerContact&&!previous) lastContact=i;
      previous=sim.chargerContact;
      const {wl,wr}=sim.reportedNow();
      samples.push([i,wl,wr]);
    }
    console.error(label,sim.lastAction?.result,sim.chargerContact);
    if(!sim.chargerContact||lastContact===null)continue;
    for(const [i,l,r] of samples)if(i>=lastContact-200&&i<=lastContact+50) console.log([label,(i-lastContact)*sim.dt,l,r].join(","));
  }finally{sim.dispose();}
}
