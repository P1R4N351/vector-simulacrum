import { rearEnvelope } from "./rear_envelope.mjs";
// Isolate the final straight mount, with all variants using the canonical Vector 2 model.
import {VectorSim} from "../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";
const mj=await mujoco(), model=readJson("public/robot/vector2/model.json");
const xml=readText("public/robot/vector2/vector-free-dock.xml");
console.log("variant,offset_mm,speed_mmps,time_s,origin_x_mm,wheel_mmps,contact,collision_pairs");
for(const variant of ["canonical","smooth_rear_guards","no_rear_guards","driven_rear_envelope"])
for(const offset of [0,2])for(const speed of [30,102]){
  const candidate=rearEnvelope(xml,variant);
  const sim=new VectorSim(mj,{xml:candidate,values:model.values,derived:model.derived,camera:model.camera,startXyMm:[-42,offset],headingDeg:0,headDeg:0,liftMm:47,noiseSeed:1});
  try{
    sim.controlLevel="override";
    sim.setWheelMotors(-speed,-speed);
    for(let i=1;i<=1400;i++){
      sim.tick(1);
      if(i%20===0||sim.chargerContact){
        const pairs=new Set();
        for(let j=0;j<sim.data.ncon;j++){
          const contact=sim.data.contact.get(j);
          const names=[contact.geom1,contact.geom2].map(id=>mj.mj_id2name(sim.model,mj.mjtObj.mjOBJ_GEOM.value,id));
          if(names.some(n=>n?.startsWith("dock_")))pairs.add(names.join("/"));
        }
        const {wl,wr}=sim.reportedNow();
        console.log([variant,offset,speed,i*sim.dt,sim.poseMm()[0],(wl+wr)/2,+sim.chargerContact,[...pairs].sort().join("|")].join(","));
      }
      if(sim.chargerContact)break;
    }
  }finally{sim.dispose();}
}
