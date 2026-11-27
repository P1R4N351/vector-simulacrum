import {VectorSim} from "../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";
const {values,derived}=readJson("public/robot/model.json");
const sim=new VectorSim(await mujoco(),{values,derived,xml:readText("public/robot/vector-contact42-free-dock.xml"),startDocked:true,calmMode:true,noiseSeed:42});
try {
  console.log("s,gx,gy,gz,ax,ay,az");
  let stamp=-1;
  for(let i=0;i<12500;i++){
    const state=sim.state;
    if(state.time_s!==stamp){
      console.log([state.time_s,...state.gyro_radps,...state.accel_mmps2].join(","));
      stamp=state.time_s;
    }
    sim.tick(1);
  }
}finally{sim.dispose();}
