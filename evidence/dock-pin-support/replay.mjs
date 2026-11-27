import {VectorSim} from "../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";

import {addPins} from "./pins.mjs";
const mj = await mujoco();
const [scene, stiffnessArg, drive="rest", version="v1"] = process.argv.slice(2);
const stiffness=Number(stiffnessArg);
const dir=version==="v2"?"public/robot/vector2":"public/robot";
const {values,derived}=readJson(dir+"/model.json");
const original=readText(dir+"/"+scene+".xml");
const sim=new VectorSim(mj,{values,derived,xml:stiffness?addPins(original,stiffness):original,
  startDocked:true,headDeg:-21.017,liftMm:32.34,calmMode:drive==="rest",noiseSeed:42});
const adr=side=>sim.model.jnt_qposadr[mj.mj_name2id(sim.model,mj.mjtObj.mjOBJ_JOINT.value,"pin_"+side)];
const indices=stiffness?["l","r"].map(adr):[];
try {
  const last=drive==="rest"?12500:400;
  console.log("time_s,x_mm,pitch_deg,charger,left_compression_mm,right_compression_mm");
  for(let tick=0;tick<=last;tick++){
    if(tick% (drive==="rest"?50:1)===0) {
      const compression=indices.map(i=>-1000*sim.data.qpos[i]);
      console.log([tick*sim.dt,sim.poseMm()[0],sim._pitch()*180/Math.PI,+sim.chargerContact,compression[0]??0,compression[1]??0].join(","));
    }
    if(drive==="depart" && tick===0) sim.setWheelMotors(60,60);
    sim.tick(1);
  }
} finally {sim.dispose();}
