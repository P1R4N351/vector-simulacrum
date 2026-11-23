import { pinOffsets } from "./pin_offsets.mjs";
// Open-loop reverse mount; change only the specified charger guide contacts.
import { rearEnvelope } from "./rear_envelope.mjs";
import { VectorSim } from "../../web/src/sim/robot.js";
import { mujoco, readJson, readText } from "../../web/test/helpers.js";
const mj=await mujoco(), model=readJson("public/robot/vector2/model.json");
const base=rearEnvelope(readText("public/robot/vector2/vector-free-dock.xml"),"driven_rear_envelope");
function padResiduals(sim) {
  const errors = pinOffsets(sim);
  return [Math.max(...errors.map(e=>Math.abs(e[0]))),Math.max(...errors.map(e=>Math.abs(e[1]))),
    Math.min(...errors.map(e=>e[2])),Math.max(...errors.map(e=>e[2]))];
}
console.log("guides,heading_deg,offset_mm,time_s,x_mm,y_mm,yaw_deg,on,pad_abs_x_mm,pad_abs_y_mm,pad_min_z_mm,pad_max_z_mm");
for (const guides of ["original","smooth","no_funnels","no_rails","no_guides"]) {
  const xml=base.replace(/<geom name="dock_(funnel|rail)_[lr]"[^>]+\/>/g,(g,part)=>{
    if(guides==="smooth")return g.replace('priority="2"','priority="3"').replace(/friction="[^"]+"/,'friction="0.01 0.005 0.0001"');
    if(guides==="no_guides" || guides==="no_"+part+"s")return g.replace('conaffinity="1"','conaffinity="0"');
    return g;
  });
  for(const heading of (guides==="original"?[-5,-3,-1.5,0,1.5,3,5]:[-3,0,3])){
    // Straight-line aim compensates for 94 mm of travel and the pads 35.5 mm aft.
    const aimedOffset=129.5*Math.tan(heading*Math.PI/180);
    const offsets=[-3,0,3].includes(heading)?[-2,0,2]:[0];
    if(heading!==0 && (guides==="original" || guides==="no_guides")) offsets.push(aimedOffset);
    for(const offset of offsets){
      const sim=new VectorSim(mj,{xml,values:model.values,derived:model.derived,camera:model.camera,
        startXyMm:[-42,offset],headingDeg:heading,headDeg:0,liftMm:47,noiseSeed:1});
      try {
        sim.controlLevel="override"; sim.setWheelMotors(-30,-30);
        for(let i=1;i<=1400;i++){
          sim.tick(1);
          if(i%20===0||sim.chargerContact){
            const [x,y]=sim.poseMm();
            console.log([guides,heading,offset,i*sim.dt,x,y,sim._yaw()*180/Math.PI,+sim.chargerContact,...padResiduals(sim)].join(","));
          }
          if(sim.chargerContact)break;
        }
      } finally {sim.dispose();}
    }
  }
}
