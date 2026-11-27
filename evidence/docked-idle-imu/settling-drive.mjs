import {VectorSim} from "../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";
const mj=await mujoco();
console.log("version,mode,drive,left_mmps,right_mmps,mean_yaw_radps,release_s,travel_mm");
for (const version of ["v1","v2"]) {
 const dir=version==="v1"?"public/robot":"public/robot/vector2";
 const {values,derived}=readJson(dir+"/model.json");
 for(const mode of ["baseline","noslip"]) for(const [drive,l,r] of [["depart",60,60],["ccw100",-100,100],["cw100",100,-100],["ccw180",-180,180],["cw180",180,-180]]) {
   let xml=readText(dir+"/vector-contact42-free-dock.xml");
   if(mode==="noslip") xml=xml.replace("<option ",'<option noslip_iterations="4" ');
   const sim=new VectorSim(mj,{values,derived,xml,startDocked:drive==="depart",headDeg:-21,noiseSeed:42});
   try {
     const x0=sim.poseMm()[0];
     let release=null,lSum=0,rSum=0,yawStart;
     sim.setWheelMotors(l,r);
     for(let t=1;t<=300;t++){
       sim.tick(1);
       if(release===null && !sim.chargerContact && drive==="depart") release=t*sim.dt;
       if(t===100) yawStart=sim.headingTotalRad();
       if(t>100){const [left,right]=sim._wheelSpeeds();lSum+=left;rSum+=right;}
     }
     console.log([version,mode,drive,lSum/200,rSum/200,(sim.headingTotalRad()-yawStart)/(200*sim.dt),release,sim.poseMm()[0]-x0].join(","));
   } finally {sim.dispose();}
 }
}
