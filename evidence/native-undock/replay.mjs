import {readFileSync} from "node:fs";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";
import {VectorSim} from "../../web/src/sim/robot.js";
const mj=await mujoco(), {values,derived,shelf_outline_mm}=readJson("public/robot/vector2/model.json");
const clips=JSON.parse(readFileSync(0,"utf8"));
console.log("clip,t,left,right,head,lift,on");
for (const [name,frames] of Object.entries({canonical:[],...clips})){
  const sim=new VectorSim(mj,{values,derived,xml:readText("public/robot/vector2/vector-free-shelf.xml"),
    shelfOutline:shelf_outline_mm,startDocked:true,headDeg:Number(process.argv[2]),liftMm:Number(process.argv[3]),noiseSeed:1});
  try{
    const track=kind=>frames.filter(f=>f.Name===kind).sort((a,b)=>a.triggerTime_ms-b.triggerTime_ms);
    const body=track("BodyMotionKeyFrame"),head=track("HeadAngleKeyFrame"),lift=track("LiftHeightKeyFrame");
    let hi=0,li=0;
    if(name==="canonical")sim.driveOffCharger();
    for(let tick=0;tick<=4250;tick++){
      const t=tick*sim.dt,ms=1000*t;
      if(name!=="canonical"){
        while(hi<head.length && head[hi].triggerTime_ms<=ms)sim.commandHeadAngle(head[hi++].angle_deg);
        while(li<lift.length && lift[li].triggerTime_ms<=ms){
          const f=lift[li++];sim.commandLiftHeight(f.height_mm,f.durationTime_ms/1000);
        }
        const f=body.findLast(f=>f.triggerTime_ms<=ms);
        let l=0,r=0;
        if(f && ms<f.triggerTime_ms+f.durationTime_ms){
          const v=f.speed,rad=f.radius_mm;
          if(rad==="STRAIGHT") l=r=v;
          else if(rad==="TURN_IN_PLACE" || rad===0){r=v*Math.PI/180*23;l=-r;}
          else{l=v*(1-23/rad);r=v*(1+23/rad);}
        }
        sim.setWheelMotors(l,r);
      }
      if(tick%2===0){const s=sim.reportedNow();console.log([name,t,s.wl,s.wr,s.head_deg,s.lift_mm,+sim.chargerContact].join(","));}
      sim.tick(1);
    }
  } finally{sim.dispose();}
}
