import {readFileSync} from "node:fs";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";
import {VectorSim} from "../../web/src/sim/robot.js";
const mj=await mujoco(), {values,derived,shelf_outline_mm}=readJson("public/robot/vector2/model.json");
const clips=JSON.parse(readFileSync(0,"utf8"));
const testTurns=process.argv.includes("--turns");
const modes=testTurns ? ["open","ramped","feedback"] : ["open"];
console.log("clip,t,left,right,head,lift,on"+(testTurns?",gyro,x_mm,y_mm,z_mm,yaw_rad":""));
for (const mode of modes) for (const [name,frames] of Object.entries(testTurns ? {straight_03:clips.straight_03} : {canonical:[],...clips})){
  const sim=new VectorSim(mj,{values,derived,xml:readText("public/robot/vector2/vector-free-shelf.xml"),
    shelfOutline:shelf_outline_mm,startDocked:true,headDeg:Number(process.argv[2]),liftMm:Number(process.argv[3]),noiseSeed:1});
  try{
    const track=kind=>frames.filter(f=>f.Name===kind).sort((a,b)=>a.triggerTime_ms-b.triggerTime_ms);
    const body=track("BodyMotionKeyFrame"),head=track("HeadAngleKeyFrame"),lift=track("LiftHeightKeyFrame");
    let hi=0,li=0,previousFrame=null,desired=0,integral=0;
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
          else if(rad==="TURN_IN_PLACE" || rad===0){
            const target=v*Math.PI/180;
            r=target*23;
            if(mode!=="open"){
              const state=sim.reportedNow();
              const measured=state.gyro_z_dps*Math.PI/180/Math.cos(state.head_deg*Math.PI/180);
              if(previousFrame!==f){desired=measured;integral=0;previousFrame=f;}
              const step=50*sim.dt;
              desired+=Math.max(-step,Math.min(step,target-desired));
              r=Math.trunc(desired*23*1.5);
              if(mode==="feedback"){
                const error=desired-measured;
                r+=Math.trunc(20*error+0.5*integral);
                if(Math.abs(r)<220)integral=Math.max(-100,Math.min(100,integral+error));
              }
            }
            l=-r;
          }
          else{l=v*(1-23/rad);r=v*(1+23/rad);}
        }
        sim.setWheelMotors(l,r);
      }
      if(tick%2===0){const s=sim.reportedNow();const row=[testTurns?mode:name,t,s.wl,s.wr,s.head_deg,s.lift_mm,+sim.chargerContact];if(testTurns)row.push(s.gyro_z_dps,...sim.poseMm(),sim._yaw());console.log(row.join(","));}
      if(testTurns && tick===2800){
        const pairs=new Set();
        const contact=sim.data.contact;
        for(let i=0;i<sim.data.ncon;i++){
          const c=contact.get(i);
          const names=[c.geom1,c.geom2].map(id=>mj.mj_id2name(sim.model,mj.mjtObj.mjOBJ_GEOM.value,id));
          pairs.add(names.join("/"));
          c.delete();
        }
        console.error(mode,[...pairs].join(" "));
      }
      sim.tick(1);
    }
  } finally{sim.dispose();}
}
