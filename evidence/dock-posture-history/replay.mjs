import {readFileSync} from "node:fs";
import {VectorSim} from "../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";
const rows=JSON.parse(readFileSync(0,"utf8"));
const mode=process.argv[2];
if (!["static","posture","wheels","full","noslip","shaft","shaft-noslip","shaft-linear"].includes(mode)) throw Error("Unknown replay");
const {values,derived,shelf_outline_mm}=readJson("public/robot/vector2/model.json");
let xml=readText("public/robot/vector2/vector-free-shelf.xml");
const shaft=mode.startsWith("shaft");
if(shaft) xml=xml.replace(/<general name="(left|right)_wheel" joint="([^"]+)" gainprm="([^"]+)"[^>]*\/>/g,
  (_,side,joint,cap)=>`<velocity name="${side}_wheel" joint="${joint}" kv="0.8" forcelimited="true" forcerange="-${cap} ${cap}"/>`);
if(mode.endsWith("noslip")) xml=xml.replace("<option ",'<option noslip_iterations="4" ');
const sim=new VectorSim(await mujoco(),{values,derived,xml,shelfOutline:shelf_outline_mm,startDocked:true,
  headDeg:rows[0][3],liftMm:rows[0][4],noiseSeed:42});
try{
  console.log("s,x_mm,pitch_deg,head_deg,lift_mm,charger,wl,wr");
  let speeds=[0,0];
  if(shaft) sim._driveTreads=()=>{
    sim.data.ctrl[sim._act.left]=speeds[0]/sim.v.wheel_front_radius_mm;
    sim.data.ctrl[sim._act.right]=speeds[1]/sim.v.wheel_front_radius_mm;
  };
  let j=0, lastHead=rows[0][3],lastLift=rows[0][4];
  for(let tick=0;;tick++){
    const t=rows[0][0]+tick*sim.dt;
    if(t>0.75) break;
    while(j+1<rows.length && rows[j+1][0]<=t+1e-10) j++;
    const [,left,right,head,lift]=rows[j];
    speeds=[left,right];
    if(mode==="shaft-linear" && j+1<rows.length){
      const next=rows[j+1], f=(t-rows[j][0])/(next[0]-rows[j][0]);
      speeds=[left+f*(next[1]-left),right+f*(next[2]-right)];
    }
    const departing=t>=-1.8;
    if(shaft || departing || ["full","posture","noslip"].includes(mode)){
      if(head!==lastHead){sim.commandHeadAngle(head);lastHead=head;}
      if(lift!==lastLift){sim.commandLiftHeight(lift);lastLift=lift;}
    }
    if(shaft || departing || ["full","wheels","noslip"].includes(mode)) sim.setWheelMotors(left,right);
    if(tick%10===0 || t>-2){
      const s=sim.reportedNow();
      console.log([t,sim.poseMm()[0],sim._pitch()*180/Math.PI,s.head_deg,s.lift_mm,+sim.chargerContact,s.wl,s.wr].join(","));
    }
    sim.tick(1);
  }
}finally{sim.dispose();}
