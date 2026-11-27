// Align only longitudinal seating to the first corrected side-camera frame.
import {readFileSync} from "node:fs";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";
import {VectorSim} from "../../web/src/sim/robot.js";
import {dockImpedance} from "../dock-impedance/variant.mjs";

const [constraint,drive,registration]=process.argv.slice(2);
if(!["base","both"].includes(constraint)||!["command","shaft"].includes(drive)||!["nominal","glyph","glyph-lip"].includes(registration))throw Error("Unknown replay condition");
const geometry=readFileSync(new URL("../dock-pad-registration/geometry.csv",import.meta.url),"utf8").trim().split(/\r?\n/).slice(1).map(r=>r.split(","));
const frame=geometry.find(r=>r[0]==="B" && r[1]===(registration==="glyph"?"4":"5") && r[2]==="span" && r[4]==="W35");
const frameTime=Number(frame[5]), cameraOffset=Number(frame[8]);
const rows=JSON.parse(readFileSync(0,"utf8")).filter(r=>r[0]>=-1.8);
const {values,derived,shelf_outline_mm}=readJson("public/robot/vector2/model.json");
const engine=await mujoco();
let xml=dockImpedance(readText("public/robot/vector2/vector-free-shelf.xml"),constraint);
if(drive==="shaft")xml=xml.replace(/<general name="(left|right)_wheel" joint="([^"]+)" gainprm="([^"]+)"[^>]*\/>/g,
 (_,side,joint,cap)=>`<velocity name="${side}_wheel" joint="${joint}" kv="0.8" forcelimited="true" forcerange="-${cap} ${cap}"/>`);
function run(shift,measureOnly){
  class Seated extends VectorSim {
    _place(...args){
      super._place(...args);
      this.data.qpos[0]+=shift/1000;
      this.mujoco.mj_forward(this.model,this.data);
    }
  }
  const sim=new Seated(engine,{values,derived,xml,startDocked:true,shelfOutline:shelf_outline_mm,headDeg:rows[0][3],liftMm:rows[0][4],noiseSeed:42});
  let wheels=[0,0];
  if(drive==="shaft")sim._driveTreads=()=>{
    sim.data.ctrl[sim._act.left]=wheels[0]/values.wheel_front_radius_mm;
    sim.data.ctrl[sim._act.right]=wheels[1]/values.wheel_front_radius_mm;
  };
  try{
    const out=[];
    let j=0,lastHead=rows[0][3],lastLift=rows[0][4];
    for(let n=0;;n++){
      const t=rows[0][0]+n*sim.dt;
      if(t>.75)break;
      while(j+1<rows.length && rows[j+1][0]<=t)j++;
      const [stamp,left,right,head,lift]=rows[j];
      wheels=[left,right];
      if(drive==="shaft" && j+1<rows.length){
        const next=rows[j+1], f=(t-stamp)/(next[0]-stamp);
        wheels=[left+f*(next[1]-left),right+f*(next[2]-right)];
      }
      if(head!==lastHead){sim.commandHeadAngle(head);lastHead=head;}
      if(lift!==lastLift){sim.commandLiftHeight(lift);lastLift=lift;}
      sim.setWheelMotors(left,right);
      const {xpos,xmat,site_xpos}=sim.data;
      const ch=sim._chassisBody,dk=sim._dockBody;
      let offset=0;
      for(let a=0;a<3;a++){
        const pad=xpos[3*ch+a]-.0355*xmat[9*ch+3*a]+.001*xmat[9*ch+3*a+2];
        const pin=sim._dockPins.reduce((sum,i)=>sum+site_xpos[3*i+a]/2,0);
        offset-=1000*(pad-pin)*xmat[9*dk+3*a];
      }
      if(measureOnly && t>=frameTime)return offset;
      out.push([t,shift,sim.poseMm()[0],offset,+sim.chargerContact]);
      sim.tick();
    }
    return out;
  } finally {sim.dispose();}
}
let shift=0;
if(registration!=="nominal"){
  let bestError=Infinity;
  let lo=-12, hi=14;
  for(let i=0;i<16;i++){
    const candidate=(lo+hi)/2;
    const error=run(candidate,true)-cameraOffset;
    if(Math.abs(error)<bestError){bestError=Math.abs(error);shift=candidate;}
    if(bestError<.03)break;
    if(error<0)lo=candidate;else hi=candidate;
  }
  if(bestError>.05)console.error("Closest tested initial offset differs by",bestError,"mm for",constraint,drive,registration);

}
console.log("s,shift_mm,chassis_x_mm,pad_outward_mm,contact");
for(const row of run(shift,false)) console.log(row.join(","));
