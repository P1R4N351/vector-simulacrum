import {readFileSync} from "node:fs";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";
import {VectorSim} from "../../web/src/sim/robot.js";

const mj=await mujoco();
const {values,derived,shelf_outline_mm}=readJson("public/robot/vector2/model.json");
const xml=readText("public/robot/vector2/vector-free-shelf.xml");
const shaftXml=xml.replace(/<general name="(left|right)_wheel" joint="([^"]+)" gainprm="([^"]+)"[^>]*\/>/g,
  (_,side,joint,cap)=>`<velocity name="${side}_wheel" joint="${joint}" kv="0.8" forcelimited="true" forcerange="-${cap} ${cap}"/>`);
if(!shaftXml.includes('<velocity name="right_wheel"')) throw Error("Missing shaft actuator");
const trials=JSON.parse(readFileSync(0,"utf8"));
console.log("trial,mode,offset,t,x_mm,on");
const offsets = process.argv.slice(2).map(Number);
if (!offsets.every(Number.isFinite)) throw Error("Seat offsets must be finite millimetres");
for(const mode of offsets.length ? ["linear"] : ["controller","shaft","linear"]){
  for(const offset of offsets.length ? offsets : [0,6]){
    class Seat extends VectorSim {
      _place(...args){
        super._place(...args);
        this.data.qpos[0]+=offset/1000;
        this.mujoco.mj_forward(this.model,this.data);
      }
    }
    for(const trial of trials){
      const shaft=mode!=="controller";
      const sim=new Seat(mj,{values,derived,xml:shaft?shaftXml:xml,shelfOutline:shelf_outline_mm,
        startDocked:true,headDeg:trial.head,liftMm:trial.lift,noiseSeed:1});
      try {
        if(!sim.chargerContact) throw Error("Expected seated start");
        let speeds=[0,0], index=0;
        if(shaft) sim._driveTreads=()=>{
          sim.data.ctrl[sim._act.left]=speeds[0]/sim.v.wheel_front_radius_mm;
          sim.data.ctrl[sim._act.right]=speeds[1]/sim.v.wheel_front_radius_mm;
        };
        const wheels=trial.wheels;
        for(let tick=0;;tick++){
          const t=wheels[0][0]+tick*sim.dt;
          if(t>1.1) break;
          while(index+1<wheels.length && wheels[index+1][0]<=t) index++;
          speeds=wheels[index].slice(1);
          if(mode==="linear" && index+1<wheels.length){
            const next=wheels[index+1],frac=(t-wheels[index][0])/(next[0]-wheels[index][0]);
            speeds=speeds.map((v,i)=>v+frac*(next[i+1]-v));
          }
          sim.setWheelMotors(...speeds);
          console.log([trial.name,mode,offset,t,sim.poseMm()[0],+sim.chargerContact].join(","));
          sim.tick(1);
        }
      }finally{sim.dispose();}
    }
  }
}
