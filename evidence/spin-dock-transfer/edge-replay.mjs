// Use the canonical chassis and gyro; prescribe measured shaft travel as in replay.mjs --travel.
import {readFileSync} from "node:fs";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";
import {VectorSim} from "../../web/src/sim/robot.js";
const rows=name=>readFileSync(new URL(name,import.meta.url),"utf8").trim().split(/\r?\n/).slice(1).map(l=>l.split(","));
const input=rows("edge-knots.csv"), heads=rows("../hw-rec-20261119/unit2-native-replay-input.csv");
const mj=await mujoco(), model=readJson("public/robot/vector2/model.json");
const stepMs=Number(process.env.VECTOR_SPIN_STEP_MS ?? 1);
if (![.5,1].includes(stepMs)) throw Error("Expected a 0.5 or 1 ms solver step");
const scene=readText("public/robot/vector2/vector-free-dock.xml").replace('timestep="0.001"','timestep="'+stepMs/1000+'"').replace(/<general name="(left|right)_wheel" joint="([^"]+)" gainprm="([^"]+)"[^>]*\/>/g,
 (_,side,joint,cap)=>`<velocity name="${side}_wheel" joint="${joint}" kv="0.8" forcelimited="true" forcerange="-${cap} ${cap}"/>`);
if (!scene.includes('<velocity name="right_wheel"')) throw Error("Missing shaft drive");
console.log("turn,scheme,ms,head_gyro_dps,left_mm,right_mm");
for (const turn of new Set(input.map(r=>r[0]))) for (const scheme of ["frame","fresh","early","center","late"]) {
  const paths=["left","right"].map(side=>input.filter(r=>r[0]===turn&&r[1]===side&&r[2]===scheme).map(r=>r.slice(3).map(Number)));
  const sim=new VectorSim(mj,{xml:scene,values:model.values,derived:{...model.derived,substeps:model.derived.substeps/stepMs},startXyMm:[0,140],headingDeg:0,
    headDeg:Number(heads.find(r=>r[0]===turn)[7])*180/Math.PI,liftMm:30.64501190185547,noiseSeed:1});
  try {
    const indices=[0,0];
    function pos(i,t) {
      const a=paths[i];
      while(indices[i]+1<a.length && a[indices[i]+1][0]<=t) indices[i]++;
      const k=indices[i];
      if(k===a.length-1) return a[k][1];
      return a[k][1]+(t-a[k][0])*(a[k+1][1]-a[k][1])/(a[k+1][0]-a[k][0]);
    }
    const dt=sim.dt*1000;
    sim.controlLevel="override";
    let speed=[0,0];
    sim._driveTreads=()=>{for(const [i,side] of ["left","right"].entries()) sim.data.ctrl[sim._act[side]]=speed[i]/sim.v.wheel_front_radius_mm;};
    for(let t=paths[0][0][0];t<=paths[0].at(-1)[0];t+=dt){
      const actual=sim._treadTravelMm();
      speed=paths.map((_,i)=>{const p=pos(i,t); return (pos(i,t+dt)-p)*1000/dt+(p-actual[i])/.05;});
      sim.setWheelMotors(...speed);
      console.log([turn,scheme,t,sim.reportedNow().gyro_z_dps,...actual].join(","));
      sim.tick(1);
    }
  } finally {sim.dispose();}
}
