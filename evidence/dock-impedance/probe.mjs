import {dockImpedance} from "./variant.mjs";
import {VectorSim,CONTROL_OVERRIDE} from "../../web/src/sim/robot.js";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";
const [mode,secondsArg,driveArg]=process.argv.slice(2);
const seconds=Number(secondsArg);
const driveAt=driveArg===undefined ? Infinity : Number(driveArg);
const {values,derived}=readJson("public/robot/model.json");
let xml=readText("public/robot/vector-contact42-free-dock.xml");
xml=dockImpedance(xml,mode);
const sim=new VectorSim(await mujoco(),{values,derived,xml,startDocked:true,controlLevel:CONTROL_OVERRIDE,headDeg:-4.18,liftMm:32.68,noiseSeed:42});
try{
  console.log("s,x_mm,y_mm,z_mm,left_mm,right_mm,contact");
  for(let n=0;n<=Math.round(seconds/sim.dt);n++){
    if(n%10===0)console.log([n*sim.dt,...sim.poseMm(),...sim._treadTravelMm(),+sim.chargerContact].join(","));
    if(n===Math.round(driveAt/sim.dt))sim.setWheelMotors(60,60);
    sim.tick();
  }
}finally{sim.dispose();}
