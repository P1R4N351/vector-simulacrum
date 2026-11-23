import {readFileSync} from "node:fs";
import {mujoco,readJson,readText} from "../../web/test/helpers.js";
import {VectorSim} from "../../web/src/sim/robot.js";

const mj = await mujoco();
const {values,derived,shelf_outline_mm} = readJson("public/robot/vector2/model.json");
const xml = readText("public/robot/vector2/vector-free-shelf.xml");
const trials = JSON.parse(readFileSync(0,"utf8"));
console.log("trial,offset_mm,start_x_mm,start_pitch_deg,release_s,travel_mm");
for (const offset of [0,2,4,5,6,8]) {
  class ShiftedStart extends VectorSim {
    _place(...args) {
      super._place(...args);
      this.data.qpos[0] += offset/1000; // World +X points out of the shelf dock.
      this.mujoco.mj_forward(this.model,this.data);
    }
  }
  for (const trial of trials) {
    const sim = new ShiftedStart(mj,{values,derived,xml,shelfOutline:shelf_outline_mm,startDocked:true,
      headDeg:trial.head,liftMm:trial.lift,noiseSeed:1});
    try {
      if (!sim.chargerContact) throw Error("shifted start lacks electrical contact");
      const x0=sim.poseMm()[0], pitch=sim._pitch()*180/Math.PI;
      const wheels=trial.wheels;
      let j=0;
      for (let t=wheels[0][0];t<=wheels.at(-1)[0];t+=sim.dt) {
        while(j+1<wheels.length && wheels[j+1][0]<=t) j++;
        sim.setWheelMotors(wheels[j][1],wheels[j][2]);
        sim.tick(1);
        if(!sim.chargerContact) {
          console.log([trial.name,offset,x0,pitch,t+sim.dt,sim.poseMm()[0]-x0].join(","));
          break;
        }
      }
    } finally {sim.dispose();}
  }
}
