import { mujoco, readJson, readText } from "../../web/test/helpers.js";
import { VectorSim } from "../../web/src/sim/robot.js";
import { readFileSync } from "node:fs";
import {addPins} from "./pins.mjs";
const input = JSON.parse(readFileSync(0, "utf8"));
const mj = await mujoco();
const {values, derived, shelf_outline_mm} = readJson("public/robot/vector2/model.json");
const noslip = process.argv[2] === "noslip";
const stiffness = noslip ? 0 : Number(process.argv[2]);
const xml=readText("public/robot/vector2/vector-free-shelf.xml");
const sim = new VectorSim(mj,{values, derived, shelfOutline:shelf_outline_mm, startDocked:true,
  headDeg:input.head, liftMm:input.lift, xml:stiffness ? addPins(xml,stiffness) : noslip ? xml.replace("<option ",'<option noslip_iterations="4" ') : xml});
try {
  const dwell = Number(process.argv[3] ?? 0);
  const seated = sim.poseMm()[0];
  sim.tick(Math.round(dwell/sim.dt));
  const start = sim.poseMm()[0];
  const pinIds = ["dock_pin_l","dock_pin_r"].map(name=>mj.mj_name2id(sim.model,mj.mjtObj.mjOBJ_GEOM.value,name));
  let j = 0;
  console.log("t_s,travel_mm,on,left_pin_n,right_pin_n,dwell_travel_mm");
  for (let t=input.wheels[0][0]; t<0.75; t+=sim.dt) {
    while (j+1<input.wheels.length && input.wheels[j+1][0]<=t) j++;
    sim.setWheelMotors(input.wheels[j][1],input.wheels[j][2]);
    const loads=[0,0];
    for(let i=0;i<sim.data.ncon;i++){
      const c=sim.data.contact.get(i);
      const side=pinIds.findIndex(id=>id===c.geom1 || id===c.geom2);
      if(side>=0 && c.efc_address>=0) loads[side]+=sim.data.efc_force[c.efc_address];
    }
    console.log([t,sim.poseMm()[0]-start,+sim.chargerContact,...loads,start-seated].join(","));
    sim.tick(1);
  }
} finally { sim.dispose(); }
