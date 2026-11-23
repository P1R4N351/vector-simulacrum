import { mujoco, readJson, readText } from "../../web/test/helpers.js";
import { VectorSim } from "../../web/src/sim/robot.js";
import { readFileSync } from "node:fs";
const input = JSON.parse(readFileSync(0, "utf8"));
const mj = await mujoco();
const {values, derived, shelf_outline_mm} = readJson("public/robot/vector2/model.json");
const sim = new VectorSim(mj,{values, derived, shelfOutline:shelf_outline_mm, startDocked:true,
  headDeg:input.head, liftMm:input.lift, xml:readText("public/robot/vector2/vector-free-shelf.xml")});
try {
  const start = sim.poseMm()[0];
  let j = 0;
  console.log("t_s,travel_mm,on");
  for (let t=input.wheels[0][0]; t<0.51; t+=sim.dt) {
    while (j+1<input.wheels.length && input.wheels[j+1][0]<=t) j++;
    sim.setWheelMotors(input.wheels[j][1],input.wheels[j][2]);
    console.log([t,sim.poseMm()[0]-start,+sim.chargerContact].join(","));
    sim.tick(1);
  }
} finally { sim.dispose(); }
