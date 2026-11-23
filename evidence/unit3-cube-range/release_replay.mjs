// Run from any directory with the canonical web dependencies installed.
import { mujoco, readText, readJson } from "../../web/test/helpers.js";
import { VectorSim } from "../../web/src/sim/robot.js";
const mj = await mujoco();
const {values, derived, shelf_outline_mm} = readJson("public/robot/vector2/model.json");
console.log("command_mmps,release_ms,last_on_mm,first_off_mm,pitch_change_deg,origin_drop_mm");
for (const speed of [25,40,60]) {
  const sim = new VectorSim(mj,{values,derived,shelfOutline:shelf_outline_mm,startDocked:true,
    headDeg:-9.5,liftMm:33.4,xml:readText("public/robot/vector2/vector-free-shelf.xml")});
  try {
    const start = sim.poseMm(), pitch = sim.state.pose_pitch_rad;
    sim.setWheelMotors(speed,speed);
    let prev=0;
    for (let i=1;i<=600;i++) {
      sim.tick(1);
      const [x,,z]=sim.poseMm();
      if (!sim.chargerContact) {
        console.log([speed,5*i,prev,x-start[0],(sim._pitch()-pitch)*180/Math.PI,start[2]-z].join(","));
        break;
      }
      prev=x-start[0];
      if(i===600) throw new Error("Dock contact held throughout replay");
    }
  } finally {sim.dispose();}
}
