// Sample a free return and subsequent departure through the canonical shelf physics.
import { mujoco, readJson, readText } from "../../web/test/helpers.js";
import { VectorSim } from "../../web/src/sim/robot.js";
const mj = await mujoco();
const { values, derived, shelf_outline_mm } = readJson("public/robot/vector2/model.json");
const localStop = process.argv[2] === "--contact-stop";
const wheels = localStop || process.argv[2] === "--wheels";
const args = process.argv.slice(wheels ? 3 : 2);
console.log("speed,phase,t_s,x_mm,sdk_x_mm,on,pitch_rad,z_mm,dock_depth_mm" + (wheels ? ",left_mmps,right_mmps,tread_left_mm,tread_right_mm" : ""));
const speeds = args.length ? args.map(Number) : [25,35,45];
if (speeds.some(speed => !Number.isFinite(speed) || speed <= 0)) throw Error("Expected positive speeds in mm/s");
for (const speed of speeds) {
  const sim = new VectorSim(mj,{values,derived,shelfOutline:shelf_outline_mm,startDocked:true,
    headDeg:0,liftMm:47.15,xml:readText("public/robot/vector2/vector-free-shelf.xml")});
  try {
    const initial = sim.poseMm()[0];
    function row(phase) {
      const [x,,z]=sim.poseMm();
      const dock=sim._dockBody, r=sim.data.xmat.subarray(9*dock), p=sim.data.xpos.subarray(3*dock);
      const depth=sim.poseMm().reduce((a,v,i)=>a+(v-1000*p[i])*r[i*3],0);
      const extra = [];
      if (wheels) {
        const { wl, wr } = sim.reportedNow();
        extra.push(wl, wr, ...sim._treadTravelMm());
      }
      console.log([speed,phase,sim.tickCount*sim.dt,x-initial,sim.state.x_mm-initial,+sim.chargerContact,sim._pitch(),z,depth,...extra].join(","));
    }
    function drive(v,phase,until) {
      sim.setWheelMotors(v,v);
      for(let i=0;i<1600;i++) {
        sim.tick(1); row(phase);
        if(until(i)) return;
      }
      throw Error("Unreached phase endpoint "+phase);
    }
    row("start");
    drive(35,"out",()=>sim.poseMm()[0]-initial>55);
    drive(0,"pause",i=>i>=100);
    drive(-speed,"return",()=>sim.chargerContact);
    if (localStop) sim._stopTreadsAtContact();
    drive(0,"seat",i=>i>=199);
    drive(35,"release",()=>!sim.chargerContact);
  } finally {sim.dispose();}
}
