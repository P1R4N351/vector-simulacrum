// Export stationary browser slots for the recorded shelf-cube comparison.
import { VectorSim } from "../src/sim/robot.js";
import { mujoco, readJson, readText } from "../test/helpers.js";

const { values, derived } = readJson("public/robot/model.json");
const sim = new VectorSim(await mujoco(), { xml: readText("public/robot/vector-free.xml"), values, derived });
console.log("time_s,slot,x,y,z");
let previous = null;
try {
  while (sim.data.time < 16.56) {
    sim.tick(1);
    const packet = sim.cubeSensor.packet;
    if (packet && packet !== previous) {
      packet.slots.forEach((slot, i) => console.log([packet.time_s, i, ...slot].join(",")));
      previous = packet;
    }
  }
} finally {
  sim.dispose();
}
