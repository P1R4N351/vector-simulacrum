// Run the web cube sensor in a stationary orientation. Retain each complete packet once.
import {readFileSync} from "node:fs";
import {CubeSensor} from "../../../web/src/sim/cube_sensor.js";
import {readJson} from "../../../web/test/helpers.js";
const values = readJson("public/robot/vector2/model.json").values;
const cases = JSON.parse(readFileSync(0,"utf8"));
for (const {force,packets,name,seed} of cases) {
  const sensor = new CubeSensor(values,0.001,seed,force);
  let last=null, saved=0;
  for (let i=1; saved<packets && i<(packets+20)*35; i++) {
    sensor.step(i/1000,force);
    if (i>300 && sensor.packet!==last && sensor.packet) {
      saved++;
      console.log(JSON.stringify([name,seed,sensor.packet.slots]));
    }
    last=sensor.packet;
  }
  if (saved!==packets) throw Error("Incomplete stationary run");
}
