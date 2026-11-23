import {readFileSync} from "node:fs";
import {CubeSensor} from "../../web/src/sim/cube_sensor.js";
import {readJson} from "../../web/test/helpers.js";
const {dt,start,forces}=JSON.parse(readFileSync(0,"utf8"));
const {values}=readJson("public/robot/vector2/model.json");
const chip=new CubeSensor(values,dt,1,forces[0]);
const rows=[];
for(let i=0;i<forces.length;i++){
  const prev=chip.packet;
  chip.step((i+1)*dt,forces[i]);
  if(chip.packet && chip.packet!==prev){
    for(const slot of chip.packet.slots) rows.push([start+chip.packet.time_s,...slot.map(v=>v/8192),chip.packet.taps]);
  }
}
console.log(JSON.stringify(rows));
