import {readFileSync} from "node:fs";
import {gunzipSync} from "node:zlib";
import {CubeListeners} from "../../web/src/sim/cube_listeners.js";

const rows=gunzipSync(readFileSync(new URL("./cube.csv.gz",import.meta.url))).toString().trim().split("\n").slice(1);
let name,listener,slots;
console.log("trial,packet_s,tap_count");
for (const row of rows) {
  const [trial,time,slot,x,y,z,taps]=row.split(",");
  if(trial!==name) {name=trial; listener=new CubeListeners();}
  if(+slot===0) slots=[];
  if(slots.length!==+slot) throw Error("Packet slots out of sequence");
  slots.push([+x,+y,+z]);
  if(slots.length===3) {
    for (const event of listener.receive({time_s:+time,taps:+taps,slots})) {
      if("tap_count" in event) console.log([trial,event.time_s,event.tap_count].join(","));
    }
  }
}
