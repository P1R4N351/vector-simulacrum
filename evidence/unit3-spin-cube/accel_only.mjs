import {readFileSync} from "node:fs";
import {gunzipSync} from "node:zlib";
import {CubeSensor} from "../../web/src/sim/cube_sensor.js";
import {readJson} from "../../web/test/helpers.js";

const {values}=readJson("public/robot/vector2/model.json");
const steps=values.cube_tick_ms/values.cube_accel_frame_period_ms;
if (!Number.isInteger(steps)) throw Error("Replay requires whole sensor frames per app tick");
const lines=gunzipSync(readFileSync(new URL("./cube.csv.gz",import.meta.url))).toString().trim().split("\n").slice(1);
console.log("trial,mode,packet_s,slot");
for (const mode of ["hold","linear"]) {
  let name,chip,previous;
  for (const line of lines) {
    const [trial,time,slot,x,y,z]=line.split(",");
    const current=[+x,+y,+z];
    if(trial!==name) {
      name=trial;
      chip=new CubeSensor(values,.002,1,[0,0,1]);
      previous=current;
      chip.previousZ=current[2]; // Capture started after the cube had powered up.
    }
    const before=chip.taps;
    for(let step=1;step<=steps;step++) {
      const frame=mode==="linear" ? current.map((v,k)=>Math.round(previous[k]+(v-previous[k])*step/steps)) :
        (step===steps ? current : previous);
      chip.store(frame);
    }
    chip.buffered=0; // The per-tick FIFO was drained.
    if(chip.taps!==before) console.log([name,mode,+time,+slot].join(","));
    previous=current;
  }
}
