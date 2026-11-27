import {readFileSync,writeFileSync} from "node:fs";
import {gunzipSync} from "node:zlib";
import {SysconFirmware} from "../../../web/src/sim/syscon_firmware.js";
import {makeSyntheticSysconFlash,readJson} from "../../../web/test/helpers.js";

const firmware=new SysconFirmware(makeSyntheticSysconFlash());
const {values:v}=readJson("public/robot/vector2/model.json");
const sceneMax=Math.max(v.tof_lift_quality,v.tof_quality_at_100mm*(v.tof_report_min_mm/100)**v.tof_quality_distance_exponent,v.tof_quality_floor);
const out=["run,reports,above_old_bridge_limit,above_scene_max,max_physical_quality,max_new_error,range_mismatches,threshold_mismatches,physical_above_threshold"];
for (const run of "ABCDEF") {
  const file=new URL("../"+run+"/range.csv.gz",import.meta.url);
  const rows=gunzipSync(readFileSync(file)).toString().trim().split(/\r?\n/).slice(1);
  let count=0,clipped=0,beyond=0,maxQ=0,maxErr=0,distMismatch=0,thrMismatch=0,physicalStrong=0;
  for (const row of rows) {
    const [,range,signal]=row.split(",");
    if (!range || !signal) continue;
    const raw=Number(range), quality=Number(signal);
    const bytes=firmware.packBodyFrame({proxSample:{raw,quality,count}});
    const packet=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);
    const q=2*packet.getUint16(88)/packet.getUint16(92);
    count++;
    clipped+=quality>65535/(90*128);
    beyond+=quality>sceneMax;
    maxQ=Math.max(quality,maxQ);
    maxErr=Math.max(maxErr,Math.abs(q-quality));
    distMismatch+=raw!==packet.getUint16(86);
    thrMismatch+=(q>v.tof_quality_min)!==(quality>v.tof_quality_min);
    physicalStrong+=quality>v.tof_quality_min;
  }
  out.push([run,count,clipped,beyond,maxQ,maxErr,distMismatch,thrMismatch,physicalStrong].join(","));
}
writeFileSync(new URL("scores.csv",import.meta.url),out.join("\n")+"\n");
console.log("Maximum canonical optical quality",sceneMax);console.log(out.join("\n"));
