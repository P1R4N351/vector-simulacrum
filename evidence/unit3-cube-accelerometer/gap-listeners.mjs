// Effects of receipt-clock gaps and consecutive identical decoded reports.
import assert from "node:assert/strict";
import { readFileSync, readdirSync, writeFileSync } from "node:fs";
import { gunzipSync } from "node:zlib";
import { fileURLToPath } from "node:url";
import { join } from "node:path";
import { CubeListeners } from "../../web/src/sim/cube_listeners.js";

const here=fileURLToPath(new URL(".",import.meta.url));
const resting=readFileSync(join(here,"resting.tsv"),"utf8").trim().split(/\r?\n/)
  .map(line=>line.split(/\s+/)).filter(([, ,p])=>p.startsWith("02") && p.length===40).map(r=>+r[1]);
const intervals=resting.slice(1).map((t,i)=>t-resting[i]);
const gaps=intervals.flatMap((t,i)=>t>.15?[i]:[]);
const mean=a=>a.reduce((s,x)=>s+x,0)/a.length;
const fast=mean(intervals.filter(t=>t<=.15)), slow=mean(intervals.filter(t=>t>.15));
const spacing=mean(gaps.slice(1).map((v,i)=>v-gaps[i]));
const root=join(here,"../unit3-wheelstand");
const changes=["run,mode,kind,packet,uptime,actual_elapsed_s,nominal_elapsed_s,forecast_elapsed_s,anchored_elapsed_s"];
const runs=["run,packets,adjacent_duplicates,raw_events,filtered_events,max_score_difference"];
for(const path of readdirSync(root,{recursive:true}).filter(p=>p.endsWith("/cube.csv.gz")).sort()){
  const run=path.slice(0,-"/cube.csv.gz".length);
  const rows=gunzipSync(readFileSync(join(root,path))).toString().trim().split(/\r?\n/).slice(1)
    .map(line=>line.split(",").map(Number));
  if(!rows.length) continue;
  assert.equal(rows.length%3,0);
  const all=new CubeListeners(), unique=new CubeListeners();
  let last="",duplicates=0,maxDelta=0;
  const totals=[0,0];
  let anchorIndex=null, anchorElapsed=0;
  for(let i=0;i<rows.length;i+=3){
    const group=rows.slice(i,i+3), n=i/3, t=group[0][1];
    assert.deepEqual(group.map(row=>row[2]),[0,1,2]);
    assert.ok(group.every(row=>row[1]===t && row[3]===group[0][3]));
    if(anchorIndex===null && i>0 && t-rows[i-3][1]>.15){
      anchorIndex=n; anchorElapsed=t-rows[0][1];
    }
    const packet={time_s:t,taps:group[0][3],slots:group.map(row=>row.slice(4,7))};
    const key=JSON.stringify([packet.taps,packet.slots]);
    if(key===last) duplicates++;
    for(const [j,events] of [all.receive(packet),key===last?[]:unique.receive(packet)].entries()){
      totals[j]+=events.length;
      for(const e of events) {
        const kind="moving" in e?(e.moving?"start":"stop"):"axis" in e?"axis"+e.axis:"tap";
        changes.push([run,j?"dedup":"all",kind,n,t,t-rows[0][1],n*.03,
          n*fast+Math.floor(n/spacing)*(slow-fast),
          anchorIndex===null?"":anchorElapsed+(n-anchorIndex)*fast+Math.floor((n-anchorIndex)/spacing)*(slow-fast)].join(","));
      }
    }
    maxDelta=Math.max(maxDelta,Math.abs(all.score-unique.score)); last=key;
  }
  runs.push([run,rows.length/3,duplicates,...totals,maxDelta].join(","));
}
writeFileSync(join(here,"gap-listener-events.csv"),changes.join("\n")+"\n");
writeFileSync(join(here,"gap-listener-runs.csv"),runs.join("\n")+"\n");
