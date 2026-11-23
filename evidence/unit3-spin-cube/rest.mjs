import {CubeSensor} from "../../web/src/sim/cube_sensor.js";
import {readJson} from "../../web/test/helpers.js";
const {values} = readJson("public/robot/vector2/model.json");
const dt = values.sim_timestep_ms/1000;
console.log("seed,rms_g");
for(let seed=0;seed<16;seed++){
  const gravity=[0,0,1];
  const chip=new CubeSensor(values,dt,seed,gravity);
  const samples=[];
  for(let tick=1;tick*dt<=3;tick++){
    const previous=chip.packet;
    chip.step(tick*dt,gravity);
    if(tick*dt>=1 && chip.packet && chip.packet!==previous) samples.push(...chip.packet.slots);
  }
  const median=[0,1,2].map(axis=>samples.map(s=>s[axis]).sort((a,b)=>a-b).slice(Math.floor((samples.length-1)/2),Math.floor(samples.length/2)+1).reduce((a,b,_,v)=>a+b/v.length,0));
  const ms=samples.reduce((sum,s)=>sum+s.reduce((a,v,k)=>a+((v-median[k])/8192)**2,0),0)/samples.length;
  console.log([seed,Math.sqrt(ms)].join(","));
}
