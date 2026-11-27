// Feed the measured head-angle path into the WireOS packet producer.
import {readFileSync} from "node:fs";
import {WireBody} from "../../../web/src/sim/wire_body.js";
const data=readFileSync(new URL("active.csv",import.meta.url),"utf8").trim().split(/\r?\n/).slice(1)
  .map(line=>line.split(",")).filter(row=>row[1]==="state").map(row=>[Number(row[0]),Number(row[4])*Math.PI/180]);
const dt=.005, scale=Math.PI/(4*366.211), ticksPerSec=48000000*256;
let idx=0, angle=data[0][1], time=0;
const state={
  dt,data:{time:0},_treadTravelMm:()=>[0,0],headAngleRad:()=>angle,liftAngleRad:()=>0,
  cliffRangesMm:()=>[0,0,0,0],rawRangeSample:()=>({quality:0,raw:0,count:0}),
  _imu:()=>[[0,0,0],[0,0,0]],tickCount:0,batteryVolts:4,
  tickWireBody(){
    time=++this.tickCount*dt;
    const stamp=data[0][0]+time;
    while(idx<data.length-2 && data[idx+1][0]<stamp) idx++;
    const [t0,a]=data[idx], [t1,b]=data[idx+1];
    angle=a+(b-a)*Math.min(1,(stamp-t0)/(t1-t0));
    this.data.time=time;
  },
};
const body=new WireBody(state);
let prev=Math.round(angle/scale), idle=0, lastFrame=0, oldSpeed=0, newSpeed=0;
console.log("s,frame_clock_dps,edge_clock_dps");
while(data[0][0]+time+dt<=data.at(-1)[0]){
  const packet=new DataView(body.frame([0,0,0,0]).buffer);
  const pos=packet.getInt32(60,true);
  if(pos!==prev){
    oldSpeed=(pos-prev)*scale/((state.tickCount-lastFrame)*dt);
    newSpeed=packet.getInt32(64,true)*scale*ticksPerSec/packet.getUint32(68,true);
    lastFrame=state.tickCount;
    idle=0;
  }else if(++idle>=5){ oldSpeed=0; newSpeed=0; }
  console.log([data[0][0]+time,oldSpeed*180/Math.PI,newSpeed*180/Math.PI].join(","));
  prev=pos;
}
