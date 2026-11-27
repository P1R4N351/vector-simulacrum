// Release the stock simulated cube on its lower rubber edge with no fork contact.
import {mujoco,readText} from "../../../web/test/helpers.js";
const mj=await mujoco();
const scene=readText("public/robot/vector2/vector-free-dock.xml");
const rad=Math.PI/180;
console.log("step_ms,angle_deg,rate_dps,final_deg,settled_new_face_s");
for (const dt of [0.001,0.0005]) {
  const model=mj.MjModel.from_xml_string(scene.replace('timestep="0.001"',`timestep="${dt}"`));
  try {
    const id=(kind,name)=>mj.mj_name2id(model,mj.mjtObj[kind].value,name);
    const joint=id("mjOBJ_JOINT","cube_root"),body=id("mjOBJ_BODY","cube");
    const q=model.jnt_qposadr[joint],v=model.jnt_dofadr[joint];
    const corner=id("mjOBJ_GEOM","cube_rubber_0");
    const h=-model.geom_pos[3*corner+2]+model.geom_size[3*corner+2];
    for (const angle of [30,36,41.3,44,45,46,50]) {
      const a=angle*rad, height=h*(Math.cos(a)+Math.sin(a));
      const arm=h*(Math.cos(a)-Math.sin(a));
      for (let rate=0;rate<=180;rate+=10) {
        const data=new mj.MjData(model);
        try {
          data.qpos.set([0.15,0.12,height,Math.cos(a/2),Math.sin(a/2),0,0],q);
          const omega=rate*rad;
          data.qvel.set([0,-omega*height,omega*arm,omega,0,0],v);
          let lastOutside=0,tilt=angle;
          for(let k=0;k<Math.round(.8/dt);k++) {
            mj.mj_step(model,data);
            const R=data.xmat.subarray(9*body);
            tilt=Math.atan2(R[7],R[8])/rad;
            if(Math.abs(tilt-90)>2) lastOutside=data.time;
          }
          console.log([dt*1000,angle,rate,tilt,lastOutside<.75?lastOutside+dt:""].join(","));
        } finally {data.delete();}
      }
    }
  } finally {model.delete();}
}
