// Export convex strips of the viewer's photo-derived links, relative to each rear pin.
//   node export_links.mjs <shoulder x mm> <shoulder z mm> <arm length mm> [sdk folder]
// The SDK folder holds vector.obj and manifest.json; it defaults to web/public/robot/sdk/.
import fs from "node:fs";
import path from "node:path";
import { pathToFileURL } from "node:url";
import { liftLink, lowerLinkPivot } from "../../web/src/game/liftlinks.js";
import { OBJLoader } from "../../web/node_modules/three/examples/jsm/loaders/OBJLoader.js";
const sdk = process.argv[5] ? pathToFileURL(path.resolve(process.argv[5]) + path.sep)
  : new URL("../../web/public/robot/sdk/",import.meta.url);
const groups = new OBJLoader().parse(fs.readFileSync(new URL("vector.obj",sdk),"utf8"));
const wheel=groups.getObjectByName("wheel_FL_geo").geometry;
wheel.computeBoundingBox();
const axle=(wheel.boundingBox.min.x+wheel.boundingBox.max.x)/2;
const spec=JSON.parse(fs.readFileSync(new URL("manifest.json",sdk))).viewer_pose;
const [ux,uz,length]=process.argv.slice(2,5).map(Number);
if(![ux,uz,length].every(Number.isFinite))throw Error("usage: export_links.mjs <shoulder x mm> <shoulder z mm> <arm length mm> [sdk folder]");
const angle=spec.lift_angle_in_default_pose_rad;
const links=[];
const shoulder=[ux/10+axle,uz/10];
for(const lower of [false,true]){
  const [x,z]=lower?lowerLinkPivot(shoulder,spec):shoulder;
  const end=[x+length/10*Math.cos(angle),z+length/10*Math.sin(angle)];
  const strips=[];
  for(const side of [-1,1]){
    const geom=liftLink(side,[x,z],end,lower).children[0].geometry;
    const p=geom.getAttribute("position");
    if(p.count!==57*24)throw Error("Update strip extraction for revised link topology");
    for(let i=0;i<56;i+=4){
      const vertices=[];
      for(let j=i*24;j<(i+5)*24;j++)
        vertices.push((p.getX(j)-x)*.01,p.getY(j)*.01,(p.getZ(j)-z)*.01);
      strips.push(vertices);
    }
  }
  links.push({pivot:[(x-axle)*10,z*10],strips});
}
console.log(JSON.stringify({angle,links}));
