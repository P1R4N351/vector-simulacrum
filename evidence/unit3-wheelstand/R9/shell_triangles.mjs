// Export viewer triangles into the replay's chassis-local metre frame.
import fs from 'node:fs';
import { OBJLoader } from '../../../web/node_modules/three/examples/jsm/loaders/OBJLoader.js';
import { frontApron } from '../../../web/src/game/shell.js';
const folder = new URL('../../../web/public/robot/sdk/', import.meta.url);
const obj = new OBJLoader().parse(fs.readFileSync(new URL('vector.obj',folder),'utf8'));
const wheel = obj.getObjectByName('wheel_FL_geo').geometry;
wheel.computeBoundingBox();
const axle = (wheel.boundingBox.min.x + wheel.boundingBox.max.x)/2;
const output = {};
for(const [name,mesh] of [...['body_geo','trackBase_L_geo','trackBase_R_geo'].map(k=>[k,obj.getObjectByName(k)]),['front apron',frontApron()]]){
  const geom = mesh.geometry.index ? mesh.geometry.toNonIndexed() : mesh.geometry;
  const p = geom.attributes.position;
  output[name] = Array.from({length:p.count},(_,i)=>[(p.getX(i)-axle)*.01,p.getY(i)*.01,p.getZ(i)*.01]);
}
console.log(JSON.stringify(output));
