// node export-shell.mjs /absolute/path/to/web > shell.json
import { resolve, join } from 'node:path';
import { pathToFileURL } from 'node:url';

const web = resolve(process.argv[2]);
const fromWeb = relative => import(pathToFileURL(join(web, relative)).href);
const { Group } = await fromWeb('node_modules/three/build/three.module.js');
const { addCrownAndBackpack } = await fromWeb('src/game/topology.js');
const body = new Group();
addCrownAndBackpack(new Group(), body, 2);
const positions = body.getObjectByName('Backpack and rear cover').geometry.attributes.position;
console.log(JSON.stringify(Array.from(positions.array)));
