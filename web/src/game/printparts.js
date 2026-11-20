// Dauler's physical replacement parts. The source STLs are in print-bed
// coordinates, in mm. Return geometry in the SDK holder's centimetres.
import * as THREE from "three";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

async function readPart(name) {
  const response = await fetch(`./robot/sdk/print/${name}.stl`);
  if (!response.ok || response.headers.get("content-type")?.includes("text/html")) return null;
  const geometry = new STLLoader().parse(await response.arrayBuffer());
  geometry.scale(0.1, 0.1, 0.1);
  return geometry;
}

export async function loadPrintParts() {
  const names = ["wheel_front_stock", "wheel_back_stock", "forks7"];
  return Promise.all(names.map(async name => {
    try {
      return await readPart(name);
    } catch (error) {
      console.warn(`Using SDK geometry for ${name}`, error);
      return null;
    }
  }));
}

export function outerWheel(geometry, faceY, material) {
  geometry.computeBoundingBox();
  const bounds = geometry.boundingBox;
  const center = bounds.getCenter(new THREE.Vector3());
  const cap = new THREE.Mesh(geometry, material);
  // The minimum print Z is the outside face. The clips project inboard.
  const side = Math.sign(faceY);
  cap.rotation.x = side * Math.PI / 2;
  cap.position.set(-center.x, faceY + side * bounds.min.z, -side * center.y);
  cap.castShadow = true;
  return cap;
}

export function printFork(geometry, values, material) {
  // Print X spans the fork. Positive print Y points up the linkage and
  // negative print Z points forward. Register the crossbar to its wrist.
  const transform = new THREE.Matrix4().set(
    0, 0, -1, 0.1 * (values.lift_front_wrt_wrist_mm - 26.52),
    -1, 0, 0, -4.801,
    0, 1, 0, 0.1 * (values.lift_xbar_top_wrt_wrist_mm - values.lift_xbar_height_mm + 36.79),
    0, 0, 0, 1,
  );
  const fork = new THREE.Mesh(geometry, material);
  fork.applyMatrix4(transform);
  fork.castShadow = true;
  return fork;
}
