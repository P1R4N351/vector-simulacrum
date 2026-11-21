// Use the SDK's optical surface when installed; the collision boxes remain visible otherwise.
import * as THREE from "three";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";

export async function loadCubeLook(sizeMm) {
  const base = "./robot/sdk/";
  try {
    const manifest = await (await fetch(base + "manifest.json")).json();
    if (!manifest.sha256?.["cube.obj"] || !manifest.sha256?.["cube.jpg"]) return null;
    const [shape, texture] = await Promise.all([
      new OBJLoader().loadAsync(base + "cube.obj"),
      new THREE.TextureLoader().loadAsync(base + "cube.jpg"),
    ]);
    texture.colorSpace = THREE.SRGBColorSpace;
    const material = new THREE.MeshStandardMaterial({ map: texture, roughness: 0.62 });
    shape.traverse((node) => {
      if (node.isMesh) {
        node.material = material;
        node.castShadow = true;
        node.receiveShadow = true;
      }
    });
    const bounds = new THREE.Box3().setFromObject(shape);
    const size = bounds.getSize(new THREE.Vector3());
    shape.scale.set(sizeMm / 1000 / size.x, sizeMm / 1000 / size.y, sizeMm / 1000 / size.z);
    shape.position.copy(bounds.getCenter(new THREE.Vector3())).multiply(shape.scale).negate();
    return new THREE.Group().add(shape);
  } catch (error) {
    console.warn("Cube optical mesh unavailable; using contact boxes", error);
    return null;
  }
}
