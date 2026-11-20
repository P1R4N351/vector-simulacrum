// Moulded links in the SDK's centimetre frame. The shoulder and wrist
// centres come from the rig. The intermediate sections follow the photos.
import * as THREE from "three";

function linkSurface(stations) {
  const centre = new THREE.CatmullRomCurve3(stations.map(([x, y, z]) => new THREE.Vector3(x, y, z)));
  const radii = new THREE.CatmullRomCurve3(stations.map((s, i) => new THREE.Vector3(i, s[3], s[4])));
  const vertices = [], indices = [];
  const rings = 56, sides = 24;
  for (let i = 0; i <= rings; i++) {
    const t = i / rings, p = centre.getPoint(t), tangent = centre.getTangent(t);
    const up = new THREE.Vector3(-tangent.z, 0, tangent.x).normalize();
    const across = up.clone().cross(tangent).normalize();
    const size = radii.getPoint(t);
    for (let j = 0; j < sides; j++) {
      const a = 2 * Math.PI * j / sides;
      const y = Math.cos(a), z = Math.sin(a);
      const point = p.clone().addScaledVector(across, size.y * Math.sign(y) * Math.abs(y) ** 0.50)
        .addScaledVector(up, size.z * Math.sign(z) * Math.abs(z) ** 0.68);
      vertices.push(...point.toArray());
      if (i) {
        const k = i * sides + j, next = i * sides + (j + 1) % sides;
        indices.push(k - sides, next - sides, k, next - sides, next, k);
      }
    }
  }
  for (let j = 1; j < sides - 1; j++) {
    indices.push(0, j + 1, j, rings * sides, rings * sides + j, rings * sides + j + 1);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

export function liftLink(side, pivot, wrist, lower) {
  const link = new THREE.Group();
  const [x, z] = pivot;
  const endX = wrist[0], endZ = wrist[1];
  const section = (t, y, rise, halfWidth, halfHeight) =>
    [x + t * (endX - x), y * side, z + t * (endZ - z) + rise, halfWidth, halfHeight];
  const rail = new THREE.Mesh(linkSurface([
    // A rolled end encloses each pin. The broad outside cheek blends
    // into a narrower shank before the two links bend toward the fork.
    section(-0.065, 2.32, -0.01, 0.045, 0.055),
    section(-0.051, 2.33, -0.005, 0.17, 0.23),
    section(-0.025, 2.34, 0, 0.235, lower ? 0.37 : 0.32),
    section(0.025, 2.35, 0.035, 0.245, lower ? 0.44 : 0.37),
    section(0.19, 2.35, lower ? 0.06 : 0.14, 0.23, lower ? 0.425 : 0.385),
    section(0.34, 2.35, lower ? 0.24 : 0.52, 0.225, lower ? 0.39 : 0.36),
    section(0.56, 2.34, lower ? 0.84 : 0.88, 0.22, 0.37),
    section(0.72, 2.31, lower ? 1.16 : 1.04, 0.218, lower ? 0.36 : 0.34),
    section(0.79, 2.27, lower ? 1.05 : 0.99, 0.215, 0.34),
    section(0.87, 2.16, 0.72, 0.205, 0.315),
    section(0.97, 2.015, 0.17, 0.19, 0.295),
    section(1, 2.005, 0, 0.18, 0.28),
  ]), new THREE.MeshStandardMaterial({color: 0x66676a, roughness: 0.40}));
  rail.castShadow = true;
  link.add(rail);

  // Revolve a stepped bearing with a radiused outer shoulder. The seat
  // stays inside the rear housing and the rail overlaps its narrower neck.
  const radius = lower ? 0.49 : 0.43;
  const profile = [[0, 1.81], [radius - 0.035, 1.81], [radius, 1.86],
    [radius, 2.17], [radius - 0.012, 2.20], [radius - 0.05, 2.23],
    [radius - 0.09, 2.23], [radius - 0.09, 2.31], [0, 2.31]];
  const bearing = new THREE.Mesh(
    new THREE.LatheGeometry(profile.map(([r, y]) => new THREE.Vector2(r, y)), 48),
    new THREE.MeshStandardMaterial({color: 0x353639, roughness: 0.44}));
  bearing.position.set(x, 0, z);
  if (side < 0) bearing.rotation.x = Math.PI;
  link.add(bearing);
  return link;
}
