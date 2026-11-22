// A NavMap leaf has no object ID. Require a single cube-sized connected region
// beyond the fixed charger before using the map to place a free cube.
export function mappedCubeXY(navMap, side, dockLipX, seatedOffset) {
  if (navMap?.frame !== "seated_robot") return null;
  const leaves = navMap.latest_objects_and_visibility.filter(row => row[3] === 3);
  if (!leaves.length) return null;
  const angle = leaves[0][4];
  const c = Math.cos(angle), s = Math.sin(angle);
  const cells = leaves.map(([x, y, w]) => ({x, y, w, u: x*c+y*s, v: -x*s+y*c}));
  const remaining = new Set(cells);
  const candidates = [];
  for (const seed of remaining) {
    const component = [seed];
    remaining.delete(seed);
    for (const cell of component) {
      for (const other of remaining) {
        const dx = Math.abs(cell.u-other.u), dy = Math.abs(cell.v-other.v);
        const span = (cell.w+other.w)/2;
        // Exported centres are rounded to 0.01 mm. Corner-only contact is separate.
        if (Math.max(dx,dy) <= span+.03 && Math.min(dx,dy) < span-.03) {
          remaining.delete(other);
          component.push(other);
        }
      }
    }
    // This placement applies to the open shelf ahead of the ramp.
    if (component.some(a => a.x - a.w*(Math.abs(c)+Math.abs(s))/2 <= seatedOffset)) continue;
    const area = component.reduce((sum,a) => sum + a.w*a.w, 0);
    const resolution = Math.max(...component.map(a => a.w));
    if (Math.abs(Math.sqrt(area)-side) > resolution || resolution >= side/2) continue;
    candidates.push(["x","y"].map(axis =>
      component.reduce((sum,a) => sum+a[axis]*a.w*a.w, 0)/area));
  }
  if (candidates.length !== 1) return null;
  const [x,y] = candidates[0];
  return [dockLipX-seatedOffset+x,y];
}
