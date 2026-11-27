// Extend each sloped contact box 0.65 mm along both ends. Preserve slope and height.
export function closeSeams(xml) {
  let count = 0;
  const changed = xml.replace(/(<geom name="dock_(?:lip|apron|bed|pocket)"[^>]+?size=")([\d.]+)/g,
    (_, prefix, halfLength) => { count++; return prefix+(Number(halfLength)+.00065); });
  if (count !== 4) throw Error('Unexpected dock profile');
  return changed;
}
