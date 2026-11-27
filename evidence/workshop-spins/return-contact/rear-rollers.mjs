// Experimental contact geometry for the first-generation web model.
export function rearRollers(source, mode) {
  if (!['welded', 'passive', 'driven'].includes(mode)) throw Error('Unknown roller mode');
  let xml = source;
  for (const side of ['l','r']) {
    const name = 'tip_rear_'+side;
    const geom = xml.match(new RegExp('<geom name="'+name+'"[^>]*?/>'))?.[0];
    if (!geom) throw Error('Missing '+name);
    const radius = Number(geom.match(/size="([^"]+)"/)[1]);
    const ends = geom.match(/fromto="([^"]+)"/)[1].split(' ').map(Number);
    const center = ends.slice(0,3).map((v,i)=>(v+ends[i+3])/2);
    const local = ends.map((v,i)=>v-center[i%3]);
    const joint = name+'_joint';
    const shape = geom.replace(/fromto="[^"]+"/,'fromto="'+local.join(' ')+'"')
                      .replace('mass="0"','mass="0.00005"');
    const hinge = mode === 'welded' ? '' : '<joint name="'+joint+'" type="hinge" axis="0 1 0"/>';
    xml = xml.replace(geom,'<body pos="'+center.join(' ')+'">'+hinge+shape+'</body>');
    if (mode !== 'driven') continue;
    const coupled = mode === 'driven' ? ' joint2="wheel_f'+side+'_joint"' : '';
    const gain = mode === 'driven' ? .0145/radius : 0;
    xml = xml.replace('</equality>','<joint joint1="'+joint+'"'+coupled+
      ' polycoef="0 '+gain+' 0 0 0" solref="0.002 1"/></equality>');
  }
  return xml;
}
