export function dockImpedance(xml,mode){
if(!["base","priority","dock","hinges","both"].includes(mode)) throw Error("Unknown mode");
if(["hinges","both"].includes(mode)){
  if(!xml.includes('frictionloss="0.024578125"')) throw Error('Expected v1/v2 drive hinge friction');
  xml=xml.replaceAll('frictionloss="0.024578125"', 'solimpfriction="0.999 0.999 0.001" frictionloss="0.024578125"');
}
if(["priority","dock","both"].includes(mode)){
  xml=xml.replace(/<geom name="dock_(?:bed|apron|lip|pocket|base)"[^>]*>/g,s=>{
    const priority=s.includes('priority=') ? s.replace(/priority="\d+"/,'priority="3"') : s.replace("<geom ",'<geom priority="3" ');
    return mode==="priority" ? priority : priority.replace("<geom ",'<geom solimp="0.999 0.999 0.001" ');
  });
}
return xml;
}
