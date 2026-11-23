export function rearEnvelope(xml,variant) {
  let candidate=xml.replace(/<geom name="tip_rear_[lr]"[^>]+\/>/g,geom=>{
    if(variant==="smooth_rear_guards")return geom.replace('priority="1"','priority="3"').replace(/friction="[^"]+"/,'friction="0.01 0.005 0.0001"');
    if(variant==="no_rear_guards")return geom.replace('contype="1"','contype="0"');
    return geom;
  });
  if(variant==="driven_rear_envelope"){
    for(const side of ["l","r"]){
      const name="tip_rear_"+side;
      const y=side==="l"?0.02395:-0.02395;
      candidate=candidate.replace(new RegExp('<geom name="'+name+'"[^>]+/>'), geom=>
        '<body pos="-0.048 '+y+' 0.01075"><joint name="'+name+'_belt" type="hinge" axis="0 1 0"/>'+
        geom.replace(/fromto="[^"]+"/,'fromto="0 -0.00575 0 0 0.00575 0"').replace('mass="0"','mass="0.000001"')+'</body>');
      candidate=candidate.replace("</equality>",'<joint joint1="'+name+'_belt" joint2="wheel_r'+side+'_joint" polycoef="0 1.028708 0 0 0" solref="0.002 1"/></equality>');
    }
  }
  return candidate;
}
