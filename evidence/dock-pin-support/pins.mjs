export function addPins(xml, stiffness) {
  const pairs = [], bodies = [];
  const dock = xml.match(/<body name="dock" pos="([-0-9.]+) 0 0" quat="6.123234e-17 0 0 1"/);
  if (!dock) throw Error("Unexpected dock transform");
  for (const [side,sign] of [["l",1],["r",-1]]) {
    const geom = new RegExp('<geom name="dock_pin_'+side+'"[^>]+/>');
    if (!geom.test(xml)) throw Error("Missing dock pin");
    xml = xml.replace(geom, tag => {
      const [x,y,z] = tag.match(/pos="([^"]+)"/)[1].split(" ").map(Number);
      bodies.push(`<body name="spring_${side}" pos="${Number(dock[1])-x} ${-y} ${z}" quat="0 0 0 1">
        <joint name="pin_${side}" type="slide" axis="0 0 1" range="-0.004 0" stiffness="${stiffness}" damping="0.35"/>
        ${tag.replace(/pos="[^"]+"/,'pos="0 0 0" mass="0.0005"')}
      </body>`);
      return "";
    });
    xml = xml.replace('<geom name="chassis_geom"', `<geom name="pad_${side}" type="box" pos="-0.0355 ${sign*0.0099} 0.0012" size="0.0059 0.0028 0.0002" mass="0" contype="0" conaffinity="0"/>
      <geom name="chassis_geom"`);
    pairs.push(`<pair geom1="dock_pin_${side}" geom2="pad_${side}" condim="3" friction="0.3 0.3 0.005 0.0001 0.0001" solref="0.004 1"/>`);
  }
  return xml.replace("</worldbody>",bodies.join("")+"</worldbody>").replace("<equality>", "<contact>"+pairs.join("")+"</contact><equality>");
}
