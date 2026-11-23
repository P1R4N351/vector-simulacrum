// Pin offsets for this dock's two rear-facing pads, expressed in its local frame.
export function pinOffsets(sim) {
  const {xpos,xmat,site_xpos}=sim.data;
  const c=xmat.subarray(9*sim._chassisBody),d=xmat.subarray(9*sim._dockBody);
  const sideSign=c[1]*d[1]+c[4]*d[4]+c[7]*d[7]>0?-1:1;
  return Array.from(sim._dockPins,(pin,side)=>{
    const y=(side===0?sideSign:-sideSign)*9.90;
    const delta=[0,1,2].map(i=>1000*(xpos[3*sim._chassisBody+i]-site_xpos[3*pin+i])
      -35.5*c[3*i]+y*c[3*i+1]+c[3*i+2]);
    return [0,1,2].map(i=>delta[0]*d[i]+delta[1]*d[3+i]+delta[2]*d[6+i]);
  });
}
