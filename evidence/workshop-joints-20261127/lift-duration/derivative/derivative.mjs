// Diagnostic alternatives for the derivative contribution; angles and offsets are radians.
export function addDerivative(sim, mode) {
  const writePI=sim._writeLift.bind(sim);
  let previousError=0;
  let previousTarget=sim._liftCmd;
  sim._writeLift=function(command) {
    const angle=this.data.qpos[this._q.lift];
    const error=command-angle;
    const [kp,ki,kd]=this.v.lift_power_gains;
    const range=5*Math.PI/180;
    const inLimitBand=[angle,command].every(a=>a>=this.d.lift_angle_min && a<=this.d.lift_angle_min+range) ||
      [angle,command].every(a=>a>=this.d.lift_angle_max-range && a<=this.d.lift_angle_max);
    const difference=mode==="error" ? error-previousError : command-previousTarget;
    const d=inLimitBand ? 0 : kd*this.dt*difference;
    const oldSum=this._liftErrorSum;
    writePI(command);
    this.data.ctrl[this._act.lift]+=d/kp;
    if(mode==="error" && command===this._liftTarget && Math.abs(error)<this.v.lift_angle_tol_deg*Math.PI/180){
      const power=kp*error+ki*oldSum+d;
      const decay=Math.abs(power)>this.v.lift_power_in_position_max ? Math.sign(power)*this.v.lift_error_sum_decay : 0;
      this._liftErrorSum=Math.max(-this.v.lift_error_sum_max,Math.min(this.v.lift_error_sum_max,oldSum-decay));
    }
    previousError=error;
    previousTarget=command;
  };
}
