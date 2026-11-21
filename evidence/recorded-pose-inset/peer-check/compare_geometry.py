"""Cross the two tread implementations at identical nominal contact widths and scoring."""
import argparse
from contextlib import contextmanager, ExitStack
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
from unittest.mock import patch

import numpy as np
import c7_transfer as c7
import fit_p2 as p2
import contact
from simulacrum import mjcf, recording, replay, params

@contextmanager
def shifted_model(inset, shifted, multiccd):
    def build(v, obstacles):
        xml = shifted.build_mjcf(dict(v, tread_contact_inset_mm=inset, tip_support_clearance_mm=.3), obstacles)
        return xml.replace('multiccd="enable"', 'multiccd="disable"') if multiccd == 'off' else xml
    with ExitStack() as stack:
        stack.enter_context(patch.object(mjcf, 'build_mjcf', build))
        stack.enter_context(patch.object(p2, 'contact_xml', lambda xml, _mm: xml))
        stack.enter_context(patch.object(contact, 'contact_xml', lambda xml, _mm: xml))
        yield

def run(p2_data,c7_data,outdir,shifted,swap_solver):
    runs,_,p2_actual=p2.read_inputs(p2_data)
    scores=[]
    def record(robot,spin,impl,inset,yaw,roll,pitch,pivot):
        scores.append(dict(robot=robot,spin=spin,implementation=impl,inset_mm=inset,
                           yaw_rad_s=yaw,roll_rad_s=roll,pitch_rad_s=pitch,center_mm=pivot))
    for m in p2_actual:
        record('unit 3',m['spin'],'recorded','',m['yaw'],m['roll'],m['pitch'],m['center_mm'])
    values=params.values(params.load(model='vector_1'))
    states,commands=recording.load(c7_data)
    spins=[s for s in recording.segments(commands,values).values() if s.left*s.right<0]
    for seg in spins:
        rows=[c7.data_row(r['t']-seg.t_cmd,r['head'],r['gyro'],r['lift'],r['wl'],r['wr'],
                          *r['pose'][:2],r['yaw'],values['imu_pitch_offset_deg']) for r in states]
        m=c7.score(rows,seg.t_stop-seg.t_cmd,'unit 1')
        record('unit 1',seg.label,'recorded','',m['mean_abs_yaw_rad_s'],m['mean_abs_roll_rad_s'],
               m['mean_abs_pitch_rad_s'],None)
    for implementation in ('roller','shifted'):
        for inset in (3.,3.75):
            with ExitStack() as stack:
                if implementation=='shifted':
                    stack.enter_context(shifted_model(inset,shifted,'off' if swap_solver else 'on'))
                elif swap_solver:
                    original=contact.contact_xml
                    def roller_with_multiccd(xml,mm):
                        return original(xml,mm).replace('multiccd="disable"', 'multiccd="enable"')
                    stack.enter_context(patch.object(contact,'contact_xml',roller_with_multiccd))
                    stack.enter_context(patch.object(p2,'contact_xml',roller_with_multiccd))
                for run in runs:
                    _,m=p2.simulated(inset,*run)
                    record('unit 3',m['spin'],implementation,inset,m['yaw'],m['roll'],m['pitch'],m['center_mm'])
                for seg in spins:
                    rows=c7.simulate(values,replay._state_at(states,seg.t_cmd),seg,inset)
                    m=c7.score(rows,seg.t_stop-seg.t_cmd,implementation)
                    record('unit 1',seg.label,implementation,inset,m['mean_abs_yaw_rad_s'],m['mean_abs_roll_rad_s'],
                           m['mean_abs_pitch_rad_s'],m['simulation_icr_x_mm'])
    prefix='swapped_multiccd' if swap_solver else 'crossed'
    outdir.mkdir(exist_ok=True,parents=True)
    c7.write_csv(outdir/f'{prefix}_geometry.csv',scores)
    aggregates=[]
    for robot in ('unit 3','unit 1'):
        obs={r['spin']:r for r in scores if r['robot']==robot and r['implementation']=='recorded'}
        for impl in ('roller','shifted'):
            for inset in (3.,3.75):
                rows=[r for r in scores if r['robot']==robot and r['implementation']==impl and r['inset_mm']==inset]
                errors=[r['yaw_rad_s']/obs[r['spin']]['yaw_rad_s']-1 for r in rows]
                aggregates.append(dict(robot=robot,implementation=impl,inset_mm=inset,
                    rms_yaw_error_pct=100*np.sqrt(np.mean(np.square(errors))),
                    mean_signed_yaw_error_pct=100*np.mean(errors),
                    **{k: np.mean([r[k] for r in rows]) for k in ('roll_rad_s','pitch_rad_s','center_mm')}))
    c7.write_csv(outdir/f'{prefix}_summary.csv',aggregates)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('p2_data',type=Path)
    p.add_argument('c7_data',type=Path)
    p.add_argument('out',type=Path)
    p.add_argument('--swap-multiccd',action='store_true',help='cross solver flag too: roller on, shifted off')
    args=p.parse_args()
    with tempfile.TemporaryDirectory() as temp:
        dest=Path(temp)/'mjcf.py'
        shutil.copyfile(mjcf.__file__,dest)
        with Path(__file__).with_name('shifted.patch').open() as patch_file:
            subprocess.run(['patch','-s',str(dest)],stdin=patch_file,check=True)
        spec=importlib.util.spec_from_file_location('simulacrum.shifted_mjcf',dest)
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        run(args.p2_data,args.c7_data,args.out,module,args.swap_multiccd)

if __name__=='__main__': main()
