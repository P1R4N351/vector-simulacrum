"""Rotate only the square tabletop to check sensitivity to the cylinder contact solver."""
import csv
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

from support_transfer import HERE, ROOT, read, run, support_models


def main():
    hybrid = read('hybrid-encoder-replay.csv')
    native = read('unit2-native-replay-input.csv')
    output = []
    for variant, xml in support_models():
        for angle in (0, 15, 30, 45):
            tree = ET.fromstring(xml)
            tree.find(".//geom[@name='table']").set('euler',f'0 0 {np.deg2rad(angle)}')
            for name in ('cw_2','ccw_2'):
                series = hybrid[hybrid['turn']==name]
                t = series['ms_from_last_edge']/1000
                wheels = np.column_stack([series['left_input_mmps'],series['right_input_mmps']])
                head = native[native['turn']==name]['head_rad'][0]
                loaded = 0
                peak_force = 0
                support_ids = None
                force = np.zeros(6)
                def observe(m, d, at):
                    nonlocal loaded, peak_force, support_ids
                    if support_ids is None:
                        support_ids = {m.geom('tip_rear_'+side).id for side in ('l','r')}
                    for i, ct in enumerate(d.contact):
                        if ct.geom1 in support_ids or ct.geom2 in support_ids:
                            mujoco.mj_contactForce(m, d, i, force)
                            if force[0] > 1e-6:
                                loaded += 1
                                peak_force = max(peak_force,force[0])
                clock,out = run([dict(head=head,lift=30.6450119)],t,wheels,None,model_xml=ET.tostring(tree),
                                observe=observe,dense=True)
                gyro = np.interp(t-.017,clock,out[:,0])
                err = gyro-series['measured_head_dps']
                rmse = [np.sqrt(np.mean(err[(t>=a)&(t<=b)]**2)) for a,b in [(-.85,-.25),(-.10,.10)]]
                output.append([variant,angle,name,*rmse,loaded,peak_force])
    with (HERE/'support-sensitivity.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow('variant table_deg turn steady_rmse_dps brake_rmse_dps loaded_contacts peak_normal_N'.split())
        w.writerows(output)


if __name__=='__main__':
    main()
