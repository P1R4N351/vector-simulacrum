"""Mirror each steady-speed input at fixed posture and vary proxy traversal order."""
from dataclasses import replace
from pathlib import Path
import sys
from unittest.mock import patch
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path[:0] = [str(here.parent/'recorded-pose-inset'), str(here.parents[1])]
import fit_p2
from contact import contact_xml

runs, _, measured = fit_p2.read_inputs(here.parent/'camera-spins')
rows = []
for (segment, start, times, values), obs in zip(runs, measured):
    for shift in (0, 1):
        for order in ('original', 'reversed', 'balanced_heat'):
            def render(xml, inset):
                root = ET.fromstring(contact_xml(xml, inset, forward_shift_mm=shift))
                if order == 'reversed':
                    chassis = root.find(".//body[@name='chassis']")
                    patches = [b for b in chassis.findall('body') if b.get('name').startswith('patch_')]
                    assert len(patches) == 4
                    for body in patches:
                        chassis.remove(body)
                    chassis.extend(reversed(patches))
                return ET.tostring(root, encoding='unicode')

            lr = [np.copysign(obs[k], speed) for k, speed in zip(('wl', 'wr'), (segment.left, segment.right))]
            effective = dict(values, tread_contact_inset_mm=0)
            if order == 'balanced_heat':
                effective['tread_heat_loss_rate_per_s'] = [np.mean(values['tread_heat_loss_rate_per_s'])] * 2
            scores = {}
            for left, right in (lr, lr[::-1]):
                seg = replace(segment, left=left, right=right)
                with patch.object(fit_p2, 'contact_xml', render):
                    trace, score = fit_p2.simulated(
                        3.75, seg, start, times, effective)
                steady = [r for r in trace if .7 <= r['t_s'] <= 1.4]
                t = [r['t_s'] for r in steady]
                heading = np.unwrap([r['heading_rad'] for r in steady])
                score['pose_rate'] = abs(np.polyfit(t, heading, 1)[0])
                scores['ccw' if right > left else 'cw'] = score
            a, b = scores['ccw'], scores['cw']
            rows.append(dict(source=segment.label, shift_mm=shift, variant=order,
                             ccw_yaw_radps=a['yaw'], cw_yaw_radps=b['yaw'],
                             yaw_asymmetry_pct=200*(a['yaw']-b['yaw'])/(a['yaw']+b['yaw']),
                             pose_asymmetry_pct=200*(a['pose_rate']-b['pose_rate'])/(a['pose_rate']+b['pose_rate']),
                             max_mirror_wheel_delta_mmps=max(abs(a['wr']-b['wl']), abs(a['wl']-b['wr']))))
fit_p2.dump(here/'mirror-spins.csv', rows)
fig, axes = plt.subplots(1, 2, sharey=True, figsize=(8,3.5), layout='constrained')
for ax, shift in zip(axes,(0,1)):
    for order in ('original','reversed','balanced_heat'):
        group = [r for r in rows if r['shift_mm']==shift and r['variant']==order]
        ax.plot([r['source'] for r in group],[r['yaw_asymmetry_pct'] for r in group],'o-',label=order)
    ax.axhline(0,color='gray',lw=.7)
    ax.set(title=f'{shift} mm forward shift',xlabel='Source of posture and wheel targets')
axes[0].set_ylabel('CCW − CW yaw magnitude (% of mean)')
axes[1].legend(title='Replay variant')
fig.savefig(here/'mirror-spins.png',dpi=160)

for shift in (0, 1):
    for variant in ('original', 'balanced_heat'):
        obs = {m['spin']: m['yaw'] for m in measured}
        errors = [100*(r[r['source'].split('_')[0] + '_yaw_radps']/obs[r['source']]-1)
                  for r in rows if r['shift_mm']==shift and r['variant']==variant and r['source'].endswith('_2')]
        print(shift, variant, 'held-out measured yaw RMS (%)', np.sqrt(np.mean(np.square(errors))))
