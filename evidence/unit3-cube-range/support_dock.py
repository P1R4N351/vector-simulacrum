"""Check the existing four-support option on the five recorded backing approaches."""
import csv
import io
import json
import subprocess
import tempfile

import numpy as np

from support_transfer import HERE, ROOT, mjcf, params
from recorded_mount import read_trial, table


def main():
    v = params.values(params.load(model='vector_2'))
    base = mjcf.build_mjcf(v, (), cube='free', dock='scene')
    assert base.strip() == (ROOT/'web/public/robot/vector2/vector-free-dock.xml').read_text().strip()
    xml = mjcf.build_mjcf(dict(v, tip_supports_driven=1), (), cube='free', dock='scene')
    trials = [read_trial(c,o)[0] for c,o in zip(table('return-scores.csv'),table('mount-target-inputs.csv'),strict=True)]
    with tempfile.NamedTemporaryFile(suffix='.xml',mode='w') as f:
        f.write(xml)
        f.flush()
        data = subprocess.check_output(['node',str(HERE/'recorded_mount.mjs'),f.name],input=json.dumps(trials),text=True)
    states = np.genfromtxt(io.StringIO(data),names=True,delimiter=',',dtype=None,encoding='utf8')
    with (HERE/'support-dock.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow('session first_contact_s outward_at_real_contact_mm lateral_at_real_contact_mm'.split())
        for trial in trials:
            r = states[states['session']==trial['name']]
            hit = r['t_s'][r['on']==1]
            w.writerow([trial['name'],hit[0] if len(hit) else '',np.interp(0,r['t_s'],r['out_mm']),
                        np.interp(0,r['t_s'],r['lat_mm'])])


if __name__=='__main__':
    main()
