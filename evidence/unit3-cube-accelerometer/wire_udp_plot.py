"""Score captured radio timing and the loopback WebSocket/UDP measurement."""
import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).parent
rows = np.genfromtxt(root/'wire-udp.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
fig,axes=plt.subplots(2,2,figsize=(10,6),layout='constrained')
with (root/'wire-udp-scores.csv').open('w') as out:
    w=csv.writer(out,lineterminator='\n')
    w.writerow(['run','reports','incorrect_reports','relay_p50_ms','relay_p95_ms','relay_p99_ms',
                'relay_max_ms','physical_over150ms','udp_over150ms','physical_span_s','udp_span_s'])
    for i,run in enumerate(('R5','R8')):
        a=rows[rows['run']==run]
        path=root.parent/'unit3-wheelstand'/run/'cube.csv.gz'
        with gzip.open(path,'rt') as f:
            uptime=np.array([float(r['uptime']) for r in csv.DictReader(f) if r['slot']=='2'])
        assert len(uptime)==len(a)
        physical=np.diff(uptime)*1000
        udp=np.diff(a['recv_last_ms'])
        latency=a['recv_last_ms']-a['send_ms']
        w.writerow([run,len(a),sum(a['max_word_error']!=0),*np.percentile(latency,[50,95,99,100]),
                    sum(physical>150),sum(udp>150),uptime[-1]-uptime[0],np.sum(udp)/1000])
        ax,delay=axes[i]
        ax.plot(a['report'][1:],physical,'.',ms=2,label='Physical robot receipt (uptime)')
        ax.plot(a['report'][1:],udp,'.',ms=2,label='Local UDP receipt')
        ax.set(title=run,xlabel='Report index',ylabel='Inter-report interval (ms)')
        delay.plot(a['report'],latency,'.',ms=2)
        delay.set(title=run,xlabel='Report index',ylabel='Send to third UDP datagram (ms)')
axes[0,0].legend(fontsize=8)
fig.savefig(root/'wire-udp.png',dpi=150)
