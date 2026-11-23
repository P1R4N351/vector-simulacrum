"""Predict tap callback receipt from prior radio packets and the frozen pickup delay."""
import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent
delay = .145906


def read(path):
    with (gzip.open if path.suffix == '.gz' else open)(path, 'rt') as f:
        return list(csv.DictReader(f))


def score(label, packets, events):
    results = []
    for host, sdk in events:
        i = np.flatnonzero(np.isclose(packets[:,0],host,rtol=0,atol=1e-6)).item()
        # Median clock offset of earlier packets only. One-second window on
        # robot uptime, independent of host-side bunching and future arrivals.
        peers = packets[:i]
        peers = peers[peers[:,1] >= packets[i,1]-1]
        if len(peers) < 3:
            continue
        mapped = packets[i,1] + np.median(peers[:,0]-peers[:,1])
        results.append((label,host,len(peers),1000*(host-mapped),
                        1000*(host+delay-sdk),1000*(mapped+delay-sdk)))
    return results


def main():
    results = []
    matched = read(root/'sdk-tap-transfer.csv')
    for label in sorted({r['run'] for r in matched}):
        data = read(root.parent/'unit3-wheelstand'/label/'cube.csv.gz')
        packets = np.array([(float(r['sec']),float(r['uptime'])) for r in data if r['slot']=='0'])
        events = [(float(r['packet_host_s']),float(r['packet_host_s'])+float(r['sdk_host_offset_ms'])/1000)
                  for r in matched if r['run']==label and r['sdk_host_offset_ms']]
        results.extend(score(label,packets,events))
    records = read(root.parent/'unit3-docked-grip/release-events.csv')
    packets = np.array([(float(r['host_seconds']),float(r['robot_uptime_s'])) for r in records if r['kind']=='packet'])
    events = [(float(r['packet_host_s']),float(r['sdk_host_s']))
              for r in read(root.parent/'unit3-docked-grip/release-event-scores.csv') if r['kind']=='object_tapped']
    results.extend(score('Docked',packets,events))
    with (root/'tap-receipt-clock.csv').open('w') as f:
        w = csv.writer(f)
        w.writerow('run packet_host_s prior_packets receipt_jitter_ms arrival_error_ms uptime_error_ms'.split())
        w.writerows(results)
    fig,ax=plt.subplots(figsize=(9,3.8),layout='constrained')
    for i,r in enumerate(results):
        ax.plot([i,i],r[-2:],color='.75')
    for col,marker,name in [(-2,'o','Packet receipt + delay'),(-1,'x','Robot uptime mapped by prior packets + delay')]:
        ax.scatter(range(len(results)),[r[col] for r in results],marker=marker,label=name)
    ax.axhline(0,color='.4',lw=.8)
    ax.set(xticks=range(len(results)),xticklabels=[r[0] for r in results],ylabel='Predicted minus observed callback time (ms)')
    ax.tick_params(axis='x',rotation=55)
    ax.legend(fontsize=8)
    fig.savefig(root/'tap-receipt-clock.png',dpi=165)
    for label,subset in [('Wheelstand',results[:-len(events)]),('Docked',results[-len(events):])]:
        print(label,len(subset),np.median(np.abs(np.array([r[-2:] for r in subset])),axis=0),
              np.max(np.abs(np.array([r[-2:] for r in subset])),axis=0))


if __name__=='__main__':
    main()
