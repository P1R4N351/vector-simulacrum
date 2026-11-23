from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parent
def read(name):
    return np.genfromtxt(root/name,delimiter=',',names=True,dtype=None,encoding='utf8')
sdk,ble,sim=map(read,('sdk-taps.csv','counter-uptime.csv','replayed-taps.csv'))
labels=[];lag=[];stamp=[];fixed=145.906
for trial in ('W33','W34','W35'):
    a,b,c=[r[r['trial']==trial] for r in (sdk,ble,sim)]
    assert len(a)==len(b)==len(c) and np.array_equal(b['tap_count'],c['tap_count'])
    assert np.allclose(b['packet_s'],c['packet_s'],rtol=0,atol=.000002)
    # Both event streams are ordered. Short match gate prevents counting remote events.
    delays=(a['sdk_receipt_s']-c['packet_s'])*1000
    assert np.all(np.abs(delays-fixed)<100)
    lag.extend(delays);labels.extend(f'{trial}.{i+1}' for i in range(len(a)))
    stamp.extend(a['sdk_stamp_ms']-1000*b['robot_uptime_s'])
residual=np.array(lag)-fixed
fig,axes=plt.subplots(2,1,figsize=(8,5.3),sharex=True,layout='constrained')
axes[0].scatter(labels,lag,label='SDK callback receipt minus packet receipt')
axes[0].axhline(fixed,ls='--',color='tab:orange',label='Earlier 145.906 ms estimate')
axes[0].set_ylabel('Receiver delay (ms)');axes[0].legend()
axes[1].scatter(labels,stamp,color='tab:green',label='SDK event stamp minus packet uptime')
axes[1].axhline(0,color='gray',lw=.8);axes[1].set_ylabel('Robot-clock difference (ms)');axes[1].legend()
fig.savefig(root/'tap-timing.png',dpi=160)
print('receiver delays',lag,'residual MAE',np.mean(np.abs(residual)),'max',max(abs(residual)),'stamp',stamp)
