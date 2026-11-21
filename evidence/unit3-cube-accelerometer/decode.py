"""Decode stock-format notifications from the robot's Bluetooth daemon."""
import csv, struct, sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

def decode(path):
    packets=[]
    battery=[]
    for line in Path(path).read_text().splitlines():
        host, uptime, payload=line.split()
        data=bytes.fromhex(payload)
        if data[0]==2 and len(data)==20:
            packets.append((int(host),float(uptime),data[1],data[2:]))
        elif data[0]==4 and len(data)==4:
            battery.append(struct.unpack_from('<H',data,2)[0]*3.6/1024)
    return packets,battery

def main():
    fig,axes=plt.subplots(len(sys.argv)-1,1,figsize=(11,3*(len(sys.argv)-1)),squeeze=False)
    for ax,name in zip(axes[:,0],sys.argv[1:]):
        pkt,volts=decode(name)
        assert len(pkt)>1
        series=[]
        for host,up,taps,data in pkt:
            for i,axes_raw in enumerate(struct.iter_unpack('<3h',data)):
                series.append([host,up,i,taps,*axes_raw])
        with Path(name).with_suffix('.csv').open('w') as out:
            w=csv.writer(out,lineterminator='\n')
            w.writerow(['pod_rx_unix_ns','robot_rx_uptime_s','slot','tap_count','x_counts','y_counts','z_counts'])
            w.writerows(series)
        rows=np.array(series)
        # Place the three slots at estimated relative tick intervals before receipt.
        # Arrival time is retained separately; no absolute shutter-clock claim.
        dt=np.median(np.diff([p[1] for p in pkt]))/3
        t=rows[:,1]-rows[0,1]+(rows[:,2]-2)*dt
        g=rows[:,-3:]/8192
        ax.plot(t,g,lw=.7,label=['Cube X','Cube Y','Cube Z'])
        ax.set(ylabel='Specific force (g)',xlabel='Seconds from first packet',title=Path(name).stem)
        ax.legend(ncol=3)
        print(name,len(pkt),len(rows),np.round(g.mean(0),4),'mean norm',np.linalg.norm(g,axis=1).mean(),'peak norm',np.linalg.norm(g,axis=1).max(),'battery',min(volts) if volts else None,'max packet gap',np.diff([p[1] for p in pkt]).max())
    fig.tight_layout()
    fig.savefig(Path(__file__).with_name('cube-xyz.png'),dpi=150)

if __name__=='__main__':main()
