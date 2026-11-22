"""Compare direct BLE acceleration packet gaps with voltage notifications."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
period=1.22746  # Fixed from the robot's independent rest capture (R9/receipt_gaps.py).


def main():
    fig,axes=plt.subplots(3,1,figsize=(9,5.5),layout='constrained',sharex=True)
    for ax,name in zip(axes,['led-accel','led-accel-repeat','dark-direct']):
        rows=list(csv.DictReader((here/(name+'.csv')).open()))
        def times(tag):
            return np.array([float(r['sec']) for r in rows if r['kind']=='packet' and r['value'].startswith(tag)])
        accel,voltage=times('02'),times('04')
        delta=np.diff(accel)
        stop=accel[1:][delta>.15]
        width=delta[delta>.15]
        expected=stop[0]+period*np.arange(len(stop))
        ax.broken_barh(list(zip(stop-width,width)),(.1,.8),color='#929fae',label='Acceleration receipt gap')
        ax.vlines(voltage,0,1,color='#d96d25',label='Voltage receipt')
        ax.scatter(expected,np.full(len(expected),.5),marker='x',color='black',label='Predicted return')
        ax.set(yticks=[],ylabel=name)
        interior=voltage[(voltage>=stop[0]-width[0])&(voltage<=accel[-1])]
        nearest=np.min(np.abs(interior[:,None]-stop),axis=1) if len(interior) else []
        print(name,len(accel),len(stop),np.round(1000*np.median(width),1),
              'median/max prediction error ms',np.round(1000*np.array([np.median(abs(expected[1:]-stop[1:])),max(abs(expected-stop))]),1),
              'nearby rail reports ms',np.round(1000*np.array(nearest),1))
    axes[-1].set_xlabel('Seconds on direct receiver clock')
    axes[0].legend(ncol=3,fontsize=8)
    fig.savefig(here/'gap-voltage.png',dpi=160)


if __name__=='__main__':
    main()
