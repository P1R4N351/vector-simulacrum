"""Replay both measured approaches with the current SDK-derived cube collision boxes."""
import csv
from copy import deepcopy
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np
from chassis_contact import replay, here, load_rows


def replace_boxes(fixture, dest):
    tree = ET.parse(fixture)
    body = tree.find(".//body[@name='cube']")
    materials = {}
    for geom in body.findall('geom'):
        material = geom.get('name').split('_')[1]
        materials[material] = geom
        body.remove(geom)
    surface = here.parents[2] / 'simulacrum/cube_surface.csv'
    with surface.open() as file:
        for index, row in enumerate(list(csv.reader(file))[1:]):
            material, *mm = row
            geom = deepcopy(materials[material])
            dims = [str(float(x)/1000) for x in mm]
            geom.attrib.update(name=f'cube_{material}_{index}', pos=' '.join(dims[:3]),
                               size=' '.join(dims[3:]))
            body.append(geom)
    tree.write(dest)


def main(fixture):
    fig, axes = plt.subplots(2, 2, figsize=(9, 5), layout='constrained')
    with TemporaryDirectory() as tmp, (here/'surface-transfer.csv').open('w') as file:
        revised = Path(tmp)/'revised.xml'
        replace_boxes(fixture, revised)
        writer = csv.writer(file, lineterminator='\n')
        writer.writerow(['run','surface','peak_deg','final_deg','peak_g','tap_changes'])
        for col, run in enumerate(['R8','R9']):
            for label, path in [('Original',fixture),('Revised',revised)]:
                scores, (t, tilt, stamps, accel) = replay((run,False,path))
                writer.writerow([run,label,*scores[4:]])
                axes[0,col].plot(t, tilt, label=label)
                axes[1,col].plot(stamps, accel[:,1], label=label)
            folder = here.parent/run
            data = np.genfromtxt(folder/'cube.csv.gz', names=True, delimiter=',')
            zero = next(float(r['sec']) for r in load_rows('actions.csv',folder) if r['action']=='roll cube')
            axes[1,col].plot(data['sec'][::3]-zero, data['y'].reshape(-1,3).mean(axis=1)/8192,
                             'k.', ms=2,label='Recorded')
            axes[0,col].set_title(run)
            for ax in axes[:,col]:
                ax.set_xlim(2.5,t[-1])
            axes[1,col].set_xlabel('Seconds after command')
    axes[0,0].set_ylabel('Simulated tilt (°)')
    axes[1,0].set_ylabel('Packet Y (g)')
    axes[1,1].legend()
    fig.savefig(here/'surface-transfer.png',dpi=160)


if __name__ == '__main__':
    main(sys.argv[1])
