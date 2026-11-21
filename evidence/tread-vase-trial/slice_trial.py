"""Bench slice Dauler's tread_oBar.stl. Writes files only."""
import argparse, os, subprocess
from pathlib import Path
import numpy as np
import trimesh

APP=Path('/Applications/UltiMaker Cura.app/Contents/Resources')
RES=APP/'share/cura/resources'

def run(source,out):
    out.mkdir(exist_ok=True,parents=True)
    tread=trimesh.load_mesh(source)
    tread.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]))
    assert tread.is_volume
    tread.apply_translation([-tread.bounds.mean(axis=0)[0],-tread.bounds.mean(axis=0)[1],-tread.bounds[0,2]])
    for slit in (0,.02,.05,.10,.20,.4):
        cut=trimesh.creation.box(extents=[40,max(.001,slit),20])
        cut.apply_translation([20,0,6])
        mesh=tread.difference(cut,engine='manifold') if slit else tread
        stem=out/f'slit-{slit:.2f}'
        mesh.export(str(stem)+'.stl')
        settings=dict(magic_spiralize='true',smooth_spiralized_contours='false',
                      wall_line_count=1,roofing_layer_count=0, flooring_layer_count=0,top_layers=0,bottom_layers=0,initial_bottom_layers=0,meshfix_union_all_remove_holes='false',infill_sparse_density=0,
                      adhesion_type='none',support_enable='false',layer_height=.2,layer_height_0=.2,
                      line_width=.4,wall_line_width_0=.4,wall_line_width=.4,initial_layer_line_width_factor=100,
                      machine_nozzle_size=.4,material_diameter=1.75,machine_gcode_flavor='RepRap (Marlin/Sprinter)',
                      speed_print=20,speed_wall_0=20,speed_layer_0=15, meshfix_union_all='true',
                      retraction_enable='false',material_print_temperature=220,material_bed_temperature=45,
                      machine_start_gcode='; Offline toolpath study',machine_end_gcode='; End of study')
        cmd=[str(APP/'CuraEngine'),'slice','-j',str(RES/'definitions/fdmprinter.def.json')]
        for k,v in settings.items(): cmd.extend(['-s',f'{k}={v}'])
        cmd+=['-e0','-j',str(RES/'definitions/fdmextruder.def.json')]
        for k,v in settings.items(): cmd.extend(['-s',f'{k}={v}'])
        cmd.extend(['-l',str(stem)+'.stl','-o',str(stem)+'.gcode'])
        env=dict(os.environ,CURA_ENGINE_SEARCH_PATH=os.pathsep.join(str(RES/p) for p in ['definitions','extruders']))
        with open(str(stem)+'.log','w') as log: subprocess.run(cmd,stdout=log,stderr=log,env=env,check=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('out',type=Path)
    a=p.parse_args();run(a.source,a.out)
