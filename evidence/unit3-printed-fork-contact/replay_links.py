"""Drive the photo-derived upper and lower lift links from the two rear pivots."""
import json
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from pathlib import Path
from functools import lru_cache


@lru_cache
def geometry(repo):
    from simulacrum import params
    v=params.values(params.load(model="vector_2"))
    x,_,z=v["lift_base_position_mm"]
    length=v["lift_arm_length_mm"]
    js=Path(__file__).with_name("export_links.mjs")
    model=json.loads(subprocess.check_output(["node",str(js),str(x),str(z),str(length)]))
    return model,z,length


def add_links(xml,repo):
    model,_,_=geometry(repo)
    assets=xml.find("asset")
    if assets is None:
        assets=ET.SubElement(xml.getroot(),"asset")
    for j,link in enumerate(model["links"]):
        body=ET.SubElement(xml.find("worldbody"),"body",name=f"rail{j}",mocap="true")
        for i,verts in enumerate(link["strips"]):
            name=f"rail{j}_{i}"
            ET.SubElement(assets,"mesh",name=name,vertex=" ".join(map(str,verts)))
            ET.SubElement(body,"geom",name=name,mesh=name,type="mesh",contype="0",conaffinity="4",
                          friction=".3 .005 .0001",priority="1",solref=".004 1")


def rail_paths(repo,x,y,h,l):
    model,z,length=geometry(repo)
    pitch=-(np.arcsin((l-z)/length)-model["angle"])
    c,s=np.cos(h/2),np.sin(h/2)
    cp,sp=np.cos(pitch/2),np.sin(pitch/2)
    quat=np.column_stack((c*cp,-s*sp,c*sp,s*cp))
    centers=[]
    for link in model["links"]:
        px,pz=link["pivot"]
        centers.append(.001*np.column_stack((x+px*np.cos(h),y+px*np.sin(h),np.full_like(x,pz))))
    return np.stack(centers,axis=1),quat


def require_printed_fork(tree):
    """Disabled stock geoms may accompany the fork's active convex meshes."""
    geoms = tree.findall(".//body[@name='fork']/geom")
    active = [g for g in geoms if int(g.get("contype", "1")) or int(g.get("conaffinity", "1"))]
    if not active or any(g.get("type") != "mesh" for g in active):
        raise ValueError("Expected a printed-fork fixture with active mesh collision pieces")
