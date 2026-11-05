"""Convex pad cells shared by CAD and the MuJoCo contact model; mm."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull

ROOT=Path(__file__).resolve().parents[1]

def cells(sections,back=-9.8,front=-12.3):
    out=[]
    def profiles(z,width,rim,front_override=None):
        facefront=front if front_override is None else front_override
        a=width/2;b=width*.2
        return [[(-a,back,z),(-b,back,z),(-b,facefront,z),(-a,facefront-rim,z)],
                [(-b,back,z),(b,back,z),(b,facefront,z),(-b,facefront,z)],
                [(b,back,z),(a,back,z),(a,facefront-rim,z),(b,facefront,z)]]
    for start,end in zip(sections,sections[1:]):
        aa,bb=profiles(*start),profiles(*end)
        for a,b in zip(aa,bb):
            vv=np.asarray(a+b,dtype=float);h=ConvexHull(vv);ff=[]
            for face,eq in zip(h.simplices,h.equations):
                face=face.copy();tr=vv[face]
                if np.dot(np.cross(tr[1]-tr[0],tr[2]-tr[0]),eq[:3])<0:face=face[[0,2,1]]
                ff.append(face.tolist())
            out.append({'vertices_mm':vv.tolist(),'faces':ff})
    return out

def solid(cell):
    import cadquery as cq
    vv=[cq.Vector(*v) for v in cell['vertices_mm']];faces=[]
    for inds in cell['faces']:
        pts=[vv[i] for i in inds];faces.append(cq.Face.makeFromWires(cq.Wire.makePolygon(pts+[pts[0]])))
    return cq.Solid.makeSolid(cq.Shell.makeShell(faces)).fix()

def pad(name,parameters):
    cfg=parameters['pads'];cc=cells(cfg[name+'_sections'],cfg['back_y'],cfg['center_front_y'])
    shapes=[solid(c) for c in cc];s=shapes[0]
    for other in shapes[1:]:s=s.fuse(other)
    return s.clean(),cc

def export_collision(parameters):
    data={}
    for short in ['proximal','distal']:
        cfg=parameters['pads'];data['pad_'+short]=cells(cfg[short+'_sections'],cfg['back_y'],cfg['center_front_y'])
    path=ROOT/'source/pad_collision.json';path.write_text(json.dumps(data,indent=2));return data
