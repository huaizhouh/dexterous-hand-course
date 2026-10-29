"""Independent trial2 B-rep authoring. All dimensions and exports are mm."""
from pathlib import Path
import hashlib
import json
import math
import numpy as np
from OCP.gp import gp_Pnt, gp_Dir, gp_Ax2, gp_Trsf, gp_Vec
from OCP.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE
from OCP.BRep import BRep_Builder
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeSphere, BRepPrimAPI_MakePrism
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse, BRepAlgoAPI_Cut, BRepAlgoAPI_Common
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from design import *

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent/'support_frames'/'金属舵机最小单元-双轴.STEP'
PARTS = HERE/'parts'; MESHES = HERE/'meshes'; REPORTS = HERE/'reports'

def collect(s, kind):
    e=TopExp_Explorer(s,kind); out=[]
    while e.More(): out.append(e.Current()); e.Next()
    return out

def solids(s): return [TopoDS.Solid_s(x) for x in collect(s,TopAbs_SOLID)]

def volume(s):
    g=GProp_GProps(); BRepGProp.VolumeProperties_s(s,g); return g.Mass()

def bounds(s):
    b=Bnd_Box(); BRepBndLib.AddOptimal_s(s,b,False,False); return list(b.Get())

def move(s,m):
    t=gp_Trsf(); t.SetValues(*[float(v) for v in m[:3].flat]); return BRepBuilderAPI_Transform(s,t,True).Shape()

def compound(shapes):
    b=BRep_Builder(); c=TopoDS_Compound(); b.MakeCompound(c)
    for s in shapes:b.Add(c,s)
    return c

def union(*shapes):
    a=shapes[0]
    for b in shapes[1:]:
        op=BRepAlgoAPI_Fuse(a,b); op.Build()
        if not op.IsDone(): raise RuntimeError('fuse failed')
        a=op.Shape()
    return a

def cut(a,*tools):
    for b in tools:
        op=BRepAlgoAPI_Cut(a,b);op.Build()
        if not op.IsDone():raise RuntimeError('cut failed')
        a=op.Shape()
    return a

def box(x,y,z,dx,dy,dz): return BRepPrimAPI_MakeBox(gp_Pnt(x,y,z),dx,dy,dz).Shape()

def cylinder(r,h,p,axis=(0,0,1)):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*p),gp_Dir(*axis)),r,h).Shape()

def tube(ro,ri,h,p,axis=(0,0,1)):return cut(cylinder(ro,h,p,axis),cylinder(ri,h,p,axis))

def sphere(r,p):return BRepPrimAPI_MakeSphere(gp_Pnt(*p),r).Shape()

def wire_solid(points,r):
    chunks=[]
    for a,b in zip(points,points[1:]):
        d=np.asarray(b)-a; L=float(np.linalg.norm(d))
        if L>1e-6:chunks.extend([cylinder(r,L,a,d/L),sphere(r,b)])
    return union(sphere(r,points[0]),*chunks)

def read_step(path):
    r=STEPControl_Reader()
    if r.ReadFile(str(path))!=IFSelect_RetDone:raise RuntimeError(f'cannot read {path}')
    r.TransferRoots();return r.OneShape()

def write_step(shape,path,single=False):
    old=Interface_Static.IVal_s('write.step.assembly')
    try:
        Interface_Static.SetIVal_s('write.step.assembly',0 if single else 1)
        w=STEPControl_Writer();w.Transfer(shape,STEPControl_AsIs)
        if w.Write(str(path))!=IFSelect_RetDone:raise RuntimeError(f'cannot write {path}')
    finally:Interface_Static.SetIVal_s('write.step.assembly',old)

def write_mesh(shape,path):
    BRepMesh_IncrementalMesh(shape,.1,False,.18,True).Perform()
    w=StlAPI_Writer();w.ASCIIMode=False
    if not w.Write(shape,str(path)):raise RuntimeError(f'cannot mesh {path}')

def bore_x(s,z=0.):return cut(s,cylinder(1.6,40,(-20,0,z),(1,0,0)))

def guide_eye(s,z,depth=5):
    # Palmar eyelet, with a printable bridge to the phalanx.
    return cut(union(s,tube(2,.65,2,(0,-8,z-1)),box(-2,-8,z-1,4,depth,2)),cylinder(.65,4,(0,-8,z-2)))

def make_root():
    foot=box(-22,-15,-15,44,30,5)
    for x in (-19,19):
        for y in (-10,10):foot=cut(foot,cylinder(1.7,7,(x,y,-16)))
    for x in (-16,13):
        ear=union(box(x,-7,-10,3,14,10),cylinder(7,3,(x,0,0),(1,0,0)))
        foot=union(foot,ear)
    foot=bore_x(foot)
    foot=union(foot,box(-2,-10,-11,4,4,4))
    return cut(foot,cylinder(.65,12,(0,-8,-17)),cylinder(.5,5,(12,4,0),(1,0,0)))

def make_proximal():
    s=union(cylinder(7,14,(-7,0,0),(1,0,0)),box(-8,-5,0,16,10,35),box(-16,-6,31,32,12,4))
    for x in (-16,13):s=union(s,box(x,-7,34,3,14,11),cylinder(7,3,(x,0,45),(1,0,0)))
    s=bore_x(bore_x(s),45)
    s=cut(s,box(-9,-2,12,18,4,15),cylinder(2.35,6.3,(7,0,0),(1,0,0)),cylinder(2.35,6.3,(-13.3,0,0),(1,0,0)))
    return cut(guide_eye(s,34,4),cylinder(.5,5,(5,4,0),(1,0,0)),cylinder(.5,5,(12,4,45),(1,0,0)))

def make_distal():
    p=BRepBuilderAPI_MakePolygon()
    for y,z in [(-5,0),(-5,29),(0,36),(5,29),(5,0)]:p.Add(gp_Pnt(-7,y,z))
    p.Close();beam=BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(p.Wire()).Face(),gp_Vec(14,0,0)).Shape()
    s=bore_x(union(cylinder(7,14,(-7,0,0),(1,0,0)),beam))
    s=guide_eye(s,8,5);s=guide_eye(s,32,6)
    # Eyelet bores must remain open after bridges are fused.
    for z in (8,32):s=cut(s,cylinder(.65,4,(0,-8,z-2)))
    return cut(s,cylinder(.5,5,(5,4,0),(1,0,0)))

def nut(x,y=0,z=0,flats=5.5,thick=2.4,hole=1.6):
    p=BRepBuilderAPI_MakePolygon();r=flats/math.sqrt(3)
    for i in range(6):
        a=math.radians(30+60*i);p.Add(gp_Pnt(x,y+r*math.cos(a),z+r*math.sin(a)))
    p.Close();s=BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(p.Wire()).Face(),gp_Vec(thick,0,0)).Shape()
    return cut(s,cylinder(hole,thick+.2,(x-.1,y,z),(1,0,0)))

def load_motor():
    ss=sorted(solids(read_step(SOURCE)),key=volume,reverse=True)
    if len(ss)!=3:raise RuntimeError('source motor must contain body and two horns')
    body=ss[0]; horns=sorted(ss[1:],key=lambda s:bounds(s)[0]);main=horns[1]
    centre=None
    for f in collect(main,TopAbs_FACE):
        a=BRepAdaptor_Surface(TopoDS.Face_s(f))
        if a.GetType()==GeomAbs_Cylinder and abs(a.Cylinder().Radius()-10)<1e-5:
            p=a.Cylinder().Axis().Location();centre=(p.Y(),p.Z());break
    if centre is None:raise RuntimeError('cannot locate source output axis')
    cx=(bounds(horns[0])[0]+bounds(main)[3])/2
    m=turn('x',180)@shift(-cx,-centre[0],-centre[1])
    body=move(body,m);main=move(main,m)
    hs=[]
    for f in collect(main,TopAbs_FACE):
        a=BRepAdaptor_Surface(TopoDS.Face_s(f))
        if a.GetType()==GeomAbs_Cylinder and abs(a.Cylinder().Radius()-1.5)<1e-5:
            p=a.Cylinder().Axis().Location(); y,z=p.Y(),p.Z()
            if abs(math.hypot(y,z)-7)<.1 and not any(math.hypot(y-u,z-v)<.01 for u,v in hs):hs.append((y,z))
    if len(hs)!=4:raise RuntimeError(f'expected four horn holes, got {hs}')
    return body,main,hs

def make_drive(holes):
    h=26.
    hub=union(cylinder(10.5,2,(h,0,0),(1,0,0)),box(h+2,-3,-3,10,6,6))
    hub=cut(hub,cylinder(1.25,15,(25,0,0),(1,0,0)))
    for y,z in holes:hub=cut(hub,cylinder(1.1,4,(25,y,z),(1,0,0)))
    drum=union(cylinder(10,1,(31,0,0),(1,0,0)),cylinder(7.4,6,(32,0,0),(1,0,0)),cylinder(10,1,(38,0,0),(1,0,0)))
    drum=cut(drum,box(30,-3.1,-3.1,10,6.2,6.2))
    drum=cut(drum,cylinder(.65,12,(30,7.8,0),(1,0,0))) # tie-off port
    retainer=compound([tube(5,1.6,.6,(39,0,0),(1,0,0)),union(cylinder(1.2,5.6,(34,0,0),(1,0,0)),cylinder(2.7,1.4,(39.6,0,0),(1,0,0)))])
    bolts=[]
    for y,z in holes:
        bolts += [union(cylinder(1,6,(22,y,z),(1,0,0)),cylinder(1.9,.8,(28,y,z),(1,0,0))),nut(22.3,y,z,4,1.2,1.05)]
    return hub,drum,retainer,compound(bolts)

def make_carrier():
    s=union(box(-29,-13,-64,58,48,5),box(-21,-9,-59,42,39,1))
    for y in (-13,33):
        s=union(s,box(-29,y,-59,58,2,36),box(-29,y,-23,58,2,3))
        # Rear holes; front holes use extended ears below.
        if y>0:
            for x in (-23,23):s=cut(s,cylinder(1.7,8,(x,34,-26)),cylinder(3,42,(x,34,-65)))
    for x in (-23,23):
        s=union(s,box(x-4,-24,-23,8,13,3))
        s=cut(s,cylinder(1.7,8,(x,-19.5,-26)))
    # Clamp pads seat against the motor's top plane; use thin rubber shims.
    for x in (-21,18):s=union(s,box(x,-13,-38,3,48,2))
    for x in (-19.5,19.5):
        s=union(s,cylinder(3,6,(x,10,-38)))
        s=cut(s,cylinder(1.25,8,(x,10,-39)))
    return s

def make_palm():
    s=cut(cylinder(100,5,(0,0,-5)),cylinder(25,7,(0,0,-6)))
    for i in range(3):
        b=base(i)
        block=box(-3,-10.8,-29,41,5.6,9)
        # In this local frame the top of the conduit meets the palm underside.
        path=[np.array(p) for p in FIXED_ROUTE[1:-1]]
        channel=wire_solid(path,1.15)
        block=cut(block,channel,cylinder(1.15,14,(0,-8,-29)),cylinder(1.15,14,(35,-8,-29)))
        s=union(s,move(block,b))
        for x,y,r in [(x,y,2.15) for x in (-19,19) for y in (-10,10)]+[(x,y,1.7) for x in (-23,23) for y in (-19.5,34)]+[(0,-8,1.15)]:
            s=cut(s,move(cylinder(r,14,(x,y,-25)),b))
    # Attachment pattern on the central ring.
    for a in (30,150,270):
        r=math.radians(a);s=cut(s,cylinder(2.2,8,(34*math.cos(r),34*math.sin(r),-6)))
    return s


def hex_z(x,y,z):
    poly=BRepBuilderAPI_MakePolygon()
    for j in range(6):
        a=math.radians(j*60);poly.Add(gp_Pnt(x+5.5/math.sqrt(3)*math.cos(a),y+5.5/math.sqrt(3)*math.sin(a),z))
    poly.Close();return cut(BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(poly.Wire()).Face(),gp_Vec(0,0,2.4)).Shape(),cylinder(1.6,2.6,(x,y,z-.1)))

def attachments():
    roots=[];carriers=[];clamps=[]
    for x in (-19,19):
        for y in (-10,10):
            roots += [union(cylinder(1.5,10,(x,y,-20)),cylinder(2.75,2.5,(x,y,-10))),tube(2.1,1.6,4,(x,y,-19.5))]
    for x in (-23,23):
        for y in (-19.5,34):
            carriers += [union(cylinder(1.5,12,(x,y,-23)),cylinder(2.75,2.5,(x,y,-25.5))),hex_z(x,y,-15)]
    for x in (-19.5,19.5):
        clamps.append(union(cylinder(1.2,6,(x,10,-38)),cylinder(2.75,2.5,(x,10,-32))))
    return compound(roots),compound(carriers),compound(clamps)

def build_parts():
    body,main,holes=load_motor();hub,drum,retainer,bolts=make_drive(holes)
    roots,carriers,clamps=attachments()
    pad=cut(union(cylinder(4,11,(0,-8.5,14)),sphere(4,(0,-8.5,14)),sphere(4,(0,-8.5,25))),box(-5,-5,9,10,10,22),cylinder(1.1,21,(0,-8,9)))
    return {'palm':make_palm(),'root_clevis':make_root(),'proximal':make_proximal(),'distal':make_distal(),
            'root_fasteners':roots,'carrier_fasteners':carriers,'clamp_screws':clamps,
            'contact_pad':pad,'winch_hub':hub,'winch_drum':drum,'motor_carrier':make_carrier(),
            'motor_body':body,'motor_horn':main,'winch_retainer':retainer,'hub_fasteners':bolts,
            'hinge_bolt':union(cylinder(1.5,39,(-17.5,0,0),(1,0,0)),cylinder(2.75,2,(-19.5,0,0),(1,0,0))),
            'hinge_nut':nut(17),'hinge_spacers':compound([tube(2.25,1.6,5.6,(x,0,0),(1,0,0)) for x in (-12.8,7.2)]),'torsion_spring_envelope':tube(3.15,2.35,3.3,(9,0,0),(1,0,0)),
            'liner':tube(1,.5,19,(0,-8,10)), 'tendon_stop':sphere(.9,(0,-8,33.9))}

CUSTOM = ('palm','root_clevis','proximal','distal','contact_pad','winch_hub','winch_drum','motor_carrier','hinge_bolt')

def assembly(parts,q=(0.,0.),reel=0.,include_cable=True):
    records=[]
    def put(name,part,m):records.append({'name':name,'part':part,'matrix':m.tolist(),'shape':move(parts[part],m)})
    put('palm','palm',np.eye(4))
    for i,n in enumerate(FINGERS):
        b=base(i);p=b@turn('x',q[0]);d=p@shift(z=45)@turn('x',q[1]);motor=b@shift(z=-48);rotor=motor@turn('x',reel)
        for part,m in [('root_clevis',b),('root_fasteners',b),('carrier_fasteners',b),('clamp_screws',b),('proximal',p),('distal',d),('contact_pad',d),('liner',d),('tendon_stop',d),('motor_carrier',b),('motor_body',motor),('motor_horn',rotor),('winch_hub',rotor),('winch_drum',rotor),('winch_retainer',rotor),('hub_fasteners',rotor)]:put(n+'_'+part,part,m)
        for j,m in enumerate((b,p@shift(z=45))):
            for part in ('hinge_bolt','hinge_nut','hinge_spacers','torsion_spring_envelope'):put(f'{n}_j{j+1}_{part}',part,m)
        if include_cable:
            cable=wire_solid(route(i,*q),.4)
            records.append({'name':n+'_tendon','part':'tendon','matrix':np.eye(4).tolist(),'shape':cable})
    return records

def export_assembly(records,path,single=False):
    shape=compound([r['shape'] for r in records]);write_step(shape,path,single)
    result=solids(read_step(path));expected=sum(len(solids(r['shape'])) for r in records)
    if len(result)!=expected or any(not BRepCheck_Analyzer(s).IsValid() for s in result):raise RuntimeError(f'STEP validation failed {path}')
    return {'file':path.name,'solids':len(result),'valid':True,'volume_mm3':volume(shape),'bounds_mm':bounds(shape)}

def main():
    for p in (PARTS,MESHES,REPORTS):p.mkdir(exist_ok=True)
    parts=build_parts();part_report=[]
    for n,s in parts.items():
        if not BRepCheck_Analyzer(s).IsValid() or volume(s)<=0:raise RuntimeError(f'invalid part {n}')
        write_mesh(s,MESHES/(n+'.stl'))
        write_step(s,PARTS/(n+'.step'),single=True)
        part_report.append({'name':n,'custom':n in CUSTOM,'solids':len(solids(s)),'volume_mm3':volume(s)})
    report=[];manifest={}
    for pose,q in [('open',(0.,0.)),('grasp',(9.,60.))]:
        reel=(NEUTRAL_LENGTH-length(route(0,*q)))/SPOOL_RADIUS*180/math.pi
        records=assembly(parts,q,reel)
        report.append(export_assembly(records,HERE/f'tendon_hand_{pose}.step'))
        if pose=='open':report.append(export_assembly(records,HERE/'tendon_hand_open_multibody.step',True))
        manifest[pose]={'joint_angles_deg':q,'reel_deg':reel,'tendon_length_mm':length(route(0,*q)),'instances':[{k:v for k,v in r.items() if k!='shape'} for r in records]}
    (REPORTS/'cad.json').write_text(json.dumps({'parts':part_report,'assemblies':report},indent=2))
    (HERE/'assembly.json').write_text(json.dumps(manifest,indent=2))
    (REPORTS/'source.json').write_text(json.dumps({'source':str(SOURCE.relative_to(HERE.parent.parent.parent)),'sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),'new_geometry_source':'build_cad.py; no prior hand design imported'},indent=2))
    print('CAD export complete',report,flush=True)

if __name__=='__main__':main()
