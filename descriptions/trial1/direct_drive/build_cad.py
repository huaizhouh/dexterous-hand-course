#!/usr/bin/env python3
"""Build three-servo direct-drive concept from support_frames; millimetres."""
import json, math
from pathlib import Path
import numpy as np
from OCP.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.BRep import BRep_Builder
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeBox, BRepPrimAPI_MakePrism
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse, BRepAlgoAPI_Cut, BRepAlgoAPI_Common
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf, gp_Pnt, gp_Dir, gp_Ax2, gp_Vec

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent.parent
SOURCE=ROOT/'support_frames'
ASSETS=HERE/'meshes'
ASSETS.mkdir(exist_ok=True)
RADIUS=70.0
PIVOT_Z=20.0
# Coordinates measured in the supplied STEP, not in a second-hand assembly.
CY=10.4198318473
CX=6.810106
CZ=51.0

def trans(x=0,y=0,z=0):
 m=np.eye(4);m[:3,3]=(x,y,z);return m

def rot(axis,angle):
 a=math.radians(angle);c=math.cos(a);s=math.sin(a);m=np.eye(4)
 if axis=='x':m[:3,:3]=[[1,0,0],[0,c,-s],[0,s,c]]
 else:m[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]
 return m

def moved(s,m):
 t=gp_Trsf();t.SetValues(*map(float,m[:3,:].flat));return BRepBuilderAPI_Transform(s,t,True).Shape()

def read(p):
 r=STEPControl_Reader()
 if r.ReadFile(str(p))!=IFSelect_RetDone:raise RuntimeError(f'STEP read failed: {p}')
 r.TransferRoots();return r.OneShape()

def solids(shape):
 e=TopExp_Explorer(shape,TopAbs_SOLID);out=[]
 while e.More():out.append(TopoDS.Solid_s(e.Current()));e.Next()
 return out

def combine(parts):
 builder=BRep_Builder();c=TopoDS_Compound();builder.MakeCompound(c)
 for part in parts:builder.Add(c,part)
 return c

def bounds(s):
 b=Bnd_Box();BRepBndLib.Add_s(s,b);return list(b.Get())

def volume(s):
 p=GProp_GProps();BRepGProp.VolumeProperties_s(s,p);return p.Mass()

def export_step(s,p):
 w=STEPControl_Writer();assert w.Transfer(s,STEPControl_AsIs)==IFSelect_RetDone
 assert w.Write(str(p))==IFSelect_RetDone

def export_stl(s,p):
 BRepMesh_IncrementalMesh(s,0.12,False,0.2,True).Perform()
 w=StlAPI_Writer();w.ASCIIMode=False
 assert w.Write(s,str(p))

def box(x,y,z,dx,dy,dz):return BRepPrimAPI_MakeBox(gp_Pnt(x,y,z),dx,dy,dz).Shape()

def difference(a,b):
 operation=BRepAlgoAPI_Cut(a,b);operation.Build()
 if not operation.IsDone():raise RuntimeError('cut failed')
 return operation.Shape()

def fuse(a,b):
 operation=BRepAlgoAPI_Fuse(a,b);operation.Build()
 if not operation.IsDone():raise RuntimeError('fuse failed')
 return operation.Shape()

def cyl_x(x,y,z,r,length):
 return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,z),gp_Dir(1,0,0)),r,length).Shape()

def hex_nut(x,y,z):
 # M2 thin nut envelope: 4 mm across flats, 1.2 mm thick.
 radius=4/math.sqrt(3)
 wire=BRepBuilderAPI_MakePolygon()
 for i in range(6):
  a=math.radians(30+60*i);wire.Add(gp_Pnt(x,y+radius*math.cos(a),z+radius*math.sin(a)))
 wire.Close()
 blank=BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(wire.Wire()).Face(),gp_Vec(1.2,0,0)).Shape()
 return difference(blank,cyl_x(x-0.1,y,z,1.05,1.4))

def screw_and_nut(side,y,z):
 # Simplified unthreaded geometry for M2 x 6 low-profile screw and thin nut.
 if side<0:
  shank=cyl_x(-28,y,z,1,6);head=cyl_x(-28.8,y,z,1.9,0.8);nut=hex_nut(-23.45,y,z)
 else:
  shank=cyl_x(22,y,z,1,6);head=cyl_x(28,y,z,1.9,0.8);nut=hex_nut(22.3,y,z)
 return fuse(shank,head),nut

def tie(x):
 # Idealised 3 x 1 mm nylon band; locked portion is not modelled.
 parts=[box(x-1.5,-30,10.2,3,65,1),box(x-1.5,-30,-22,3,65,1),
        box(x-1.5,-30,-22,3,1,33.2),box(x-1.5,34,-22,3,1,33.2)]
 a=parts[0]
 for b in parts[1:]:a=fuse(a,b)
 return a

def root(yaw):
 a=math.radians(yaw)
 return trans(-RADIUS*math.sin(a),RADIUS*math.cos(a),PIVOT_Z)@rot('z',yaw)

def build():
 components=solids(read(SOURCE/'金属舵机最小单元-双轴.STEP'))
 if len(components)!=3:raise ValueError(f'expected three unit solids, got {len(components)}')
 components.sort(key=volume,reverse=True)
 servo=components[0]
 horns=sorted(components[1:],key=lambda s: bounds(s)[0])
 follower,main=horns
 # Bring the hole patterns on both horns to the U-bracket pattern. The two
 # horns are separate parts; their angular positions in the source differ.
 axis=trans(CX,CY,CZ)
 follower=moved(follower,axis@rot('x',-0.17)@trans(-CX,-CY,-CZ))
 main=moved(main,axis@rot('x',5.65)@trans(-CX,-CY,-CZ))
 bracket=solids(read(HERE/'source_parts/长U支架.step'))[0]
 # Standard long-U spindle: (27, -12.5, 50).  Place its ears against
 # the two horn outer faces, then point its length upwards.
 bracket_in_unit=trans(CX-27,CY+12.5,CZ-50)
 normalize=rot('x',180)@trans(-CX,-CY,-CZ)
 local={'servo':moved(servo,normalize),'horn_follower':moved(follower,trans(0.05,0,0)@normalize),
        'horn_main':moved(main,normalize),'long_u':moved(bracket,normalize@bracket_in_unit)}
 # Silicone block rests against the distal inside of the long-U crosspiece.
 local['pad']=box(-14,-12.5,20,28,8,30)
 # One custom palm: plate and three risers for zip-tie mounting. The two
 # through-slots per tie straddle each motor along its radial direction.
 palm=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0,0,0),gp_Dir(0,0,1)),112,4).Shape()
 # A centre aperture clears the simulation support and offers access below.
 palm=difference(palm,BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0,0,-1),gp_Dir(0,0,1)),37,6).Shape())
 # Four 4.4 mm wrist/fixture attachment holes on a 90 mm square.
 for x in (-45,45):
  for y in (-45,45):
   palm=difference(palm,BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,y,-1),gp_Dir(0,0,1)),2.2,6).Shape())
 for yaw in (0,120,240):
  m=root(yaw)@trans(0,0,-PIVOT_Z)
  palm=fuse(palm,moved(box(-18,-8,4,36,36,6),m))
  for x in (-18,18):
   for y in (-30,36):palm=difference(palm,moved(box(x-2,y-4,-1,4,8,6),m))
 palm=TopoDS.Solid_s(solids(palm)[0])
 local['palm']=palm
 hardware=[]
 for side in (-1,1):
  for y,z in ((-7,0),(7,0),(0,-7),(0,7)):
   hardware.extend(screw_and_nut(side,y,z))
 straps=[tie(-18),tie(18)]
 for n,shape in enumerate(hardware):
  if not BRepCheck_Analyzer(shape).IsValid():raise RuntimeError(f'invalid fastener {n}')
 export_stl(combine(hardware),ASSETS/'fasteners.stl')
 export_stl(combine(straps),ASSETS/'ties.stl')
 for name,s in local.items():
  if not BRepCheck_Analyzer(s).IsValid():raise RuntimeError(f'invalid part {name}')
  export_stl(s,ASSETS/(name+'.stl'))
 for name in ('palm','pad'):
  export_step(local[name],HERE/(name+'.step'))
 records=[('palm',palm)]
 for i,yaw in enumerate((0,120,240),1):
  for name in ('servo','horn_follower','horn_main','long_u','pad'):
   records.append((f'finger{i}_{name}',moved(local[name],root(yaw))))
  for n,shape in enumerate(hardware):records.append((f'finger{i}_fastener{n}',moved(shape,root(yaw))))
  for n,shape in enumerate(straps):records.append((f'finger{i}_tie{n}',moved(shape,root(yaw))))
 model=combine([s for _,s in records]);out=HERE/'direct_drive_open.step';export_step(model,out)
 # Export a single product with 70 separate solids.
 mode=Interface_Static.IVal_s('write.step.assembly')
 try:
  Interface_Static.SetIVal_s('write.step.assembly',0)
  export_step(model,HERE/'direct_drive_open_multibody.step')
 finally:Interface_Static.SetIVal_s('write.step.assembly',mode)
 reread=read(out);read_solids=solids(reread)
 if len(read_solids)!=len(records) or any(not BRepCheck_Analyzer(s).IsValid() for s in read_solids):raise RuntimeError('STEP re-import failed')
 report={'solids':len(read_solids),'step_reimport_valid':True,'bounds_mm':bounds(reread),'overlaps':[],'instances':[]}
 for n,s in records:report['instances'].append({'name':n,'bounds_mm':bounds(s),'volume_mm3':volume(s)})
 # Exact solid intersections; tolerance intentionally tight enough to catch
 # incorrectly placed bracket ears or support risers.
 for i,(an,a) in enumerate(records):
  ab=bounds(a)
  for bn,b in records[i+1:]:
   bb=bounds(b)
   if any(min(ab[k+3],bb[k+3])-max(ab[k],bb[k])<1e-5 for k in range(3)):continue
   op=BRepAlgoAPI_Common(a,b);op.Build()
   if not op.IsDone():raise RuntimeError(f'common failed {an} {bn}')
   v=volume(op.Shape())
   if v>0.05:report['overlaps'].append({'a':an,'b':bn,'volume_mm3':v})
 (HERE/'cad_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
 print(json.dumps({'n_solids':len(read_solids),'overlaps':report['overlaps']},indent=2),flush=True)

if __name__=='__main__':build()
