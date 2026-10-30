"""ThreeFinger V5. Analytic B-rep in millimetres; no proprietary CAD required.

The teacher's actual STEP is imported, separated by measured volumes and
transformed rigidly. No servo envelope is substituted for the supplied model.
The CAD source is editable; STEP contains static solids, not SW motion mates.
"""
from pathlib import Path
import argparse, csv, hashlib, json, math
import cadquery as cq
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder

ROOT = Path(__file__).resolve().parents[1]
P = json.loads((ROOT/'source/parameters.json').read_text())
L = P['proximal_axis_distance']
D = P['distal_axis_to_tip']
RAD = P['root_radius']
ZROOT = P['root_height']
COLORS = {
 'palm':(.22,.34,.42), 'base':(.22,.34,.42), 'root':(.74,.81,.84),
 'proximal':(.80,.86,.89), 'distal':(.80,.86,.89), 'pad_proximal':(.10,.51,.55),
 'pad_distal':(.10,.51,.55), 'motor_body':(.14,.16,.19),
 'main_horn':(.62,.64,.66), 'slave_horn':(.62,.64,.66),
 'cradle':(.31,.40,.46), 'clamp':(.31,.40,.46), 'spool':(.91,.43,.17),
 'guide_front':(.33,.46,.50),'guide_back':(.33,.46,.50),
 'tendon':(.86,.13,.13),'elastic':(.93,.70,.10),
 'column':(.39,.49,.54), 'metal':(.66,.68,.70), 'liner':(.19,.23,.24)
}

def box(x,y,z,dx,dy,dz):
 return cq.Solid.makeBox(dx,dy,dz,cq.Vector(x,y,z))
def cyl(r,h,p,axis=(0,0,1)):
 return cq.Solid.makeCylinder(r,h,cq.Vector(*p),cq.Vector(*axis))
def sphere(r,p): return cq.Solid.makeSphere(r,cq.Vector(*p),angleDegrees1=-90)
def comp(ss): return cq.Compound.makeCompound(list(ss))
def fuse(*ss):
 s=ss[0]
 for t in ss[1:]: s=s.fuse(t)
 return s.clean()
def cut(s,*ss):
 for t in ss: s=s.cut(t)
 return s.clean()
def tube(ro,ri,h,p,axis=(0,0,1)):
 return cut(cyl(ro,h,p,axis),cyl(ri,h+.02,np.asarray(p)-np.asarray(axis)*.01,axis))
def transform(s,m):
 # Rigid OCCT placement preserves analytic planes and cylinders.
 from OCP.gp import gp_Trsf
 from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
 t=gp_Trsf();t.SetValues(*[float(x) for x in m[:3,:].ravel()])
 return cq.Shape.cast(BRepBuilderAPI_Transform(s.wrapped,t,True).Shape())
def T(x=0,y=0,z=0):
 m=np.eye(4);m[:3,3]=(x,y,z);return m
def R(axis,deg):
 a=math.radians(deg);c,s=math.cos(a),math.sin(a);m=np.eye(4)
 if axis=='x': m[:3,:3]=((1,0,0),(0,c,-s),(0,s,c))
 else: m[:3,:3]=((c,-s,0),(s,c,0),(0,0,1))
 return m
def point(m,p): return (m@np.r_[p,1])[:3]
def frame(i):
 a=math.radians(120*i)
 return T(-RAD*math.sin(a),RAD*math.cos(a),ZROOT)@R('z',120*i)
def rounded_plate(side,thick,z,r=9):
 return cq.Workplane('XY').rect(side,side).extrude(thick).edges('|Z').fillet(r).val().translate((0,0,z))
def hose(points,r):
 ss=[sphere(r,p) for p in points]
 for a,b in zip(points,points[1:]):
  v=np.asarray(b)-np.asarray(a); le=float(np.linalg.norm(v))
  if le>1e-7: ss.append(cyl(r,le,a,v/le))
 return fuse(*ss)
def screw(r,length,p,axis=(0,0,1),head_r=None,head_h=2.4):
 # p is the underside of the head; shank goes in +axis direction.
 v=np.asarray(axis);p=np.asarray(p)
 return fuse(cyl(r,length,p,v),cyl(head_r or r*1.85,head_h,p-v*head_h,v))
def hexnut(rhole,flats,thick,p,axis='z'):
 s=cq.Workplane('XY').polygon(6,flats/math.cos(math.pi/6)).extrude(thick).val()
 s=cut(s,cyl(rhole,thick+.2,(0,0,-.1)))
 if axis=='x': s=s.rotate((0,0,0),(0,1,0),90)
 return s.translate(p)

def peg(z,base_y=None):
 # Neck 3.2 mm dia, 6 mm seating width, 7.2 mm captive head.
 base_y=P['root_peg_base_y'] if base_y is None else base_y
 width=P['elastic_neck_width']
 return fuse(cyl(1.6,width,(0,base_y,z),(0,1,0)),
             cyl(3.6,1.5,(0,base_y+width,z),(0,1,0)))
def pivot_bore(s,z): return cut(s,cyl(1.65,40,(-20,0,z),(1,0,0)))
def root_part():
 s=box(-21,-14,-18,42,28,4)
 for x in (-12.3,8.3):
  s=fuse(s,box(x,-8.5,-14,4,17,14),cyl(8.5,4,(x,0,0),(1,0,0)))
 s=fuse(s,box(-12.3,5,-14,24.6,P['root_peg_base_y']-5,7.6),peg(-8))
 # Dorsal stop shoulders: extension stop is a clearance surface, not an
 # unverified 0-degree angular claim.
 for x in (-12.3,8.3): s=fuse(s,box(x,7,3,4,4,7))
 s=pivot_bore(s,0)
 s=cut(s,cyl(1.55,7,(0,GUIDE_Y,-19)))
 for x in (-17,17):
  for y in (-4,4): s=cut(s,cyl(1.7,6,(x,y,-19)))
 return s

def proximal_part():
 s=fuse(cyl(7,16,(-8,0,0),(1,0,0)),box(-8,-5.5,0,16,11,L-14))
 s=fuse(s,box(-12.3,-5.5,L-17,24.6,11,4))
 for x in (-12.3,8.3):
  s=fuse(s,box(x,-8.5,L-14,4,17,14),cyl(8.5,4,(x,0,L),(1,0,0)))
 # A closed palmar tendon tunnel, integral with the contact-pad backing.
 s=fuse(s,cyl(2.3,L-24,(0,-7.5,12)),box(-8,-9.8,18,16,4.5,26),
        box(-5,5,10,10,P['root_peg_base_y']-5,8),box(-8,5,L-17,16,P['pip_peg_base_y']-5,8),peg(14),peg(L-13,P['pip_peg_base_y']))
 s=cut(s,cyl(1.2,L-22,(0,-7.5,11)))
 return pivot_bore(pivot_bore(s,0),L)

def distal_part():
 # Straight industrial phalanx, tapered at the distal end.
 poly=[(-8,0),(8,0),(8,D-9),(5,D),(-5,D),(-8,D-9)]
 # XZ plane extrudes in -Y. Centre the section on y=0.
 beam=cq.Workplane('XZ').polyline(poly).close().extrude(11).val().translate((0,5.5,0))
 s=fuse(cyl(7,16,(-8,0,0),(1,0,0)),beam,
        cyl(2.3,32,(0,-7.5,10)),box(-7,-9.8,12,14,4.5,D-12),
        box(-4,-9.8,40,8,5.3,8),box(-5,5,10,10,P['pip_peg_base_y']-5,8),peg(14,P['pip_peg_base_y']))
 s=cut(s,cyl(1.2,39,(0,-7.5,9)),cyl(2.5,4.1,(0,-7.5,44)))
 return pivot_bore(s,0)

def pad(width,z0,height):
 s=box(-width/2,-12.3,z0,width,2.5,height)
 # Glue-on replaceable TPU/silicone contact surface, shallow transverse ribs.
 for z in np.arange(z0+2,z0+height-1,4):
  s=fuse(s,box(-width/2+.6,-12.9,float(z),width-1.2,.6,1.4))
 return s

MOUNT_X=(-29.5,25.5)
MOUNT_Y=(-18.,36.)
GUIDE_Y=-10.9
def guide_path():
 # World-relative-to-root route: servo z=-56; fixed channel below palm.
 pts=[(33,GUIDE_Y,-38),(33,GUIDE_Y,-35)]
 for a in np.linspace(0,math.pi/2,9)[1:]:
  pts.append((29+4*math.cos(a),GUIDE_Y,-35+4*math.sin(a)))
 pts.append((4,GUIDE_Y,-31))
 for a in np.linspace(0,math.pi/2,9)[1:]:
  pts.append((4-4*math.sin(a),GUIDE_Y,-27-4*math.cos(a)))
 pts.append((0,GUIDE_Y,-18))
 return [np.array(p,dtype=float) for p in pts]

def palm_part():
 outer=rounded_plate(P['palm_side'],6,0)
 s=cut(outer,cyl(87,8,(0,0,-1)))
 s=fuse(s,cyl(46,6,(0,0,0)))
 for i in range(3):
  # Wide radial webs preserve root and four cradle mounting lands.
  s=fuse(s,transform(box(-34,31,0,68,74,6),R('z',120*i)))
 s=s.intersect(outer).clean()
 s=cut(s,cq.Workplane('XY').rect(54,54).extrude(8).edges('|Z').fillet(7).val().translate((0,0,-1)))
 for i in range(3):
  b=frame(i)
  for x in (-17,17):
   for y in (-4,4): s=cut(s,transform(cyl(1.7,8,(x,y,-25)),b))
  for x in MOUNT_X:
   for y in MOUNT_Y: s=cut(s,transform(cyl(1.7,8,(x,y,-25)),b))
  s=cut(s,transform(box(-3.2,GUIDE_Y-3.5,-25,6.4,7,8),b))
  for x in (24,34): s=cut(s,transform(cyl(1.35,8,(x,GUIDE_Y+6.1,-25)),b))
 for x in (-96,96):
  for y in (-96,96): s=cut(s,cyl(2.2,8,(x,y,-1)))
 return s

def base_part():
 s=rounded_plate(P['palm_side'],6,-68)
 opening=cq.Workplane('XY').rect(138,126).extrude(8).edges('|Z').fillet(12).val().translate((0,0,-69))
 s=cut(s,opening)
 for x in (-96,96):
  for y in (-96,96): s=cut(s,cyl(2.2,8,(x,y,-69)))
 # Four optional fixing holes, away from the hand-grip opening.
 for x in (-78,78):
  for y in (-78,78): s=cut(s,cyl(2.2,8,(x,y,-69)))
 return s

def cradle_part():
 # Coordinates are relative to servo output-axis centre.
 s=box(-33,-22.5,-14.5,62,62.5,3.5)
 # Side support faces with replaceable rubber liners.
 for y in (-13.5,30.5): s=fuse(s,box(-25.7,y,-11,47.3,3,7))
 for x in MOUNT_X:
  for y in MOUNT_Y:
   s=fuse(s,cyl(3.5,21.6,(x,y,-11)))
   s=cut(s,cyl(1.7,28,(x,y,-15)))
   # Nut pocket is accessible from the underside, M3 hex, 5.7 AF.
   pocket=cq.Workplane('XY').polygon(6,5.7/math.cos(math.pi/6)).extrude(2.5).val().translate((x,y,-14.5))
   s=cut(s,pocket)
 return cut(s,cyl(13.5,3,(26.5,0,0),(1,0,0)))
def clamp_part():
 s=box(-33,-22.5,10.6,62,62.5,3)
 s=cut(s,box(21,-14.5,10.5,9,29,3.2))
 # Clear the central output shaft region; cover is above the housing.
 for x in MOUNT_X:
  for y in MOUNT_Y:
   s=fuse(s,cyl(3.5,18.4,(x,y,13.6)))
   s=cut(s,cyl(1.7,23,(x,y,10)))
 return s

def servo_parts():
 path=ROOT/'reference/teacher_dual_axis_servo_original.stp'
 ss=sorted(cq.importers.importStep(str(path)).val().Solids(),key=lambda s:-s.Volume())
 if len(ss)!=8: raise ValueError('Expected the actual 8-solid teacher servo STEP')
 # The original lower housing has a small topology defect. OCCT healing
 # changes its volume by 0.000128 mm^3; preserve the original file unchanged.
 from OCP.ShapeFix import ShapeFix_Shape
 for k,s in enumerate(ss):
  if not s.isValid():
   fixer=ShapeFix_Shape(s.wrapped);fixer.Perform();ss[k]=cq.Shape.cast(fixer.Shape())
   if not ss[k].isValid(): raise ValueError(f'Cannot heal source solid {k}')
 # Exclude the two supplied U-brackets (indices 2,3). They are not required
 # for this tendon design; preserve originals in reference, without machining.
 norm=R('z',-90)@T(-10,0,0)
 return { 'motor_body':transform(comp([ss[k] for k in (0,1,4,7)]),norm),
         'main_horn':transform(ss[5],norm),'slave_horn':transform(ss[6],norm) }

def spool_part():
 s=fuse(cyl(13,2,(27,0,0),(1,0,0)),cyl(10.5,6,(29,0,0),(1,0,0)),cyl(13,1.5,(35,0,0),(1,0,0)))
 s=cut(s,cyl(3.3,10,(26.8,0,0),(1,0,0)))
 for y,z in [(0,7),(7,0),(0,-7),(-7,0)]:
  s=cut(s,cyl(1.1,10,(26.8,y,z),(1,0,0)),cyl(2.1,7.6,(29,y,z),(1,0,0)))
 # Cable tie-through in a flange, off the 4 mounting screws.
 a=math.radians(45);y,z=11.3*math.cos(a),11.3*math.sin(a)
 s=cut(s,cyl(.65,2.4,(26.8,y,z),(1,0,0)))
 return s

def guide_parts():
 # A serviceable split block: half-round channels print open and close around
 # the tendon. 4 mm corner radii; use PTFE-safe dry lubricant after deburring.
 path=guide_path()
 blank=fuse(box(-3,GUIDE_Y-3.3,-38,39.5,6.6,14),
            box(-3,GUIDE_Y-3.3,-24,6,6.6,6))
 for x in (24,34): blank=fuse(blank,box(x-3,GUIDE_Y+3,-30,6,5.8,6))
 bore=hose(path,1.3)
 blank=cut(blank,bore)
 for x in (24,34): blank=cut(blank,cyl(1.35,10,(x,GUIDE_Y+6.1,-32)))
 front=blank.intersect(box(-4,GUIDE_Y-4,-39,42,4,23)).clean()
 back=blank.intersect(box(-4,GUIDE_Y,-39,42,10,23)).clean()
 # Two transverse M2 clamp screws fasten the halves, away from the channel.
 for x in (10,20):
  front=cut(front,cyl(1.1,12,(x,GUIDE_Y-4,-35),(0,1,0)))
  back=cut(back,cyl(1.1,12,(x,GUIDE_Y-4,-35),(0,1,0)))
 return front,back

def parts():
 gf,gb=guide_parts()
 pp={'palm':palm_part(),'base':base_part(),'root':root_part(),
     'proximal':proximal_part(),'distal':distal_part(),
     'pad_proximal':__import__('pad_geometry').pad('proximal',P)[0],'pad_distal':__import__('pad_geometry').pad('distal',P)[0],
     'cradle':cradle_part(),'clamp':clamp_part(),'spool':spool_part(),
     'guide_front':gf,'guide_back':gb,'column':tube(6,2.2,62,(0,0,-62)),
     'footpad':cut(cyl(7,7,(0,0,-75)),cyl(5.4,6,(0,0,-73.8)))}
 pp.update(servo_parts())
 return pp

def tendon_points(i,q1,q2):
 b=frame(i);pr=b@R('x',q1);di=pr@T(z=L)@R('x',q2)
 pts=[point(b,(33,GUIDE_Y,-56))]+[point(b,p) for p in guide_path()]
 pts += [point(b,(0,GUIDE_Y,-8)),point(pr,(0,-7.5,12)),point(pr,(0,-7.5,L-12)),
         point(di,(0,-7.5,10)),point(di,(0,-7.5,44))]
 return pts
def cable_length(q1,q2):
 pts=tendon_points(0,q1,q2)
 return sum(float(np.linalg.norm(b-a)) for a,b in zip(pts,pts[1:]))
def elastic_paths(b,pr,di):
 paths=[]
 for m0,m1,z0,z1,joint,yy,cy in [(b,pr,-8,14,b,P['root_band_line_y'],P['root_band_corner_y']),
   (pr@T(z=L),di,-13,14,pr@T(z=L),P['pip_band_line_y']-1.5,P['pip_band_corner_y']-1.5),
   (pr@T(z=L),di,-13,14,pr@T(z=L),P['pip_band_line_y']+1.5,P['pip_band_corner_y']+1.5)]:
  # One closed loop per hinge: two strands and two captive rounded ends.
  path=[point(m0,(2.2,yy,z0)),point(joint,(2.2,cy,0)),point(m1,(2.2,yy,z1))]
  path += [point(m1,(2.2*math.cos(a),yy,z1+2.2*math.sin(a))) for a in np.linspace(0,math.pi,10)[1:]]
  path += [point(joint,(-2.2,cy,0)),point(m0,(-2.2,yy,z0))]
  path += [point(m0,(2.2*math.cos(a),yy,z0+2.2*math.sin(a))) for a in np.linspace(math.pi,2*math.pi,10)[1:]]
  paths.append(path)
 return paths

def make_assembly(pp,q1,q2,name,flex=True,fasteners=True):
 assy=cq.Assembly(name=name); instances=[]
 def add(key,shape,m=np.eye(4),group='',kind=None):
  shape=transform(shape,m)
  color=COLORS.get(kind or key,COLORS['metal'])
  assy.add(shape,name=key,color=cq.Color(*color))
  instances.append({'name':key,'kind':kind or key,'group':group,'matrix_mm':m.tolist(),'shape':shape})
 add('palm',pp['palm']);add('base',pp['base'])
 for j,(x,y) in enumerate([(x,y) for x in (-96,96) for y in (-96,96)]):
  add(f'column_{j}',pp['column'],T(x,y,0),kind='column')
  add(f'footpad_{j}',pp['footpad'],T(x,y,0),kind='liner')
  if fasteners:
   add(f'column_bolt_{j}',screw(2,80,(x,y,6.8),(0,0,-1),3.5,3),kind='metal')
   add(f'column_nut_{j}',hexnut(2.1,7,3.2,(x,y,-72)),kind='metal')
   for k,z in enumerate((6,-68.8)):
    add(f'column_washer_{j}_{k}',tube(4.5,2.15,.8,(x,y,z)),kind='metal')
 for i in range(3):
  group=f'finger_{chr(65+i)}';b=frame(i);pr=b@R('x',q1);di=pr@T(z=L)@R('x',q2)
  motor=b@T(z=-56)
  takeup=cable_length(0,0)-cable_length(q1,q2)
  winch=motor@R('x',-math.degrees(takeup/P['spool_cable_center_radius']))
  for key,m in [('root',b),('proximal',pr),('distal',di),('pad_proximal',pr),('pad_distal',di),
                ('cradle',motor),('clamp',motor),('motor_body',motor),('main_horn',winch),
                ('slave_horn',winch),('spool',winch),('guide_front',b),('guide_back',b)]:
   add(f'{group}_{key}',pp[key],m,group,kind=key)
  # Shim sheets. Their compression is set during trial assembly.
  add(f'{group}_liner_bottom',box(-22,-9.5,-11,43,39,.8),motor,group,'liner')
  add(f'{group}_liner_top',box(-22,-9.5,10.05,43,39,.55),motor,group,'liner')
  if fasteners:
   for j,jm in enumerate((b,pr@T(z=L))):
    add(f'{group}_pin_{j}',screw(1.5,30,(-12.8,0,0),(1,0,0),2.75,2.4),jm,group,'metal')
    add(f'{group}_pin_nut_{j}',hexnut(1.6,5.5,2.4,(12.8,0,0),'x'),jm,group,'metal')
    for side in (-1,1):
     # Standard 0.3 mm stainless/PTFE thrust washer, OD6, ID3.2.
     add(f'{group}_washer_{j}_{side}',tube(3,1.6,.3,(-8.3 if side<0 else 8,0,0),(1,0,0)),jm,group,'metal')
     add(f'{group}_outer_washer_{j}_{side}',tube(3.2,1.6,.5,(-12.8 if side<0 else 12.3,0,0),(1,0,0)),jm,group,'metal')
   for x in (-17,17):
    for y in (-4,4):
     add(f'{group}_rootbolt_{x}_{y}',screw(1.5,14,(x,y,-14),(0,0,-1),2.75,2.4),b,group,'metal')
     add(f'{group}_rootnut_{x}_{y}',hexnut(1.6,5.5,2.4,(x,y,-26.4)),b,group,'metal')
   for x in MOUNT_X:
    for y in MOUNT_Y:
     add(f'{group}_cradlebolt_{x}_{y}',screw(1.5,55,(x,y,38),(0,0,-1),2.75,2.4),motor,group,'metal')
     add(f'{group}_cradlenut_{x}_{y}',hexnut(1.6,5.5,2.4,(x,y,-14.4)),motor,group,'metal')
   for k,(y,z) in enumerate([(0,7),(7,0),(0,-7),(-7,0)]):
    add(f'{group}_spoolbolt_{k}',screw(1,8,(30,y,z),(-1,0,0),1.9,1.5),winch,group,'metal')
    add(f'{group}_spoolboltwasher_{k}',tube(2,1.05,1,(29,y,z),(1,0,0)),winch,group,'metal')
    add(f'{group}_spoolnut_{k}',hexnut(1.05,4,1.6,(22.9,y,z),'x'),winch,group,'metal')
   # 0.5 mm standoff between source horn face and new spool flange.
   for k,(y,z) in enumerate([(0,7),(7,0),(0,-7),(-7,0)]):
    add(f'{group}_hornwasher_{k}',tube(2,1.05,.5,(26.5,y,z),(1,0,0)),winch,group,'metal')
   for x in (10,20):
    add(f'{group}_guidebolt_{x}',screw(1,8,(x,GUIDE_Y-3.3,-35),(0,1,0),1.9,1.5),b,group,'metal')
    # Hex nut axis is transformed from z to y.
    n=hexnut(1.05,4,1.6,(0,0,0)).rotate((0,0,0),(1,0,0),-90).translate((x,GUIDE_Y+3.3,-35))
    add(f'{group}_guidenut_{x}',n,b,group,'metal')
   for x in (24,34):
    add(f'{group}_guide_mount_{x}',screw(1.25,16,(x,GUIDE_Y+6.1,-18),(0,0,-1),2.3,1.8),b,group,'metal')
    add(f'{group}_guide_mount_nut_{x}',hexnut(1.3,5,2,(x,GUIDE_Y+6.1,-32)),b,group,'metal')
  if flex:
   add(f'{group}_tendon',hose(tendon_points(i,q1,q2),.4),group=group,kind='tendon')
   for j,path in enumerate(elastic_paths(b,pr,di)):
    add(f'{group}_elastic_{j}',hose(path,.55),group=group,kind='elastic')
 return assy,instances

def save_instances(items,path):
 data=[{k:v for k,v in x.items() if k!='shape'} for x in items]
 path.write_text(json.dumps(data,indent=2))

def export_single_product(shape,path):
 from OCP.Interface import Interface_Static
 from OCP.STEPControl import STEPControl_Writer,STEPControl_AsIs
 from OCP.IFSelect import IFSelect_RetDone
 old=Interface_Static.IVal_s('write.step.assembly')
 try:
  Interface_Static.SetIVal_s('write.step.assembly',0)
  w=STEPControl_Writer();w.Transfer(shape.wrapped,STEPControl_AsIs)
  if w.Write(str(path))!=IFSelect_RetDone:raise RuntimeError('single product export failed')
 finally:Interface_Static.SetIVal_s('write.step.assembly',old)

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--pose',default='ALL');args=ap.parse_args()
 print('Building analytic solids',flush=True);__import__('pad_geometry').export_collision(P);pp=parts()
 status={}
 printable=set(pp)-{'motor_body','main_horn','slave_horn'}
 for key,s in pp.items():
  assert s.isValid() and s.Volume()>0,(key,'invalid geometry')
  if key in printable: assert len(s.Solids())==1,(key,'must be a single connected printable solid')
  status[key]={'valid':s.isValid(),'solids':len(s.Solids()),'volume_mm3':s.Volume(),
               'bbox_mm':[getattr(s.BoundingBox(),a) for a in ['xmin','ymin','zmin','xmax','ymax','zmax']]}
  cq.exporters.export(s,str(ROOT/'parts_step'/f'{key}.step'))
  if key in printable:
   # Put the local part above the STL z=0 plane. Orient on a flat lateral face
   # for finger hinges; axes along Z reduce support in the pin bores.
   m=s
   if key in {'root','proximal','distal'}: m=m.rotate((0,0,0),(0,1,0),90)
   bb=m.BoundingBox();m=m.translate((-bb.xmin,-bb.ymin,-bb.zmin))
   mesh_path=ROOT/'print_stl'/f'{key}.stl'
   cq.exporters.export(m,str(mesh_path),tolerance=.055,angularTolerance=.12)
   import trimesh
   mesh=trimesh.load_mesh(mesh_path,process=True)
   before=len(mesh.faces)
   # OCCT emits two zero-area seam triangles on each split guide. Remove only
   # degenerate/duplicate facets; never fill an intentional hole or modify CAD.
   mesh.update_faces(mesh.nondegenerate_faces(height=1e-7))
   mesh.update_faces(mesh.unique_faces());mesh.remove_unreferenced_vertices()
   if len(mesh.faces)!=before:mesh.export(mesh_path)
   assert mesh.is_watertight and mesh.is_winding_consistent and mesh.body_count==1,(key,'bad print mesh')
   status[key]['mesh_degenerate_facets_removed']=before-len(mesh.faces)
 print('All component solids valid',flush=True)
 for name,qs in P['poses'].items():
  if args.pose not in ('ALL',name):continue
  print('Exporting',name,qs,flush=True)
  assy,items=make_assembly(pp,*qs,name)
  assy.export(str(ROOT/'cad'/f'{name}_ASSEMBLY.step'))
  if name=='OPEN':
   single=comp(x['shape'] for x in items)
   export_single_product(single,ROOT/'00_OPEN_IN_SOLIDWORKS.step')
  save_instances(items,ROOT/'checks'/f'{name}_instances.json')
  print('Exported',len(items),'components',flush=True)
 src=ROOT/'reference/teacher_dual_axis_servo_original.stp'
 data={'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),
       'teacher_source_solids':8,'unused_source_solid_indices':[2,3],
       'parts':status,'parameters':P,'cadquery_version':cq.__version__,
       'neutral_route_length_mm':cable_length(0,0),
       'poses':{n:{'q_deg':q,'route_length_mm':cable_length(*q),
                    'takeup_mm':cable_length(0,0)-cable_length(*q),
                    'spool_rotation_deg':math.degrees((cable_length(0,0)-cable_length(*q))/P['spool_cable_center_radius'])}
                for n,q in P['poses'].items()}}
 (ROOT/'checks/build_report.json').write_text(json.dumps(data,indent=2))
 print('Done',flush=True)
if __name__=='__main__':main()
