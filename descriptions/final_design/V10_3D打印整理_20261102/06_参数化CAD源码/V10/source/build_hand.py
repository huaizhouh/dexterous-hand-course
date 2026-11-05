"""ThreeFinger V10. Fixed outward root mount and tapered soft fingertips; mm.

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
 'palm':(.22,.34,.42), 'handle':(.20,.29,.34), 'root':(.74,.81,.84),
 'proximal':(.80,.86,.89), 'distal':(.80,.86,.89), 'pad_proximal':(.10,.51,.55),
 'pad_distal':(.10,.51,.55), 'motor_body':(.14,.16,.19),
 'main_horn':(.62,.64,.66), 'slave_horn':(.62,.64,.66),
 'cradle':(.31,.40,.46), 'clamp':(.31,.40,.46), 'spool':(.91,.43,.17),
 'guide_front':(.33,.46,.50),'guide_back':(.33,.46,.50),
 'tendon':(.86,.13,.13),'elastic':(.93,.70,.10),
 'metal':(.66,.68,.70), 'liner':(.19,.23,.24)
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

def hinge_frame(i):
 return frame(i)@R('x',-P['root_mount_outward_deg'])
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
 # Strengthened neck and captive flange; seating width holds parallel loops.
 base_y=P['root_peg_base_y'] if base_y is None else base_y
 width=P['elastic_neck_width']
 return fuse(cyl(P['elastic_neck_diameter']/2,width,(0,base_y,z),(0,1,0)),
             cyl(P['elastic_cap_diameter']/2,1.5,(0,base_y+width,z),(0,1,0)))
def pivot_bore(s,z,diameter=None,parent=False):
 s=cut(s,cyl((diameter or P['joint_hole_diameter'])/2,40,(-20,0,z),(1,0,0)))
 if parent:
  outer=P['joint_fork_outer_width']/2;r=P['joint_outer_washer_od']/2+.25
  s=cut(s,cyl(r,20-outer,(-20,0,z),(1,0,0)),cyl(r,P['joint_nut_relief_end_x']-outer+.01,(outer-.01,0,z),(1,0,0)))
 return s

def stop_components():
 """Shared physical stop primitives. Fixed noses are rotated to the specified inward bias angle.

 Positive rotation flexes inward. Opposing wing/nose faces coincide at the
 specified opening angle; further extension causes real solid contact.
 """
 out=[]
 if not P.get('mechanical_extension_stops_enabled',True):return out
 def build(b):
  cc=np.asarray(b['center_unrotated_mm']);size=np.asarray(b['size_mm'])
  return transform(box(*(cc-size/2),*size),R('x',b['rotation_x_deg']))
 for g in __import__('joint_geometry').geometry(P)['items']:
  stem,foot,nose,wing=[build(g[k]) for k in ('stem','foot','nose','wing')]
  out.append({'side':g['side'],'parent':fuse(stem,foot,nose),'nose':nose,'wing':wing})
 return out

def sleeve_part():
 return tube(P['joint_sleeve_od']/2,P['joint_sleeve_id']/2,P['joint_sleeve_length'],(-P['joint_sleeve_length']/2,0,0),(1,0,0))

def nut_cage():
 # Open hex pocket keeps the dry-clamped bolt/spacer stationary for the viscous
 # damper. Relief around washer/nut leaves axial clamp load on the metal tube.
 x=P['joint_fork_outer_width']/2;end=P['joint_nut_cage_end_x'];relief=P['joint_nut_relief_end_x']
 s=cyl(P['joint_nut_cage_outer_radius'],end-x,(x,0,0),(1,0,0))
 s=cut(s,cyl(P['joint_nut_relief_radius'],relief-x+.02,(x-.01,0,0),(1,0,0)))
 pocket=cq.Workplane('XY').polygon(6,P['joint_nut_pocket_af']/math.cos(math.pi/6)).extrude(end-relief+.2).val().rotate((0,0,0),(0,1,0),90).translate((relief-.01,0,0))
 return cut(s,pocket)

def root_part():
 # Upper fork tilts around its hinge; mounting plate and servo stay horizontal.
 # Supporting ribs bridge the tilted fork into the original four-bolt plate.
 s=box(-P['joint_fork_outer_width']/2,5,-14,P['joint_fork_outer_width'],P['root_peg_base_y']-5,7.6)
 outer=P['joint_fork_outer_width']/2;inner=P['joint_fork_inner_width']/2;t=outer-inner
 for x in (-outer,inner):
  s=fuse(s,box(x,-8.5,-14,t,17,14),cyl(8.5,t,(x,0,0),(1,0,0)))
 s=fuse(s,box(-outer,5,-14,2*outer,P['root_peg_base_y']-5,7.6),peg(-8))
 for stop in stop_components():s=fuse(s,stop['parent'])
 s=fuse(s,nut_cage())
 s=pivot_bore(s,0,P['joint_fork_bore_diameter'],parent=True)
 s=transform(s,R('x',-P['root_mount_outward_deg']))
 s=fuse(s,box(-21,-14,-18,42,28,4))
 for x in (-outer,inner):s=fuse(s,box(x,-11.5,-16,t,19,4.5))
 s=cut(s,cyl(1.55,12,(0,GUIDE_Y,-19)))
 for x in (-17,17):
  for y in (-4,4): s=cut(s,cyl(1.7,6,(x,y,-19)))
 return s

def proximal_part():
 s=fuse(cyl(7,16,(-8,0,0),(1,0,0)),box(-8,-P['finger_body_thickness']/2,0,16,P['finger_body_thickness'],L-14))
 outer=P['joint_fork_outer_width']/2;inner=P['joint_fork_inner_width']/2;t=outer-inner
 s=fuse(s,box(-outer,-P['proximal_crossbar_depth']/2,L-20,2*outer,P['proximal_crossbar_depth'],P['proximal_crossbar_thickness']))
 for x in (-outer,inner):
  s=fuse(s,box(x,-8.5,L-14,t,17,14),cyl(8.5,t,(x,0,L),(1,0,0)))
 # A closed palmar tendon tunnel, integral with the contact-pad backing.
 backing_width=P.get('proximal_pad_backing_width_mm',16.)
 s=fuse(s,cyl(2.3,L-24,(0,-7.5,12)),box(-backing_width/2,-9.8,14,backing_width,4.5,36),
        box(-5,5,10,10,P['root_peg_base_y']-5,8),box(-8,5,L-17,16,P['pip_peg_base_y']-5,8),peg(14),peg(L-13,P['pip_peg_base_y']))
 s=cut(s,cyl(1.2,L-18,(0,-7.5,11)),box(-1.4,-10,L-13,2.8,3.9,5.2))
 for stop in stop_components():s=fuse(s,stop['wing'],stop['parent'].translate((0,0,L)))
 s=fuse(s,nut_cage().translate((0,0,L)))
 return pivot_bore(pivot_bore(s,0),L,P['joint_fork_bore_diameter'],parent=True)

def distal_part():
 # Straight industrial phalanx, tapered at the distal end.
 poly=[(-8,0),(8,0),(8,D-14),(4,D-5),(3,D),(-3,D),(-4,D-5),(-8,D-14)]
 # XZ plane extrudes in -Y. Centre the section on y=0.
 beam=cq.Workplane('XZ').polyline(poly).close().extrude(P['finger_body_thickness']).val().translate((0,P['finger_body_thickness']/2,0))
 backing_width=P.get('distal_pad_backing_width_mm',14.)
 backing_profile=[(-backing_width/2,10),(backing_width/2,10),
                  (backing_width/2,34),(4,43),(3,D),(-3,D),(-4,43),(-backing_width/2,34)]
 backing=cq.Workplane('XZ').polyline(backing_profile).close().extrude(4.5).val().translate((0,-5.3,0))
 s=fuse(cyl(7,16,(-8,0,0),(1,0,0)),beam,
        cyl(2.3,32,(0,-7.5,10)),backing,
        box(-3,-9.8,40,6,5.3,6),box(-5,5,10,10,P['pip_peg_base_y']-5,8),peg(14,P['pip_peg_base_y']))
 s=cut(s,cyl(1.2,39,(0,-7.5,9)),cyl(2.5,4.1,(0,-7.5,44)))
 for stop in stop_components():s=fuse(s,stop['wing'])
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
 h=P['palm_thickness'];rail=P['palm_mount_rail_width'];spoke=P['palm_spoke_width']
 s=cyl(P['palm_hub_diameter']/2,h,(0,0,0))
 for i in range(3):
  # Narrow rails connect the existing four servo bolts. No outer plate/rim.
  lobe=box(-spoke/2,20,0,spoke,RAD+15-20,h)
  landing=cq.Workplane('XY').rect(56,30).extrude(h).edges('|Z').fillet(3).val().translate((0,RAD,0))
  lobe=fuse(lobe,landing)
  for x in MOUNT_X:
   lobe=fuse(lobe,box(x-rail/2,RAD+MOUNT_Y[0],0,rail,MOUNT_Y[1]-MOUNT_Y[0],h))
   for y in MOUNT_Y:lobe=fuse(lobe,cyl(rail/2,h,(x,RAD+y,0)))
  for y in MOUNT_Y:lobe=fuse(lobe,box(MOUNT_X[0],RAD+y-rail/2,0,MOUNT_X[1]-MOUNT_X[0],rail,h))
  # Small ears remain under the existing split-guide mounting screws.
  for x in (24,34):lobe=fuse(lobe,cyl(5,h,(x,RAD+GUIDE_Y+6.1,0)))
  lobe=fuse(lobe,box(23,RAD+GUIDE_Y+1.1,0,11,10,h))
  s=fuse(s,transform(lobe,R('z',120*i)))
 s=cut(s,cyl(P['handle_cable_bore_diameter']/2,h+2,(0,0,-1)))
 for i in range(3):
  b=frame(i)
  for x in (-17,17):
   for y in (-4,4): s=cut(s,transform(cyl(1.7,8,(x,y,-25)),b))
  for x in MOUNT_X:
   for y in MOUNT_Y: s=cut(s,transform(cyl(1.7,8,(x,y,-25)),b))
  s=cut(s,transform(box(-3.2,GUIDE_Y-3.5,-25,6.4,7,8),b))
  for x in (24,34): s=cut(s,transform(cyl(1.35,8,(x,GUIDE_Y+6.1,-25)),b))
 for x,y in handle_bolts():s=cut(s,cyl(P['handle_mount_hole_diameter']/2,h+2,(x,y,-1)))
 return s

def handle_bolts():
 rr=P['handle_mount_bolt_circle_diameter']/2
 return [(rr*math.cos(math.radians(a)),rr*math.sin(math.radians(a))) for a in (30,150,270)]

def handle_part():
 rr=P['handle_grip_diameter']/2;bottom=P['handle_bottom'];bh=P['handle_butt_height']
 top=P['handle_grip_top_z'];ft=P['handle_flange_thickness']
 s=fuse(cyl(P['handle_flange_diameter']/2,ft,(0,0,-ft)),
        cq.Solid.makeCone(rr,rr+1,-ft-top,cq.Vector(0,0,top)),
        cyl(rr,P['handle_grip_length'],(0,0,top-P['handle_grip_length'])),
        cq.Solid.makeCone(P['handle_butt_diameter']/2,rr,bh,cq.Vector(0,0,bottom)))
 s=cut(s,cyl(P['handle_cable_bore_diameter']/2,-bottom+2,(0,0,bottom-1)))
 for z in np.arange(top-P['handle_grip_length']+8,top-7,12):s=cut(s,tube(rr+1,rr-1,2,(0,0,float(z))))
 for x,y in handle_bolts():
  s=cut(s,cyl(P['handle_mount_hole_diameter']/2,ft+2,(x,y,-ft-1)))
  pocket=cq.Workplane('XY').polygon(6,7.3/math.cos(math.pi/6)).extrude(3.4).val().translate((x,y,-ft))
  s=cut(s,pocket)
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
 pp={'palm':palm_part(),'handle':handle_part(),'root':root_part(),
     'proximal':proximal_part(),'distal':distal_part(),
     'pad_proximal':__import__('pad_geometry').pad('proximal',P)[0],'pad_distal':__import__('pad_geometry').pad('distal',P)[0],
     'cradle':cradle_part(),'clamp':clamp_part(),'spool':spool_part(),
     'guide_front':gf,'guide_back':gb}
 pp.update(servo_parts())
 pp['joint_sleeve']=sleeve_part()
 return pp

def tendon_points(i,q1,q2):
 b=frame(i);pr=hinge_frame(i)@R('x',q1);di=pr@T(z=L)@R('x',q2)
 pts=[point(b,(33,GUIDE_Y,-56))]+[point(b,p) for p in guide_path()]
 pts += [point(b,(0,GUIDE_Y,-8)),point(pr,(0,-7.5,12)),point(pr,(0,-7.5,L-12)),
         point(di,(0,-7.5,10)),point(di,(0,-7.5,44))]
 return pts
def cable_length(q1,q2):
 pts=tendon_points(0,q1,q2)
 return sum(float(np.linalg.norm(b-a)) for a,b in zip(pts,pts[1:]))
def elastic_paths(b,pr,di):
 paths=[]
 h=b@R('x',-P['root_mount_outward_deg'])
 for m0,m1,z0,z1,joint,yy,cy in [(h,pr,-8,14,h,P['root_band_line_y'],P['root_band_corner_y']),
   (pr@T(z=L),di,-13,14,pr@T(z=L),P['pip_band_line_y']-1.5,P['pip_band_corner_y']-1.5),
   (pr@T(z=L),di,-13,14,pr@T(z=L),P['pip_band_line_y']+1.5,P['pip_band_corner_y']+1.5)]:
  # One closed loop per hinge: two strands and two captive rounded ends.
  path=[point(m0,(3.2,yy,z0)),point(joint,(3.2,cy,0)),point(m1,(3.2,yy,z1))]
  path += [point(m1,(3.2*math.cos(a),yy,z1+3.2*math.sin(a))) for a in np.linspace(0,math.pi,10)[1:]]
  path += [point(joint,(-3.2,cy,0)),point(m0,(-3.2,yy,z0))]
  path += [point(m0,(3.2*math.cos(a),yy,z0+3.2*math.sin(a))) for a in np.linspace(math.pi,2*math.pi,10)[1:]]
  paths.append(path)
 return paths

def make_assembly(pp,q1,q2,name,flex=True,fasteners=True):
 assy=cq.Assembly(name=name); instances=[]
 def add(key,shape,m=np.eye(4),group='',kind=None):
  shape=transform(shape,m)
  color=COLORS.get(kind or key,COLORS['metal'])
  assy.add(shape,name=key,color=cq.Color(*color))
  instances.append({'name':key,'kind':kind or key,'group':group,'matrix_mm':m.tolist(),'shape':shape})
 add('palm',pp['palm']);add('handle',pp['handle'])
 for j,(x,y) in enumerate(handle_bolts()):
  if fasteners:
   add(f'handle_bolt_{j}',screw(2,16,(x,y,6.8),(0,0,-1),3.5,3),kind='metal')
   add(f'handle_nut_{j}',hexnut(2.1,7,3.2,(x,y,-8)),kind='metal')
   add(f'handle_washer_{j}',tube(4.5,2.15,.8,(x,y,6)),kind='metal')
 for i in range(3):
  group=f'finger_{chr(65+i)}';b=frame(i);pr=hinge_frame(i)@R('x',q1);di=pr@T(z=L)@R('x',q2)
  motor=b@T(z=-56)
  takeup=cable_length(*P['poses']['OPEN'])-cable_length(q1,q2)
  winch=motor@R('x',-math.degrees(takeup/P['spool_cable_center_radius']))
  for key,m in [('root',b),('proximal',pr),('distal',di),('pad_proximal',pr),('pad_distal',di),
                ('cradle',motor),('clamp',motor),('motor_body',motor),('main_horn',winch),
                ('slave_horn',winch),('spool',winch),('guide_front',b),('guide_back',b)]:
   add(f'{group}_{key}',pp[key],m,group,kind=key)
  # Shim sheets. Their compression is set during trial assembly.
  add(f'{group}_liner_bottom',box(-22,-9.5,-11,43,39,.8),motor,group,'liner')
  add(f'{group}_liner_top',box(-22,-9.5,10.05,43,39,.55),motor,group,'liner')
  if fasteners:
   for j,jm in enumerate((hinge_frame(i),pr@T(z=L))):
    sleeve_end=P['joint_sleeve_length']/2;wt=P['joint_outer_washer_thickness']
    add(f'{group}_sleeve_{j}',pp['joint_sleeve'],jm,group,'metal')
    add(f'{group}_pin_{j}',screw(1.5,P['joint_pin_bolt_length'],(-sleeve_end-wt,0,0),(1,0,0),2.75,2.4),jm,group,'metal')
    add(f'{group}_pin_nut_{j}',hexnut(1.6,5.5,P['joint_pin_nut_height'],(sleeve_end+wt,0,0),'x'),jm,group,'metal')
    for side in (-1,1):
     it=P['joint_thrust_washer_thickness'];half=P['joint_rotor_width']/2
     add(f'{group}_washer_{j}_{side}',tube(P['joint_thrust_washer_od']/2,P['joint_thrust_washer_id']/2,it,(-half-it if side<0 else half,0,0),(1,0,0)),jm,group,'metal')
     add(f'{group}_outer_washer_{j}_{side}',tube(P['joint_outer_washer_od']/2,P['joint_outer_washer_id']/2,wt,(-sleeve_end-wt if side<0 else sleeve_end,0,0),(1,0,0)),jm,group,'metal')
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

def export_test_parts(pp):
 out=ROOT/'calibration';out.mkdir(exist_ok=True)
 test={'joint_trial_parent':pp['root'],
       'joint_trial_child':pp['proximal'].intersect(box(-30,-30,-20,60,70,54))}
 coupon=box(0,0,0,80,18,6)
 for i,d in enumerate([7.05,7.10,7.20,7.25,7.30]):coupon=cut(coupon,cyl(d/2,8,(10+15*i,9,-1)))
 test['bore_coupon_705_710_720_725_730']=coupon
 for key,shape in test.items():
  assert shape.isValid() and len(shape.Solids())==1
  cq.exporters.export(shape,str(out/(key+'.step')))
  if key!='bore_coupon_705_710_720_725_730':shape=shape.rotate((0,0,0),(0,1,0),90)
  bb=shape.BoundingBox();shape=shape.translate((-bb.xmin,-bb.ymin,-bb.zmin))
  cq.exporters.export(shape,str(out/(key+'.stl')),tolerance=.04,angularTolerance=.1)
  import trimesh
  mesh=trimesh.load_mesh(out/(key+'.stl'));assert mesh.is_watertight and mesh.body_count==1


def main():
 ap=argparse.ArgumentParser();ap.add_argument('--pose',default='ALL');args=ap.parse_args()
 for directory in ('parts_step','print_stl','checks','cad'):(ROOT/directory).mkdir(exist_ok=True)
 print('Building analytic solids',flush=True);__import__('pad_geometry').export_collision(P)
 __import__('joint_geometry').export(P,ROOT/'source/joint_contact.json');pp=parts()
 status={}
 printable=set(pp)-{'motor_body','main_horn','slave_horn','joint_sleeve'}
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
   if key=='handle':m=m.rotate((0,0,0),(1,0,0),180)
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
 export_test_parts(pp)
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
 inertials=__import__('mass_properties').moving_links(pp,P)
 data={'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),
       'teacher_source_solids':8,'unused_source_solid_indices':[2,3],
       'parts':status,'parameters':P,'cadquery_version':cq.__version__,'moving_inertials':inertials,
       'neutral_route_length_mm':cable_length(*P['poses']['OPEN']),
       'poses':{n:{'q_deg':q,'route_length_mm':cable_length(*q),
                    'takeup_mm':cable_length(*P['poses']['OPEN'])-cable_length(*q),
                    'spool_rotation_deg':math.degrees((cable_length(*P['poses']['OPEN'])-cable_length(*q))/P['spool_cable_center_radius'])}
                for n,q in P['poses'].items()}}
 (ROOT/'checks/build_report.json').write_text(json.dumps(data,indent=2))
 print('Done',flush=True)
if __name__=='__main__':main()
