#!/usr/bin/env python3
"""Create MuJoCo model from the CAD-generated component meshes."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent

def add(parent,tag,**attrs):return ET.SubElement(parent,tag,{k:str(v) for k,v in attrs.items()})

def build():
 root=ET.Element('mujoco',model='direct_drive_three_finger')
 add(root,'compiler',angle='radian',meshdir='meshes',autolimits='true')
 add(root,'option',timestep='0.001',gravity='0 0 -9.81',integrator='implicitfast',iterations='100')
 add(root,'statistic',center='0 0 0.04',extent='0.3')
 visual=add(root,'visual');add(visual,'global',offwidth='1100',offheight='900');add(visual,'headlight',ambient='0.5 0.5 0.5',diffuse='0.65 0.65 0.65')
 assets=add(root,'asset')
 add(assets,'texture',name='sky',type='skybox',builtin='gradient',rgb1='0.3 0.37 0.45',rgb2='0.75 0.8 0.85',width='512',height='3072')
 for n in ('servo','horn_follower','horn_main','long_u','pad','palm','fasteners','ties'):
  add(assets,'mesh',name=n,file=n+'.stl',scale='0.001 0.001 0.001')
 world=add(root,'worldbody')
 add(world,'light',pos='0 -0.2 0.4',dir='0 0 -1',diffuse='0.8 0.8 0.8')
 add(world,'geom',name='pedestal',type='cylinder',pos='0 0 0.01',size='0.034 0.01',rgba='0.48 0.48 0.5 1',friction='2 0.02 0.002',solref='0.003 1',solimp='0.99 0.999 0.0001')
 rig=add(world,'body',name='rig')
 add(rig,'joint',name='lift',type='slide',axis='0 0 1',range='0 0.06',damping='30')
 add(rig,'inertial',pos='0 0 0.01',mass='2',diaginertia='0.02 0.02 0.02')
 add(rig,'geom',name='palm',type='mesh',mesh='palm',rgba='0.21 0.27 0.34 1',contype='0',conaffinity='0')
 # Conservative primitive contact ring leaves the 74 mm centre aperture open.
 for n,pos,sz in [('left','-0.0745 0 0.002','0.0375 0.07 0.002'),('right','0.0745 0 0.002','0.0375 0.07 0.002'),('front','0 0.0745 0.002','0.037 0.0375 0.002'),('back','0 -0.0745 0.002','0.037 0.0375 0.002')]:
  add(rig,'geom',name='palm_contact_'+n,type='box',pos=pos,size=sz,rgba='0 0 0 0',mass='0',contype='1',conaffinity='1')
 for i,degrees in enumerate((0,120,240),1):
  a=math.radians(degrees);x=-0.07*math.sin(a);y=0.07*math.cos(a)
  unit=add(rig,'body',name=f'unit{i}',pos=f'{x:.12f} {y:.12f} 0.02',euler=f'0 0 {a:.12f}')
  add(unit,'geom',name=f'ties{i}',type='mesh',mesh='ties',rgba='0.8 0.8 0.78 1',contype='0',conaffinity='0')
  add(unit,'geom',name=f'servo{i}',type='mesh',mesh='servo',rgba='0.18 0.21 0.24 1',contype='0',conaffinity='0')
  finger=add(unit,'body',name=f'finger{i}')
  add(finger,'joint',name=f'joint{i}',type='hinge',axis='1 0 0',range='0 0.75',damping='0.025',armature='0.0005')
  add(finger,'inertial',pos='0 0 0.02',mass='0.22',diaginertia='0.0002 0.0002 0.0002')
  for n,col in [('horn_follower','0.85 0.65 0.22 1'),('horn_main','0.85 0.65 0.22 1'),('long_u','0.65 0.71 0.76 1'),('pad','0.12 0.12 0.12 1'),('fasteners','0.72 0.72 0.68 1')]:
   add(finger,'geom',name=f'{n}{i}',type='mesh',mesh=n,rgba=col,contype='0',conaffinity='0')
  add(finger,'geom',name=f'pad_collision{i}',type='box',pos='0 -0.0085 0.035',size='0.014 0.004 0.015',rgba='0 0 0 0',friction='2 0.02 0.002',solref='0.003 1',solimp='0.99 0.999 0.0001',mass='0',contype='1',conaffinity='1')
 obj=add(world,'body',name='object',pos='0 0 0.05')
 add(obj,'freejoint',name='object_free')
 add(obj,'geom',name='object_collision',type='sphere',size='0.03',mass='0.05',rgba='0.92 0.36 0.15 1',friction='2 0.02 0.002',solref='0.003 1',solimp='0.99 0.999 0.0001')
 acts=add(root,'actuator')
 add(acts,'position',name='lift',joint='lift',kp='5000',ctrlrange='0 0.06',forcerange='-150 150',forcelimited='true')
 for i in range(1,4): add(acts,'position',name=f'servo{i}',joint=f'joint{i}',kp='2.0',ctrllimited='true',ctrlrange='0 0.75',forcelimited='true',forcerange='-1.0 1.0')
 ET.indent(root,space='  ')
 ET.ElementTree(root).write(HERE/'hand.xml',encoding='unicode')

if __name__=='__main__':build()
