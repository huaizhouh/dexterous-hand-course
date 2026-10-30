"""CPU orthographic CAD preview, shaded from tessellated deliverable geometry."""
from pathlib import Path
import argparse,json,sys
import numpy as np
import cadquery as cq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from build_hand import ROOT,COLORS,parts,make_assembly,frame,R,T,L,transform

def render(items,path,eye=(1.5,-2.7,1.7),title='',labels=None,figsize=(11,10)):
 import os
 os.environ.setdefault('PYOPENGL_PLATFORM','egl')
 os.environ.setdefault('EGL_PLATFORM','surfaceless')
 import pyrender,trimesh
 from PIL import Image,ImageDraw,ImageFont
 scene=pyrender.Scene(bg_color=[245,247,249,255],ambient_light=[.24,.24,.24])
 allv=[]
 for item in items:
  vv,ff=item['shape'].tessellate(.12,.16)
  v=np.array([p.toTuple() for p in vv]);allv.append(v)
  # Flat faces keep the industrial geometry legible; curved surfaces use
  # smooth normals within each tessellated B-rep face.
  mesh=trimesh.Trimesh(vertices=v,faces=np.array(ff),process=False)
  mat=pyrender.MetallicRoughnessMaterial(baseColorFactor=(*COLORS.get(item['kind'],COLORS['metal']),1),
           metallicFactor=.15 if item['kind'] in ('metal','main_horn','slave_horn') else .02,roughnessFactor=.65)
  scene.add(pyrender.Mesh.from_trimesh(mesh,material=mat,smooth=False))
 v=np.concatenate(allv);center=(v.min(axis=0)+v.max(axis=0))/2
 eye=np.array(eye,dtype=float);eye/=np.linalg.norm(eye)
 right=np.cross([0,0,1],eye);right/=np.linalg.norm(right);up=np.cross(eye,right)
 basis=np.array([right,up,eye]).T
 proj=(v-center)@basis
 width,height=int(figsize[0]*170),int(figsize[1]*170)
 aspect=width/height
 ym=max(np.ptp(proj[:,1])*.59,np.ptp(proj[:,0])*.59/aspect)
 xm=ym*aspect
 pose=np.eye(4);pose[:3,:3]=basis;pose[:3,3]=center+eye*700
 camera=pyrender.OrthographicCamera(xmag=xm,ymag=ym,znear=.1,zfar=2000)
 scene.add(camera,pose=pose)
 for direction,intensity in [((1,-2,3),1.7),((-3,-1,1),.45),((0,3,2),.7)]:
  d=np.array(direction,dtype=float);d/=np.linalg.norm(d);rr=np.cross([0,0,1],d);rr/=np.linalg.norm(rr)
  lp=np.eye(4);lp[:3,:3]=np.array([rr,np.cross(d,rr),d]).T;lp[:3,3]=center+d*500
  scene.add(pyrender.DirectionalLight(color=np.ones(3),intensity=intensity),pose=lp)
 renderer=pyrender.OffscreenRenderer(width,height)
 rgb,_=renderer.render(scene,flags=pyrender.RenderFlags.NONE);renderer.delete()
 im=Image.fromarray(rgb);draw=ImageDraw.Draw(im)
 font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',max(18,int(width/65)))
 draw.rectangle((0,0,width,65),fill=(245,247,249));draw.text((22,16),title,font=font,fill=(35,53,67))
 # Figure labels are laid out on a separate engineering drawing page.
 im.save(path)
 print(path,flush=True)

def main():
 pp=parts()
 for name,qs in json.loads((ROOT/'source/parameters.json').read_text())['poses'].items():
  _,items=make_assembly(pp,*qs,name,True,True)
  render(items,ROOT/'preview'/f'{name}.png',title=f'THREEFINGER V5  /  {name.replace("_"," ")}',figsize=(11,11))
  if name=='OPEN':
   # One-finger view includes its real servo and guide block.
   one=[x for x in items if x['group']=='finger_A']
   render(one,ROOT/'preview/FINGER_A_DETAIL.png',eye=(2,-2,1.1),title='110 mm finger / two passive hinges / one tendon',figsize=(9,12),
       labels=[('Captive elastic pegs',(0,78,38),(35,20)),('Internal tendon path',(0,57.5,65),(-130,15)),
               ('Actual teacher servo',(0,71,-31),(30,-40)),('Horn-mounted spool',(33,65,-32),(40,10))])
   render(items,ROOT/'preview/TOP_LAYOUT.png',eye=(.002,-.001,1),title='210 x 210 mm rectangular frame / 120-degree layout',figsize=(10,10))
 # Detail of open split tendon guide, one half offset for visibility.
 render([{'shape':pp['guide_back'],'kind':'guide_back'},
         {'shape':pp['guide_front'].translate((0,-10,0)),'kind':'guide_front'}],
        ROOT/'preview/SPLIT_GUIDE.png',eye=(1,-2,1.4),title='Split tendon guide / accessible radiused channel',figsize=(9,5))
if __name__=='__main__':main()
