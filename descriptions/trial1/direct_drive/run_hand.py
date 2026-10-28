#!/usr/bin/env python3
"""Run the CAD-matched MuJoCo direct-drive hand using conda newton mjpython."""
import argparse
from pathlib import Path
import time
import numpy as np
import mujoco

HERE=Path(__file__).resolve().parent

def target(t):
 t=t%18.0
 if t<1.5:return 0.0
 if t<5.5:return .70*(t-1.5)/4
 if t<15:return .70
 if t<17:return .70*(17-t)/2
 return 0.0

def lift_target(t):
 t=t%18.0
 if t<7:return 0.0
 if t<10:return .04*(t-7)/3
 if t<11:return .04
 if t<14:return .04*(14-t)/3
 return 0.0


def check(model):
 data=mujoco.MjData(model)
 fingers=np.array([mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,f'joint{i}') for i in range(1,4)])
 addrs=model.jnt_qposadr[fingers]
 oid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'object_collision')
 padids={mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,f'pad_collision{i}') for i in range(1,4)}
 pedestal=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'pedestal')
 palmids={mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'palm_contact_'+n) for n in ('left','right','front','back')}
 seen=set();maxforce=0.;max_simultaneous=0;peak=0.
 for _ in range(18000):
  data.ctrl[0]=lift_target(data.time);data.ctrl[1:]=target(data.time)
  mujoco.mj_step(model,data)
  if not np.isfinite(data.qpos).all():raise RuntimeError('non-finite state')
  current=set();peak=max(peak,float(data.body('object').xpos[2]))
  for j in range(data.ncon):
   con=data.contact[j]
   if pedestal in (con.geom1,con.geom2) and ({con.geom1,con.geom2}&palmids):
    raise RuntimeError('pedestal collides with palm')
   matched={con.geom1,con.geom2}&padids
   if oid in (con.geom1,con.geom2) and matched:
    force=np.zeros(6);mujoco.mj_contactForce(model,data,j,force)
    if force[0]>1e-3:current.update(matched)
    maxforce=max(maxforce,force[0])
  seen.update(current);max_simultaneous=max(max_simultaneous,len(current))
 if max_simultaneous!=3 or maxforce<=0:raise RuntimeError('three simultaneous loaded contacts not observed')
 if peak<0.075:raise RuntimeError('object not lifted')
 if abs(data.body('object').xpos[2]-0.05)>0.004:raise RuntimeError('object not returned to pedestal')
 if np.max(np.abs(data.qpos[addrs]))>0.04:raise RuntimeError('fingers did not reopen')
 print('PASS: three simultaneous contacts, object lifted and returned, fingers reopened')
 print('peak object center:',round(peak*1000,2),'mm; maximum normal force:',round(maxforce,3),'N')
 print('final finger angles:',np.round(np.degrees(data.qpos[addrs]),3),'deg; object:',np.round(data.body('object').xpos,5),'m')
 # Verify independent actuation without contact or gravity.
 gravity=model.opt.gravity.copy();model.opt.gravity[:]=0
 try:
  obj_joint=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'object_free')
  for k in range(3):
   d=mujoco.MjData(model);adr=model.jnt_qposadr[obj_joint];d.qpos[adr:adr+3]=(0,0,1)
   d.ctrl[1+k]=0.45
   for _ in range(4000):mujoco.mj_step(model,d)
   q=d.qpos[addrs]
   if abs(q[k]-.45)>.04 or np.max(np.abs(np.delete(q,k)))>.02:
    raise RuntimeError(f'finger {k+1} independent actuation failed: {q}')
  print('PASS: each finger moves independently')
 finally:model.opt.gravity[:]=gravity


def main():
 p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');p.add_argument('--seconds',type=float,default=0,help='Exit viewer after N simulation seconds; 0 means until closed')
 args=p.parse_args()
 model=mujoco.MjModel.from_xml_path(str(HERE/'hand.xml'))
 if args.check:check(model);return
 from mujoco import viewer as mj_viewer
 data=mujoco.MjData(model)
 with mj_viewer.launch_passive(model,data) as viewer:
  print('MuJoCo viewer opened', flush=True)
  viewer.cam.azimuth=130;viewer.cam.elevation=-23;viewer.cam.distance=.45;viewer.cam.lookat[:]=(0,0,.045)
  now=time.monotonic()
  while viewer.is_running() and (not args.seconds or data.time<args.seconds):
   data.ctrl[0]=lift_target(data.time);data.ctrl[1:]=target(data.time)
   mujoco.mj_step(model,data);viewer.sync()
   wait=data.time-(time.monotonic()-now)
   if wait>0:time.sleep(wait)

if __name__=='__main__':main()
