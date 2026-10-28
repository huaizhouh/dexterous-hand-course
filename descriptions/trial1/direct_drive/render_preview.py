#!/usr/bin/env python3
"""Render two frames of the same MuJoCo CAD-mesh model; run with mjpython."""
from pathlib import Path
import numpy as np
from PIL import Image
import mujoco
from run_hand import target, lift_target

here=Path(__file__).resolve().parent
m=mujoco.MjModel.from_xml_path(str(here/'hand.xml'));d=mujoco.MjData(m)
cam=mujoco.MjvCamera();cam.azimuth=130;cam.elevation=-23;cam.distance=.45;cam.lookat[:]=(0,0,.045)
with mujoco.Renderer(m,height=900,width=1100) as render:
 for t,name in [(0.,'preview_open.png'),(7.,'preview_grasp.png'),(10.5,'preview_lift.png')]:
  while d.time<t:
   d.ctrl[0]=lift_target(d.time);d.ctrl[1:]=target(d.time);mujoco.mj_step(m,d)
  mujoco.mj_forward(m,d);render.update_scene(d,cam)
  Image.fromarray(render.render()).save(here/name)
  print('rendered',name,flush=True)
