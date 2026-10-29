"""Render the actual MJCF meshes and simulation states; run using mjpython."""
from pathlib import Path
from PIL import Image
import mujoco as mj
from run_hand import Simulation,commands

HERE=Path(__file__).resolve().parent
s=Simulation();cam=mj.MjvCamera();cam.azimuth=135;cam.elevation=-22;cam.distance=.5;cam.lookat[:]=(0,0,.02)
with mj.Renderer(s.model,height=1100,width=1400) as r:
    for t,file in [(0.,'preview_open.png'),(6.5,'preview_grasp.png'),(10.5,'preview_lift.png')]:
        while s.data.time<t:s.step(commands(s.data.time))
        mj.mj_forward(s.model,s.data);r.update_scene(s.data,cam);Image.fromarray(r.render()).save(HERE/file)
        print('rendered',file,flush=True)
    s.reset();cam.elevation=25;cam.lookat[:]=(0,0,-.02)
    r.update_scene(s.data,cam);Image.fromarray(r.render()).save(HERE/'preview_drive.png')
