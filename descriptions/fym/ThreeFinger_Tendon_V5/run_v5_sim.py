#!/usr/bin/env python3
"""V5 tendon hand simulation. Keep this script in the complete V5 project root.

Python 3.11; tested with mujoco==3.3.7 and NumPy. No CADQuery/Newton required.
  python run_v5_sim.py --check
  python run_v5_sim.py --mode motion
  python run_v5_sim.py
  python run_v5_sim.py --manual
  python run_v5_sim.py --check --object all

V5 STL meshes are restored from their print orientation using build_report.
TPU pad collisions use the same convex cells as the CAD; rigid link collisions
are approximated by boxes/cylinders. The motor housing visual is approximate.
Three physical winch joints pull three unilateral elastic cables. Cable tensions
act through MuJoCo spatial tendons and exert equal reel reaction torques. The six
finger joints have no position actuators. Nine physical rubber loops are
represented by six effective spatial springs (one root, two parallel PIP loops).
Only the hand base is moved by a virtual test fixture. Objects are free bodies;
they receive gravity and contact forces, never attachment or scripted lifting.

Generated XML alone does not contain the Python cable/reel force coupling.
Always launch this script, not just the exported XML.

Assumed, NOT measured: friction, masses, band stiffness, cable stiffness, servo
torque and servo gains. Demonstrations do not certify a real grasp or CAD design.
The simulated extension/flexion limits are assumptions, not verified hard stops.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import queue
import struct
import sys
import time
import xml.etree.ElementTree as ET

try:
    import numpy as np
    import mujoco
except ImportError as exc:
    raise SystemExit(
        "Missing package: " + str(exc) + "\nRun in Anaconda Prompt:\n"
        '  conda activate newton\n  python -m pip install "mujoco==3.3.7" numpy\n'
    )

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "v5_sim_output"
VERSION = "V5-SIM-1.0"
NAMES = ("A", "B", "C")
DT = 0.001
DURATION = 18.0
COLORS = {
    "palm": ".22 .34 .42 1", "base": ".22 .34 .42 1",
    "column": ".39 .49 .54 1", "root": ".74 .81 .84 1",
    "proximal": ".80 .86 .89 1", "distal": ".80 .86 .89 1",
    "pad_proximal": ".10 .51 .55 1", "pad_distal": ".10 .51 .55 1",
    "cradle": ".31 .40 .46 1", "clamp": ".31 .40 .46 1",
    "spool": ".91 .43 .17 1", "guide_front": ".33 .46 .50 1",
    "guide_back": ".33 .46 .50 1", "footpad": ".19 .23 .24 1",
}
ASSUMPTIONS = {
    "motor_torque_limit_Nm": 0.15,
    "motor_kp_Nm_per_rad": 4.0,
    "motor_kv_Nms_per_rad": 0.025,
    "cable_stiffness_N_per_m": 2500.0,
    "cable_damping_Ns_per_m": 1.5,
    "tension_limit_N": 20.0,
    "root_band_stiffness_N_per_m": 100.0,
    "distal_band_stiffness_N_per_m": 250.0,
    "band_free_length_ratio": 0.98,
    "pad_friction": 1.0,
    "object_friction": 0.8,
    "proximal_mass_kg": 0.016,
    "distal_mass_kg": 0.013,
    "lift_height_m": 0.07,
    "collision": "CAD convex cells for TPU pads; analytic boxes/cylinders for rigid links",
    "servo_housing_visual": "approximate box; printed parts are actual V5 meshes",
    "calibration": "All mechanical/material values here are assumptions, not measured hardware data.",
}
OBJECTS = {
    "sphere": dict(type="sphere", size=(.030,), mass=.035, center=.0425, base=.136, takeup=12., label="60 mm sphere"),
    "pingpong": dict(type="sphere", size=(.020,), mass=.0027, center=.0325, base=.136, takeup=10., label="40 mm 2.7 g ping-pong ball"),
    "cube": dict(type="box", size=(.020,.020,.020), mass=.025, center=.042, base=.138, takeup=14., label="40 mm cube"),
    "cylinder": dict(type="cylinder", size=(.020,.025), mass=.025, center=.043, base=.140, takeup=14., label="40 x 50 mm cylinder"),
    "capsule": dict(type="capsule", size=(.010,.025), mass=.015, center=.045, base=.146, takeup=16., label="20 x 70 mm capsule"),
    "small_sphere": dict(type="sphere", size=(.005,), mass=.002, center=.069, base=.146, takeup=16., label="10 mm sphere (raised support)"),
}

def vec(values):
    return " ".join(f"{float(v):.10g}" for v in values)

def add(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k,v in attrs.items()})

def load_inputs():
    wanted = [ROOT/"source/parameters.json", ROOT/"checks/build_report.json", ROOT/"print_stl"]
    if not all(p.exists() for p in wanted):
        raise RuntimeError("Keep run_v5_sim.py in the V5 ROOT folder beside print_stl, source, simulation and checks.\n"
                           f"Current script folder: {ROOT}")
    p = json.loads(wanted[0].read_text(encoding="utf-8"))
    b = json.loads(wanted[1].read_text(encoding="utf-8"))
    OUT.mkdir(exist_ok=True)
    return p,b

def read_stl(path):
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0] if len(raw)>=84 else -1
    if 84+50*count == len(raw):
        dtype=np.dtype([("normal","<f4",(3,)),("v","<f4",(3,3)),("attr","<u2")])
        triangles = np.frombuffer(raw,dtype=dtype,count=count,offset=84)["v"].astype(float)
    else:
        points=[list(map(float,l.split()[1:4])) for l in raw.decode("ascii").splitlines() if l.strip().startswith("vertex ")]
        triangles=np.asarray(points,dtype=float).reshape(-1,3,3)
    vertices,inv=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
    return vertices,inv.reshape(-1,3)

def mesh_assets(report):
    """Recover original CAD coordinates; print STLs are translated and rotated."""
    folder=OUT/"meshes";folder.mkdir(exist_ok=True)
    assets={};bounds={}
    for name in COLORS:
        src=ROOT/"print_stl"/(name+".stl")
        if not src.exists():raise RuntimeError(f"Missing V5 mesh: {src}")
        v,f=read_stl(src)
        bb=np.asarray(report["parts"][name]["bbox_mm"],dtype=float)
        if name in ("root","proximal","distal"):
            # Printed R_y(90) frame, translation subtracts rotated minimum.
            v += np.array([bb[2],bb[1],-bb[3]])
            v = np.column_stack((-v[:,2],v[:,1],v[:,0]))
        else:v += bb[:3]
        recovered=np.r_[v.min(axis=0),v.max(axis=0)]
        if np.max(np.abs(recovered-bb))>.06:
            raise RuntimeError(f"STL/CAD alignment check failed: {name}: {recovered} vs {bb}")
        bounds[name]=recovered.tolist()
        v *= .001
        obj="\n".join([*("v "+vec(x) for x in v),*("f "+" ".join(str(int(i)+1) for i in face) for face in f)])+"\n"
        content=obj.encode("ascii");assets["meshes/"+name+".obj"]=content
        target=folder/(name+".obj")
        if not target.exists() or target.read_bytes()!=content:target.write_bytes(content)
    return assets,bounds

def make_model(p,report,mode,objname,args):
    ASSUMPTIONS['root_band_stiffness_N_per_m']=p['root_band_effective_stiffness_N_per_m']
    ASSUMPTIONS['distal_band_stiffness_N_per_m']=p['distal_band_effective_stiffness_N_per_m']
    assets,bounds=mesh_assets(report)
    L=p["proximal_axis_distance"]*.001
    D=p["distal_axis_to_tip"]*.001
    radius=p["root_radius"]*.001
    rootz=p["root_height"]*.001
    cfg=OBJECTS[objname]
    default_profile=json.loads((ROOT/'simulation/grasp_profiles.json').read_text())[objname]
    selected_profile=getattr(args,'profile_override',None) or default_profile
    topz=selected_profile['base_m']+selected_profile.get('approach_m',.025) if mode=='grasp' else .185
    xml=ET.Element("mujoco",model="V5_tendon_hand")
    add(xml,"compiler",angle="radian",autolimits="true",inertiafromgeom="false")
    add(xml,"option",timestep=DT,gravity="0 0 -9.81",integrator="implicitfast",cone="elliptic",iterations=80)
    vis=add(xml,"visual");add(vis,"global",offwidth=1200,offheight=900)
    add(vis,"headlight",ambient=".3 .3 .3",diffuse=".7 .7 .7",specular=".1 .1 .1")
    add(vis,"map",znear=".005",zfar="20")
    add(vis,"scale",jointwidth=".0015",jointlength=".012",forcewidth=".002",contactwidth=".003")
    default=add(xml,"default")
    add(default,"joint",damping=".0015",armature="0.000003",frictionloss=".00005",solreflimit=".01 1")
    add(default,"geom",friction="1 .005 .0001",solref=".015 1",solimp=".9 .95 .001",margin=".0001",condim=4)
    add(default,"site",size=".0005",rgba=".8 .1 .1 0",group=4)
    asset=add(xml,"asset")
    add(asset,"texture",name="sky",type="skybox",builtin="gradient",rgb1=".48 .62 .76",rgb2=".9 .94 .98",width=512,height=3072)
    add(asset,"texture",name="floor_tex",type="2d",builtin="checker",rgb1=".22 .28 .33",rgb2=".32 .39 .45",width=512,height=512)
    add(asset,"material",name="floor_mat",texture="floor_tex",texrepeat="8 8",reflectance=".08")
    for n in COLORS:add(asset,"mesh",name=n,file="meshes/"+n+".obj")
    pad_cells=json.loads((ROOT/"source/pad_collision.json").read_text())
    for part,cells in pad_cells.items():
        for j,cell in enumerate(cells):
            add(asset,"mesh",name=f"contact_{part}_{j}",vertex=vec(np.asarray(cell['vertices_mm']).ravel()*.001),face=" ".join(str(i) for face in cell['faces'] for i in face))
    world=add(xml,"worldbody")
    add(world,"light",pos=".3 -.3 .6",dir="-.3 .3 -.8",diffuse=".8 .8 .8")
    add(world,"light",pos="-.3 .2 .4",dir=".3 -.2 -.5",diffuse=".35 .35 .35")
    add(world,"geom",name="floor",type="plane",size=".7 .7 .02",material="floor_mat")
    # A virtual test fixture holds and translates only the hand base.
    hand=add(world,"body",name="hand",mocap="true",pos=vec((0,0,topz)),quat="0 1 0 0")
    def visual(body,n,name=None,pos=None,quat=None):
        kw=dict(name=name or n,mesh=n,type="mesh",contype=0,conaffinity=0,group=1,rgba=COLORS[n])
        if pos is not None:kw["pos"]=vec(pos)
        if quat is not None:kw["quat"]=vec(quat)
        return add(body,"geom",**kw)
    def collision(body,n,size,pos,typ="box",**kwargs):
        return add(body,"geom",name=n,type=typ,size=vec(size),pos=vec(pos),rgba=".6 .2 .8 0",group=3,**kwargs)
    visual(hand,"palm");visual(hand,"base")
    for i,(x,y) in enumerate([(x,y) for x in (-.096,.096) for y in (-.096,.096)]):
        visual(hand,"column",f"column_{i}",(x,y,0));visual(hand,"footpad",f"foot_{i}",(x,y,0))
    for x,y,sx,sy in [(0,.096,.105,.009),(0,-.096,.105,.009),(.096,0,.009,.087),(-.096,0,.009,.087)]:
        collision(hand,f"palm_rim_{x}_{y}",(sx,sy,.003),(x,y,.003))
    contact=add(xml,"contact")
    tendons=add(xml,"tendon")
    actuators=add(xml,"actuator")
    for i,n in enumerate(NAMES):
        angle=i*2*math.pi/3
        base=add(hand,"body",name=n+"_root",pos=vec((-radius*math.sin(angle),radius*math.cos(angle),rootz)),quat=vec((math.cos(angle/2),0,0,math.sin(angle/2))))
        visual(base,"root",n+"_root_mesh")
        for key in ("guide_front","guide_back"):visual(base,key,n+"_"+key)
        motor=add(base,"body",name=n+"_motor",pos="0 0 -.056")
        for key in ("cradle","clamp"):visual(motor,key,n+"_"+key)
        # The teacher servo is present as an approximate static housing only.
        add(motor,"geom",name=n+"_motor_housing",type="box",size=".0237 .0202 .010",pos="-.0021 .010 0",contype=0,conaffinity=0,rgba=".13 .15 .18 1",group=1)
        spool=add(motor,"body",name=n+"_spool_body")
        add(spool,"inertial",pos=".032 0 0",mass=".004",diaginertia=".0000004 .0000004 .0000004")
        add(spool,"joint",name=n+"_winch_joint",axis="-1 0 0",range="0 2.2",armature=".00003",damping=".002",frictionloss=0)
        visual(spool,"spool",n+"_spool_mesh")
        add(actuators,"position",name=n+"_winch",joint=n+"_winch_joint",kp=ASSUMPTIONS["motor_kp_Nm_per_rad"],kv=ASSUMPTIONS["motor_kv_Nms_per_rad"],ctrlrange="0 2.1",forcerange=f'-{args.torque} {args.torque}')
        prox=add(base,"body",name=n+"_prox")
        add(prox,"inertial",pos="0 0 .031",mass=ASSUMPTIONS["proximal_mass_kg"],diaginertia=".000006 .000006 .0000008")
        add(prox,"joint",name=n+"_q1",axis="1 0 0",range="0 .785398")
        visual(prox,"proximal",n+"_prox_mesh");visual(prox,"pad_proximal",n+"_prox_pad_mesh")
        collision(prox,n+"_prox_bone",(.008,.0055,.023),(0,0,.025))
        # Include the palmar backing and tunnel omitted by the old rectangular
        # backbone proxy: these rigid features can meet across fingers.
        collision(prox,n+"_prox_backing_bone",(.008,.00225,.013),(0,-.00755,.031))
        collision(prox,n+"_prox_tunnel_bone",(.0023,.019),(0,-.0075,.031),typ='cylinder')
        for j in range(len(pad_cells['pad_proximal'])):
            add(prox,'geom',name=f'{n}_prox_contact_{j}',type='mesh',mesh=f'contact_pad_proximal_{j}',group=3,rgba='.6 .2 .8 0',friction=f'{args.friction} .005 .0001')
        distal=add(prox,"body",name=n+"_dist",pos=vec((0,0,L)))
        add(distal,"inertial",pos=vec((0,0,D/2)),mass=ASSUMPTIONS["distal_mass_kg"],diaginertia=".000003 .000003 .0000005")
        add(distal,"joint",name=n+"_q2",axis="1 0 0",range="0 1.22173")
        visual(distal,"distal",n+"_dist_mesh");visual(distal,"pad_distal",n+"_dist_pad_mesh")
        collision(distal,n+"_dist_bone",(.007,.0055,D/2),(0,0,D/2))
        collision(distal,n+"_dist_backing_bone",(.007,.00225,(D-.012)/2),(0,-.00755,(D+.012)/2))
        collision(distal,n+"_dist_tunnel_bone",(.0023,.016),(0,-.0075,.026),typ='cylinder')
        for j in range(len(pad_cells['pad_distal'])):
            add(distal,'geom',name=f'{n}_dist_contact_{j}',type='mesh',mesh=f'contact_pad_distal_{j}',group=3,rgba='.6 .2 .8 0',friction=f'{args.friction} .005 .0001')
        add(contact,"exclude",body1=n+"_prox",body2=n+"_dist")
        # Exact variable-length V4 cable route; fixed under-palm length is
        # included as sites. All via-points use the CAD local coordinates.
        fixed=[(.033,-.0109,-.056),(.033,-.0109,-.038),(.033,-.0109,-.035)]
        fixed += [(.029+.004*math.cos(a),-.0109,-.035+.004*math.sin(a)) for a in np.linspace(0,math.pi/2,9)[1:]]
        fixed += [(.004,-.0109,-.031)]
        fixed += [(.004-.004*math.sin(a),-.0109,-.027-.004*math.cos(a)) for a in np.linspace(0,math.pi/2,9)[1:]]
        fixed += [(0,-.0109,-.018),(0,-.0109,-.008)]
        names=[]
        for j,pt in enumerate(fixed):
            sn=f"{n}_cable_fixed_{j}";add(base,"site",name=sn,pos=vec(pt));names.append(sn)
        for body,suffix,pt in [(prox,"p0",(0,-.0075,.012)),(prox,"p1",(0,-.0075,L-.012)),(distal,"d0",(0,-.0075,.010)),(distal,"d1",(0,-.0075,.044))]:
            sn=n+"_"+suffix;add(body,"site",name=sn,pos=vec(pt));names.append(sn)
        cable=add(tendons,"spatial",name=n+"_cable",width=".00045",rgba=".9 .1 .1 1")
        for sn in names:add(cable,"site",site=sn)
        add(actuators,"motor",name=n+"_rope_force_internal",tendon=n+"_cable",ctrlrange="-20 0",forcerange="-20 0")
        # Effective stiffness includes both strands of a closed loop, and the
        # two parallel PIP loops. These are calibration targets, not measured
        # rubber properties. No rigid angle coupling is imposed.
        root_y=p['root_band_line_y']*.001;pip_y=p['pip_band_line_y']*.001
        for j,parent,child,low,high,corner,stiff in [
            (1,base,prox,(0,root_y,-.008),(0,root_y,.014),(0,p['root_band_corner_y']*.001,0),ASSUMPTIONS["root_band_stiffness_N_per_m"]),
            (2,prox,distal,(0,pip_y,L-.013),(0,pip_y,.014),(0,p['pip_band_corner_y']*.001,L),ASSUMPTIONS["distal_band_stiffness_N_per_m"])]:
            ns=[]
            for body,suffix,pt in [(parent,"low",low),(parent,"back",corner),(child,"high",high)]:
                sn=f"{n}_band{j}_{suffix}";add(body,"site",name=sn,pos=vec(pt));ns.append(sn)
            band=add(tendons,"spatial",name=f"{n}_return{j}",width=".00065",rgba=".95 .72 .08 1",stiffness=stiff,damping=".12")
            for sn in ns:add(band,"site",site=sn)
    if mode=="grasp":
        halfheight = cfg["size"][0] if cfg["type"]=="sphere" else cfg["size"][-1]+(cfg["size"][0] if cfg["type"]=="capsule" else 0)
        support=cfg["center"]-halfheight
        if support>0:
            add(world,"geom",name="pedestal",type="cylinder",size=vec((min(.012,cfg["size"][0]*.65),support/2)),pos=vec((0,0,support/2)),rgba=".53 .59 .64 1",friction=".5 .005 .0001")
        obj=add(world,"body",name="object",pos=vec((0,0,cfg["center"])))
        add(obj,"freejoint",name="object_free")
        # Explicit analytic inertias for an independent free rigid body.
        mass=cfg["mass"]; shape=cfg["type"]; size=cfg["size"]
        if shape=="sphere":
            inertia=(.4*mass*size[0]**2,)*3
        elif shape=="box":
            a,b,c=size;inertia=tuple(mass/3*x for x in (b*b+c*c,a*a+c*c,a*a+b*b))
        elif shape=="cylinder":
            r,h=size;it=mass*(3*r*r+4*h*h)/12;inertia=(it,it,.5*mass*r*r)
        else:
            r,h=size;vc=math.pi*r*r*2*h;vs=4/3*math.pi*r**3
            mc=mass*vc/(vc+vs);ms=mass-mc
            it=mc*(3*r*r+4*h*h)/12+ms*(.4*r*r+h*h+.75*h*r)
            inertia=(it,it,.5*mc*r*r+.4*ms*r*r)
        add(obj,"inertial",pos="0 0 0",mass=mass,diaginertia=vec(inertia))
        add(obj,"geom",name="object_contact",type=cfg["type"],size=vec(cfg["size"]),rgba=".84 .29 .22 1",friction=".8 .005 .0001")
    raw=ET.tostring(xml,encoding="unicode")
    xmlpath=OUT/(mode+"_"+objname+".xml");xmlpath.write_text(raw,encoding="utf-8")
    model=mujoco.MjModel.from_xml_string(raw,assets=assets)
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    # Two-value deadband [0, Lfree] makes rubber tension-only.
    for n in NAMES:
        for j in (1,2):
            tid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_TENDON,f"{n}_return{j}")
            model.tendon_lengthspring[tid]=[0,data.ten_length[tid]*ASSUMPTIONS["band_free_length_ratio"]]
    mujoco.mj_forward(model,data)
    sim=Simulation(model,data,p,mode,objname,args,topz)
    sim.mesh_alignment=bounds
    return sim

def smooth(x):
    x=float(np.clip(x,0,1));return x*x*(3-2*x)

class Simulation:
    def __init__(self,model,data,p,mode,objname,args,topz):
        self.m,self.d,self.p,self.mode,self.objname,self.args,self.topz=model,data,p,mode,objname,args,topz
        self.r=p["spool_cable_center_radius"]*.001
        def ids(kind,names):return np.array([mujoco.mj_name2id(model,kind,n) for n in names],dtype=int)
        self.cable=ids(mujoco.mjtObj.mjOBJ_TENDON,[n+"_cable" for n in NAMES])
        self.jmotor=ids(mujoco.mjtObj.mjOBJ_JOINT,[n+"_winch_joint" for n in NAMES])
        self.jfinger=ids(mujoco.mjtObj.mjOBJ_JOINT,[n+q for n in NAMES for q in ("_q1","_q2")])
        self.actmotor=ids(mujoco.mjtObj.mjOBJ_ACTUATOR,[n+"_winch" for n in NAMES])
        self.actrope=ids(mujoco.mjtObj.mjOBJ_ACTUATOR,[n+"_rope_force_internal" for n in NAMES])
        self.neutral=data.ten_length[self.cable].copy()
        self.objbody=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"object")
        self.objgeom=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"object_contact")
        self.geomfinger={}
        for g in range(model.ngeom):
            n=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,g) or ""
            if n[:2] in ("A_","B_","C_") and ("contact" in n or "bone" in n):self.geomfinger[g]=n[0]
        self.profile=getattr(args,'profile_override',None) or json.loads((ROOT/'simulation/grasp_profiles.json').read_text())[objname]
        self.topz=self.profile['base_m'] if mode=='grasp' else topz
        self.final_command=np.full(3,args.takeup_mm) if args.takeup_mm is not None else (np.full(3,7.) if mode=='motion' else np.asarray(self.profile['final_mm'],dtype=float))
        self.manual_height=self.profile.get('approach_m',.025) if mode=='grasp' else 0.
        self.records=[];self.tension=np.zeros(3);self.manual_target=np.zeros(3);self.numerical_ok=True;self.next_record=0

    def step(self,manual=False,active=None):
        d,m=self.d,self.m
        t=d.time;pre=np.asarray(self.profile['pre_mm'],dtype=float);final=np.asarray(self.profile['final_mm'],dtype=float)
        if self.mode=='motion':pre=np.full(3,3.);final=np.full(3,7.)
        if self.args.takeup_mm is not None:final=np.full(3,self.args.takeup_mm)
        if t<3:desired=pre*smooth((t-1)/2)
        elif t<5:desired=pre
        elif t<8:desired=pre+(final-pre)*smooth((t-5)/3)
        elif t<15:desired=final
        else:desired=final*(1-smooth((t-15)/2))
        lift=smooth((t-9)/2) if t<13 else 1-smooth((t-13)/2)
        clearance=self.profile.get('approach_m',.025)
        approach=clearance*((1-smooth((t-3)/2))+smooth((t-15)/2))
        if not manual:
            if active is not None:desired*=np.asarray(active)
            d.ctrl[self.actmotor]=desired*.001/self.r
        else:d.ctrl[self.actmotor]=self.manual_target*.001/self.r
        d.mocap_pos[0]=[0,0,self.topz+(approach+ASSUMPTIONS['lift_height_m']*lift if self.mode=='grasp' and not manual else self.manual_height)]
        q=d.qpos[m.jnt_qposadr[self.jmotor]]
        vel=d.qvel[m.jnt_dofadr[self.jmotor]]
        extension=d.ten_length[self.cable]-self.neutral+self.r*q
        speed=d.ten_velocity[self.cable]+self.r*vel
        # Slack cables cannot push; spool receives the equal reaction torque.
        force=ASSUMPTIONS["cable_stiffness_N_per_m"]*extension+ASSUMPTIONS["cable_damping_Ns_per_m"]*speed
        self.tension=np.where(extension>0,np.clip(force,0,ASSUMPTIONS["tension_limit_N"]),0)
        d.ctrl[self.actrope]=-self.tension
        d.qfrc_applied[:]=0
        d.qfrc_applied[m.jnt_dofadr[self.jmotor]]=-self.r*self.tension
        before=float(d.time)
        mujoco.mj_step(m,d)
        bad=any(d.warning[k].number for k in (mujoco.mjtWarning.mjWARN_BADQPOS,mujoco.mjtWarning.mjWARN_BADQVEL,mujoco.mjtWarning.mjWARN_BADQACC))
        if bad or d.time<before or not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel)) or np.max(np.abs(d.qvel))>500:
            self.numerical_ok=False;raise RuntimeError("Numerical instability detected; see saved report.")
        if d.time>=self.next_record:
            self.record();self.next_record+=.05

    def record(self):
        d,m=self.d,self.m;fingers=set();support=False;floor_contacts=0;self_contacts=0
        for i,c in enumerate(d.contact[:d.ncon]):
            aa,bb=int(c.geom1),int(c.geom2);fa=self.geomfinger.get(aa);fb=self.geomfinger.get(bb)
            other=bb if fa else aa
            isfloor=bool((fa or fb) and mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,other)=='floor')
            if isfloor or (fa and fb and fa!=fb):
                cf=np.zeros(6);mujoco.mj_contactForce(m,d,i,cf)
                if cf[0]>.005:
                    floor_contacts+=int(isfloor);self_contacts+=int(bool(fa and fb and fa!=fb))
        if self.objgeom>=0:
            for ci,c in enumerate(d.contact[:d.ncon]):
                cf=np.zeros(6);mujoco.mj_contactForce(m,d,ci,cf)
                if cf[0]<.0005:continue
                if c.geom1==self.objgeom:other=int(c.geom2)
                elif c.geom2==self.objgeom:other=int(c.geom1)
                else:continue
                if other in self.geomfinger:fingers.add(self.geomfinger[other])
                elif mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,other) in ("floor","pedestal"):support=True
        self.records.append({"time_s":float(d.time),"joint_deg":np.degrees(d.qpos[m.jnt_qposadr[self.jfinger]]).tolist(),
            "tension_N":self.tension.tolist(),"actual_reel_takeup_mm":(d.qpos[m.jnt_qposadr[self.jmotor]]*self.r*1000).tolist(),
            "object_z_m":float(d.xpos[self.objbody,2]) if self.objbody>=0 else None,
            "hand_z_m":float(d.mocap_pos[0,2]),"object_contact_fingers":sorted(fingers),"object_on_support":support,"finger_floor_contacts":floor_contacts,"cross_finger_contacts":self_contacts,
            "object_xyz_m":d.xpos[self.objbody].tolist() if self.objbody>=0 else None})

    def result(self):
        hold=[r for r in self.records if 11.5<=r["time_s"]<=12.8]
        last=[r for r in self.records if r["time_s"]>=17.8]
        qmax=max((max(abs(x) for x in r["joint_deg"]) for r in self.records),default=0)
        ret=max((max(abs(x) for x in r["joint_deg"]) for r in last),default=999)
        success=None;lift=0;drift=0
        if self.mode=="grasp" and hold:
            lift=min(r["object_z_m"] for r in hold)-OBJECTS[self.objname]["center"]
            drift=float(np.linalg.norm(np.ptp(np.asarray([r['object_xyz_m'] for r in hold]),axis=0)))
            success=bool(lift>=.04 and drift<.008 and all(len(r["object_contact_fingers"])>=2 and not r["object_on_support"] for r in hold))
        floor_samples=sum(r['finger_floor_contacts']>0 for r in self.records)
        return {"script_version":VERSION,"mujoco_version":mujoco.__version__,"mode":self.mode,"object":self.objname,
            "numerically_stable":self.numerical_ok,"observed_grasp_lift":success,
            "minimum_object_lift_during_hold_mm":lift*1000,"maximum_joint_angle_deg":qmax,
            "return_error_deg":ret,"rebound_below_4deg":ret<4,"full_cycle_success":bool(success and ret<4 and floor_samples==0 and self.numerical_ok),
            "hold_drift_mm":drift*1000,"finger_floor_contact_samples":floor_samples,
            "takeup_command_mm":self.final_command.tolist(),"profile":self.profile,"object_config":OBJECTS[self.objname] if self.mode=="grasp" else None,
            "hand_translation_is_virtual_test_fixture":True,"assumptions":dict(ASSUMPTIONS,motor_torque_limit_Nm=self.args.torque,pad_friction=self.args.friction),
            "scope":"Simplified contact simulation, not a hardware pass/fail certificate.",
            "no_object_attachment":True,"mesh_alignment_bbox_mm":self.mesh_alignment,"samples":self.records}

    def reset(self):
        mujoco.mj_resetData(self.m,self.d);mujoco.mj_forward(self.m,self.d)
        self.records=[];self.next_record=0;self.manual_target[:]=0
        self.manual_height=self.profile.get('approach_m',.025) if self.mode=='grasp' else 0.

def snapshot(sim,path):
    try:from PIL import Image
    except ImportError:raise RuntimeError("--snapshot needs Pillow: python -m pip install pillow")
    cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.12];cam.distance=.54;cam.azimuth=135;cam.elevation=-20
    with mujoco.Renderer(sim.m,height=900,width=1200) as renderer:
        renderer.update_scene(sim.d,camera=cam)
        Image.fromarray(renderer.render()).save(path)

def save_report(sim,suffix=""):
    path=OUT/(f"{sim.mode}_{sim.objname}{suffix}_report.json")
    result=sim.result();path.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print(f"[result] stable={result['numerically_stable']} grasp_lift={result['observed_grasp_lift']} "
          f"lift={result['minimum_object_lift_during_hold_mm']:.1f}mm return={result['return_error_deg']:.2f}deg")
    print("[report]",path)
    return result

def run_headless(sim,args,active=None):
    saved=False
    while sim.d.time<DURATION:
        sim.step(active=active)
        if args.snapshot and not saved and sim.d.time>=12:
            snapshot(sim,args.snapshot);saved=True
    return save_report(sim,"_independent" if active is not None else "")

def run_gui(sim,args):
    import mujoco.viewer
    events=queue.SimpleQueue();paused=False;cycles=0;last_note=-1
    with mujoco.viewer.launch_passive(sim.m,sim.d,key_callback=lambda key:events.put(key)) as viewer:
        viewer.cam.lookat[:]=[0,0,.12];viewer.cam.distance=.55;viewer.cam.azimuth=135;viewer.cam.elevation=-20
        print("[keys] P=pause, R=reset, O=open, C=close (manual), Esc=exit")
        print("[manual] 1/2/3 select finger, 0 selects all; Up/Down takeup +/-0.5 mm; PageUp/PageDown hand height +/-1 mm.")
        selected=None
        while viewer.is_running():
            started=time.perf_counter()
            while not events.empty():
                key=events.get()
                if key in (80,112):paused=not paused
                elif key in (82,114):sim.reset();last_note=-1
                elif key in (79,111):sim.manual_target[:]=0
                elif key in (67,99):sim.manual_target[:]=sim.final_command
                elif key in (266,267):sim.manual_height=np.clip(sim.manual_height+(.001 if key==266 else -.001),-.020,.100)
                elif key in (49,50,51):selected=key-49
                elif key==48:selected=None
                elif key==256:return
                elif key in (265,264):
                    idx=slice(None) if selected is None else selected
                    sim.manual_target[idx]=np.clip(sim.manual_target[idx]+(.5 if key==265 else -.5),0,18)
            if not paused:
                for _ in range(max(1,round(1/(60*DT)))):sim.step(manual=args.manual)
            viewer.sync()
            sec=int(sim.d.time)
            if sec!=last_note:
                last_note=sec
                q=sim.d.qpos[sim.m.jnt_qposadr[sim.jfinger]]
                print(f"t={sim.d.time:5.1f}s q_deg={np.round(np.degrees(q),1)} T_N={np.round(sim.tension,2)}")
            if not args.manual and sim.d.time>=DURATION:
                save_report(sim);cycles+=1
                if args.cycles and cycles>=args.cycles:break
                sim.reset();last_note=-1
            delay=1/60-(time.perf_counter()-started)
            if delay>0:time.sleep(delay)

def main():
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode",choices=("grasp","motion"),default="grasp")
    ap.add_argument("--object",choices=tuple(OBJECTS)+("all",),default="pingpong")
    ap.add_argument("--check",action="store_true",help="headless motion + selected grasp checks")
    ap.add_argument("--headless",action="store_true",help="run an 18-second cycle without a window")
    ap.add_argument("--manual",action="store_true",help="manual reel takeup using keyboard")
    ap.add_argument("--cycles",type=int,default=0,help="GUI cycles, 0=repeat until window is closed")
    ap.add_argument("--takeup-mm",type=float,help="override final reel takeup, range 0..18 mm")
    ap.add_argument("--torque",type=float,default=ASSUMPTIONS["motor_torque_limit_Nm"],help="assumed motor torque limit in Nm")
    ap.add_argument("--friction",type=float,default=ASSUMPTIONS["pad_friction"],help="assumed finger friction")
    ap.add_argument("--snapshot",type=Path,help="optional screenshot at t=12 s; requires Pillow and graphics support")
    args=ap.parse_args()
    if args.takeup_mm is not None and not 0<=args.takeup_mm<=18:ap.error("--takeup-mm must be 0..18")
    if not 0<args.torque<=1 or not 0<args.friction<=3:ap.error("Use torque 0..1 and friction 0..3")
    if args.manual and (args.headless or args.check):ap.error("Manual mode needs a GUI")
    if args.object=="all" and not args.check:ap.error("--object all is available with --check")
    p,report=load_inputs()
    print(f"[{VERSION}] MuJoCo {mujoco.__version__}; root radius {p['root_radius']} mm; finger {p['proximal_axis_distance']+p['distal_axis_to_tip']} mm")
    print("[model] 3 winches -> 3 unilateral tendons -> 6 passive joints; 9 rubber loops represented by 6 effective springs")
    print("[assumptions] torque/friction/band stiffness are uncalibrated. TPU contacts follow CAD cells; rigid collisions are simplified.")
    if args.check:
        print("[check] motion and elastic rebound (no object)")
        motion=make_model(p,report,"motion","sphere",args);a=run_headless(motion,args)
        if a['maximum_joint_angle_deg']<5 or not a['rebound_below_4deg']:raise RuntimeError("Motion/rebound check failed")
        print("[check] only finger A is commanded")
        indep=make_model(p,report,"motion","sphere",args);b=run_headless(indep,args,active=[1,0,0])
        others=max(abs(x) for r in b['samples'] for x in r['joint_deg'][2:])
        if others>3:raise RuntimeError(f"Unexpected uncommanded motion: {others:.2f} deg")
        names=list(OBJECTS) if args.object=="all" else [args.object]
        results=[]
        for name in names:
            print("[check] contact/lift:",name)
            results.append(run_headless(make_model(p,report,"grasp",name,args),args))
        passed=sum(bool(r["full_cycle_success"]) for r in results)
        print(f"[grasp summary] {passed}/{len(results)} complete simulated cycles passed. NOT a hardware certificate.")
        if passed<len(results):return 2
    else:
        sim=make_model(p,report,args.mode,args.object,args)
        if args.headless:
            result=run_headless(sim,args)
            if sim.mode=="grasp" and not result["full_cycle_success"]:return 2
        else:run_gui(sim,args)

if __name__=="__main__":
    try:raise SystemExit(main())
    except KeyboardInterrupt:print("\nStopped.")
    except Exception as exc:
        print(f"\nERROR: {exc}",file=sys.stderr)
        raise SystemExit(1)
