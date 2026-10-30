"""Reproduce V4/V5 contact tests. Run from any directory; MuJoCo + NumPy."""
import argparse,copy,importlib.util,json,math,types
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
POSES=[('center',0,0,0),('x+1mm',.001,0,0),('x-1mm',-.001,0,0),
       ('y+1mm',0,.001,0),('y-1mm',0,-.001,0),('diag+2mm',.002,-.002,0),('diag-2mm',-.002,.002,0)]

def load_module(design):
    path=ROOT/'run_v5_sim.py' if design=='v5' else ROOT/'comparison/baseline_v4/run_v4_sim.py'
    spec=importlib.util.spec_from_file_location('bench_'+design,path);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    if design=='v4':
        mod.OBJECTS['pingpong']=dict(type='sphere',size=(.020,),mass=.0027,center=.0325,base=.136,takeup=10.,label='40 mm 2.7 g ping-pong ball (added benchmark)')
    return mod

def trial(mod,design,name,pose):
    label,dx,dy,yaw=pose;p,b=mod.load_inputs()
    args=types.SimpleNamespace(torque=.15,friction=1.,takeup_mm=None,snapshot=None,profile_override=None)
    sim=mod.make_model(p,b,'grasp',name,args);m,d=sim.m,sim.d;mj=mod.mujoco
    j=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'object_free');a=m.jnt_qposadr[j]
    m.qpos0[a]+=dx;m.qpos0[a+1]+=dy;theta=math.radians(yaw)/2;m.qpos0[a+3:a+7]=[math.cos(theta),0,0,math.sin(theta)]
    mj.mj_resetData(m,d);mj.mj_forward(m,d)
    hold_start,hold_end,release=(11.5,12.8,17.8) if design=='v5' else (7.5,8.8,13.8)
    samples=[];next_sample=0;floor_samples=0
    while d.time<mod.DURATION:
        sim.step()
        if d.time<next_sample:continue
        next_sample+=.05;fingers=set();support=False;floor=False
        for ci,c in enumerate(d.contact[:d.ncon]):
            aa,bb=int(c.geom1),int(c.geom2);cf=np.zeros(6);mj.mj_contactForce(m,d,ci,cf)
            if cf[0]<.0005:continue
            if sim.objgeom in (aa,bb):
                other=bb if aa==sim.objgeom else aa
                if other in sim.geomfinger:fingers.add(sim.geomfinger[other])
                elif mj.mj_id2name(m,mj.mjtObj.mjOBJ_GEOM,other) in ('floor','pedestal'):support=True
            for gg,other in [(aa,bb),(bb,aa)]:
                if gg in sim.geomfinger and mj.mj_id2name(m,mj.mjtObj.mjOBJ_GEOM,other)=='floor' and cf[0]>.005:floor=True
        floor_samples+=floor
        samples.append({'time_s':float(d.time),'object_xyz_m':d.xpos[sim.objbody].tolist(),
            'contact_fingers':sorted(fingers),'support':support,'finger_floor':floor,
            'joint_deg':np.degrees(d.qpos[m.jnt_qposadr[sim.jfinger]]).tolist(),
            'tensions_N':sim.tension.tolist(),'winch_torque_Nm':d.actuator_force[sim.actmotor].tolist()})
    hold=[r for r in samples if hold_start<=r['time_s']<=hold_end];last=[r for r in samples if r['time_s']>=release]
    position=np.asarray([r['object_xyz_m'] for r in hold]);lift=(position[:,2].min()-mod.OBJECTS[name]['center'])*1000
    drift=float(np.linalg.norm(np.ptp(position,axis=0)))*1000
    ret=max(abs(q) for r in last for q in r['joint_deg'])
    contacts=min(len(r['contact_fingers']) for r in hold)
    grasp=bool(lift>=40 and contacts>=2 and not any(r['support'] for r in hold) and drift<8)
    full=bool(grasp and ret<4 and floor_samples==0)
    return {'design':design,'object':name,'pose':label,'dx_m':dx,'dy_m':dy,'yaw_deg':yaw,
        'lift_success':grasp,'full_cycle_success':full,'minimum_lift_mm':float(lift),
        'hold_drift_mm':drift,'return_error_deg':float(ret),'minimum_contact_fingers':contacts,
        'finger_floor_contact_samples':int(floor_samples),'numerically_stable':sim.numerical_ok,
        'torque_limit_Nm':.15,'pad_friction':1.0,'samples':samples}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--design',choices=['v4','v5','both'],default='both')
    ap.add_argument('--full',action='store_true',help='7 defined planar placements per object; center only otherwise')
    ap.add_argument('--object',default='all',choices=['all','sphere','pingpong','cube','cylinder','capsule','small_sphere'])
    args=ap.parse_args();designs=['v4','v5'] if args.design=='both' else [args.design]
    names=['sphere','pingpong','cube','cylinder','capsule','small_sphere'] if args.object=='all' else [args.object]
    out=ROOT/'checks/benchmarks';out.mkdir(exist_ok=True)
    for design in designs:
        mod=load_module(design);results=[]
        for name in names:
            for pose in POSES if args.full else POSES[:1]:
                r=trial(mod,design,name,pose);file=f"{design}_{name}_{pose[0].replace('+','plus').replace('-','minus')}.json"
                (out/file).write_text(json.dumps(r,indent=2));results.append({k:v for k,v in r.items() if k!='samples'})
                print(design,name,pose[0],'FULL_PASS',r['full_cycle_success'],'lift',round(r['minimum_lift_mm'],1),'return',round(r['return_error_deg'],2),flush=True)
        summary={'design':design,'protocol':'Same object geometries/masses, 0.15 Nm torque cap, friction 1.0, floor and self-contact enabled. Controller and pad/return geometry differ as documented.',
            'criteria':'>=40 mm minimum lift throughout hold; >=2 fingers with >0.5 mN normal contact; no object support; <8 mm hold drift; <4 deg released joint error; no finger-floor contact.',
            'note':'V4 ping-pong case added for comparison; not part of original five V4 cases. Results are deterministic samples under assumptions, not a measured hardware success rate.',
            'results':results}
        path=out/f'{design}_summary.json';path.write_text(json.dumps(summary,indent=2));print(path,flush=True)
if __name__=='__main__':main()
