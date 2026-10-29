"""Run with conda newton: mjpython descriptions/trial2/run_hand.py."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import mujoco as mj
from design import FINGERS, SPOOL_RADIUS, NEUTRAL_LENGTH

HERE=Path(__file__).resolve().parent

def blend(t):
    t=np.clip(t,0.,1.);return t*t*(3-2*t)

def commands(t):
    pull=1.8*blend((t-1)/4) if t<15 else 1.8*(1-blend((t-15)/3))
    lift=.035*blend((t-7)/3) if t<11 else .035*(1-blend((t-11)/3))
    return np.array([lift,pull,pull,pull])

class Simulation:
    def __init__(self,model=None):
        self.model=model or mj.MjModel.from_xml_path(str(HERE/'hand.xml'))
        self.data=mj.MjData(self.model)
        self.eq=[mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_EQUALITY,n+'_taut_cable') for n in FINGERS]
        self.flex=[mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_TENDON,n+'_flexor') for n in FINGERS]
        self.reel=[self.model.jnt_qposadr[mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_JOINT,n+'_reel')] for n in FINGERS]
        self.finger_q=[self.model.jnt_qposadr[mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_JOINT,n+'_'+j)] for n in FINGERS for j in ('MCP','PIP')]
        self.reactions=np.zeros(3)
        mj.mj_forward(self.model,self.data)

    def reset(self):
        mj.mj_resetData(self.model,self.data);self.reactions[:]=0;mj.mj_forward(self.model,self.data)

    def gap(self,i):
        return self.data.ten_length[self.flex[i]]+SPOOL_RADIUS/1000*self.data.qpos[self.reel[i]]-NEUTRAL_LENGTH/1000

    def step(self,control):
        d=self.data
        # A cable carries tension only. Remove a taut constraint that would
        # push; restore it when the geometric span takes up the paid-out cable.
        for i,e in enumerate(self.eq):
            if d.eq_active[e] and self.reactions[i]>.01:d.eq_active[e]=False
            elif not d.eq_active[e] and self.gap(i)>2e-5:d.eq_active[e]=True
        d.ctrl[:]=control;mj.mj_step(self.model,d)
        for i,e in enumerate(self.eq):
            rows=np.flatnonzero((d.efc_type==mj.mjtConstraint.mjCNSTR_EQUALITY)&(d.efc_id==e))
            self.reactions[i]=float(d.efc_force[rows[0]]) if len(rows) else 0.

def verify():
    s=Simulation();m,d=s.model,s.data
    ball=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'test_ball')
    pads={mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,n+'_pad_contact') for n in FINGERS}
    peak_z=.073;contacts=0;max_tension=0.;max_error=0.;snapshots={}
    for k in range(20000):
        s.step(commands(d.time))
        if not np.isfinite(d.qpos).all():raise RuntimeError('non-finite state')
        loaded=set()
        for c in d.contact:
            if ball in (c.geom1,c.geom2):loaded.update({c.geom1,c.geom2}&pads)
        contacts=max(contacts,len(loaded));peak_z=max(peak_z,float(d.body('test_object').xpos[2]))
        max_tension=max(max_tension,float(max(-s.reactions)))
        max_error=max(max_error,max(abs(s.gap(i)) for i,e in enumerate(s.eq) if d.eq_active[e]) if any(d.eq_active[s.eq]) else 0)
        if k in (6499,10499,19999):snapshots[str(round(d.time,1))]={'q_deg':np.degrees(d.qpos[s.finger_q]).tolist(),'reel_deg':np.degrees(d.qpos[s.reel]).tolist(),'ball_xyz_m':d.body('test_object').xpos.tolist()}
    if contacts!=3:raise RuntimeError(f'only {contacts} pads contacted')
    if peak_z<.095:raise RuntimeError(f'ball did not lift: {peak_z}')
    if abs(d.body('test_object').xpos[2]-.073)>.004:raise RuntimeError('ball not returned to pedestal')
    if np.max(abs(d.qpos[s.finger_q]))>.04:raise RuntimeError('passive joints did not reopen')
    # Move the object away and isolate each cable, with gravity disabled.
    m.opt.gravity[:]=0
    free=m.jnt_qposadr[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'object_free')]
    for i,n in enumerate(FINGERS):
        s.reset();d.qpos[free:free+3]=(0,0,1);mj.mj_forward(m,d)
        control=np.zeros(4);control[i+1]=.65
        for _ in range(3000):s.step(control)
        q=d.qpos[s.finger_q].reshape(3,2)
        if np.min(q[i])<.06 or np.max(abs(np.delete(q,i,axis=0)))>.025:raise RuntimeError(f'independent tendon test failed {n}: {q}')
    result={'pass':True,'simultaneous_pad_contacts':contacts,'lift_mm':(peak_z-.073)*1000,'max_tendon_tension_N':max_tension,'max_active_cable_error_mm':max_error*1000,'independent_fingers':3,'snapshots':snapshots,'assumptions':{'ball_mass_kg':.03,'ball_radius_m':.022,'pad_friction':1.2,'reel_torque_limit_Nm':.075,'MCP_spring_Nm_per_rad':.06,'PIP_spring_Nm_per_rad':.05}}
    (HERE/'reports'/'simulation.json').write_text(json.dumps(result,indent=2))
    print('PASS: 3 independent tendons, 6 passive joints; grasp/lift/return/reopen')
    print(f'Lift {result["lift_mm"]:.2f} mm; max tension {max_tension:.3f} N; active cable error {max_error*1000:.3f} mm')

def main():
    p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');p.add_argument('--seconds',type=float,default=0);args=p.parse_args()
    if args.check:verify();return
    from mujoco import viewer as v
    s=Simulation();d=s.data
    with v.launch_passive(s.model,d) as view:
        print('trial2 viewer ready: 3 tendons / 6 finger joints',flush=True)
        view.cam.azimuth=135;view.cam.elevation=-23;view.cam.distance=.48;view.cam.lookat[:]=(0,0,.025)
        begin=time.monotonic();elapsed=0.;count=0
        while view.is_running() and (not args.seconds or elapsed<args.seconds):
            if d.time>=20:s.reset()
            s.step(commands(d.time));elapsed+=s.model.opt.timestep;count+=1
            if count%12==0:
                view.sync();delay=begin+elapsed-time.monotonic()
                if delay>0:time.sleep(delay)

if __name__=='__main__':main()
