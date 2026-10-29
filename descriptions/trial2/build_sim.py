"""Author MJCF with spatial tendons coupled to physical winch joints."""
from pathlib import Path
import math
import xml.etree.ElementTree as E
from design import *

HERE=Path(__file__).resolve().parent
NAMES=('palm','root_clevis','proximal','distal','contact_pad','winch_hub','winch_drum','motor_carrier','motor_body','motor_horn','winch_retainer','hub_fasteners','hinge_bolt','hinge_nut','torsion_spring_envelope','liner','tendon_stop','hinge_spacers','root_fasteners','carrier_fasteners','clamp_screws')

def sub(p,t,**kw):return E.SubElement(p,t,{k:str(v) for k,v in kw.items()})
def xyz(p):return ' '.join(f'{float(v)/1000:.10f}' for v in p)
def visual(p,part,name,pos=None,color='0.68 0.73 0.78 1'):
    kw=dict(name=name,type='mesh',mesh=part,rgba=color,contype='0',conaffinity='0',mass='0')
    if pos is not None:kw['pos']=xyz(pos)
    return sub(p,'geom',**kw)

def main():
    m=E.Element('mujoco',model='trial2_three_tendon_two_joint_hand')
    sub(m,'compiler',angle='radian',meshdir='meshes',autolimits='true')
    sub(m,'option',timestep='.001',gravity='0 0 -9.81',integrator='implicitfast',iterations='100',cone='elliptic')
    sub(m,'statistic',center='0 0 0.025',extent='.35')
    v=sub(m,'visual');sub(v,'global',offwidth='1400',offheight='1100');sub(v,'headlight',ambient='.5 .5 .5',diffuse='.6 .6 .6')
    default=sub(m,'default');sub(default,'geom',friction='1.2 .01 .001',solref='.004 1',solimp='.99 .999 .0001')
    asset=sub(m,'asset');sub(asset,'texture',type='skybox',builtin='gradient',rgb1='.18 .25 .33',rgb2='.78 .83 .88',width='512',height='3072')
    for n in NAMES:sub(asset,'mesh',name=n,file=n+'.stl',scale='.001 .001 .001')
    world=sub(m,'worldbody');sub(world,'light',pos='.1 -.2 .5',dir='0 0 -1',diffuse='.8 .8 .8')
    sub(world,'geom',name='test_pedestal',type='cylinder',pos='0 0 0',size='.018 .049',rgba='.45 .5 .56 1')
    sub(world,'geom',name='test_tray',type='cylinder',pos='0 0 .05',size='.042 .001',rgba='.45 .5 .56 1')
    sub(world,'geom',name='floor',type='plane',pos='0 0 -.075',size='.3 .3 .01',rgba='.2 .26 .32 1')
    rig=sub(world,'body',name='test_fixture',gravcomp='1')
    sub(rig,'joint',name='fixture_lift',type='slide',axis='0 0 1',range='0 .045',damping='30')
    sub(rig,'inertial',pos='0 0 0',mass='1',diaginertia='.01 .01 .01')
    visual(rig,'palm','palm',color='.16 .28 .39 1')
    # Convex primitives around the true circular opening. Keep the pedestal clear.
    for name,p,sz in [('left',(-57.5,0,-2.5),(32.5,55,2.5)),('right',(57.5,0,-2.5),(32.5,55,2.5)),('back',(0,-57.5,-2.5),(25,32.5,2.5)),('front',(0,57.5,-2.5),(25,32.5,2.5))]:
        sub(rig,'geom',name='palm_contact_'+name,type='box',pos=xyz(p),size=xyz(sz),rgba='0 0 0 0',mass='0')
    tendon=sub(m,'tendon');eq=sub(m,'equality');act=sub(m,'actuator')
    sub(act,'position',name='fixture_lift',joint='fixture_lift',kp='4000',kv='40',ctrlrange='0 .045',forcerange='-100 100')
    for i,n in enumerate(FINGERS):
        origin=base(i)[:3,3]
        b=sub(rig,'body',name=n+'_base',pos=xyz(origin),euler=f'0 0 {i*2*math.pi/3:.12f}')
        visual(b,'root_fasteners',n+'_root_fasteners');visual(b,'carrier_fasteners',n+'_carrier_fasteners');visual(b,'clamp_screws',n+'_clamp_screws');visual(b,'root_clevis',n+'_root',color='.19 .39 .53 1');visual(b,'motor_carrier',n+'_carrier',color='.25 .32 .38 1')
        visual(b,'motor_body',n+'_motor',(0,0,-48),color='.14 .16 .19 1')
        for part in ('hinge_bolt','hinge_nut','hinge_spacers','torsion_spring_envelope'):visual(b,part,n+'_root_'+part,color='.6 .56 .42 1')
        fixed_names=[]
        for j,p in enumerate(FIXED_ROUTE):
            sn=f'{n}_fixed_{j}';sub(b,'site',name=sn,pos=xyz(p),size='.0007',rgba='0 0 0 0');fixed_names.append(sn)
        w=sub(b,'body',name=n+'_winch',pos='0 0 -.048')
        sub(w,'joint',name=n+'_reel',type='hinge',axis='1 0 0',range='0 2.8',damping='.001',armature='.000005')
        sub(w,'inertial',pos='.03 0 0',mass='.012',diaginertia='.000002 .000002 .000002')
        for part in ('motor_horn','winch_hub','winch_drum','winch_retainer','hub_fasteners'):visual(w,part,n+'_'+part,color='.83 .58 .18 1')
        p=sub(b,'body',name=n+'_proximal')
        sub(p,'joint',name=n+'_MCP',type='hinge',axis='1 0 0',range='0 1.05',stiffness='.06',springref='0',damping='.003',armature='.00001')
        sub(p,'inertial',pos='0 0 .023',mass='.04',diaginertia='.00002 .00002 .000006')
        visual(p,'proximal',n+'_prox_mesh')
        sub(p,'geom',name=n+'_prox_contact',type='capsule',fromto='0 0 .008 0 0 .03',size='.005',rgba='0 0 0 0',mass='0')
        for j,pt in enumerate(PROX_GUIDES):sub(p,'site',name=n+'_prox_guide'+str(j),pos=xyz(pt),size='.0007',rgba='0 0 0 0')
        for part in ('hinge_bolt','hinge_nut','hinge_spacers','torsion_spring_envelope'):visual(p,part,n+'_PIP_'+part,(0,0,45),color='.6 .56 .42 1')
        d=sub(p,'body',name=n+'_distal',pos='0 0 .045')
        sub(d,'joint',name=n+'_PIP',type='hinge',axis='1 0 0',range='0 1.3',stiffness='.05',springref='0',damping='.003',armature='.000008')
        sub(d,'inertial',pos='0 0 .018',mass='.025',diaginertia='.000006 .000006 .000003')
        visual(d,'distal',n+'_dist_mesh')
        visual(d,'contact_pad',n+'_pad_mesh',color='.13 .18 .21 1')
        visual(d,'liner',n+'_liner',color='.9 .9 .85 1');visual(d,'tendon_stop',n+'_stop',color='.84 .61 .2 1')
        sub(d,'geom',name=n+'_pad_contact',type='capsule',fromto='0 -.0085 .014 0 -.0085 .025',size='.004',rgba='0 0 0 0',mass='0')
        sub(d,'geom',name=n+'_dist_contact',type='capsule',fromto='0 0 .008 0 0 .029',size='.004',rgba='0 0 0 0',mass='0')
        for j,pt in enumerate(DIST_GUIDES):sub(d,'site',name=n+'_dist_guide'+str(j),pos=xyz(pt),size='.0007',rgba='0 0 0 0')
        sp=sub(tendon,'spatial',name=n+'_flexor',width='.0004',rgba='.94 .2 .09 1')
        for sn in fixed_names+[n+'_prox_guide'+str(j) for j in range(len(PROX_GUIDES))]+[n+'_dist_guide'+str(j) for j in range(len(DIST_GUIDES))]:sub(sp,'site',site=sn)
        ft=sub(tendon,'fixed',name=n+'_unrolled');sub(ft,'joint',joint=n+'_reel',coef=str(SPOOL_RADIUS/1000))
        sub(eq,'tendon',name=n+'_taut_cable',tendon1=n+'_flexor',tendon2=n+'_unrolled',polycoef='0 -1 0 0 0',solref='.003 1',solimp='.99 .999 .0001')
        sub(act,'position',name=n+'_reel_motor',joint=n+'_reel',kp='.12',kv='.009',ctrlrange='0 2.6',forcerange='-.075 .075')
    obj=sub(world,'body',name='test_object',pos='0 0 .073');sub(obj,'freejoint',name='object_free')
    sub(obj,'geom',name='test_ball',type='sphere',size='.022',mass='.03',rgba='.92 .39 .13 1')
    E.indent(m,space='  ');E.ElementTree(m).write(HERE/'hand.xml',encoding='unicode')
    print('MJCF written; neutral cable length',NEUTRAL_LENGTH,'mm')

if __name__=='__main__':main()
