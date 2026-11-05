"""Joint stop box primitives shared by CAD, tolerances and MuJoCo; mm."""
import json,math
from pathlib import Path

def geometry(p):
    if not p.get('mechanical_extension_stops_enabled',True):
        return {'open_angle_deg':p['joint_open_stop_angle_deg'],'items':[]}
    def box(lo,hi,angle=0):
        c=[(a+b)/2 for a,b in zip(lo,hi)];s=[b-a for a,b in zip(lo,hi)]
        a=math.radians(angle);co,si=math.cos(a),math.sin(a)
        center=[c[0],c[1]*co-c[2]*si,c[1]*si+c[2]*co]
        return {'center_unrotated_mm':c,'center_mm':center,'size_mm':s,
                'rotation_x_deg':angle,'quat_wxyz':[math.cos(a/2),math.sin(a/2),0,0]}
    items=[]
    for side in (-1,1):
        x0=p['joint_stop_x0'] if side>0 else -p['joint_stop_x1'];x1=p['joint_stop_x1'] if side>0 else -p['joint_stop_x0']
        wx0=p['joint_wing_x0'] if side>0 else -p['joint_stop_x1'];wx1=p['joint_stop_x1'] if side>0 else -p['joint_wing_x0']
        stem=box([x0,p['joint_stop_stem_front_y'],p['joint_stop_stem_z0']],[x1,p['joint_stop_back_y'],p['joint_stop_z1']+1])
        foot=box([x0,p['joint_stop_foot_front_y'],p['joint_stop_stem_z0']],[x1,p['joint_stop_back_y'],p['joint_stop_foot_top_z']])
        nose=box([x0,p['joint_stop_front_y'],p['joint_stop_z0']],[x1,p['joint_stop_back_y'],p['joint_stop_z1']],p['joint_open_stop_angle_deg'])
        wing=box([wx0,p['joint_wing_front_y'],p['joint_stop_z0']],[wx1,p['joint_stop_front_y'],p['joint_stop_z1']])
        items.append({'side':side,'stem':stem,'foot':foot,'nose':nose,'wing':wing})
    return {'open_angle_deg':p['joint_open_stop_angle_deg'],'items':items}

def export(p,path):
    data=geometry(p);Path(path).write_text(json.dumps(data,indent=2));return data
