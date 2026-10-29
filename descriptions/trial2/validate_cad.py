"""Sample B-rep interference, including guides and the actual cable spans."""
import argparse
import json
from build_cad import *

def conflicts(records):
    hits=[];fits=[]
    boxes={r['name']:bounds(r['shape']) for r in records}
    for i,a in enumerate(records):
        aa=boxes[a['name']]
        for b in records[i+1:]:
            bb=boxes[b['name']]
            if any(min(aa[k+3],bb[k+3])-max(aa[k],bb[k])<1e-6 for k in range(3)):continue
            op=BRepAlgoAPI_Common(a['shape'],b['shape']);op.Build()
            if not op.IsDone():raise RuntimeError('boolean common failed')
            v=volume(op.Shape())
            if v<.03:continue
            entry={'a':a['name'],'b':b['name'],'volume_mm3':v}
            same=a['name'][0]==b['name'][0]
            if same and {a['part'],b['part']} in ({'motor_body','motor_horn'},{'tendon','tendon_stop'}):fits.append(entry)
            else:hits.append(entry)
    return hits,fits

def main():
    p=argparse.ArgumentParser();p.add_argument('--quick',action='store_true');a=p.parse_args()
    parts=build_parts();results=[]
    poses=[(0,0),(15,35)] if a.quick else [(0,0),(10,15),(15,35),(9,60),(20,45),(25,30)]
    for q in poses:
        reel=(NEUTRAL_LENGTH-length(route(0,*q)))/SPOOL_RADIUS*180/math.pi
        rows=assembly(parts,q,reel);hits,fits=conflicts(rows)
        results.append({'q_deg':q,'unexpected':hits,'declared_fits':fits})
        print(q,'unexpected',hits,flush=True)
    (REPORTS/'interference.json').write_text(json.dumps(results,indent=2))
    if any(r['unexpected'] for r in results):raise SystemExit('unresolved interference')

if __name__=='__main__':main()
