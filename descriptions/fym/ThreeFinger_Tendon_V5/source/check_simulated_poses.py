"""Exact B-rep rigid-link checks at representative simulated joint angles.

Complements the simplified rigid-link collision proxies in MuJoCo. No claim
about continuous-time collision clearance or loaded part deformation is made.
"""
import itertools,json,argparse
import numpy as np
from build_hand import ROOT,parts,frame,R,T,L,transform

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--runtime',action='store_true',help='read latest run_v5_sim reports instead of benchmark traces');args=ap.parse_args()
    pp=parts();results=[]
    for name in ('sphere','pingpong','cube','cylinder','capsule','small_sphere'):
        path=ROOT/f'v5_sim_output/grasp_{name}_report.json' if args.runtime else ROOT/f'checks/benchmarks/v5_{name}_center.json'
        trace=json.loads(path.read_text())['samples']
        for t in (3.,8.5,12.):
            row=min(trace,key=lambda x:abs(x['time_s']-t));items=[]
            for i in range(3):
                q1,q2=row['joint_deg'][i*2:i*2+2]
                b=frame(i);pr=b@R('x',q1);di=pr@T(z=L)@R('x',q2)
                for key,m in [('root',b),('proximal',pr),('distal',di)]:
                    s=transform(pp[key],m);bb=s.BoundingBox()
                    items.append((f'{chr(65+i)}_{key}',s,np.array([bb.xmin,bb.ymin,bb.zmin,bb.xmax,bb.ymax,bb.zmax])))
            problems=[];pairs=0
            for (na,a,aa),(nb,b,bb) in itertools.combinations(items,2):
                if not np.all(np.minimum(aa[3:],bb[3:])-np.maximum(aa[:3],bb[:3])>1e-5):continue
                pairs+=1;v=max(0.,a.intersect(b).Volume())
                if v>.03:problems.append({'a':na,'b':nb,'intersection_mm3':v})
            results.append({'object':name,'time_s':row['time_s'],'joint_deg':row['joint_deg'],'candidate_pairs':pairs,'interferences':problems})
            print(name,t,'rigid interferences',len(problems),flush=True)
    report={'scope':'Exact root/proximal/distal solids at 18 sampled states from centered simulations. Excludes flexible pads, fasteners, bands and continuous trajectories. Static full-assembly checks are separate.',
        'threshold_mm3':.03,'results':results,'unresolved':sum(len(r['interferences']) for r in results)}
    (ROOT/'checks/simulated_pose_geometry.json').write_text(json.dumps(report,indent=2))
    if report['unresolved']:raise SystemExit(1)
if __name__=='__main__':main()
