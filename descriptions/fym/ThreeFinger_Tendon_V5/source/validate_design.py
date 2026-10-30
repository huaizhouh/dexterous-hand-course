"""Repeatable geometry checks. Sampling is not physical grasp validation."""
import itertools,json,math
from pathlib import Path
import numpy as np
import cadquery as cq
from build_hand import *

def bbox(s):
 b=s.BoundingBox();return np.array([b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax])
def overlaps(a,b,tol=1e-5): return np.all(np.minimum(a[3:],b[3:])-np.maximum(a[:3],b[:3])>tol)
def common_volume(a,b):
 if not overlaps(bbox(a),bbox(b)): return 0.
 return max(0.,a.intersect(b).Volume())

def check_pose(pp,qs,name):
 _,items=make_assembly(pp,*qs,name,flex=False,fasteners=False)
 # Soft shims are intentionally compressed in final installation. Imported
 # servo subcomponents are not checked against each other as they are source
 # assembly geometry, with a source-healing disclosure in the report.
 rigid=[x for x in items if x['kind']!='liner']
 for x in rigid:x['bbox']=bbox(x['shape'])
 problems=[];checked=0
 for a,b in itertools.combinations(rigid,2):
  if not overlaps(a['bbox'],b['bbox']):continue
  checked+=1
  print('  checking',a['name'],b['name'],flush=True)
  v=max(0.,a['shape'].intersect(b['shape']).Volume())
  if v>.03:problems.append([a['name'],b['name'],round(v,6)])
 print(name,'pairs',checked,'interferences',problems,flush=True)
 return {'q_deg':qs,'candidate_pairs_tested':checked,'threshold_mm3':.03,'interferences':problems}

def main():
 pp=parts();report={}
 for n,qs in P['poses'].items():report[n]=check_pose(pp,qs,n)
 report['local_joint_grid']=[]
 for q1 in (0,5,10,15,25,35,45):
  pr=transform(pp['proximal'],R('x',q1))
  v=common_volume(pp['root'],pr)
  if v>.03:report['local_joint_grid'].append({'root_q':q1,'volume_mm3':v})
 for q2 in (0,5,10,20,35,50,60,70):
  di=transform(pp['distal'],T(z=L)@R('x',q2))
  v=common_volume(pp['proximal'],di)
  if v>.03:report['local_joint_grid'].append({'distal_q':q2,'volume_mm3':v})
 report['tendon_geometry']={}
 for q1,q2 in [*P['poses'].values(),(35,70)]:
  eps=.01;le=cable_length(q1,q2)
  # Effective moment arms = -d(route length)/dq, mm, with q in radians.
  arms=[-(cable_length(q1+eps,q2)-le)/math.radians(eps),
        -(cable_length(q1,q2+eps)-le)/math.radians(eps)]
  report['tendon_geometry'][f'{q1},{q2}']={'length_mm':le,'takeup_mm':cable_length(0,0)-le,'moment_arms_mm':arms}
 # Whole-hand intermediate poses along each defined example path.
 report['intermediate_poses']=[]
 for end in [q for n,q in P['poses'].items() if n!='OPEN']:
  for f in (.25,.5,.75):
   qs=[q*f for q in end]
   report['intermediate_poses'].append(check_pose(pp,qs,f'path_{end}_{f}'))
 # Verify the principal STEP deliverables using a new import.
 report['step_reimport']=[]
 for p in [ROOT/'00_OPEN_IN_SOLIDWORKS.step']+list((ROOT/'cad').glob('*.step')):
  s=cq.importers.importStep(str(p)).val()
  valid=all(x.isValid() and x.Volume()>0 for x in s.Solids())
  report['step_reimport'].append({'path':str(p.relative_to(ROOT)),'solids':len(s.Solids()),'valid_solids':valid})
  print('Reimport',p.name,len(s.Solids()),valid,flush=True)
 report['limits']='Static and sampled B-rep checks only; no SolidWorks runtime, load, friction or real object verification.'
 (ROOT/'checks/geometry_validation.json').write_text(json.dumps(report,indent=2))
 problems=sum(len(v['interferences']) for k,v in report.items() if isinstance(v,dict) and 'interferences' in v)
 problems+=sum(len(v['interferences']) for v in report['intermediate_poses'])
 problems+=len(report['local_joint_grid'])
 print('Unresolved geometric interferences:',problems,flush=True)
 if problems or not all(x['valid_solids'] for x in report['step_reimport']):raise SystemExit(1)
if __name__=='__main__':main()
