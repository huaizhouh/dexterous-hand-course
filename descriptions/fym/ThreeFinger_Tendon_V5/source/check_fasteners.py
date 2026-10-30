"""Check fasteners against rigid solids for one representative radial module."""
import json,itertools
import cadquery as cq
from build_hand import *
from validate_design import bbox,overlaps

def main():
 pp=parts();_,items=make_assembly(pp,0,0,'FASTENER_CHECK',flex=False,fasteners=True)
 items=[x for x in items if x['group']=='finger_A' or x['name'] in ('palm','base','column_0','column_bolt_0','column_nut_0','column_washer_0_0','column_washer_0_1')]
 for x in items:x['bbox']=bbox(x['shape'])
 problems=[];tested=0
 for a,b in itertools.combinations(items,2):
  if 'metal' not in (a['kind'],b['kind']):continue
  if not overlaps(a['bbox'],b['bbox']):continue
  tested+=1
  v=max(0.,a['shape'].intersect(b['shape']).Volume())
  if v>.03:
   problems.append([a['name'],b['name'],v]);print(problems[-1],flush=True)
 report={'pose_deg':[0,0],'scope':'Finger A fasteners vs all same-module rigid parts and palm; one frame column. Other module geometry is identical by radial transform.',
         'candidate_pairs':tested,'threshold_mm3':.03,'unresolved':problems,
         'not_checked':'Thread engagement, real screw-head profiles, tool access and bolt-preload deformation need physical trial assembly.'}
 (ROOT/'checks/fastener_validation.json').write_text(json.dumps(report,indent=2))
 print('Fastener pairs',tested,'unresolved',len(problems),flush=True)
 if problems:raise SystemExit(1)
if __name__=='__main__':main()
