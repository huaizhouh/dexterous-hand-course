#!/usr/bin/env python3
"""Sample solid interference of actual delivered STEP over flexion angles."""
import json
import numpy as np
from build_cad import HERE, root, rot, moved, read, solids, bounds, volume, BRepAlgoAPI_Common, combine, export_step, BRepCheck_Analyzer

report=json.loads((HERE/'cad_validation.json').read_text())
items=solids(read(HERE/'direct_drive_open.step'))
if len(items)!=len(report['instances']):raise RuntimeError('Unexpected STEP solid count')
for s,record in zip(items,report['instances']):
 if abs(volume(s)-record['volume_mm3'])>1e-3 or max(abs(a-b) for a,b in zip(bounds(s),record['bounds_mm']))>0.1:
  raise RuntimeError('STEP solid order changed')
results=[]
for angle in (0,10,20,30,40):
 records=[]
 for s,record in zip(items,report['instances']):
  name=record['name']
  if name.startswith('finger') and not name.endswith('_servo') and '_tie' not in name:
   finger=int(name[6]);m=root((finger-1)*120)
   s=moved(s,m@rot('x',angle)@np.linalg.inv(m))
  records.append((name,s))
 conflicts=[]
 for i,(name,a) in enumerate(records):
  ab=bounds(a)
  for name2,b in records[i+1:]:
   # The supplied servo unit already embeds each horn into its own axle.
   if name.endswith('_servo') and name2.startswith(name[:7]) and 'horn_' in name2:continue
   bb=bounds(b)
   if any(min(ab[k+3],bb[k+3])-max(ab[k],bb[k]) < 1e-5 for k in range(3)):continue
   common=BRepAlgoAPI_Common(a,b);common.Build()
   if not common.IsDone():raise RuntimeError(f'Common failed: {name} {name2}')
   v=volume(common.Shape())
   if v>0.05:conflicts.append({'a':name,'b':name2,'volume_mm3':v})
 if angle==40 and not conflicts:
  target=HERE/'direct_drive_40deg.step';export_step(combine([s for _,s in records]),target)
  check=solids(read(target))
  if len(check)!=len(records) or any(not BRepCheck_Analyzer(s).IsValid() for s in check):
   raise RuntimeError('40 degree STEP reimport validation failed')
 results.append({'angle_deg':angle,'overlaps_over_0_05_mm3':conflicts})
 print(angle,conflicts,flush=True)
(HERE/'motion_validation.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
if any(r['overlaps_over_0_05_mm3'] for r in results):raise SystemExit('Motion interference requires correction')
