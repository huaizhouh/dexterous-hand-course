"""B-rep checks for compact stops, wider backings and the outward V10 mount."""
from __future__ import annotations

import itertools
import json
from pathlib import Path

import cadquery as cq
import numpy as np

from build_hand import ROOT, P, L, T, R, frame, hinge_frame, transform, stop_components


def bounds(shape) -> np.ndarray:
    b=shape.BoundingBox()
    return np.array([b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax])


def overlap(a,b) -> float:
    aa,bb=bounds(a),bounds(b)
    if not np.all(np.minimum(aa[3:],bb[3:])-np.maximum(aa[:3],bb[:3])>1e-6):
        return 0.
    return max(0.,a.intersect(b).Volume())


def main() -> int:
    pp={p.stem:cq.importers.importStep(str(p)).val() for p in (ROOT/'parts_step').glob('*.step')}
    invalid=[name for name,s in pp.items() if not all(x.isValid() and x.Volume()>0 for x in s.Solids())]
    stops=stop_components()
    rows=[]
    for joint,angles in [('root',[4,10,20,35,45,60]),('pip',[4,15,30,50,70])]:
        for angle in angles:
            parent=pp['root'] if joint=='root' else pp['proximal']
            parent_frame=R('x',-P['root_mount_outward_deg']) if joint=='root' else T(z=L)
            child_frame=parent_frame@R('x',angle)
            child=transform(pp['proximal'] if joint=='root' else pp['distal'],child_frame)
            volume=overlap(parent,child)
            intentional=sum(overlap(transform(s['nose'],parent_frame),transform(s['wing'],child_frame)) for s in stops)
            rows.append(dict(joint=joint,angle_deg=angle,intersection_mm3=volume,
                             intentional_stop_mm3=intentional,unexpected_mm3=max(0.,volume-intentional),
                             passed=volume-intentional<=.03))
    opening_controls=[]
    for joint in ('root','pip'):
        parent=pp['root'] if joint=='root' else pp['proximal']
        base=R('x',-P['root_mount_outward_deg']) if joint=='root' else T(z=L)
        child=transform(pp['proximal'] if joint=='root' else pp['distal'],base)
        volume=overlap(parent,child)
        opening_controls.append(dict(joint=joint,forced_angle_deg=0,interference_mm3=volume,
                                     expected_mechanical_stop_interference=volume>.03))
    whole=[]
    for name,(q1,q2) in P['poses'].items():
        objects=[]
        for i in range(3):
            base=frame(i);prox=hinge_frame(i)@R('x',q1);dist=prox@T(z=L)@R('x',q2)
            for part,matrix in [('root',base),('proximal',prox),('distal',dist)]:
                objects.append((i,part,transform(pp[part],matrix)))
        collisions=[]
        for (i,a,aa),(j,b,bb) in itertools.combinations(objects,2):
            if i==j:
                continue
            volume=overlap(aa,bb)
            if volume>.03:
                collisions.append(dict(a=f'{i}_{a}',b=f'{j}_{b}',volume_mm3=volume))
        whole.append(dict(pose=name,joint_deg=[q1,q2],cross_finger_rigid_interferences=collisions,
                          passed=not collisions))
    passed=not invalid and all(r['passed'] for r in rows+whole) and all(r['expected_mechanical_stop_interference'] for r in opening_controls)
    report=dict(passed=passed,invalid_step_parts=invalid,joint_sweep=rows,
                mechanical_stop_negative_controls=opening_controls,whole_hand_poses=whole,
                scope='Sampled exact rigid-solid geometry and negative opening controls. TPU/bands may touch; no material, fatigue or real motor qualification.')
    (ROOT/'checks/v10_geometry.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
    return 0 if passed else 2


if __name__=='__main__':
    raise SystemExit(main())
