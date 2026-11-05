"""CAD-volume masses and full inertia, with explicit unmeasured densities/lumps."""
from __future__ import annotations

import cadquery as cq
import numpy as np


def moving_links(parts: dict, parameters: dict) -> dict:
    result={}
    for name in ('proximal','distal'):
        entries=[]
        for part,density in [(name,parameters['rigid_density_kg_m3']),
                             ('pad_'+name,parameters['pad_density_kg_m3'])]:
            shape=parts[part]
            rho=density*1e-9
            center=np.asarray(cq.Shape.centerOfMass(shape).toTuple())*.001
            entries.append((shape.Volume()*rho,center,np.asarray(cq.Shape.matrixOfInertia(shape))*rho*1e-6))
        extra=parameters['moving_hardware_lumps'][name]
        entries.append((extra['mass_kg'],np.asarray(extra['center_mm'])*.001,np.zeros((3,3))))
        mass=sum(m for m,_,_ in entries)
        center=sum(m*c for m,c,_ in entries)/mass
        inertia=np.zeros((3,3))
        for m,c,i in entries:
            r=c-center
            inertia+=i+m*(np.dot(r,r)*np.eye(3)-np.outer(r,r))
        result[name]=dict(mass_kg=mass,center_m=center.tolist(),
                          fullinertia_kg_m2=[float(inertia[i,j]) for i,j in [(0,0),(1,1),(2,2),(0,1),(0,2),(1,2)]],
                          eigenvalues_kg_m2=np.linalg.eigvalsh(inertia).tolist(),
                          assumptions='Uniform assumed rigid/pad densities; moving hardware point mass. Not measured.')
    return result
