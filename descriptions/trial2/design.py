"""Shared dimensions, rigid transforms and tendon routing. CAD uses mm."""
import math
import numpy as np

BASE_RADIUS = 60.0
PIVOT_HEIGHT = 15.0
PROXIMAL_LENGTH = 45.0
SPOOL_RADIUS = 7.8  # cable centre radius, not drum solid radius
FINGERS = ("A", "B", "C")

def shift(x=0., y=0., z=0.):
    m = np.eye(4); m[:3, 3] = (x, y, z); return m

def turn(axis, degrees):
    a = math.radians(degrees); c, s = math.cos(a), math.sin(a); m = np.eye(4)
    if axis == "x": m[:3, :3] = ((1,0,0),(0,c,-s),(0,s,c))
    else: m[:3, :3] = ((c,-s,0),(s,c,0),(0,0,1))
    return m

def base(i):
    a = math.radians(i*120)
    return shift(-BASE_RADIUS*math.sin(a), BASE_RADIUS*math.cos(a), PIVOT_HEIGHT) @ turn("z", i*120)

def apply(m, p): return (m @ np.r_[p, 1.])[:3]

# Fixed, radiused routing beneath the palm. Two 3 mm bend-radius elbows.
FIXED_ROUTE = [(35., -8., -48.), (35., -8., -26.)]
FIXED_ROUTE += [(32+3*math.cos(a), -8., -26+3*math.sin(a)) for a in np.linspace(0, math.pi/2, 7)[1:]]
FIXED_ROUTE += [(3., -8., -23.)]
FIXED_ROUTE += [(3-3*math.sin(a), -8., -20-3*math.cos(a)) for a in np.linspace(0, math.pi/2, 7)[1:]]
FIXED_ROUTE += [(0., -8., -6.)]
PROX_GUIDES = [(0., -8., 32.5), (0., -8., 35.5)]
DIST_GUIDES = [(0., -8., 6.5), (0., -8., 9.5), (0., -8., 30.5), (0., -8., 34.)]

def route(i, q1=0., q2=0.):
    root = base(i); p = root @ turn("x", q1)
    d = p @ shift(z=PROXIMAL_LENGTH) @ turn("x", q2)
    return [apply(root, s) for s in FIXED_ROUTE] + [apply(p, s) for s in PROX_GUIDES] + [apply(d, s) for s in DIST_GUIDES]

def length(points): return sum(float(np.linalg.norm(b-a)) for a,b in zip(points, points[1:]))

NEUTRAL_LENGTH = length(route(0))
