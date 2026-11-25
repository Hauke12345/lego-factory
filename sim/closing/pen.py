import sys; sys.path.insert(0, '/home/claude/scripts'); sys.path.insert(0, '/home/claude/lego-factory/sim/closing')
import numpy as np, ldr
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
import contacts as C
from asm2 import T, R
Tr = np.array(ldr.part('3873')['tris'], float)
def band(x0, x1):
    ps = []
    for t in Tr:
        if t[:, 0].min() >= x0 - .01 and t[:, 0].max() <= x1 + .01:
            p = Polygon(t[:, [2, 1]])
            if p.area > 1e-6: ps.append(p.buffer(0))
    return unary_union(ps)
hook = band(7.5, 12)                      # cheeks + hooks (z, y) in the part frame
pinR = max(np.hypot(*(v[[2, 1]] - [16, 0])) for t in Tr for v in t if 8 <= abs(v[0]) <= 12 and abs(v[2] - 16) < 3 and abs(v[1]) < 3)
print('pin radius', round(pinR, 2), ' hook region area', round(hook.area, 1))
# the hook alone (exclude the pin disk at z 16 of the same link)
hook = hook.difference(Point(16, 0).buffer(3))
def pen(beta):
    M = T(*C.J) @ R('x', beta) @ T(*(-C.J)) @ C.MX
    c = (np.linalg.inv(C.M0) @ M @ np.array([0, 0, 16, 1.0]))[[2, 1]]
    disk = Point(*c).buffer(pinR, 64)
    inter = disk.intersection(hook)
    if inter.is_empty: return c, 0.0
    # penetration depth ~ pin radius minus distance from centre to the hook boundary outside
    d = hook.exterior.distance(Point(*c)) if hasattr(hook, 'exterior') else hook.boundary.distance(Point(*c))
    inside = hook.contains(Point(*c))
    return c, (pinR + d) if inside else (pinR - d)
for b in (40, 30, 20, 15, 12, 10, 8, 6, 4, 2, 1, 0):
    c, p = pen(b); print(f'beta {b:4.1f}: pin centre (z {c[0]:6.2f}, y {c[1]:6.2f})  overlap with link0 hook {p:5.2f} LDU')
