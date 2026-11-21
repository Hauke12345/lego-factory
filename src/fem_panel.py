"""Side-elevation polygons of every bridge part + the truck's real parts (truck-local 3D, coloured)."""
import json, math, numpy as np
from scipy.spatial import ConvexHull
import sys; sys.path.insert(0, '/home/claude/scripts'); import bridge as BR
D = json.load(open('/home/claude/render_b/build_check.json'))
def l2s(c): return [int(255*(12.92*x if x <= 0.0031308 else 1.055*x**(1/2.4) - 0.055)) for x in c]
out = {'bridge': {}, 'truck': []}
# parked pose used in check_build: T(0, 0.20, Z_BOX) @ Rz(-90)
t0 = np.array([0, 0.20, BR.Z_BOX]); Rz = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]])      # Rz(-90)
for n, p in D['parts'].items():
    if p['group'] == 'truck':
        for sh in p['shapes']:
            w = np.array(sh['verts']); loc = (Rz.T @ (w - t0).T).T                         # world -> truck-local
            loc = np.unique(np.round(loc, 5), axis=0)
            if len(loc) > 160: loc = loc[np.linspace(0, len(loc) - 1, 160).astype(int)]
            out['truck'].append({'v': loc.tolist(), 'color': l2s(sh['color']), 'name': sh['name']})
        continue
    M = np.array(p['M']); v = np.array(p['verts']); w = (M[:3, :3] @ v.T).T + M[:3, 3]
    yz = w[:, 1:3]
    try: h = ConvexHull(yz); poly = yz[h.vertices].tolist()
    except Exception: poly = yz.tolist()
    out['bridge'][n] = {'poly': poly, 'x': float(w[:, 0].mean()), 'group': p['group']}
json.dump(out, open('/home/claude/render_b/fem_panel.json', 'w')); print(len(out['bridge']), len(out['truck']))
