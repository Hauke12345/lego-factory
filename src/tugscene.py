"""scene assembly for a simulation sample: static yard + robot + trailer."""
import numpy as np, tug, tugsim
_static = None
def static():
    global _static
    if _static is None:
        P = []; tug.yard(P); tug.ramps(P); _static = P
    return _static
def dynamic(s):
    P = []
    f = np.array([-np.sin(s['psi']), -np.cos(s['psi'])])
    hf = hr = 46.0
    if abs(s['z']) < 5 and s['x'] > -2960:                       # on the rails / ramp: two idlers on the profile, bottom run must clear the step edges
        xf, xr = s['x'] + 80*f[0], s['x'] - 80*f[0]
        hf, hr = tug.wheel_h(xf), tug.wheel_h(xr)
        c = 160.0/np.hypot(160.0, hf - hr)
        xs = np.linspace(xf, xr, 81); line = np.interp(xs, [xf, xr], [hf, hr]) - 38.0/c
        lift = max(0.0, max(tug.surf(x) - l for x, l in zip(xs, line)))
        hf += lift; hr += lift
    M = tug.pose(s['x'], s['z'], s['psi'], hf, hr)
    Q = []; tug.robot(M, Q, hook=s['hook'], track_phase=(s['dl'], s['dr']) if 'dl' in s else s['dist'])
    # the robot rests on its lowest link edge: lift it until no link plate corner is below the surface under it
    C = np.array([[x, -8, z, 1] for x in (-26, 26) for z in (-1, 11)], float).T
    lift = 0.0
    for p in Q:
        if p['part'] != '3873': continue
        for w in (p['M'] @ C).T:
            h = -w[1]
            if abs(abs(w[2]) - 130) <= 20 and w[0] > tug.XS0 - 330: sup = tug.surf(w[0])
            elif w[0] < tug.X0 + 1e-6 or abs(w[2]) > 400: sup = 8.0                       # yard floor tiles start at X0
            else: sup = 0.0
            lift = max(lift, sup - h)
    if abs(s['z']) < 5 and s['x'] > -2960:                          # on the ramp: refine against the real slope and rail geometry
        lift += _ramp_extra([p for p in Q if p['part'] == '3873'], lift)
    for p in Q: p['M'] = tug.T(0, -lift, 0) @ p['M']
    P += Q
    if s['trl']: tug.trailer(tug.trailer_M(*s['trl']), P)
    return P

_RAMP = None
def _ramp_extra(links, lift):
    """smallest extra lift (0.25 steps) at which no track link touches a rail or slope brick."""
    global _RAMP
    import lcheck
    from mathutils.bvhtree import BVHTree
    def tree(w):
        v = w.reshape(-1, 3); return BVHTree.FromPolygons(v.tolist(), np.arange(len(v)).reshape(-1, 3).tolist(), all_triangles=True)
    if _RAMP is None:
        _RAMP = [(p, tree(lcheck.world_tris(p)), lcheck.world_tris(p).reshape(-1, 3)) for p in static() if p['name'].startswith(('railt', 'rs', 'rail'))]
    W0 = [(lcheck.world_tris(dict(p, M=tug.T(0, -lift, 0) @ p['M']))) for p in links]
    lo_ = [w.reshape(-1, 3).min(0) for w in W0]; hi_ = [w.reshape(-1, 3).max(0) for w in W0]
    for extra in np.arange(0, 4.01, 0.25):
        hit = False
        for w, lo, hi in zip(W0, lo_, hi_):
            ws = w + np.array([0, -extra, 0])
            for p, t, V in _RAMP:
                vmin, vmax = V.min(0), V.max(0)
                if np.any(vmax < lo - [1, extra + 1, 1]) or np.any(vmin > hi + 1): continue
                if tree(ws).overlap(t): hit = True; break
            if hit: break
        if not hit: return float(extra)
    return 4.0
