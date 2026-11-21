"""Shot list for the combined film: factory line (line2) then the tug's first job (tug simulation log).
Each shot: pieces of the underlying timeline, camera keys (Blender metres), caption.
film time -> (segment, state, camera). 24 fps.
"""
import numpy as np, pickle
import line2, worker2, tug, asm2

FPS = 24
LDU = 0.0004
def bl(x, y, z): return np.array([x*LDU, -z*LDU, -y*LDU])
HUB_LOCAL = np.array([*worker2.HC, 1.0])

# ---------------------------------------------------------------- timelines
PH = {n: d for n, d in line2.PHASES}; PH_START = {}; _acc = 0.0
for n, d in line2.PHASES: PH_START[n] = _acc; _acc += d
PH_TOT = _acc
def line_t(phase, f): return (PH_START[phase] + np.clip(f, 0, 0.999)*PH[phase])/PH_TOT

_LOG = None
def log():
    global _LOG
    if _LOG is None: _LOG = pickle.load(open('/home/claude/factory/prev/tug_log2.pkl', 'rb'))[0]
    return _LOG
def tug_state(ts):
    L = log(); i = int(np.clip(np.floor(ts/0.05), 0, len(L) - 2)); a, b = L[i], L[i + 1]
    f = float(np.clip((ts - a['t'])/(b['t'] - a['t']), 0, 1)); s = dict(a)
    for k in ('x', 'z', 'psi', 'hook', 'dist', 'v', 'dl', 'dr', 'cl', 'cr'):
        if k not in a: continue
        s[k] = a[k] + f*(b[k] - a[k])
    if a['trl'] and b['trl']: s['trl'] = tuple(np.array(a['trl']) + f*(np.array(b['trl']) - np.array(a['trl'])))
    return s

def ease(u): return u*u*(3 - 2*u)
def ease2(u): return 0.5 - 0.5*np.cos(np.pi*np.clip(u, 0, 1))

# ---------------------------------------------------------------- shots
# piece: ('line', phase, f0, f1, seconds) or ('tug', t0, t1, seconds)
# cam: list of keys (u, pos, target, lens, fstop); pos/target either absolute (Blender m) or relative to the tracked hub when 'track' is set
SHOTS = [
    dict(name='establish', cap='Station 1 · a robot body arrives on its own pallet', pieces=[('line', 'karrive', 0, 1, 4.0)],
         cam=[(0, (1.10, 0.90, 0.58), (0.10, 0.0, 0.03), 24, 0), (1, (0.86, 0.36, 0.28), (0.50, 0.0, 0.10), 34, 0)]),
    dict(name='kfeed', cap='Station 1 · the kits follow by train, one wagon per port', pieces=[('line', 'kfeed', 0, 1, 4.5)],
         cam=[(0, (1.62, 0.92, 0.52), (0.86, -0.05, 0.02), 24, 0), (1, (1.20, 0.70, 0.36), (0.62, -0.02, 0.05), 28, 0)]),
    dict(name='plug', cap='Station 1 · plug pushed into the hub port, kit pinned to the shoulder', pieces=[('line', 'kpush', 0, 1, 6.0)],
         cam=[(0, (0.68, -0.33, 0.27), (0.51, -0.04, 0.115), 45, 8.0), (1, (0.62, -0.24, 0.21), (0.51, -0.045, 0.115), 50, 8.0)]),
    dict(name='kit_out', cap='Station 1 · forks retract, robot moves on, empty wagons pulled back', pieces=[('line', 'kretract', 0, 1, 1.5), ('line', 'travel1', 0, 1, 2.5)],
         track=True, cam=[(0, (0.16, 0.30, 0.10), (0.0, 0.0, 0.0), 32, 0), (1, (-0.10, 0.27, 0.07), (0.0, 0.0, 0.0), 32, 0)]),
    dict(name='feed', cap='Station 2 · track units arrive by train while the body waits', pieces=[('line', 'feed', 0, 1, 4.0)],
         cam=[(0, (-0.98, 0.80, 0.46), (-0.30, -0.02, 0.02), 24, 0), (1, (-0.62, 0.62, 0.32), (-0.12, 0.0, 0.05), 28, 0)]),
    dict(name='tracks', cap='Station 2 · track units pushed onto pin and motor output', pieces=[('line', 'push', 0, 1, 3.0), ('line', 'retract', 0, 1, 1.4), ('line', 'sleds', 0, 0.5, 1.3)],
         cam=[(0, (0.19, -0.33, 0.21), (-0.02, -0.04, 0.10), 42, 8.0), (1, (0.09, -0.29, 0.15), (-0.03, -0.03, 0.10), 46, 8.0)]),
    dict(name='travel', cap='', pieces=[('line', 'travel', 0, 1, 3.0)],
         track=True, cam=[(0, (0.30, 0.42, 0.36), (0.0, 0.0, 0.0), 28, 0), (1, (0.10, 0.40, 0.22), (0.0, 0.0, 0.0), 30, 0)]),
    dict(name='lift', cap='Station 3 · stand-up: two hub-driven arms swing the deck upright', pieces=[('line', 'lift', 0, 1, 4.5)],
         cam=[(0, (-0.42, 0.40, 0.07), (-0.72, 0.0, 0.11), 28, 0), (1, (-0.50, 0.50, 0.12), (-0.75, 0.0, 0.17), 28, 0)]),
    dict(name='drive', cap='Station 3 · the robot drives off on its own tracks', pieces=[('line', 'drive', 0, 1, 3.5)],
         track=True, cam=[(0, (-0.44, 0.22, 0.05), (0.0, 0.0, 0.03), 34, 9.0), (1, (-0.34, 0.30, 0.07), (0.0, 0.0, 0.03), 36, 9.0)]),
    dict(name='ramp', cap='First job · done by a tug variant (built earlier): sensor under the belly, hook motor on the back', pieces=[('tug', 0.0, 5.7, 4.5)],
         track=True, cam=[(0, (-0.06, 0.44, 0.10), (-0.02, 0.0, 0.03), 32, 0), (1, (-0.20, 0.38, 0.07), (-0.02, 0.0, 0.02), 32, 0)]),
    dict(name='follow', cap='First job · following the line edge by reflected light', pieces=[('tug', 5.7, 16.9, 6.0)],
         track=True, cam=[(0, (0.30, 0.30, 0.34), (-0.03, 0.0, 0.0), 30, 0), (1, (0.05, 0.42, 0.30), (-0.03, 0.0, 0.0), 30, 0)]),
    dict(name='hook', cap='First job · red marker: turn round, back up, hook down', pieces=[('tug', 16.9, 23.1, 3.0), ('tug', 23.1, 26.6, 4.2)],
         cam=[(0, (-1.58, -0.26, 0.19), (-1.64, 0.14, 0.05), 40, 8.0), (1, (-1.71, -0.16, 0.10), (-1.665, 0.14, 0.035), 50, 8.0)]),
    dict(name='tow', cap='First job · towing the trailer back along the same edge', pieces=[('tug', 26.6, 40.3, 5.0)],
         track=True, cam=[(0, (-0.20, -0.30, 0.22), (0.05, 0.0, 0.0), 30, 0), (1, (0.25, -0.28, 0.20), (0.05, 0.0, 0.0), 30, 0)]),
    dict(name='end', cap='First job · blue marker: unhook, drive clear', pieces=[('tug', 40.3, 43.29, 3.0), ('tug', 43.29, 43.29, 1.0)],
         cam=[(0, (-0.98, 0.42, 0.16), (-1.22, 0.0, 0.04), 32, 0), (1, (-0.98, 0.48, 0.20), (-1.21, -0.01, 0.04), 30, 0)]),
    dict(name='next', cap='Mass production · finished variants wait, the next body is already arriving', pieces=[('line', 'karrive', 0, 1, 3.0), ('line', 'kfeed', 0, 0.8, 3.5)],
         cam=[(0, (0.30, 1.20, 0.62), (0.45, -0.10, 0.06), 26, 0), (1, (0.75, 1.30, 0.80), (0.62, -0.12, 0.04), 24, 0)]),
]
def frames():
    """list of dicts per film frame: shot index, local u, segment, state, hub position (Blender m)."""
    out = []
    for si, sh in enumerate(SHOTS):
        tot = sum(p[-1] for p in sh['pieces']); n = int(round(tot*FPS)); acc = 0.0
        bounds = []
        for p in sh['pieces']: bounds.append((acc, acc + p[-1], p)); acc += p[-1]
        for k in range(n):
            ts = (k + 0.5)/FPS; u = ts/tot
            for a, b, p in bounds:
                if ts <= b or p is bounds[-1][2]: break
            f = np.clip((ts - a)/(b - a), 0, 1)
            if p[0] == 'line':
                f = p[2] + f*(p[3] - p[2]); st = line2.cycle(line_t(p[1], f)); seg = 'line'
                px = st['px']; M0 = asm2.core_M0(px)
                import line1
                P_ = line2.build(dict(st, s0=False, s1=False, s2=False))
                hub = [q for q in P_ if q['name'] == 'rob_hub'][0]['M'] @ np.array([0, 0, 0, 1.0])
            else:
                tsim = p[1] + f*(p[2] - p[1]); st = tug_state(tsim); seg = 'tug'
                M = tug.pose(st['x'], st['z'], st['psi']); hub = M @ HUB_LOCAL
            out.append(dict(shot=si, u=u, seg=seg, st=st, hub=bl(*hub[:3])))
    # smooth the tracked hub within each shot (no smoothing across cuts)
    for si in range(len(SHOTS)):
        idx = [i for i, f in enumerate(out) if f['shot'] == si]; H = np.array([out[i]['hub'] for i in idx])
        sig = 10.0; K = np.arange(-30, 31); w = np.exp(-K**2/(2*sig**2)); w /= w.sum()
        Hp = np.pad(H, ((30, 30), (0, 0)), mode='edge'); Hs = np.array([np.convolve(Hp[:, c], w, mode='valid') for c in range(3)]).T
        for j, i in enumerate(idx): out[i]['hub_s'] = Hs[j]
    return out

def _catmull(keys, u, k):
    """interpolate component k of the keys at u with Catmull-Rom through the eased key times."""
    us = [q[0] for q in keys]; vals = [np.array(q[k], float) for q in keys]
    if len(keys) == 2: return vals[0] + ease2(u)*(vals[1] - vals[0])
    u = np.clip(u, 0, 1); i = max(0, min(len(us) - 2, int(np.searchsorted(us, u, side='right') - 1)))
    t = ease2((u - us[i])/(us[i + 1] - us[i]))
    p0 = vals[max(i - 1, 0)]; p1, p2 = vals[i], vals[i + 1]; p3 = vals[min(i + 2, len(vals) - 1)]
    return 0.5*((2*p1) + (-p0 + p2)*t + (2*p0 - 5*p1 + 4*p2 - p3)*t*t + (-p0 + 3*p1 - 3*p2 + p3)*t**3)

def camera(fr):
    sh = SHOTS[fr['shot']]; keys = sh['cam']; u = fr['u']
    pos = _catmull(keys, u, 1); tgt = _catmull(keys, u, 2)
    lens = float(_catmull(keys, u, 3)); fst = float(_catmull(keys, u, 4))
    if sh.get('track'): pos = pos + fr['hub_s']; tgt = tgt + fr['hub_s']
    return pos, tgt, lens, fst

if __name__ == '__main__':
    F = frames()
    print(len(F), 'frames', len(F)/FPS, 's')
    for si, sh in enumerate(SHOTS):
        n = sum(1 for f in F if f['shot'] == si); print(f'{si:2d} {sh["name"]:10s} {n:4d} frames  {n/FPS:4.1f} s')
