"""Peripheral station: ports C and D. Each side gets a kit (Technic brick 1x8 on the hub's three shoulder holes,
carrying a Boost motor or a colour/distance sensor) and its plug is pushed into the hub port on the same side.

Robot lies on its back, head upstream (+x). Hub ports open sideways (+-z) at x = px-20, 55 LDU above the hole line.
Carrier = sled + two tower bars + a sliding fork: crossbar on five axles (2 guides through the kit, 2 push rods with pads,
1 slide axle) and the plug shoe (stud floor flush with the hub ledge, side rims). A separate plug rod, guided on the shoe
and on the tower, pushes the plug's rear face above the cable. The station paddle is a stepped wall: its upper step meets
the plug rod 30 LDU early (plug runs ahead into the shoe mouth), then the lower step pushes the two push rods 40 LDU home.
A rubber band (not modelled as geometry yet) returns the fork.
"""
import numpy as np
import ldr, asm2, cell
from asm2 import T, R, col3, add, P, YP, Y0, LBG, DBG, BLK, WHT, RED, YEL, BLU, ORG

TRK, LEAD, ZMK, RCK = 85.0, 30.0, 590, 60
PORT_Y = Y0 - 55
M0REF_INV = np.linalg.inv(asm2.core_M0(0.0))

# ---------------------------------------------------------------- synthetic soft parts (cable ribbon)
_SYN = {}
_orig_part = ldr.part
def _part(name):
    return _SYN[name] if name in _SYN else _orig_part(name)
ldr.part = _part

def _catmull(P_, n=10):
    P_ = np.asarray(P_, float); Q = np.vstack([P_[0], P_, P_[-1]]); out = []
    for i in range(1, len(Q) - 2):
        p0, p1, p2, p3 = Q[i-1], Q[i], Q[i+1], Q[i+2]
        for u in np.linspace(0, 1, n, endpoint=False):
            out.append(0.5*((2*p1) + (-p0 + p2)*u + (2*p0 - 5*p1 + 4*p2 - p3)*u*u + (-p0 + 3*p1 - 3*p2 + p3)*u**3))
    out.append(P_[-1]); return np.array(out)

def ribbon(name, pts, wide, hw=8.0, ht=1.5, col=0):
    """flat cable: pts (N,3) control points, wide (N,3) direction of the cable's wide axis at each control point."""
    C = _catmull(pts); W = _catmull(wide)
    t = np.gradient(C, axis=0); t /= np.linalg.norm(t, axis=1, keepdims=True)
    W = W - (W*t).sum(1, keepdims=True)*t; W /= np.linalg.norm(W, axis=1, keepdims=True)
    N = np.cross(t, W)
    ring = [C + a*hw*W + b*ht*N for a, b in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    tris = []
    for i in range(len(C) - 1):
        for k in range(4):
            a, b = ring[k], ring[(k + 1) % 4]
            tris += [[a[i], a[i+1], b[i+1]], [a[i], b[i+1], b[i]]]
    tris = np.array(tris); L = np.linalg.norm(np.diff(C, axis=0), axis=1).sum()
    _SYN[name] = dict(tris=tris, cols=np.full(len(tris), col), snaps=[], title='~cable')
    return L

CABLE_LEN = {}
def cable_part(kind, s, q):
    qi = int(round(q/10.0)); name = f'~cab_{kind}_{"A" if s > 0 else "B"}_{qi}'
    if name in _SYN: return name
    q = qi*10.0
    xe, ye, ze = (216, Y0 - 98 + 7, 100) if kind == 'motor' else (196, Y0 - 74 + 8, 80)
    r = (ze - 50)/2; xu = xe + 38; yc = Y0 - 50
    pts = [(-20, PORT_Y, 60 + q), (-20, PORT_Y, 72 + q), (-12, PORT_Y - 2, 88 + q), (-2, PORT_Y - 6, 104 + q), (10, PORT_Y - 6, 118 + q),
           (28, PORT_Y - 6, 124 + q), (55, PORT_Y - 2, 110 + q/2), (80, yc - 1, 80), (100, yc, 56), (125, yc, 50), (xu - 20, yc, 50), (xu, yc, 50),
           (xu + r, (yc + ye)/2, 50 + r), (xu, ye - 2, ze), (xe + 14, ye, ze), (xe, ye, ze)]
    X, Yv, Z = (1, 0, 0), (0, -1, 0), (0, 0, 1)
    wide = [X, X, (0.6, -0.8, 0), Yv, Yv, Yv, Yv, Yv, Yv, Yv, Yv, Yv, Yv, (0, -0.7, 0.7), Z, Z]
    pts = np.array(pts, float); pts[:, 2] *= s
    wide = np.array(wide, float); wide[:, 2] *= s
    CABLE_LEN[name] = ribbon(name, pts, wide)
    return name

# ---------------------------------------------------------------- cable: real length, physical bends only, surplus in an S-fold
import cable as _cab
def kit_cable(kind, s):
    name = f'~kitcab_{kind}_{"A" if s > 0 else "B"}'
    if name in _cab._SYN: return name
    zr, ytop, xu, ex, ey = (100, -355.5, 228, 216, Y0 - 98 + 7) if kind == 'motor' else (80, -355.5, 208, 196, Y0 - 98 + 8)   # run lies on the device's studs
    def build(Lf):
        r = _cab.Route((-20, PORT_Y, 54), (0, 0, 1), (1, 0, 0))          # starts inside the plug's cable stub
        r.to((-20, PORT_Y, zr - 10)).B((1, 0, 0), 90, 10)                # straight out of the plug, then up (rises between shoe wall and rim)
        r.S(20, 90).to((-20, ytop + 10, zr)).B((0, 0, 1), 90, 10)        # twist flat, turn towards the device over the top of the kit
        r.to((xu, ytop, zr))
        if Lf > 0: r.fold_pair((0, -1, 0), Lf)                           # surplus length: one flat S-fold stacked upwards
        yl = ytop - (14 if Lf > 0 else 0)
        r.B((0, 0, 1), 180, (ey - yl)/2).to((ex, ey, zr))                # down round the device's back and into its cable exit
        return r
    r, Lf = _cab.solve_fold(build, 0.0, 300.0)
    tris = r.mesh()
    if s < 0: tris = tris*np.array([1, 1, -1]); tris = tris[:, ::-1]      # mirrored side
    _cab._SYN[name] = dict(tris=tris, cols=np.full(len(tris), 0), snaps=[], title='~cable')
    CABLE_LEN[name] = (round(r.L, 1), round(Lf, 1))
    return name

# ---------------------------------------------------------------- kit (in the robot's lying reference frame, px = 0)
def kit(side, kind, G, body, delta=0.0, q=0.0, plug_body=None):
    """G: world transform of the kit frame (identity = seated on a robot lying at px=0). delta: plug offset from home."""
    s = 1 if side == 'A' else -1
    a = lambda n, p, c, M, b=body: add(f'kit{side}_{n}', p, c, G @ M, b)
    a('brick', '3702', DBG, T(160, Y0 - 10, 70*s))
    for x in (100, 120, 140): a(f'pin{x}', '2780', BLK, T(x, Y0, 60*s) @ R('y', 90))
    a('plate', '3460', DBG, T(160, Y0 - 18, 70*s))            # spacer, so the wide plate clears the fork crossbar's studs
    a('plate2', '3034', DBG, T(160, Y0 - 26, 80*s))           # 2x8: its outer stud row carries the motor's centre row
    if kind == 'sensor':
        a('riser', '3001', DBG, T(160, Y0 - 50, 80*s))                 # sensor raised one brick: its top is level with the motor's
        a('sensor', '26912', WHT, T(160, Y0 - 98, 80*s) @ R('y', 90))
    else: a('motor', '26913', WHT, T(160, Y0 - 98, 100*s) @ R('y', 90))
    a('plug', '23816a', LBG, T(-20, PORT_Y, (40 + delta)*s) @ (R('y', 180) if s > 0 else np.eye(4)), plug_body or body)
    P.append(dict(name=f'kit{side}_cable', part=kit_cable(kind, s), col=BLK, M=G @ np.eye(4), body=body,
                  ends=(f'kit{side}_plug', f'kit{side}_{kind}')))

# ---------------------------------------------------------------- carrier with fork and plug shoe
def carrier(side, sx, e, q):
    s = 1 if side == 'A' else -1; b = f'ksled{side}'; fb = f'fork{side}'
    a = lambda n, p, c, M, body=b: add(f'pal{side}K_{n}', p, c, M, body)
    a('base', '3027', LBG, T(sx, YP - 8, 260*s))
    for z in (250, 310):
        for dx in (-140, 140):
            for k in range(5): a(f'col{z}_{dx}_{k}', '3004', DBG if k % 2 else BLU, T(sx + dx, -116 - 24*k, z*s))
            for k in range(3): a(f'colp{z}_{dx}_{k}', '3023', DBG, T(sx + dx, -220 - 8*k, z*s))
        a(f'bar{z}', '3703', BLU, T(sx, Y0 - 10, z*s))
    xp = sx - 100                                                     # port x
    # ---- fork ----
    f = lambda n, p, c, M: a(n, p, c, M, fb)
    f('xbar', '3703', YEL, T(sx, Y0 - 10, (110 + e)*s))           # half a stud further out than before: puts the shoe on the stop's stud grid
    for i, dx in enumerate((40, 120)):                                # push rods: pad on the kit brick, end on the paddle
        f(f'rod{i}', '50451', BLK, T(sx + dx, Y0, (240 + e)*s) @ R('y', 90))
        f(f'rod{i}_pad', '32123a', RED, T(sx + dx, Y0, (85 + e)*s)); f(f'rod{i}_b', '32123a', YEL, T(sx + dx, Y0, (125 + e)*s))
    for i, dx in enumerate((80, 140)):                                # guides through the kit brick
        f(f'guide{i}', '50451', LBG, T(sx + dx, Y0, (220 + e)*s) @ R('y', 90))
        f(f'guide{i}_a', '32123a', YEL, T(sx + dx, Y0, (85 + e)*s)); f(f'guide{i}_b', '32123a', YEL, T(sx + dx, Y0, (125 + e)*s))
    f('slide', '3708', LBG, T(sx - 80, Y0, (210 + e)*s) @ R('y', 90))
    f('slide_b', '32123a', YEL, T(sx - 80, Y0, (125 + e)*s)); f('slide_stop', '32123a', YEL, T(sx - 80, Y0, (150 + e)*s))     # meets the inner tower bar at e = TRK
    for k in range(3): f(f'sh_p{k}', '3710', DBG, T(xp, Y0 - 18 - 8*k, (110 + e)*s))
    f('sh_floor', '3032', LBG, T(xp, Y0 - 42, (120 + e)*s) @ R('y', 90))                # 4x6 plate: stud tops = plug underside = hub ledge
    f('sh_stop', '3069b', RED, T(xp, Y0 - 50, (70 + e)*s))                                # stop tile: pushes the plug's lower back edge, under the cable
    f('sh_wall', '3010', YEL, T(xp - 30, Y0 - 66, (120 + e)*s) @ R('y', 90))
    f('sh_rim', '3710', YEL, T(xp + 30, Y0 - 50, (120 + e)*s) @ R('y', 90))

def paddle(side, theta, xp):
    s = 1 if side == 'A' else -1; zf = asm2.face_z(theta, ZMK, RCK); b = f'pusherK{side}'
    a = lambda n, p, c, M: add(f'psK{side}_{n}', p, c, M, b)
    a('pl', '4282', DBG, T(xp, -240, s*(zf + 20)))
    a('wall', '2465', RED, T(xp, -264, s*(zf + 10))); a('wall2', '2465', DBG, T(xp, -264, s*(zf + 30)))

def fork_state(theta):
    """fork offset e (0 = home) and plug lead q from the paddle position."""
    zf = asm2.face_z(theta, ZMK, RCK)
    e = float(np.clip(zf - 400, 0, TRK)); q = 0.0      # both return with the paddle (rubber bands)
    return e, q

def station(pxk, st, lanes=True):
    """st: ksx (carrier x), ktheta, kdone, kcfg (dict side -> kind), belts."""
    for x in (pxk - 320, pxk + 320):
        for z in (330, -310) + ((970, -950) if x > pxk else ()): add(f'baseK{x}_{z}', '3811', LBG, T(x, 0, z))
    if lanes:
        asm2.lane('K', 0, st['belt_m'], pxk); asm2.lane('KA', 260, st['belt_k'], pxk); asm2.lane('KB', -260, st['belt_k'], pxk)
    e, q = fork_state(st['ktheta'])
    for side in ('A', 'B'):
        kind = st['kcfg'].get(side)
        if not kind: continue
        s = 1 if side == 'A' else -1
        carrier(side, st['ksx'], e, q)
        if not st['kdone']:
            kit(side, kind, T(st['ksx'] - 80, 0, s*e), f'kit{side}', delta=q, q=q, plug_body=f'plug{side}')
        asm2.pusher(side, st['ktheta'], XM=pxk + 80, ZM=ZMK, tag='K', riser=False, rc=RCK)
        paddle(side, st['ktheta'], pxk + 80)

def theta_rest():
    """crank angle at which the paddle's lower face stands 115 back from home (partial swing of the r=60 crank)."""
    from scipy.optimize import brentq
    return brentq(lambda th: asm2.face_z(th, ZMK, RCK) - 515, 0.5, np.pi)
