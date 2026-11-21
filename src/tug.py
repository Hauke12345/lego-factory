"""Tug: single-hub navigating robot. Colour/distance sensor on the lowest chest studs looking down (eye 15 mm above the floor),
Boost motor low on the back driving a hook arm; the hook peg drops into the open frame of a trailer's drawbar.
Yard: white tiles on baseplates with a black tile line, a red marker (pick-up) and a blue marker (drop-off);
ramps of 18-degree slope bricks lead down from the stand-up station's exit rails.
World = LDraw coords (LDU, -y up). Baseplate top y=0, floor tile top y=FLOOR.
"""
import numpy as np
import ldr, asm2, worker2, lcheck, periph
from asm2 import T, R, col3, LBG, DBG, BLK, WHT, RED, YEL, BLU, ORG
from cell import ux

FLOOR = -8.0
YC = -91                     # hitch motor centre (robot frame)
HOOK_UP, HOOK_DOWN = 30.0, -25.0
DH = 122.0                   # hitch point behind the drive centre
LT = 232.0                   # trailer: eye contact point -> skid centroid
SENS = 72.0                  # sensor spot ahead of the drive centre
BTRK = 272.0                 # track gauge
lcheck.RESTING.update({'trailer': 'slides on the floor', 'tug': 'stands on its tracks'})

# ---------------------------------------------------------------- cables (robot frame)
import cable as _cab
def tug_cables():
    if '~tugcabR' in _cab._SYN: return '~tugcabR', '~tugcabL'
    # right: plug -> round in front of the chest -> flat S-folds on the chest (layers stacked towards it) -> down into the belly sensor's top
    def bR(yb, yt=-290):
        r = _cab.Route((54, -191, -55), (1, 0, 0), (0, -1, 0))
        r.to((60, -191, -55)).B((0, 1, 0), 180, 24)                      # U-turn forwards, on edge, past the track beam
        r.S(20, 90).B((0, 0, 1), 90, 40)                                  # twist flat, curve up to the robot's centre line
        r.S(20, 90).to((0, yt, -103))                                     # twist to lie flat against the chest; layer 1 up
        r.B((-1, 0, 0), 180, _cab.FOLD_R).to((0, yb, -96))                # fold towards the chest; layer 2 down
        r.B((1, 0, 0), 180, _cab.FOLD_R).to((0, yt, -89))                 # layer 3 up
        r.B((-1, 0, 0), 180, _cab.FOLD_R).to((0, -87, -82))               # layer 4 down into the sensor's cable exit
        return r
    # left: plug -> backwards along the hub's left side (outside the track beam) -> flat S-folds hanging down -> into the hitch motor
    def bL(yb, n=1):
        z1 = 37 - 14*n
        r = _cab.Route((-54, -191, -55), (-1, 0, 0), (0, 1, 0))
        r.to((-80, -191, -55)).B((0, 1, 0), 90, 10)                       # turn backwards outside the track beam
        r.S(20, 90).to((-90, -191, z1 - 10)).B((1, 0, 0), -90, 10)        # twist flat, bend down
        for k in range(n):
            r.to((-90, yb, z1 + 14*k)).B(np.cross(r.T, (0, 0, 1)), 180, _cab.FOLD_R)
            r.to((-90, -181, z1 + 14*k + 7)).B(np.cross(r.T, (0, 0, 1)), 180, _cab.FOLD_R)
        r.S(24, 90).to((-90, -101, 37)).B((0, 0, 1), -90, 10)             # last layer: twist, then turn towards the motor
        r.S(24, 90)                                                       # twist upright into the motor's cable exit
        return r
    def solve(build, a, b):
        fa = build(a).L - _cab.L_REAL; fb = build(b).L - _cab.L_REAL
        if fa*fb > 0: raise ValueError(f'cable length 625 not reachable: {fa + 625:.0f} .. {fb + 625:.0f}')
        for _ in range(40):
            m = 0.5*(a + b); fm = build(m).L - _cab.L_REAL
            if fa*fm <= 0: b, fb = m, fm
            else: a, fa = m, fm
        return build(0.5*(a + b))
    rR = solve(bR, -270, -100)
    rL = solve(lambda yb: bL(yb, 2), -40, -170)
    for n, r in (('~tugcabR', rR), ('~tugcabL', rL)): _cab.register(n, r); CABLES[n] = round(r.L, 1)
    return '~tugcabR', '~tugcabL'
CABLES = {}

# ---------------------------------------------------------------- robot
def robot(M0, P, hook=HOOK_UP, track_phase=0.0, tag='tug', body='tug'):
    worker2.build(M0=M0, tag=tag, body=body, P=P, modules={'core', 'trackL', 'trackR', 'head'}, track_phase=track_phase)
    a = lambda n, p, c, M, b=body: P.append(dict(name=f'{tag}_{n}', part=p, col=c, M=M0 @ M, body=b))
    # belly sensor: thin half on the two lowest chest stud rows, thick half hooks under the hub, eye looking down
    a('eye', '26912', WHT, T(0, -51, -90) @ col3((1, 0, 0), (0, 0, 1), (0, -1, 0)))
    # hitch motor on the lowest four back rows, output on the right side
    MM = np.eye(4); MM[:3, :3] = np.column_stack([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
    a('hmotor', '26913', WHT, T(0, YC, 30) @ MM)
    a('h_axle', '4519', BLK, T(70, YC, 64), 'hook')
    A = T(0, YC, 64) @ R('x', hook)
    a('h_arm', '32449', RED, A @ T(80, 0, 0) @ ux([0, 0, 1]) @ T(0, 0, 30), 'hook')
    a('h_cross', '32073', BLK, A @ T(40, 0, 60), 'hook')
    a('h_block', '6536', DBG, A @ T(0, 0, 60) @ col3((1, 0, 0), (0, 0, 1), (0, -1, 0)), 'hook')
    a('h_peg', '6558', BLK, A @ T(0, 20, 80) @ R('z', 90), 'hook')
    # plugs and cables: sensor -> right port, motor -> left port. Physical flat-cable routes, 25 cm each, surplus in flat S-folds
    a('plugR', '23816a', LBG, T(40, -191, -55) @ col3((0, -1, 0), (0, 0, 1), (-1, 0, 0)))
    a('plugL', '23816a', LBG, T(-40, -191, -55) @ col3((0, 1, 0), (0, 0, 1), (1, 0, 0)))
    # cable clamps. chest: 1x2 brick + 2 plate walls either side of the folded right cable, 2x4 plate across (the stack sits in the channel)
    Wy = col3((0, 1, 0), (0, 0, 1), (1, 0, 0))                         # brick long axis along y, body towards the chest (+z)
    for wx in (-30, 30):
        a(f'cc_w{wx}', '3004', DBG, T(wx, -291, -90) @ Wy)
        for k, z in enumerate((-98, -106)): a(f'cc_p{wx}_{k}', '3023', DBG, T(wx, -291, z) @ Wy)
    a('cc_cover', '3020', LBG, T(0, -291, -114) @ col3((1, 0, 0), (0, 0, 1), (0, -1, 0)))
    # left side: two long pins in the track beam's outer holes carry thin liftarms across the hanging S-folds
    for y in (-171, -131):
        a(f'lc_pin{y}', '6558', BLK, T(-90, y, 0))
        a(f'lc_arm{y}', '32065', DBG, T(-105, y, 20) @ R('z', 90))        # 7x0.5 thin beam (round holes), its third hole on the pin
    nR, nL = tug_cables()
    P.append(dict(name=f'{tag}_cabR', part=nR, col=BLK, M=M0 @ np.eye(4), body=body, ends=(f'{tag}_plugR', f'{tag}_eye')))
    P.append(dict(name=f'{tag}_cabL', part=nL, col=BLK, M=M0 @ np.eye(4), body=body, ends=(f'{tag}_plugL', f'{tag}_hmotor')))
    return P

def pose(x, z, psi, hc_front=None, hc_rear=None):
    """robot matrix from plan position, heading and the two wheel-centre heights (above baseplate top)."""
    f = 46.0 if hc_front is None else hc_front; r = 46.0 if hc_rear is None else hc_rear
    pitch = np.degrees(np.arctan2(r - f, 160.0))
    return T(x, -(f + r)/2, z) @ R('y', np.degrees(psi)) @ R('x', pitch) @ T(0, 11, 0)

# ---------------------------------------------------------------- trailer (local: origin = eye contact point on the floor, -z forward)
def trailer(M, P, body='trailer'):
    a = lambda n, p, c, Mm: P.append(dict(name=f'trl_{n}', part=p, col=c, M=M @ Mm, body=body))
    a('eye', '64179', YEL, T(0, FLOOR - 54, 42))                      # front inner edge 8 ahead of the peg axis
    for x in (-20, 0, 20): a(f'pin{x}', '2780', BLK, T(x, FLOOR - 54, 112) @ R('y', 90))
    a('mast_t', '3701', DBG, T(0, FLOOR - 64, 122)); a('mast_b', '3010', DBG, T(0, FLOOR - 40, 122))
    a('deck', '3028', LBG, T(0, FLOOR - 16, 232) @ R('y', 90))
    for x in (-40, 40):
        for z in (152, 312): a(f'skid{x}_{z}', '2654a', DBG, T(x, FLOOR - 8, z))
    a('cargo_m', '26913', WHT, T(0, FLOOR - 88, 252)); a('cargo_s', '26912', WHT, T(0, FLOOR - 64, 152) @ R('y', 90))
    return P

def trailer_M(x, z, phi):
    """trailer matrix: contact point at (x, z), forward direction = heading phi (same convention as the robot)."""
    return T(x, 0, z) @ R('y', np.degrees(phi))

# ---------------------------------------------------------------- yard: tiles on baseplates
X0, Z0, NI, NJ = -2800, -630, 48, 32            # cell (i, j): x in [X0-40(i+1), X0-40i], z in [Z0+40j, Z0+40(j+1)]
PATH = [(-2800, -10), (-3300, -10), (-3700, -410), (-4100, -410)]
RC = 400.0
def path_points(step=4.0):
    pts = [np.array(PATH[0], float)]
    for k in range(1, len(PATH) - 1):
        a, b, c = (np.array(PATH[k + d], float) for d in (-1, 0, 1))
        u = (b - a)/np.linalg.norm(b - a); v = (c - b)/np.linalg.norm(c - b)
        th = np.arccos(np.clip(np.dot(u, v), -1, 1)); tl = RC*np.tan(th/2)                 # fillet of radius RC for any bend angle
        p0, p1 = b - tl*u, b + tl*v; n = v - np.dot(v, u)*u; n /= np.linalg.norm(n); ctr = p0 + RC*n
        seg = np.linalg.norm(p0 - pts[-1]); pts += [pts[-1] + u*s for s in np.arange(step, seg, step)] + [p0]
        a0 = np.arctan2(*(p0 - ctr)[::-1]); a1 = np.arctan2(*(p1 - ctr)[::-1]); da = (a1 - a0 + np.pi) % (2*np.pi) - np.pi
        pts += [ctr + RC*np.array([np.cos(a0 + da*t), np.sin(a0 + da*t)]) for t in np.linspace(0, 1, 40)[1:]]
    a, b = pts[-1], np.array(PATH[-1], float); u = (b - a)/np.linalg.norm(b - a)
    pts += [a + u*s for s in np.arange(step, np.linalg.norm(b - a), step)] + [b]
    return np.array(pts)

def _grid():
    G = np.zeros((NI, NJ), int)                       # 0 white, 1 black, 2 red, 3 blue
    pp = path_points()
    for i in range(NI):
        for j in range(NJ):
            c = np.array([X0 - 40*i - 20, Z0 + 40*j + 20])
            d = np.linalg.norm(pp - c, axis=1).min()
            if d < 45: G[i, j] = 1
    for i in range(NI):
        for j in range(NJ):
            if G[i, j] == 1:
                x = X0 - 40*i - 20
                if x < -4040: G[i, j] = 2
                if -3000 < x < -2920: G[i, j] = 3
    return G
GRID = _grid()
REFL = {0: 85.0, 1: 6.0, 2: 45.0, 3: 18.0}
def cell_at(x, z):
    i = int(np.floor((X0 - x)/40)); j = int(np.floor((z - Z0)/40))
    return GRID[i, j] if 0 <= i < NI and 0 <= j < NJ else -1

def yard(P):
    a = lambda n, p, c, M: P.append(dict(name=n, part=p, col=c, M=M, body='frame'))
    for x in (-2880, -3520, -4160, -4800):
        for z in (330, -310): a(f'ybase{x}_{z}', '3811', LBG, T(x, 0, z))
    used = np.zeros((NI, NJ), bool); COL = {1: BLK, 2: RED, 3: BLU}
    def place(i, j, w, h, part, col, rot):
        used[i:i + w, j:j + h] = True
        xc = X0 - 40*i - 20*w; zc = Z0 + 40*j + 20*h
        a(f'railtile_{i}_{j}', part, col, T(xc, FLOOR, zc) @ (R('y', 90) if rot else np.eye(4)))
    for kind in (1, 2, 3):                             # line and markers: 2x4 tiles where two cells line up, else 2x2
        for i in range(NI):
            for j in range(NJ):
                if GRID[i, j] != kind or used[i, j]: continue
                if i + 1 < NI and GRID[i + 1, j] == kind and not used[i + 1, j]: place(i, j, 2, 1, '87079', COL[kind], False)
                elif j + 1 < NJ and GRID[i, j + 1] == kind and not used[i, j + 1]: place(i, j, 1, 2, '87079', COL[kind], True)
                else: place(i, j, 1, 1, '3068b', COL[kind], False)
    free = lambda i, j, w, h: i + w <= NI and j + h <= NJ and not used[i:i + w, j:j + h].any() and (GRID[i:i + w, j:j + h] == 0).all()
    for i in range(NI):                                # first row of big tiles straddles the z = 10 baseplate seam
        for j in range(2, NJ, 4):
            if free(i, j, 8, 4): place(i, j, 8, 4, '90498', WHT, False)
    for w, h, part, rot in ((8, 4, '90498', False), (4, 8, '90498', True), (3, 3, '10202', False), (2, 1, '87079', False), (1, 2, '87079', True), (1, 1, '3068b', False)):
        for i in range(NI):
            for j in range(NJ):
                if free(i, j, w, h): place(i, j, w, h, part, WHT, rot)
    return P

# ---------------------------------------------------------------- exit rails (end part) and ramps
XS0 = -2490                                           # first slope's stud row
def ramps(P):
    a = lambda n, p, c, M: P.append(dict(name=n, part=p, col=c, M=M, body='frame'))
    for z in (330, -310): a(f'rbase_{z}', '3811', LBG, T(-2240, 0, z))
    for s in (1, -1):
        z = 130*s
        for k in range(4):                            # last stretch of the stand-up station's rails
            xs = [(-2080 - 160*i, '3007') for i in range(3)] if k % 2 == 0 else [(-2040, '3001')] + [(-2160 - 160*i, '3007') for i in range(2)] + [(-2440, '3001')]
            for xc, part in xs: a(f'rail{s}_{k}_{xc}', part, DBG if k % 2 else LBG, T(xc, -24 - 24*k, z))
        for k in range(2):
            xs = [(-2080 - 160*i, '3034') for i in range(3)] if k == 0 else [(-2040, '3020')] + [(-2160 - 160*i, '3034') for i in range(2)] + [(-2440, '3020')]
            for xc, part in xs: a(f'railp{s}_{k}_{xc}', part, LBG, T(xc, -104 - 8*k, z))
        for xc in range(-2040, -2441, -80): a(f'railt{s}_{xc}', '87079', BLK, T(xc, -120, z))
        a(f'railt{s}_lip', '3069b', BLK, T(XS0, -120, z) @ R('y', 90))
        for k in range(4):
            xo = XS0 - 80*k; top = -112 + 24*k
            a(f'railt_slope{s}_{k}', '30363', DBG, T(xo, top, z) @ R('y', 90))
            nb = 3 - k
            for b in range(nb):
                if b == 0 and k < 2: continue                 # one 2x8 brick under slopes 0 and 1 ties the two baseplates
                a(f'rs{s}_{k}_b{b}', '3001', LBG if b % 2 else DBG, T(xo - 30, -24 - 24*b, z))
            if k == 0: a(f'rs{s}_tie', '3007', DBG, T(XS0 - 70, -24, z))
            for p in range(2): a(f'rs{s}_{k}_p{p}', '3020', LBG, T(xo - 30, -24*nb - 8 - 8*p, z))
    return P

def surf(x):
    """track surface height above baseplate top along the ramp lanes."""
    if x >= XS0 - 10: return 120.0
    k = int(np.floor((XS0 + 10 - x)/80.0))
    if k > 3: return 8.0
    xo = XS0 - 80*k; H = 112.0 - 24*k
    if x > xo - 12: return H + (8.0 if k == 0 else 4.0)
    return H - 4.8 - 15.2*((xo - 12) - x)/58.0
_XS = np.arange(-2900, -2380, 2.0); _SF = np.array([surf(x) for x in _XS])
def wheel_h(x):
    """height of a 38-radius wheel centre resting on the surface at x."""
    if x < -2850: return 46.0
    if x > -2440: return 158.0
    m = np.abs(_XS - x) < 38
    return float((_SF[m] + np.sqrt(38.0**2 - (_XS[m] - x)**2)).max())

if __name__ == '__main__':
    import sys
    what = sys.argv[1] if len(sys.argv) > 1 else 'robot'
    P = []
    if what == 'robot': robot(pose(0, 0, 0), P, hook=float(sys.argv[2]) if len(sys.argv) > 2 else HOOK_UP)
    elif what == 'trailer': trailer(trailer_M(0, 0, 0), P)
    elif what == 'hitched':
        robot(pose(0, 0, 0), P, hook=HOOK_DOWN); trailer(trailer_M(0, DH, 0), P)
    elif what == 'yard': yard(P); ramps(P); print('cells: black', (GRID == 1).sum(), 'red', (GRID == 2).sum(), 'blue', (GRID == 3).sum())
    print(len(P), 'parts')
    rep = lcheck.run(P)
    print('PASS' if not rep['clashes'] and not rep['bad_fits'] and not rep['unconnected'] else 'FAIL')
