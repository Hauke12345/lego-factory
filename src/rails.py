"""Train-track feeders for the side lanes: real LEGO track (53401 straight, 53400 R40 curve), module wagons on
pin-mounted bogie wheels, and a two-motor loco with a Powered Up hub that pushes / pulls each wagon.

Plan coordinates (x, z) on the baseplate; heading h = unit vector in (x, z). Track sits on the baseplate (rail top y = -24).
"""
import numpy as np
import asm2, lcheck
from asm2 import T, R, col3, add, LBG, DBG, BLK, WHT, RED, YEL, BLU, ORG

RC = 802.26                       # centre-line radius of the R40 curve (fitted to the 53400 rails)
SEG = 320.0                       # straight piece length
ARC = np.radians(22.5)            # one curve piece
Y_TRK = -16                       # track origin (sleeper top); its underside rests on a carpet of smooth tiles (curved track can't lock onto studs)
RAIL_TOP = -32

def frame(p, h, y=0.0):
    """4x4: local x along heading h, local y down, local z = h x y (right-handed), at plan point p, height y."""
    hx, hz = h; M = np.eye(4)
    M[:3, 0] = [hx, 0, hz]; M[:3, 1] = [0, 1, 0]; M[:3, 2] = [-hz, 0, hx]; M[:3, 3] = [p[0], y, p[1]]
    return M

class Path:
    """chain of straights and arcs; s runs from the far (feed) end towards and through the station lane."""
    def __init__(self, p, h):
        self.p0 = np.array(p, float); self.h0 = np.array(h, float); self.segs = []; self.L = 0.0
        self.p = self.p0.copy(); self.h = self.h0.copy()
    def S(self, L):
        self.segs.append(('S', self.p.copy(), self.h.copy(), L)); self.p = self.p + self.h*L; self.L += L; return self
    def A(self, ang, turn):                         # turn = +1 left (towards +z when heading +x), -1 right
        n = turn*np.array([-self.h[1], self.h[0]])  # towards the centre
        c = self.p + n*RC
        self.segs.append(('A', self.p.copy(), self.h.copy(), RC*ang, c, turn, ang))
        th = turn*ang; Rm = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        self.p = c + Rm @ (self.p - c); self.h = Rm @ self.h; self.L += RC*ang; return self
    def at(self, s):
        s = float(s)
        for sg in self.segs:
            L = sg[3]
            if s <= L or sg is self.segs[-1]:
                if sg[0] == 'S': return sg[1] + sg[2]*s, sg[2].copy()
                _, p, h, L, c, turn, ang = sg; th = turn*s/RC
                Rm = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
                return c + Rm @ (p - c), Rm @ h
            s -= L
        return self.p.copy(), self.h.copy()
    def pose(self, s, half):
        """vehicle with two support points +-half along the path: centre and chord heading."""
        a, _ = self.at(s - half); b, _ = self.at(s + half)
        h = (b - a)/np.linalg.norm(b - a); return 0.5*(a + b), h
    def pieces(self, tag):
        out = []
        for k, sg in enumerate(self.segs):
            if sg[0] == 'S':
                _, p, h, L = sg
                for i in range(int(round(L/SEG))):
                    q = p + h*(SEG*(i + 0.5)); out.append((f'{tag}_s{k}_{i}', '53401', frame(q, h, Y_TRK)))
            else:
                _, p, h, L, c, turn, ang = sg
                for i in range(int(round(ang/ARC))):
                    th = turn*ARC*(i + 0.5); Rm = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
                    q = c + Rm @ (p - c); t = Rm @ h
                    # 53400: origin on the centre line at the arc middle, tangent local +x, centre of curvature on local -z
                    toc = (c - q)/np.linalg.norm(c - q)
                    X = np.array([t[0], 0, t[1]]); Z = -np.array([toc[0], 0, toc[1]]); Y = np.cross(Z, X)
                    if Y[1] < 0: X = -X; Y = -Y
                    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2] = X, Y, Z; M[:3, 3] = [q[0], Y_TRK, q[1]]
                    out.append((f'{tag}_c{k}_{i}', '53400', M))
        return out

# ---------------------------------------------------------------- lane paths
LANE_XJ = {'A': -1450.0 + RC, 'B': -1450.0 + RC, 'KA': 1920.0, 'KB': 1920.0, 'UA': -2880.0}   # where the curve meets the lane straight
LEAD = {'A': 4, 'B': 4}                                  # lead-in straights (A/B: long, the track-unit handover sits on them)
LANE_DEF = {'TA': (-80.0, 1.0), 'TB': (80.0, 1.0), 'SA': (260.0, 1.0), 'SB': (-260.0, 1.0)}   # straight lanes: canonical (zc, hx)
STRAIGHT = {}                                            # lane -> (start plan point, heading, length): registered by tline
def lane_path(lane):
    """'A','B' (track station, wagons travel +x) and 'KA','KB' (port station, wagons travel -x).
    Lead-in straight from the side, a 90-degree R40 curve, then the lane straight past the station."""
    if lane in STRAIGHT:
        p0, h, L = STRAIGHT[lane]; return Path(p0, h).S(L)
    zc, hx = _lane(lane); sz = np.sign(zc); xj = LANE_XJ[lane]; nlane = 4; nl = LEAD.get(lane, 1)
    p0 = (xj - hx*RC, zc + sz*(RC + nl*SEG))
    for turn in (1, -1):
        P = Path(p0, (0, -sz)); P.S(nl*SEG).A(np.pi/2, turn)
        if abs(P.p[1] - zc) < 1 and abs(P.p[0] - xj) < 1 and P.h[0]*hx > 0.99: break
    else: raise RuntimeError('lane path does not close')
    P.S(nlane*SEG)
    return P

_PATHS = {}
def path(lane):
    if lane not in _PATHS: _PATHS[lane] = lane_path(lane)
    return _PATHS[lane]

def s_of_x(lane, x):
    """path parameter of a point on the lane straight with plan x."""
    P = path(lane); s0 = LEAD.get(lane, 1)*SEG + RC*np.pi/2; p, h = P.at(s0)
    return s0 + (x - p[0])/h[0]

def track(lane):
    for n, part, M in path(lane).pieces('trk' + lane): add(n, part, DBG, M, 'frame')

def baseplates(lane):
    """32x32 baseplates under the curve and lead-in (centres on the 640 grid used by the stations)."""
    P = path(lane); pts = [P.at(s)[0] for s in np.linspace(0, P.L, 80)]
    cells = set()
    for x, z in pts:
        for dx in (-100, 0, 100):
            for dz in (-100, 0, 100):
                cx = 640*np.floor((x + dx)/640) + 320; cz = 640*np.floor((z + dz - 10)/640) + 330
                cells.add((cx, cz))
    return cells

TAKEN = set()
def tiles(lane, taken):
    """smooth 2x4 tiles under the track band (on the baseplate stud grid: x centres = 0 mod 80... z centres = 10 mod 20)."""
    P = path(lane); pts = np.array([P.at(s)[0] for s in np.arange(0, P.L + 1, 10)])
    from scipy.spatial import cKDTree
    tr = cKDTree(pts)
    lo, hi = pts.min(0) - 120, pts.max(0) + 120
    xs = np.arange(80*np.floor(lo[0]/80), hi[0], 80); zs = np.arange(40*np.floor(lo[1]/40) - 10, hi[1], 40)
    for x in xs:
        for z in zs:
            if (x, z) in taken: continue
            # cell 80 x 40 centred at (x, z): keep if any corner is within the track's half width (+ margin)
            c = np.array([[x + dx, z + dz] for dx in (-40, 0, 40) for dz in (-20, 0, 20)])
            if tr.query(c)[0].min() < 84:
                taken.add((x, z)); add(f'tile_{x}_{z}', '87079', DBG, T(x, -8, z), 'frame')

# ---------------------------------------------------------------- wagon wheels (under a carrier's 6x16 base)
# per corner: 1x4 plate + Technic brick 1x4 under the base, an L-liftarm (2x4) pinned in two brick holes with its short leg down,
# a bogie wheel (38340) pinned in the short leg's lower hole. Axle height -46 puts the tread 1 LDU over the rail top
# (the wheel rides on the flange fillet; pin play absorbs it) - LEGO train wheels never land exactly on the stud grid.
WB = 60                                 # half wheelbase (on the base's stud grid): a 6-stud wheelbase runs on R40 without binding
def wagon_wheels(sx, zc, body, tag):
    for dx in (-WB, WB):
        for sg in (-1, 1):
            n = f'{tag}_wh{dx}_{sg}'; x = sx + dx; z = zc + sg*10; d = sg; xc = x + 20*d          # L corner over the brick's end hole
            add(n + 'b', '3701', DBG, T(x, -84, z), body)                         # Technic brick 1x4 under the base: holes at y -74, x -20/0/20
            # L-liftarm 32140 (long leg local z: axle end at 0, corner at 60; short leg local x to (20, 0, 60)):
            # long leg under the brick along x, short leg straight down, holes along world z
            Lm = np.eye(4); Lm[:3, 0] = (0, 1, 0); Lm[:3, 1] = (0, 0, d); Lm[:3, 2] = (d, 0, 0)
            add(n + 'L', '32140', DBG, T(xc - 60*d, -74, zc + sg*30) @ Lm, body)
            for k, px in enumerate((xc, x)):
                add(n + f'pin{k}', '2780', BLK, T(px, -74, zc + sg*20) @ R('y', 90), body)
            Wm = np.eye(4); Wm[:3, 1] = (0, 1, 0); Wm[:3, 2] = (0, 0, -sg); Wm[:3, 0] = np.cross(Wm[:3, 1], Wm[:3, 2])
            add(n + 'w', '38340', BLK, T(xc, -54, zc + sg*48) @ Wm, body)           # pin into the short leg, tread on the rail
    lcheck.RESTING.setdefault(body, 'rolls on the rails')

# ---------------------------------------------------------------- loco: a Boost Move Hub on a 6x16 deck, its two built-in
# motors drive the front axle through 40T (on the outputs) -> 24T (on the wheel axle) gears; no cables needed.
# Free-running wheels 55423 (r 22.9 with rubber ring) on 8L axles in Technic bricks hung 5 plates under the deck.
LOCO_YB = RAIL_TOP - 22.9 - 50                    # deck top: wheel axle 50 below it, tread on the rail
AX_F, AX_R = 140.0, -60.0                         # axle x in the hub frame (front axle under the outputs)
LOCO_MID = 0.5*(AX_F + AX_R); LOCO_HALF = 0.5*(AX_F - AX_R)
def loco(W, tag, body):
    """W: 4x4 world frame of the loco (origin on the track centre line midway between the axles, x forward)."""
    yb = LOCO_YB
    a = lambda n, p, c, M: add(f'{tag}_{n}', p, c, W @ T(-LOCO_MID, 0, 0) @ M, body)
    a('deck', '3027', DBG, T(0, yb, 0))
    a('hub', '26910', WHT, T(0, yb - 96, 0) @ R('y', 90))                          # outputs at x = +140, z = -+60, y = yb - 30
    for k, xa in enumerate((AX_F, AX_R)):
        for sz in (-1, 1):
            z = 30*sz
            a(f'sup{k}{sz}_b', '3004', DBG, T(xa, yb + 8, z))
            a(f'sup{k}{sz}_p', '3023', DBG, T(xa, yb + 32, z))
            a(f'sup{k}{sz}_t', '3700', DBG, T(xa, yb + 40, z))                     # hole at yb + 50
            Wm = np.eye(4); Wm[:3, 1] = (0, 1, 0); Wm[:3, 2] = (0, 0, -sz); Wm[:3, 0] = np.cross(Wm[:3, 1], Wm[:3, 2])
            a(f'wh{k}{sz}', '55423', BLK, T(xa, yb + 50, 51*sz) @ Wm)               # flange inside the rail
        a(f'ax{k}', '3707', BLK, T(xa, yb + 50, 0) @ R('y', 90))
    # only the output on the side away from the main line drives (the inner gear train would brush the flap pallet's hinge lugs)
    zs = [sz for sz in (-1, 1) if abs((W @ T(-LOCO_MID, 0, 0) @ np.array([AX_F, 0, 70*sz, 1.0]))[2]) >= abs((W @ T(-LOCO_MID, 0, 0) @ np.array([AX_F, 0, -70*sz, 1.0]))[2])]
    for sz in zs:
        a(f'out{sz}', '4519', BLK, T(AX_F, yb - 30, 74*sz) @ R('y', 90))            # 3L axle in the hub output (16 deep)
        a(f'g40{sz}', '3649', LBG, T(AX_F, yb - 30, 70*sz))
        a(f'g24{sz}', '3648b', LBG, T(AX_F, yb + 50, 70*sz))
    lcheck.RESTING.setdefault(body, 'rolls on the rails')

def loco_world(lane, s):
    """loco frame from path parameter of its axle midpoint."""
    P = path(lane); c, h = P.pose(s, LOCO_HALF); return frame(c, h)

# ---------------------------------------------------------------- magnetic couplers (2607 holder on one stud, 2959c01 magnet on its hinge)
# the holders swivel on their single stud, so on a curve both arms line up on the line between the two studs
ARM = 26.0                                      # stud -> magnet centre
def coupler_at(stud, d, tag, body):
    """stud: world point of the holder's stud (holder top); d: unit plan direction of the arm."""
    Rm = np.eye(4); z = -np.array([d[0], 0, d[1]]); Rm[:3, 2] = z; Rm[:3, 1] = (0, 1, 0); Rm[:3, 0] = np.cross(Rm[:3, 1], z)
    add(f'{tag}_mh', '2607', DBG, T(*stud) @ Rm, body)
    add(f'{tag}_mg', '2959c01', DBG, T(*stud) @ Rm @ T(0, 9, -ARM), body)

def _lane(lane):
    if lane in LANE_DEF: return LANE_DEF[lane]
    return (260.0 if lane in ('A', 'KA', 'UA') else -260.0), (1.0 if lane in ('A', 'B') else -1.0)
def cart_frame(lane, s):
    c, h = path(lane).pose(s, WB); return frame(c, h)
def cart_stud(lane, s):
    zc, hx = _lane(lane); return (cart_frame(lane, s) @ np.array([-150, -84, 10*hx, 1.0]))[:3]      # rear stud row, on the cart's +10 side
def cart(lane, s, build, sx0, body, tag):
    """build(sx0) adds a carrier (+ module) at its lane-straight position sx0; it gets wheels and is moved to path parameter s."""
    zc, hx = _lane(lane)
    n0 = len(asm2.P); build(sx0)
    wagon_wheels(sx0, zc, body, tag)
    G = cart_frame(lane, s) @ np.linalg.inv(frame((sx0, zc), (hx, 0)))
    for p in asm2.P[n0:]: p['M'] = G @ p['M']
    return G

LOCO_GAP = 150 + 2*ARM + 16 + (LOCO_MID + 150)    # cart centre -> loco axle midpoint on a straight (loco runs drive-end first, away from the cart)
def _loco_frame(lane, sl):
    c, h = path(lane).pose(sl, LOCO_HALF); return frame(c, -h)
def _loco_stud(lane, sl):
    zc, hx = _lane(lane)
    return (_loco_frame(lane, sl) @ T(-LOCO_MID, 0, 0) @ np.array([-150, LOCO_YB + 16, -10*hx, 1.0]))[:3]
def loco_s(lane, s_cart):
    """loco position whose magnet just touches the cart's: studs 2*ARM + 16 apart (plan distance), also on curves."""
    cs = cart_stud(lane, s_cart); D = 2*ARM + 16
    f = lambda sl: np.linalg.norm((_loco_stud(lane, sl) - cs)[[0, 2]]) - D
    lo, hi = s_cart - LOCO_GAP - 80, s_cart - LOCO_GAP + 30                   # f(lo) > 0, f(hi) < 0 (approaching, before overlap)
    for _ in range(50):
        mid = 0.5*(lo + hi)
        if f(mid) > 0: lo = mid
        else: hi = mid
    return 0.5*(lo + hi)
def train_couplers(lane, s_cart, tagc, bodyc, tagl, bodyl):
    sl = loco_s(lane, s_cart); W = _loco_frame(lane, sl)
    cs = cart_stud(lane, s_cart); ls = _loco_stud(lane, sl)
    d = (ls - cs)[[0, 2]]; d /= np.linalg.norm(d)
    coupler_at(cs, d, tagc, bodyc); coupler_at(ls, -d, tagl, bodyl)
    return sl, W
def loco_at(lane, s_cart, tag, body, tagc, bodyc):
    zc, hx = _lane(lane)
    sl, W = train_couplers(lane, s_cart, tagc, bodyc, tag, body)               # drive end leads away from the cart
    loco(W, tag, body)
    add(f'{tag}_cpl_p', '3022', DBG, W @ T(-LOCO_MID, 0, 0) @ T(-140, LOCO_YB + 8, 0), body)
