"""Chain station: builds the track unit's drive set (3 sprockets + a closed 34-link chain) from single parts.

Geometry is written in the left track unit's robot frame (worker2: x right, y down, z back), in the pose where the finished
drive set sits exactly where the S4 supply wagon's guides take it over ("tilted" pose). The build table tilts 90 deg about a
hinge axle along robot y (lane direction) into its "horizontal" pose: stubs pointing up, chain plane horizontal, so the open
chain lies flat on the table while it is built link by link.

  table      : plates (Technic plates with holes carry the three stub axles), smooth tiles under the chain, the drive Move Hub
               underneath (output B -> bevel pair -> 24T -> 8T on the drive stub; output A -> rack -> transfer frame)
  stubs      : axles at the three sprocket axes, tips 2 LDU short of the S4 guide tips
  rods       : three push rods through the table behind the sprockets, on a thin-liftarm frame driven by the rack: they slide
               the finished drive set off the stubs onto the S4 guides (56 LDU)
  hinge      : 1x2 axle-hole Technic bricks under the table on an axle into the station's tilt Move Hub (fixed pedestal)
Links are added one at a time at the bottom run (slot = link 19 of worker2's loop), radially from outside, each new link
snapping onto the tail; between insertions the drive stub turns the chain on by one pitch.
"""
import numpy as np
import asm2, worker2, lcheck
from asm2 import T, R, col3, add, P, LBG, DBG, BLK, WHT, RED, YEL, ORG, BLU

AT, PITCH, PER = worker2.track_path()
K_SLOT = 20                                       # worker2 link index in the insertion slot (bottom run, entering the driven idler)
X_TOP = -110.0                                    # table surface (the chain's inner edge)
X_HINGE, Z_HINGE = -60.0, 140.0                   # hinge axle (along robot y): on the ground stud grid (canonical z 60, height 110)
DRV = (-71.0, 0.0); IDL = [(-11.0, 80.0), (-11.0, -80.0)]
PO = col3((0, 1, 0), (1, 0, 0), (0, 0, -1))      # plate/brick: studs facing -x (towards the chain), long axis (local x) along robot y
PZ = col3((0, 0, 1), (1, 0, 0), (0, 1, 0))       # same, long axis along robot z
AXX = R('z', 0)                                   # axles: local x along robot x
THIN_Z = col3((1, 0, 0), (0, 1, 0), (0, 0, 1))   # thin liftarm: holes along local y ... set per use

def link_M(s):
    """worker2 link frame at loop parameter s (left unit)."""
    p, t = AT(s)
    out = -np.array([-t[1], t[0]])
    Z = np.array([0, t[0], t[1]]); Yv = np.array([0, out[0], out[1]]); X = np.cross(Yv, Z)
    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2] = X, Yv, Z; M[:3, 3] = [-136, p[0], p[1]]
    return M
S_SLOT = K_SLOT*PITCH
MAG = np.array([0.0, 14.0, 0.0])                  # magazine step (radially out from the slot, +y)

def chain_links(n, adv):
    """links in place after n insertions, the chain advanced by `adv` pitches since the first one (adv = n - 1 + fraction).
    Link j (j = 0 first inserted) sits at loop parameter S_SLOT + (adv - j)*PITCH (it travels the loop direction +s)."""
    out = []
    for j in range(n):
        s = S_SLOT + (adv - j)*PITCH
        k = int(round((s/PITCH))) % 34                     # final worker2 index (identity of the slot it ends in)
        out.append((j, link_M(s)))
    return out

# ---------------------------------------------------------------- table
YR = [-1.0 + 20*k for k in range(-11, 3)]         # stud rows y -221 .. 39
ZC = [10.0 + 20*k for k in range(-12, 8)]         # stud columns z -230 .. 150
def _cells_gap(points, r=1):
    """cells touching corner points (2x2 around each)."""
    g = set()
    for y, z in points:
        for dy in (-10, 10):
            for dz in (-10, 10): g.add((y + dy, z + dz))
    return g
STUB_PTS = [DRV] + IDL
ROD_PTS = [(-71.0, 20.0), (-11.0, 60.0), (-11.0, -60.0)]

def _row_runs(cells, axis):
    """split a set of cells into straight runs along robot y (axis 0) or z (axis 1)."""
    runs = []
    keyf = (lambda c: (c[1], c[0])) if axis == 0 else (lambda c: (c[0], c[1]))
    cs = sorted(cells, key=keyf); cur = []
    for c in cs:
        if cur and keyf(c)[0] == keyf(cur[-1])[0] and abs(keyf(c)[1] - keyf(cur[-1])[1] - 20) < 1e-6: cur.append(c)
        else:
            if cur: runs.append(cur)
            cur = [c]
    if cur: runs.append(cur)
    return runs

PLATES = {8: '3460', 6: '3666', 4: '3710', 3: '3623', 2: '3023', 1: '3024'}
TILES = {4: '2431', 3: '63864', 2: '3069b', 1: '3070b'}
def _lay(a, name, cells, x_top, axis, parts, col, stagger=True):
    k = 0
    for r, run in enumerate(_row_runs(cells, axis)):
        i = 0
        while i < len(run):
            cap = 4 if (stagger and i == 0 and r % 2) else 99          # brick bond: odd rows start with a short piece
            n = max(L for L in parts if L <= min(len(run) - i, cap))
            seg = run[i:i + n]; y = np.mean([c[0] for c in seg]); z = np.mean([c[1] for c in seg])
            Mo = PO if axis == 0 else PZ
            a(f'{name}{k}', parts[n], col, T(x_top, y, z) @ Mo); k += 1; i += n
    return k

IDM = IDL[1]                                      # driven sprocket: the idler at z -80 (the slot feeds straight onto it)
SHAFT = (-11.0, -120.0)                           # x-shaft: 24T meshing the 8T on the driven idler stub (40 apart)
X_GEAR = -76.0                                    # gear layer under the table (underside x -86)
M2O = (-36.0, -120.0)                             # chain motor output (x, z), axis along y, front face y -51 (2 plates below the table)
M1O = (-52.0, 80.0)                               # transfer motor output (x, z), axis along y, front face y -111
MOT_Y = col3((0, 0, -1), (1, 0, 0), (0, -1, 0))   # Boost motor: top (studs) -> -x (table), output along +y

def table(a, adv=0.0):
    """a(name, part, colour, M): the table and everything fixed to it (robot frame, tilted pose). Its two Boost motors
    are powered from the tilt hub (ports C, D): no hub rides on the table. adv: chain advance in pitches (turns the drive train:
    sprockets 30 deg per link, 2 teeth; chain runs -z along the bottom run, so the driven idler turns negatively about +x)."""
    th_idl = -30.0*adv; th_sh = -th_idl*8/24          # 8T on the idler stub -> 24T on the x-shaft (opposite sense, 1/3);
                                                       # the 90-deg double-bevel pair turns the motor shaft (+y) at the shaft's rate
    allc = {(y, z) for y in YR for z in ZC}
    tech = [('3738', (-71.0, 0.0))]                                  # holes z -60 .. 60: drive stub (0), drive rod (20)
    for z in (80.0, -80.0): tech.append(('32001', (-11.0, z)))       # holes z +-(40 .. 120): idler stubs, rods (+-60), x-shaft (-120)
    tcells = set()
    for part, (y0, z0) in tech:
        n = {'3738': 8, '32001': 6}[part]
        for dy in (-10, 10):
            for k in range(n): tcells.add((y0 + dy, z0 - 10*(n - 1) + 20*k))
    holes = STUB_PTS + ROD_PTS
    tg = _cells_gap(holes); sg = _cells_gap([SHAFT])
    _lay(a, 'tile', allc - tg - set(GUIDE_CELLS) - set(MOUTH_CELLS), X_TOP, 1, TILES, LBG)   # smooth top (posts stand in the gaps)
    _lay(a, 'L1_p', allc - tg, X_TOP + 8, 0, PLATES, DBG)            # plates along y, 2x2 gaps where axles/rods pass
    for part, (y0, z0) in tech: a(f'L2_t{y0:.0f}_{z0:.0f}', part, YEL, T(X_TOP + 16, y0, z0) @ PZ)
    _lay(a, 'L2_p', allc - tcells, X_TOP + 16, 1, PLATES, LBG)       # Technic plates (the bearings) + plates along z
    # stubs (axles in the Technic-plate holes; sprockets ride on them)
    a('stubD', '32073', BLK, T(-110, *DRV))                         # 5L: -160 .. -60
    a('stubDb', '32123a', YEL, T(-81, *DRV) @ R('y', 90))
    for i, (y, z) in enumerate(IDL):
        drv = (y, z) == IDM
        a(f'stubI{i}', '32073', BLK, T(-110, y, z) @ R('x', th_idl if drv else 0.0))
        if drv: a('g8', '3647', LBG, T(X_GEAR, y, z) @ R('x', th_idl) @ R('y', 90))
        else: a(f'stubIb{i}', '32123a', YEL, T(-81, y, z) @ R('y', 90))
    # chain drive: Boost motor -> bevel pair -> x-shaft -> 24T -> 8T on the driven idler stub
    a('shaft', '32073', BLK, T(-44, *SHAFT) @ R('x', th_sh))                        # 5L: -94 .. 6 (bearing in the Technic plate)
    a('g24', '3648b', LBG, T(X_GEAR, *SHAFT) @ R('x', th_sh) @ R('y', 90))
    a('bevS', '32269', LBG, T(M2O[0] + 30, *SHAFT) @ R('x', th_sh) @ R('y', 90))    # crossing at x -52; double bevels 30 from it
    for k in range(2): a(f'm2sh{k}', '3020', DBG, T(-86 + 8*k, -131, M2O[1]) @ PO)   # 2x4 shims under the motor's studs
    a('m2', '26913', WHT, T(-70, -111, M2O[1]) @ MOT_Y)              # front y -51
    a('m2ax', '3704', BLK, T(M2O[0], -51, M2O[1]) @ R('y', th_sh) @ R('z', 90))     # 2L: y -71 .. -31
    a('bevM', '32269', LBG, T(M2O[0], SHAFT[0] - 30, M2O[1]) @ R('y', th_sh) @ R('x', 90))
    # transfer motor (crank in the plane y -111 .. -81)
    a('m1', '26913', WHT, T(-86, -171, M1O[1]) @ MOT_Y)              # front y -111
    guides(a)
    # hinge: axle-hole bricks under the table, axle along y
    for y in (-201.0, -41.0, 19.0):
        for k in range(2): a(f'hs{y:.0f}_{k}', '3023', DBG, T(-86 + 8*k, y, Z_HINGE) @ PZ)
        a(f'hb{y:.0f}', '32064b', DBG, T(-70, y, Z_HINGE) @ PZ)

# ---------------------------------------------------------------- transfer: rods + frame + crank
XFER = 38.0                                       # rod travel = drive-set travel: 38 out, onto the S4 guides (their pads reach it after 18)
SET_OUT = XFER
ROD_TIP = -126.6                                  # rod tips touch the sprocket faces (they carry the sprockets when the table lies flat)
RODS = {'A': (-71.0, 20.0), 'B': (-11.0, 60.0), 'C': (-11.0, -60.0)}
RCR, LCR = 40.0, 40.0                             # crank radius, conrod length (thin 3L, axle holes at both ends)
Z_SL = 40.0                                       # slider pin: 6536 on rod A, pin hole along y at z 40
X_SL0 = -16.6                                     # slider x with the rods back
def _crank_P(ph):
    """crank pin Q (x, z) and slider x for crank angle ph (from +x towards -z)."""
    Q = np.array(M1O) + RCR*np.array([np.cos(ph), -np.sin(ph)]); dz = Q[1] - Z_SL
    return Q, Q[0] - np.sqrt(max(LCR**2 - dz*dz, 0))
def crank_phi(u):
    """crank angle for slider travel u (0 .. XFER), near the toggle with the rods back."""
    target = X_SL0 - u; lo, hi = 0.0, np.radians(80)
    for _ in range(60):
        mid = (lo + hi)/2
        if _crank_P(mid)[1] > target: lo = mid
        else: hi = mid
    return (lo + hi)/2
def _frame(dz_local, ylocal):
    """liftarm frame: local z along dz_local, local y along ylocal."""
    z = np.asarray(dz_local, float); y = np.asarray(ylocal, float); return col3(np.cross(y, z), y, z)
BAR = col3((0, -1, 0), (1, 0, 0), (0, 0, 1))      # thin liftarm, holes along x, length along z
ARM_Y = col3((0, 0, 1), (1, 0, 0), (0, 1, 0))     # thin liftarm, holes along x, length along y
ALONG_Z = R('y', 90)                              # axles / bushes / gears with local axis -> robot x ... (local z -> x)
def transfer(a, u, body='xfer'):
    """push rods at travel u (0 = back). Rod A carries the slider (6536) and the arm to the bar that carries rods B, C."""
    b = lambda n, p, c, M: a(n, p, c, M, body)
    for k, (y, z) in RODS.items(): b(f'rod{k}', '44294', LBG, T(ROD_TIP + 70 - u, y, z))
    b('slider', '6536', DBG, T(X_SL0 - u, -71, 20) @ R('x', 90))         # axle hole on rod A, pin hole (z 40) along y
    b('arm', '32449', RED, T(-1.6 - u, -41, 20) @ ARM_Y)                # rod A (axle hole) .. joint (axle hole)
    b('bar', '32065', RED, T(-11.6 - u, -11, 0) @ BAR)                   # rods B, C, joint
    b('armpin', '3749', BLK, T(-6.6 - u, -11, 20))                      # pin half in the bar, axle half in the arm
    for k, xs in (('A', (8.4,)), ('B', (-21.6, -1.6)), ('C', (-21.6, -1.6))):
        y, z = RODS[k]
        for x in xs: b(f'rb{k}{x}', '32123a', YEL, T(x - u, y, z) @ ALONG_Z)

def crank(a, u, body='crank'):
    """double thin crank (32449 x2: axle hole on the motor axle, round hole 40 out) in y -111 .. -91, axle-pin crank pin,
    thin 3L conrod (y -91 .. -81) to an axle-pin in the slider's pin hole."""
    ph = crank_phi(u); Q, px = _crank_P(ph); C = np.array(M1O)
    dQ = np.array([np.cos(ph), 0.0, -np.sin(ph)])
    a('cr_ax', '3704', BLK, T(C[0], -111, C[1]) @ R('z', 90), body)          # 2L: y -131 .. -91 (20 in the motor output)
    for y in (-106, -96): a(f'cr_arm{y}', '32449', ORG, T(C[0], y, C[1]) @ _frame(dQ, (0, 1, 0)) @ T(0, 0, 30), body)
    a('cr_pin', '3749', BLK, T(Q[0], -91, Q[1]) @ R('z', 90), body)          # pin half in the cranks, axle half in the conrod
    dP = np.array([px - Q[0], 0.0, Z_SL - Q[1]])/LCR
    a('conrod', '6632', ORG, T(Q[0], -86, Q[1]) @ _frame(dP, (0, 1, 0)), 'conrod')
    a('sl_pin', '3749', BLK, T(px, -81, Z_SL) @ R('z', -90), 'conrod')      # pin half in the slider's pin hole, axle half in the conrod
    return ph

def drive_set(a, n=34, adv=33.0, sprockets=True):
    if sprockets:
        Q = []; worker2.build(M0=np.eye(4), tag='x', body='x', P=Q, modules={'trackL'})
        for q in Q:
            if '_spr_' in q['name']: a(q['name'].split('tkL')[1], q['part'], q['col'], q['M'])
    for j, M in chain_links(n, adv): a(f'link{j:02d}', '3873', BLK, M)

def hinge_M(theta):
    """table pose: theta 0 = tilted (stubs towards S4), 90 = horizontal (stubs up). Rotation about the hinge axle (robot y)."""
    return T(X_HINGE, 0, Z_HINGE) @ R('y', -theta) @ T(-X_HINGE, 0, -Z_HINGE)

# ---------------------------------------------------------------- chain guides (the open chain is pushed round the loop)
SLOT_ZONE = (-100.0, -40.0)                       # bottom-run z range kept open for the slot and the magazine mouth
def _outline():
    ss = np.linspace(0, PER, 1200, endpoint=False)
    pts = np.array([AT(s)[0] for s in ss]); return pts
def guide_cells(gap=(0.6, 6.0)):
    """grid cells (stud centres) for round posts just outside the chain's outer faces (pitch line + 8)."""
    pts = _outline(); from matplotlib.path import Path as _P
    inside = _P(pts)
    out = []
    for y in YR:
        for z in ZC:
            if inside.contains_point((y, z)): continue
            dy_ = np.maximum(np.abs(pts[:, 0] - y) - 10, 0); dz_ = np.maximum(np.abs(pts[:, 1] - z) - 10, 0)
            d = np.min(np.hypot(dy_, dz_)) - 8.0                     # square 1x1 post to the links' outer faces
            if not (gap[0] <= d <= gap[1]): continue
            if y > 19 and SLOT_ZONE[0] - 10 <= z <= SLOT_ZONE[1] + 10: continue
            out.append((y, z))
    return out
GUIDE_CELLS = guide_cells()
MOUTH_CELLS = [(39.0, -110.0), (39.0, -90.0), (39.0, -50.0)]   # posts either side of the slot mouth (one on a jumper: z -100)
def guides(a):
    """two-high 1x1 posts on the bottom plate layer (their tile cells are left out); the mouth posts guide the front link
    of the queue into the slot (inner faces z -90 and -60; the link spans z -88.7 .. -62.7)."""
    a('mj', '3794b', DBG, T(-110, 39, -100) @ PZ)
    for k in range(2):
        a(f'mp-100_{k}', '3005', DBG, T(-134 - 24*k, 39, -100) @ PO)
        a(f'mp-50_{k}', '3005', DBG, T(-126 - 24*k, 39, -50) @ PO)
    for (y, z) in guide_cells():
        for k in range(2): a(f'gp{y:.0f}_{z:.0f}_{k}', '3005', DBG, T(-126 - 24*k, y, z) @ PO)
