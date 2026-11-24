"""Chain station at the supply train's refill stop (beyond the build lane's end): builds the drive set from single parts.

Canonical frame = the S4/S5 supply wagon's cart frame at the refill stop (x along the lane, z across: SA lane at z 260).
  table      : chainst geometry (robot frame) placed by core_M0(0) @ hinge_M(theta); theta 90 = flat (building), 0 = upright
  hinge      : 16L axle along x at (y -110, z 60) through the table's axle-hole bricks; bearing column at x -278, joiner +
               2L axle into the tilt Move Hub (internal motor A tilts; ports C, D feed the table's two Boost motors)
  links      : arrive single file on a tread belt (x -298 .. -848, top y -164) between two side rails; the queue's pressure
               pushes the front link off the belt end, across the table edge, between two mouth posts into the slot
  sprockets  : brought by supply wagon S5 (behind S4) and pushed onto the upright table's stubs by a crank pusher here
Cycle (u: seconds from the build wagon leaving S4, see tline.REF): S4 parks (3.5) -> table up -> rods slide the finished set
38 onto S4's guides -> table down -> train moves on one wagon -> table up -> S5 pushes 3 sprockets onto the stubs -> table
down -> 34 links + 2 advances (u 19 .. 43.5, wrapping into the next cycle).
"""
import numpy as np
import asm2, chainst as cs, lcheck, tline, tunit, rails, worker2
from asm2 import T, R, col3, add, P, LBG, DBG, BLK, WHT, RED, YEL, ORG, BLU

def table_G(theta):
    return asm2.core_M0(0.0) @ cs.hinge_M(theta)

# ---------------------------------------------------------------- timeline
REF = tline.REF
TL = 0.68                                          # seconds per link (insertion 0.3 + advance)
T_INS = 0.3
U_B = REF['home'][1]                               # chain building starts (table flat again, belt brought forward)
N_ADV = 36                                         # 34 links + 2: the sprockets turn 3 whole turns, axles line up with the next set
Q_BACK = (2.3, 3.3)                                # belt backs the queue off the table before it tilts
Q_FWD = (REF['down2'][1], REF['down2'][1] + 0.5)
def _ez(x): return 0.5 - 0.5*np.cos(np.pi*np.clip(x, 0, 1))
def _seg(u, k): a, b = REF[k]; return _ez((u - a)/(b - a))

def state(u):
    """station state at refill time u (0 .. 41)."""
    st = dict(u=u)
    th = 90.0
    if REF['up1'][0] <= u < REF['down1'][1]:
        th = 90*(1 - _seg(u, 'up1')) if u < REF['up1'][1] else (90*_seg(u, 'down1') if u >= REF['down1'][0] else 0.0)
    elif REF['up2'][0] <= u < REF['down2'][1]:
        th = 90*(1 - _seg(u, 'up2')) if u < REF['up2'][1] else (90*_seg(u, 'down2') if u >= REF['down2'][0] else 0.0)
    st['theta'] = th
    st['rod'] = cs.XFER*_seg(u, 'push') if u < REF['ret'][0] else cs.XFER*(1 - _seg(u, 'ret'))
    # drive set on the table: the finished one until it has gone onto S4, the new sprockets once S5 has pushed them on
    old = u < REF['ret'][0]
    st['dset_u'] = st['rod'] if old else 0.0
    st['spr'] = old or u >= REF['s5push'][1]
    st['spr_late'] = REF['s5push'][0] <= u < REF['s5push'][1]          # S5 still holds them (tline draws them)
    # chain building
    v = (u - U_B) % 41.0
    building = v < N_ADV*TL
    if not building:                               # complete (from u 43.5 - 41 = 2.5 until the transfer), or none
        st.update(nl=34 if old else 0, adv=float(N_ADV), ins=None, nins=34 if old else 0)
    else:
        j = int(v // TL); f = v - j*TL
        if j < 34 and f < T_INS: st.update(nl=j, adv=float(j), ins=(j, _ez(f/T_INS)), nins=j + _ez(f/T_INS))
        else:
            k = min(j + 1, 34); st.update(nl=k, adv=j + _ez((f - (T_INS if j < 34 else 0))/(TL - (T_INS if j < 34 else 0))),
                                           ins=None, nins=k)
    # queue on the belt: total links fed (continuous counter over cycles) and the back-off before the table tilts
    st['qback'] = 28*(_ez((u - Q_BACK[0])/(Q_BACK[1] - Q_BACK[0])) if u < Q_FWD[0] else 1 - _ez((u - Q_FWD[0])/(Q_FWD[1] - Q_FWD[0])))
    if Q_BACK[1] <= u < Q_FWD[0]: st['qback'] = 28.0
    if u < Q_BACK[0] or u >= Q_FWD[1]: st['qback'] = 0.0
    st['qfed'] = st['nins'] if building else (34 if old or u >= U_B else 0)
    st['qpos'] = 14*st['qfed'] - st['qback']                       # belt travel this cycle (link spacing 14)
    return st

# ---------------------------------------------------------------- fixed parts (canonical)
HUB_C = (102.0, -176.0, -80.0)                     # tilt hub origin (identity rotation): output face x 42, axis x, at (y -110, z 60)
def fixed(a):
    for k in range(4): a(f'col{k}', '3003', DBG if k % 2 else LBG, T(-298, -24*(k + 1), 70))
    a('colT', '6541', DBG, T(-288, -120, 60) @ R('y', 90))                           # hole along x at height 110
    a('thub', '26910', WHT, T(*HUB_C))
    for zc in (-210.0, 50.0):
        for k in range(3): a(f'hst{zc:.0f}_{k}', '3003', DBG if k % 2 else LBG, T(102, -24*(k + 1), zc))
        for xc in (92.0, 112.0): a(f'hj{xc:.0f}_{zc:.0f}', '3794b', DBG, T(xc, -80, zc) @ R('y', 90))

def hinge_parts(a, theta):
    rot = R('x', -theta)
    a('hax', '50451', BLK, T(-138, -110, 60) @ rot, 'table')                       # 16L: x -298 .. 22
    a('hjoin', '6538b', DBG, T(22, -110, 60) @ rot @ R('y', 90), 'table')
    a('hax2', '3704', BLK, T(42, -110, 60) @ rot, 'table')                         # 2L: 22 .. 62

# ---------------------------------------------------------------- link belt (canonical)
ZQ = -155.0                                        # queue centre line (slot link: z -168.7 .. -142.7)
BX0, BX1 = -308.0, -858.0                          # belt sprocket axles (front: 1x1 Technic bricks, rear: 1x2)
BY = -126.0                                        # axle height (belt top face -164: 4 above the flat table)
BZ = -155.7
B_NL = 80
def _belt_path():
    L = BX0 - BX1; per = 2*L + 2*np.pi*30
    def at(s):
        s = s % per
        if s < L: return np.array([BX1 + s, BY - 30]), np.array([1.0, 0.0])                     # top run, moving +x
        s -= L
        if s < np.pi*30: a_ = s/30; return np.array([BX0 + 30*np.sin(a_), BY - 30*np.cos(a_)]), np.array([np.cos(a_), np.sin(a_)])
        s -= np.pi*30
        if s < L: return np.array([BX0 - s, BY + 30]), np.array([-1.0, 0.0])
        s -= L; a_ = s/30; return np.array([BX1 - 30*np.sin(a_), BY + 30*np.cos(a_)]), np.array([-np.cos(a_), -np.sin(a_)])
    return at, per
def belt_links(phase):
    at, per = _belt_path(); pitch = per/B_NL; out = []
    for i in range(B_NL):
        (x, y), t = at(i*pitch + phase)
        Z = np.array([t[0], t[1], 0.0]); Yv = np.array([-t[1], t[0], 0.0])    # local -y outward (away from the loop centre)
        X = np.cross(Yv, Z); M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2] = X, Yv, Z; M[:3, 3] = (x, y, BZ)
        out.append(M)
    return out

def belt_frame(a):
    for x in (BX0,):                                                                   # front: 1x1 Technic bricks (hole over the stud)
        for z in (-200.0, -100.0):
            for k in range(4): a(f'bcf{z:.0f}_{k}', '3005', DBG if k % 2 else LBG, T(x, -24*(k + 1), z))
            a(f'bcfp{z:.0f}', '3024', DBG, T(x, -104, z)); a(f'bcfp2{z:.0f}', '3024', DBG, T(x, -112, z))
            a(f'bcfT{z:.0f}', '6541', DBG, T(x, -136, z))                               # hole (along z) at -126
    for z in (-200.0, -100.0):                                                         # rear: 1x2 Technic bricks (hole between studs)
        for k in range(4): a(f'bcr{z:.0f}_{k}', '3004', DBG if k % 2 else LBG, T(BX1, -24*(k + 1), z))
        a(f'bcrp{z:.0f}', '3023', DBG, T(BX1, -104, z)); a(f'bcrp2{z:.0f}', '3023', DBG, T(BX1, -112, z))
        a(f'bcrT{z:.0f}', '3700', DBG, T(BX1, -136, z))

BHUB = T(-718, -192, -10) @ col3((0, 0, 1), (0, 1, 0), (-1, 0, 0))   # belt hub: output A (local x -60) at (x -848, y -126, z -70)
def belt_hub(a):
    a('bhub', '26910', WHT, BHUB)
    for xc in (-838.0, -598.0):
        for k in range(4): a(f'bhs{xc:.0f}_{k}', '3003', DBG if k % 2 else LBG, T(xc, -24*(k + 1), -10))

# ---------------------------------------------------------------- side rails for the queue (canonical)
RAIL_COLS = (-378.0, -518.0, -658.0, -798.0)        # 1x4 Technic-brick column tops (holes at y -190, x +-20, 0)
def rails_q(a):
    for x in RAIL_COLS:
        for z in (-200.0, -110.0):                   # right column: on four jumpers (half-stud shift along z)
            k0 = 0
            if z > -150:
                for dx in (-30, -10, 10, 30): a(f'rq{x:.0f}_{z:.0f}_j{dx}', '3794b', DBG, T(x + dx, -8, z) @ R('y', 90))
            else: a(f'rq{x:.0f}_{z:.0f}_p', '3710', DBG, T(x, -8, z))
            for k in range(7): a(f'rq{x:.0f}_{z:.0f}_{k}', '3010', DBG if k % 2 else LBG, T(x, -8 - 24*(k + 1), z))
            a(f'rq{x:.0f}_{z:.0f}_t', '3701', LBG, T(x, -200, z))
            for dx in (-20.0, 20.0):
                a(f'rq{x:.0f}_{z:.0f}_pin{dx:.0f}', '2780', BLK, T(x + dx, -190, z + (10 if z < -150 else -10)) @ R('y', 90))
    BX = col3((0, 1, 0), (0, 0, 1), (1, 0, 0))      # beam: length (local z) along x, holes (local y) along z, local x along y
    for k, x0 in enumerate((-298.0, -598.0)):        # thick 15L beams, front hole x0: inner faces z -170 and -140
        a(f'rqL{k}', '32278', RED, T(x0 - 140, -190, -180) @ BX)
        a(f'rqR{k}', '32278', RED, T(x0 - 140, -190, -130) @ BX)

# ---------------------------------------------------------------- queue (canonical, the table's flat-pose frame)
N_Q = 34
def queue(a, st):
    G = table_G(90.0); g = st['ins'][1] if st['ins'] else 0.0
    base = cs.link_M(cs.S_SLOT)
    for i in range(N_Q):
        off = 14*(i + 1) + st['qback'] - 14*g                     # i = 0: the front link (the one going into the slot)
        yr = 19 + off
        lift = 0.0 if yr - 5 < 49 else -4.0                      # on the belt: 4 above the table top
        a(f'q{i:02d}x', '3873', BLK, G @ T(lift, off, 0) @ base, 'queue')

B_RUN = 30*(2*(BX0 - BX1) + 2*np.pi*30)/B_NL      # belt travel per cycle: 30 pitches (it slips under the blocked queue)
def belt_phase(st):
    v = (st['u'] - U_B) % 41.0
    return B_RUN*min(v/(N_ADV*TL), 1.0) - st['qback']

# ---------------------------------------------------------------- whole station (side A canonical; tline maps/mirrors)
def build_canon(st, tst):
    """st: station state; tst: tline state (S5 push angle). Adds to asm2.P in the canonical frame; returns the parts added."""
    n0 = len(P)
    G = table_G(st['theta'])
    at = lambda n, p, c, M, b='ctable': add('cs_' + n, p, c, G @ M, b)
    cs.table(at, st['adv'] if st['spr'] or st['nl'] else 0.0)
    cs.transfer(lambda n, p, c, M, b='xfer': at(n, p, c, M, 'cxfer'), st['rod'])
    cs.crank(lambda n, p, c, M, b='crank': at(n, p, c, M, 'c' + b), st['rod'])
    D = T(-st['dset_u'], 0, 0)
    if st['spr']:
        rot = -(st['adv'] - N_ADV)*30.0                        # chain runs -z on the bottom run: negative about +x
        Q = []; worker2.build(M0=np.eye(4), tag='x', body='x', P=Q, modules={'trackL'})
        for q in Q:
            if '_spr_' in q['name']:
                n = q['name'].split('tkL')[1]; M = q['M']
                Rz = T(*M[:3, 3]) @ R('x', rot) @ T(*(-M[:3, 3]))   # sprockets turn about their axes (robot x)
                at(n, q['part'], q['col'], D @ Rz @ M, 'cdset' + n[-3:])
    if st['nl']:
        for j, M in cs.chain_links(st['nl'], st['adv']): at(f'link{j:02d}', '3873', BLK, D @ M, 'cchain')
    fixed(lambda n, p, c, M, b='frame': add('cs_' + n, p, c, M, b))
    hinge_parts(lambda n, p, c, M, b='ctable': add('cs_' + n, p, c, M, 'ctable'), st['theta'])
    belt_frame(lambda n, p, c, M, b='frame': add('cs_' + n, p, c, M, b))
    for k, M in enumerate(belt_links(belt_phase(st))): add(f'cs_belt{k:02d}', '3873', BLK, M, 'cbelt')
    ph = belt_phase(st); rot = np.degrees(ph/30.0)             # top run moves +x: sprockets turn positively about +z
    add('cs_bax0', '3706', BLK, T(BX0, BY, -150) @ R('y', 90) @ R('x', -rot), 'cbelt0')           # 6L: z -210 .. -90
    add('cs_bax1', '3707', BLK, T(BX1, BY, -130) @ R('y', 90) @ R('x', -rot), 'cbelt1')           # 8L: z -210 .. -50 (into the belt hub)
    for k, x in enumerate((BX0, BX1)): add(f'cs_bspr{k}', '3648b', LBG, T(x, BY, BZ) @ R('z', rot), f'cbelt{k}')
    belt_hub(lambda n, p, c, M, b='frame': add('cs_' + n, p, c, M, b))
    rails_q(lambda n, p, c, M, b='frame': add('cs_' + n, p, c, M, b))
    queue(lambda n, p, c, M, b='queue': add('cs_' + n, p, c, M, 'cqueue'), st)
    lcheck.RESTING.update({'ctable': 'turns on the hinge axle', 'cxfer': 'slides in the table', 'ccrank': 'on the motor',
                           'cconrod': 'crank linkage', 'cdsetdrv': 'on its stub', 'cdsetidf': 'on its stub', 'cdsetidb': 'on its stub',
                           'cchain': 'round the sprockets', 'cbelt': 'tread belt on its sprockets', 'cqueue': 'stands on the belt',
                           'cbelt0': 'turns in the belt frame', 'cbelt1': 'driven by the belt hub'})
    return P[n0:]

if __name__ == '__main__':
    for u in (0, 3, 5, 6, 7.5, 9, 12, 14, 15.5, 17, 18.7, 19.2, 25, 40.5):
        s = state(u); print(u, {k: (round(v, 2) if isinstance(v, float) else v) for k, v in s.items()})
