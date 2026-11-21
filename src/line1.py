"""Line 1: track station (asm2, x=0) -> stand-up station (x=PXE). Main flow runs in -x; the robot lies on its back,
head upstream, on a pallet whose deck is a hinged flap. At the stand-up station two geared arms (Move Hub, 8T:40T)
lift the flap's side lugs; flap and robot swing 90 deg about the hinge, the tracks land on two rails and the robot drives off.
"""
import numpy as np
from scipy.optimize import brentq
import asm2, worker2, lcheck
from asm2 import T, R, col3, add, P, YP, Y0, LBG, DBG, BLK, WHT, RED, YEL, BLU, ORG

PXE = -1640.0            # pallet stop at the stand-up station
X2 = -1280               # lane 2 centre
HY, QY, RL = -178, -158, 240   # hinge height, arm pivot height, lug radius
RAIL = -120
lcheck.RESTING.update({'flap': 'rests on the pallet piers / on the lift arms'})

def flapM(px, a):
    return T(px - 180, HY, 0) @ R('z', -np.degrees(a)) @ T(-(px - 180), -HY, 0)

def arm_angle(a):
    if a < 1e-6: return 0.0
    f = lambda b: -RL*np.cos(a)*np.sin(b) + (QY - HY + RL*np.sin(a))*np.cos(b) - 15
    return brentq(f, -0.2, np.pi/2 + 0.2)

def fpallet(px, a):
    b = lambda n, p, c, M: add('pal_' + n, p, c, M, 'pallet')
    F = flapM(px, a)
    f = lambda n, p, c, M: add('pal_f' + n, p, c, F @ M, 'flap')
    b('base', '3027', LBG, T(px - 40, YP - 8, 0))
    for z in (-30, 30):
        b(f'hj{z}', '3794b', DBG, T(px - 180, -100, z))                     # jumper: half-stud shift so the 1x1 column sits on the hinge line
        for k in range(2): b(f'hp{z}_{k}', '3024', DBG, T(px - 180, -108 - 8*k, z))
        for k in range(2): b(f'hcol{z}_{k}', '3005', DBG, T(px - 180, -140 - 24*k, z))
        b(f'hbr{z}', '6541', BLK, T(px - 180, -188, z))
        b(f'hbush{z}', '32123a', YEL, T(px - 180, HY, z/2))
    b('haxle', '3706', BLK, T(px - 180, HY, 0) @ R('y', 90))
    for z in (-50, 50):
        for dx in (-80, 80):
            for k in range(2): b(f'pier{z}_{dx}_{k}', '3010', DBG, T(px + dx, -116 - 24*k, z))
            for k in range(2): b(f'pierp{z}_{dx}_{k}', '3710', DBG, T(px + dx, -148 - 8*k, z))
            b(f'piert{z}_{dx}', '2431', LBG, T(px + dx, -164, z))
    # ---- flap ----
    for z in (-50, 50):
        f(f'beam{z}', '3703', BLK, T(px - 40, -188, z))
        for dx in (-120, 40): f(f'sp{z}_{dx}', '3460', DBG, T(px + dx, -196, z))
        s = 1 if z > 0 else -1
        f(f'lug{z}', '3707', LBG, T(px + 60, HY, 110*s) @ R('y', 90))
        for zz in (35, 65): f(f'lugb{z}_{zz}', '32123a', YEL, T(px + 60, HY, zz*s))
    for dx in (-120, 0, 120): f(f'deck{dx}', '3036', LBG, T(px + dx, -204, 0) @ R('y', 90))
    f('top', '3027', LBG, T(px, -212, 0))
    for dx in (-120, -40, 40, 120):
        for dz in (-40, 0, 40): f(f'tile{dx}_{dz}', '87079', DBG, T(px + dx, -220, dz))
    for sz in (-70, 70):
        for k in range(3): f(f'lip{sz}_{k}', '3460', BLU, T(px, -212 - 8*k, sz))
    for sx in (-170, 170):
        f(f'end{sx}', '3010', BLU, T(px + sx, -228, 0) @ R('y', 90))
        f(f'endp{sx}', '3710', BLU, T(px + sx, -236, 0) @ R('y', 90))

def standup_station(b_arm):
    qx = PXE - 180
    for x in (-1600, -960, -2240):
        for z in (330, -310): add(f'base{x}_{z}', '3811', LBG, T(x, 0, z))
    # rails: 2-wide walls at |z| = 130, tile top at RAIL, from x=-1760 to -2560
    for s in (1, -1):
        z = 130*s
        for k in range(4):
            if k % 2 == 0: xs = [(-1920 - 160*i, '3007') for i in range(4)]                      # -1840 .. -2480
            else: xs = [(-1880, '3001')] + [(-2000 - 160*i, '3007') for i in range(3)] + [(-2440, '3001')]
            for xc, part in xs: add(f'rail{s}_{k}_{xc}', part, DBG if k % 2 else LBG, T(xc, -24 - 24*k, z))
        for k in range(2):
            if k == 0: xs = [(-1920 - 160*i, '3034') for i in range(4)]
            else: xs = [(-1880, '3020')] + [(-2000 - 160*i, '3034') for i in range(3)] + [(-2440, '3020')]
            for xc, part in xs: add(f'railp{s}_{k}_{xc}', part, LBG, T(xc, -104 - 8*k, z))
        for xc in range(-1880, -2441, -80): add(f'railt{s}_{xc}', '87079', BLK, T(xc, RAIL, z))
        # ---- lift arm: beam 15 on the pivot axle, 40T gear pegged to it, 8T on the hub output ----
        d = np.array([np.cos(b_arm), -np.sin(b_arm), 0.0]); A = col3((np.sin(b_arm), np.cos(b_arm), 0), (0, 0, 1), d)
        body = f'arm{"A" if s > 0 else "B"}'
        Q = np.array([qx, QY, 0.0])
        add(f'{body}_beam', '32278', YEL, T(*(Q + 140*d + [0, 0, 180*s])) @ A, body)
        add(f'{body}_g40', '3649', LBG, T(qx, QY, 200*s) @ R('z', -np.degrees(b_arm)), body)
        add(f'{body}_axle', '3705', BLK, T(qx, QY, 205*s) @ R('y', 90), body)
        add(f'{body}_peg', '3704', BLK, T(*(Q + 20*d + [0, 0, 190*s])) @ R('y', 90), body)
        add(f'{body}_bush', '32123a', YEL, T(qx, QY, 235*s), body)
        for k in range(6): add(f'{body}_post{k}', '3010', DBG if k % 2 else LBG, T(qx, -24 - 24*k, 220*s))
        add(f'{body}_postT', '3701', BLK, T(qx, -168, 220*s))
        hx = qx - 200; hz = 290*s; pb = f'pin{"A" if s > 0 else "B"}'
        add(f'{body}_hub', '26910', WHT, T(hx, QY - 66, hz) @ R('y', 90))
        for zz in (hz - 50, hz + 50):
            for k in range(5): add(f'{body}_ped{k}_{zz}', '2465', LBG if k % 2 else DBG, T(hx, -24 - 24*k, zz))
            for dx in (-80, 80): add(f'{body}_pedp{dx}_{zz}', '3460', DBG, T(hx + dx, -128, zz))
        add(f'{pb}_axle', '4519', BLK, T(qx - 60, QY, 216*s) @ R('y', 90), pb)
        add(f'{pb}_g8', '3647', DBG, T(qx - 60, QY, 200*s) @ R('z', np.degrees(b_arm)*5 + 7.5), pb)

STATE = dict(asm2.STATE, a=0.0, drive=0.0, free=False, belt_n=0.0, s1=True, s2=True)

def build(st=None):
    st = dict(STATE, **(st or {})); P.clear()
    px = st['px']
    if st['s1']:
        for x, z in ((-320, 330), (320, 330), (-320, -310), (320, -310), (-320, 970), (-320, -950)): add(f'base{x}_{z}', '3811', LBG, T(x, 0, z))
        asm2.lane('M', 0, st['belt_m']); asm2.lane('A', 260, st['belt_s']); asm2.lane('B', -260, st['belt_s'])
    if st['s2']:
        asm2.lane('N', 0, st['belt_n'], X2)
        standup_station(arm_angle(st['a']))
    fpallet(px, st['a'])
    M0 = asm2.core_M0(px)
    Mr = (T(-st['drive'], 0, 0) @ flapM(px, np.pi/2) if st['free'] else flapM(px, st['a'])) @ M0
    worker2.build(M0=Mr, tag='rob', body='robot', P=P, modules={'core'})
    for side, mod, s in (('A', 'trackL', 1), ('B', 'trackR', -1)):
        if st['seated']: worker2.build(M0=Mr, tag='rob', body='robot', P=P, modules={mod}, track_phase=st['drive'])
        if not st['s1']: continue
        asm2.carrier(side, st['sx'], st['e'])
        if not st['seated']: worker2.build(M0=T(st['sx'] + 40 - px, 0, s*st['d']) @ M0, tag='rob', body=f'unit{side}', P=P, modules={mod})
        asm2.pusher(side, st['theta'])
    return P

PHASES = [('arrive', 2.2), ('push', 1.3), ('retract', 1.0), ('sleds', 1.6), ('travel', 4.5), ('lift', 2.5), ('drive', 3.0), ('lower', 1.5)]
RUN = 420.0; DRV = 430.0
def cycle(t):
    tot = sum(d for _, d in PHASES); u = (t % 1.0)*tot
    for name, dur in PHASES:
        if u <= dur: f = u/dur; break
        u -= dur
    ease = lambda x: 0.5 - 0.5*np.cos(np.pi*np.clip(x, 0, 1))
    st = dict(STATE, phase=name, beta=0.0); g = ease(f); TR = asm2.TR
    after = dict(e=0.0, d=0.0, seated=True)
    if name == 'arrive': st.update(px=RUN*(1 - g), sx=-40 - RUN*(1 - g), belt_m=-RUN*g, belt_s=RUN*g)
    elif name == 'push':
        th = np.pi*(1 - g); e = float(np.clip(asm2.face_z(th) - 380, 0, TR)); st.update(theta=th, e=e, d=e, belt_m=-RUN, belt_s=RUN)
    elif name == 'retract': st.update(after, theta=-np.pi*g, belt_m=-RUN, belt_s=RUN)
    elif name == 'sleds': st.update(after, sx=-40 + RUN*g, belt_m=-RUN, belt_s=RUN*(1 + g))
    elif name == 'travel': st.update(after, sx=-40 + RUN, px=PXE*g, belt_m=-RUN + PXE*g, belt_n=PXE*g, belt_s=2*RUN)
    else:
        st.update(after, sx=-40 + RUN, px=PXE, belt_m=-RUN + PXE, belt_n=PXE, belt_s=2*RUN)
        if name == 'lift': st.update(a=np.pi/2*g)
        elif name == 'drive': st.update(a=np.pi/2, free=True, drive=DRV*g)
        else: st.update(a=np.pi/2*(1 - g), free=True, drive=DRV)
    return st

if __name__ == '__main__':
    import sys
    st = cycle(float(sys.argv[1])) if len(sys.argv) > 1 else {}
    if len(sys.argv) > 2: st['s1'] = False
    build(st); print(len(P), 'parts', st.get('phase'))
    rep = lcheck.run(P)
    print('PASS' if not rep['clashes'] and not rep['bad_fits'] and not rep['unconnected'] else 'FAIL')
