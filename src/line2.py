"""Line 2: peripheral station (x=PXK) -> track station (x=0) -> stand-up station (x=PXE). Main flow -x."""
import numpy as np
import asm2, worker2, lcheck, line1, periph, rails
from asm2 import T, R, add, P, LBG
from line1 import PXE, X2, flapM, arm_angle, fpallet, standup_station

BASE_TAKEN = {(x, z) for x in (-320, 320) for z in (330, -310)} | {(-320, 970), (-320, -950)} | {(x, z) for x in (960, 1600) for z in (330, -310)} | {(1600, 970), (1600, -950)} | {(x, z) for x in (-1600, -960, -2240) for z in (330, -310)}
PXK = 1280.0; RUNK = 340.0; RUN = 420.0; DRV = 430.0
TH0 = periph.theta_rest()
lcheck.RESTING.update({'forkA': 'slides in the tower bars', 'forkB': 'slides in the tower bars'})
STATE = dict(line1.STATE, px=PXK, ksx=PXK + 80, tcart={'A': 560.0, 'B': 560.0}, kcart={'A': 560.0, 'B': 560.0}, ktheta=TH0, kdone=False, kcfg={'A': 'motor', 'B': 'sensor'}, belt_k=0.0, s0=True)

SHOW_X = (360.0, 700.0, 1040.0); SHOW_Z = 820.0
def showroom():
    """finished variants standing behind the line between the stations, facing it (they don't block the camera side)."""
    import tug
    M = [tug.pose(x, SHOW_Z, 0.0) for x in SHOW_X]
    for x in (320, 960): add(f'baseS{x}_970', '3811', LBG, T(x, 0, 970))
    tug.robot(M[0], P, tag='v1', body='show1')                                                # tug: sensor down, hook motor
    worker2.build(M0=M[1], tag='v2', body='show2', P=P)                                       # builder: gun arm + tool arm
    worker2.build(M0=M[2], tag='v3', body='show3', P=P, modules={'core', 'trackL', 'trackR', 'head'})
    for side in ('A', 'B'): periph.kit(side, 'motor', M[2] @ periph.M0REF_INV, 'show3', pre='v3k')     # two-motor variant
    for k in (1, 2, 3): lcheck.RESTING.setdefault(f'show{k}', 'stands on its tracks')

def build(st=None):
    st = dict(STATE, **(st or {})); P.clear(); px = st['px']; rails.TAKEN.clear()
    if st['s0']: periph.station(PXK, st)
    if st['s1']:
        for x, z in ((-320, 330), (320, 330), (-320, -310), (320, -310), (-320, 970), (-320, -950)): add(f'base{x}_{z}', '3811', LBG, T(x, 0, z))
        asm2.lane('M', 0, st['belt_m'])
        for L in ('A', 'B'): rails.track(L); rails.tiles(L, rails.TAKEN)
        for cx, cz in sorted(set().union(*[rails.baseplates(L) for L in ('A', 'B', 'KA', 'KB')]) - BASE_TAKEN): add(f'baseT{cx}_{cz}', '3811', LBG, T(cx, 0, cz))
    if st['s2']:
        asm2.lane('N', 0, st['belt_m'], X2); standup_station(arm_angle(st['a']))
    fpallet(px, st['a'])
    M0 = asm2.core_M0(px)
    Mr = (T(-st['drive'], 0, 0) @ flapM(px, np.pi/2) if st['free'] else flapM(px, st['a'])) @ M0
    if st['free']:                                   # standing on the exit rails: rest on the lowest link edge (rail top 120 above the baseplate)
        Q = []; worker2.build(M0=Mr, tag='tmp', body='tmp', P=Q, modules={'trackL', 'trackR'}, track_phase=st['drive'])
        C = np.array([[x, -8, z, 1] for x in (-26, 26) for z in (-1, 11)], float).T; lift = 0.0
        for q in Q:
            if q['part'] != '3873': continue
            for w in (q['M'] @ C).T:
                if abs(abs(w[2]) - 130) <= 20: lift = max(lift, 120 - (-w[1]))
        Mr = T(0, -lift*min(1.0, st['drive']/12.0), 0) @ Mr      # ramps in as the hub slides out of the flap's end wall (sub-mm overlap at handover, see notes)
    if st.get('show', True) and st['s1']: showroom()
    worker2.build(M0=Mr, tag='rob', body='robot', P=P, modules={'core'})
    if st['kdone']:
        for side, kind in st['kcfg'].items():
            if kind: periph.kit(side, kind, Mr @ periph.M0REF_INV, 'robot')
    for side, mod, s in (('A', 'trackL', 1), ('B', 'trackR', -1)):
        if st['seated']: worker2.build(M0=Mr, tag='rob', body='robot', P=P, modules={mod}, track_phase=st['drive'])
        if not st['s1']: continue
        def build(sx, side=side, mod=mod, s=s):
            asm2.carrier(side, sx, st['e'])
            if not st['seated']: worker2.build(M0=T(sx + 40 - px, 0, s*st['d']) @ asm2.core_M0(px), tag='rob', body=f'unit{side}', P=P, modules={mod})
        sc = st['tcart'][side]
        rails.cart(side, sc, build, -40.0, f'sled{side}', f'tw{side}')
        rails.loco_at(side, sc, f'tloco{side}', f'tloco{side}', f'tw{side}', f'sled{side}')
        asm2.pusher(side, st['theta'])
    return P

PHASES = [('karrive', 2.0), ('kfeed', 3.0), ('kpush', 2.0), ('kretract', 1.2), ('travel1', 3.5), ('feed', 3.0), ('push', 1.3), ('retract', 1.0),
          ('sleds', 2.0), ('travel', 4.5), ('lift', 2.5), ('drive', 3.0), ('lower', 1.5)]
S_WAIT = 560.0                                       # wagons wait on the lead-in, loco still on the track
def s_station(lane): return rails.s_of_x(lane, PXK + 80 if lane.startswith('K') else -40.0)
def cycle(t, cfg=None):
    tot = sum(d for _, d in PHASES); u = (t % 1.0)*tot
    for name, dur in PHASES:
        if u <= dur: f = u/dur; break
        u -= dur
    ease = lambda x: 0.5 - 0.5*np.cos(np.pi*np.clip(x, 0, 1))
    st = dict(STATE, phase=name, beta=0.0); g = ease(f); TR = asm2.TR
    if cfg: st['kcfg'] = cfg
    idx = [n for n, _ in PHASES].index(name)
    after = lambda n: idx > [m for m, _ in PHASES].index(n)
    # port station: the body arrives on its own, then the two kit trains come round the curves
    if name == 'karrive': st.update(px=PXK + RUNK*(1 - g), belt_m=-RUNK*g)
    ks = {sd: S_WAIT for sd in 'AB'}
    if name == 'kfeed': ks = {sd: S_WAIT + (s_station('K' + sd) - S_WAIT)*g for sd in 'AB'}
    if after('kfeed'): ks = {sd: s_station('K' + sd) for sd in 'AB'}
    if name == 'travel1': ks = {sd: s_station('K' + sd) + (S_WAIT - s_station('K' + sd))*g for sd in 'AB'}      # empty wagons pulled back
    if after('travel1'): ks = {sd: S_WAIT for sd in 'AB'}
    st['kcart'] = ks
    if not after('karrive') and name != 'karrive': pass
    if name in ('kfeed',): st.update(belt_m=-RUNK)
    if name == 'kpush': st.update(ktheta=TH0*(1 - g), belt_m=-RUNK)
    if name == 'kretract': st.update(ktheta=TH0*g, kdone=True, belt_m=-RUNK)
    if after('kretract'): st.update(kdone=True)
    # track station: body travels over, then the track-unit trains arrive
    if name == 'travel1': st.update(px=PXK*(1 - g), belt_m=-RUNK - PXK*g)
    if after('travel1'): st.update(px=0.0, belt_m=-RUNK - PXK)
    ts = {sd: S_WAIT for sd in 'AB'}
    if name == 'feed': ts = {sd: S_WAIT + (s_station(sd) - S_WAIT)*g for sd in 'AB'}
    if after('feed'): ts = {sd: s_station(sd) for sd in 'AB'}
    if name == 'push':
        th = np.pi*(1 - g); e = float(np.clip(asm2.face_z(th) - 380, 0, TR)); st.update(theta=th, e=e, d=e)
    if name == 'retract':
        th = -np.pi*g; st.update(theta=th, e=float(np.clip(asm2.face_z(th) - 380, 0, TR)), d=0.0, seated=True)
    if after('retract'): st.update(d=0.0, seated=True)
    if name == 'sleds': ts = {sd: s_station(sd) + RUN*g for sd in 'AB'}                   # empty wagons leave forwards
    if after('sleds'): ts = {sd: s_station(sd) + RUN for sd in 'AB'}
    st['tcart'] = ts
    if name == 'travel': st.update(px=PXE*g, belt_m=-RUNK - PXK + PXE*g)
    if after('travel'): st.update(px=PXE, belt_m=-RUNK - PXK + PXE)
    if name == 'lift': st.update(a=np.pi/2*g)
    if name == 'drive': st.update(a=np.pi/2, free=True, drive=DRV*g)
    if name == 'lower': st.update(a=np.pi/2*(1 - g), free=True, drive=DRV)
    return st

if __name__ == '__main__':
    import sys
    st = cycle(float(sys.argv[1])) if len(sys.argv) > 1 else {}
    if len(sys.argv) > 2:
        for k in ('s0', 's1', 's2'): st[k] = k in sys.argv[2]
    build(st); print(len(P), 'parts', st.get('phase'), 'cable lengths (LDU):', {k: round(v) for k, v in periph.CABLE_LEN.items()})
    rep = lcheck.run(P)
    print('PASS' if not rep['clashes'] and not rep['bad_fits'] and not rep['unconnected'] else 'FAIL')
