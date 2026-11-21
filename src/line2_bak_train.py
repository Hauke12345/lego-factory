"""Line 2: peripheral station (x=PXK) -> track station (x=0) -> stand-up station (x=PXE). Main flow -x."""
import numpy as np
import asm2, worker2, lcheck, line1, periph
from asm2 import T, R, add, P, LBG
from line1 import PXE, X2, flapM, arm_angle, fpallet, standup_station

PXK = 1280.0; RUNK = 340.0; RUN = 420.0; DRV = 430.0
TH0 = periph.theta_rest()
lcheck.RESTING.update({'forkA': 'slides in the tower bars', 'forkB': 'slides in the tower bars'})
STATE = dict(line1.STATE, px=PXK, ksx=PXK + 80, ktheta=TH0, kdone=False, kcfg={'A': 'motor', 'B': 'sensor'}, belt_k=0.0, s0=True)

def build(st=None):
    st = dict(STATE, **(st or {})); P.clear(); px = st['px']
    if st['s0']: periph.station(PXK, st)
    if st['s1']:
        for x, z in ((-320, 330), (320, 330), (-320, -310), (320, -310), (-320, 970), (-320, -950)): add(f'base{x}_{z}', '3811', LBG, T(x, 0, z))
        asm2.lane('M', 0, st['belt_m']); asm2.lane('A', 260, st['belt_s']); asm2.lane('B', -260, st['belt_s'])
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
    worker2.build(M0=Mr, tag='rob', body='robot', P=P, modules={'core'})
    if st['kdone']:
        for side, kind in st['kcfg'].items():
            if kind: periph.kit(side, kind, Mr @ periph.M0REF_INV, 'robot')
    for side, mod, s in (('A', 'trackL', 1), ('B', 'trackR', -1)):
        if st['seated']: worker2.build(M0=Mr, tag='rob', body='robot', P=P, modules={mod}, track_phase=st['drive'])
        if not st['s1']: continue
        asm2.carrier(side, st['sx'], st['e'])
        if not st['seated']: worker2.build(M0=T(st['sx'] + 40 - px, 0, s*st['d']) @ asm2.core_M0(px), tag='rob', body=f'unit{side}', P=P, modules={mod})
        asm2.pusher(side, st['theta'])
    return P

PHASES = [('karrive', 2.0), ('kpush', 2.0), ('kretract', 1.2), ('travel1', 3.5), ('push', 1.3), ('retract', 1.0), ('sleds', 1.6),
          ('travel', 4.5), ('lift', 2.5), ('drive', 3.0), ('lower', 1.5)]
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
    if name == 'karrive': st.update(px=PXK + RUNK*(1 - g), ksx=PXK + 80 + RUNK*(1 - g), belt_m=-RUNK*g, belt_k=-RUNK*g)
    else: st.update(belt_k=-RUNK)
    if name == 'kpush': st.update(ktheta=TH0*(1 - g), belt_m=-RUNK)
    if name == 'kretract': st.update(ktheta=TH0*g, kdone=True, belt_m=-RUNK)
    if after('kretract'): st.update(kdone=True)
    # track station
    if name == 'travel1': st.update(px=PXK*(1 - g), sx=-40 - RUN*(1 - g), belt_m=-RUNK - PXK*g, belt_s=RUN*g)
    elif not after('travel1'): st.update(sx=-40 - RUN)
    if after('travel1'): st.update(px=0.0, belt_m=-RUNK - PXK, belt_s=RUN)
    if name == 'push':
        th = np.pi*(1 - g); e = float(np.clip(asm2.face_z(th) - 380, 0, TR)); st.update(theta=th, e=e, d=e)
    if name == 'retract':
        th = -np.pi*g; st.update(theta=th, e=float(np.clip(asm2.face_z(th) - 380, 0, TR)), d=0.0, seated=True)
    if after('retract'): st.update(d=0.0, seated=True)
    if name == 'sleds': st.update(sx=-40 + RUN*g, belt_s=RUN*(1 + g))
    if after('sleds'): st.update(sx=-40 + RUN, belt_s=2*RUN)
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
