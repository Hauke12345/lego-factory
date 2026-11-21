"""Line 3: one line, two robot configurations. Lower modules go on while the robot lies on its back, upper ones once it stands.

  port station (x=PXK, lying)   builder: shoulder kits (motor C, sensor D)      tug: nothing
  track station (x=0, lying)    builder: plain track units                      tug: right unit with the belly sensor + plug D
  stand-up flap (x=PXE)         the robot drives off on its own tracks along the exit rails
  shoulder station (x=RX)       builder: nothing                                tug: hook module + plug C on the left shoulder

The per-robot config only changes which wagons the side trains bring. Main flow -x.
"""
import numpy as np
import asm2, worker2, lcheck, line1, line2, periph, rails, lowmods, shoulder, mods, tline
from asm2 import T, R, add, P, LBG
from line1 import PXE, X2, flapM, arm_angle, fpallet, standup_station

CFG = {'builder': dict(kcfg={'A': 'motor', 'B': 'sensor'}, eye=False, shoulder=False),
       'tug':     dict(kcfg={}, eye=True, shoulder=True)}
PXK, RUNK, RUN = line2.PXK, line2.RUNK, line2.RUN
BASE_TAKEN = line2.BASE_TAKEN
TH0 = line2.TH0
STATE = dict(line2.STATE, cfg='tug', ucart=520.0, utheta=np.pi, ue=shoulder.TR, umod='wagon', hook=mods.SH_UP, show=False)

def robot_M(st):
    """robot frame -> world (worker2 convention) for the robot's current state."""
    px = st['px']; M0 = asm2.core_M0(px)
    if not st['free']: return flapM(px, st['a']) @ M0
    d = st['drive']
    Mr = T(-d, 0, 0) @ flapM(px, np.pi/2) @ M0
    Q = []; worker2.build(M0=Mr, tag='tmp', body='tmp', P=Q, modules={'trackL', 'trackR'}, track_phase=d)
    C = np.array([[x, -8, z, 1] for x in (-26, 26) for z in (-1, 11)], float).T; lift = 0.0
    for q in Q:
        if q['part'] != '3873': continue
        for w in (q['M'] @ C).T:
            if abs(abs(w[2]) - 130) <= 20: lift = max(lift, 120 - (-w[1]))
    return T(0, -lift*min(1.0, d/12.0), 0) @ Mr

def build(st=None):
    st = dict(STATE, **(st or {})); P.clear(); px = st['px']; rails.TAKEN.clear()
    cfg = CFG[st['cfg']]; st['kcfg'] = cfg['kcfg']
    if st['s0']: periph.station(PXK, st)
    if st['s1']:
        for x, z in ((-320, 330), (320, 330), (-320, -310), (320, -310), (-320, 970), (-320, -950)): add(f'base{x}_{z}', '3811', LBG, T(x, 0, z))
        asm2.lane('M', 0, st['belt_m'])
        for L in ('A', 'B'): rails.track(L); rails.tiles(L, rails.TAKEN)
        cells = set().union(*[rails.baseplates(L) for L in ('A', 'B', 'KA', 'KB')])
        if st.get('tline', True):
            cells |= tline.layout(rails.TAKEN)
            n0 = len(P)
            for sd in ('A', 'B'): tline.build_side(sd, st['tl'])
            for p in P[n0:]:                                            # baseplates under the station pushers
                if (p['body'] == 'frame' and p['name'].startswith('ps')) or p['name'][1:].startswith('cs_'):
                    x, z = p['M'][0, 3], p['M'][2, 3]
                    cells.add((640*np.floor(x/640) + 320, 640*np.floor((z - 10)/640) + 330))
        for cx, cz in sorted(cells - BASE_TAKEN): add(f'baseT{cx}_{cz}', '3811', LBG, T(cx, 0, cz))
    if st['s2']:
        asm2.lane('N', 0, st['belt_m'], X2); standup_station(arm_angle(st['a']))
        shoulder.layout(base_taken=BASE_TAKEN | cells if st['s1'] else BASE_TAKEN)
        shoulder.station(dict(st, mod_on_wagon=cfg['shoulder'] and st['umod'] == 'wagon', robot_M_station=shoulder.robot_M(shoulder.RX, shoulder.D_STOP)))
    fpallet(px, st['a'])
    Mr = robot_M(st)
    worker2.build(M0=Mr, tag='rob', body='robot', P=P, modules={'core'})
    if st['kdone']:
        for side, kind in st['kcfg'].items():
            if kind: periph.kit(side, kind, Mr @ periph.M0REF_INV, 'robot')
    if cfg['shoulder'] and st['umod'] == 'robot': shoulder.module(Mr, 0.0, hook=st['hook'], body='robot', plug_body='robot')
    for side, mod, s in (('A', 'trackL', 1), ('B', 'trackR', -1)):
        eye = cfg['eye'] and side == 'B'
        if st['seated']:
            worker2.build(M0=Mr, tag='rob', body='robot', P=P, modules={mod}, track_phase=st['drive'])
            if eye: lowmods.eye_on_unit(Mr, 'robot')
        if not st['s1']: continue
        def bld(sx, side=side, mod=mod, s=s, eye=eye):
            asm2.carrier(side, sx, st['e'])
            if eye: lowmods.plug_shoe(sx, st['e'], side)
            if not st['seated'] or st.get('tl', {}).get('reloaded'):
                d = st['d'] if not st['seated'] else asm2.TR; pxu = px if not st['seated'] else sx + 40
                Mu = T(sx + 40 - pxu, 0, s*d) @ asm2.core_M0(pxu)
                eye = eye and not st['seated']                         # a freshly built unit is plain (eye bracket station: later)
                worker2.build(M0=Mu, tag='rob' if not st['seated'] else 'ru', body=f'unit{side}', P=P, modules={mod})
                if eye: lowmods.eye_on_unit(Mu, f'unit{side}')
        sc = st['tcart'][side]
        rails.cart(side, sc, bld, -40.0, f'sled{side}', f'tw{side}')
        rails.loco_at(side, sc, f'tloco{side}', f'tloco{side}', f'tw{side}', f'sled{side}')
        asm2.pusher(side, st['theta'])
    ride_over_belt_ends(st)
    return P

_LV = None
def ride_over_belt_ends(st):
    """links wrapping round a belt's end sprocket stand up to 1.5 LDU proud of the run: a pallet crossing there rides up on them."""
    global _LV
    if st['free'] and st['a'] > 0: return
    import ldr
    if _LV is None:
        V = ldr.part('3873')['tris'].reshape(-1, 3); _LV = np.c_[V, np.ones(len(V))].T
    base = next(p for p in P if p['name'] == 'pal_base'); bx = base['M'][0, 3]
    top = -84.0
    for p in P:
        if not p['name'].startswith('tread') or abs(p['M'][0, 3] - bx) > 190: continue
        W = p['M'] @ _LV; m = (np.abs(W[0] - bx) < 160) & (np.abs(W[2]) < 60)
        if m.any(): top = min(top, W[1][m].min())
    lift = -84.0 - top
    if lift <= 0.05: return
    riders = {'pallet', 'flap', 'robot'}
    for p in P:
        if p['body'] in riders: p['M'] = T(0, -lift, 0) @ p['M']

D_STOP = shoulder.D_STOP; DRV0 = line2.DRV
PHASES = line2.PHASES[:-2] + [('drive', 4.0), ('lower', 1.5), ('ufeed', 5.0), ('upush', 1.5), ('uretract', 1.0), ('uleave', 2.0)]
U_WAIT = 520.0                                        # module wagon waits up on the curve, well clear of the robots driving past
def cycle(t, cfg='tug'):
    tot = sum(d for _, d in PHASES); u = (t % 1.0)*tot
    for name, dur in PHASES:
        if u <= dur: f = u/dur; break
        u -= dur
    names = [n for n, _ in PHASES]
    idx = names.index(name)
    ease = lambda x: 0.5 - 0.5*np.cos(np.pi*np.clip(x, 0, 1)); g = ease(f)
    if name in names[:names.index('drive')]:
        st = line2.cycle(sum(d for _, d in PHASES[:idx]) / sum(d for _, d in line2.PHASES) + f*dict(PHASES)[name]/sum(d for _, d in line2.PHASES), CFG[cfg]['kcfg'])
    else:
        st = line2.cycle(0.0 + (sum(d for _, d in line2.PHASES[:-2]) + 1e-6)/sum(d for _, d in line2.PHASES))      # state at the start of 'drive'
        st.update(px=PXE, belt_m=-RUNK - PXK + PXE, a=np.pi/2, free=True, seated=True, d=0.0, kdone=True,
                  tcart={sd: line2.s_station(sd) + RUN for sd in 'AB'}, kcart={sd: line2.S_WAIT for sd in 'AB'})
        if name == 'drive': st.update(drive=D_STOP*g)
        else: st.update(drive=D_STOP)
        if name == 'lower': st.update(a=np.pi/2*(1 - g))
        if idx > names.index('lower'): st.update(a=0.0)
    st.update(phase=name, cfg=cfg, ucart=U_WAIT, utheta=np.pi, ue=shoulder.TR, umod='wagon', hook=mods.SH_UP)
    ta = (t % 1.0)*tot
    st['tl'] = tline.state(ta, tot)
    st['tcart'] = {sd: tline.track_cart(ta, line2.s_station(sd), RUN) for sd in 'AB'}
    if CFG[cfg]['shoulder']:
        S0 = shoulder.s_station()
        if name == 'ufeed': st['ucart'] = U_WAIT + (S0 - U_WAIT)*g
        if idx > names.index('ufeed'): st['ucart'] = S0
        if name == 'upush':
            th = np.pi*(1 - g); st.update(utheta=th, ue=shoulder.e_of(th))
        if name == 'uretract':
            th = -np.pi*g; st.update(utheta=th, ue=shoulder.e_of(th), umod='robot')
        if name == 'uleave': st.update(ucart=S0 + RUN*g, ue=shoulder.TR, umod='robot')
    return st

if __name__ == '__main__':
    import sys, io, contextlib
    cfg = sys.argv[2] if len(sys.argv) > 2 else 'tug'
    st = cycle(float(sys.argv[1]), cfg)
    build(st)
    with contextlib.redirect_stdout(io.StringIO()): rep = lcheck.run(P, verbose=False)
    print(len(P), 'parts', st['phase'], 'clash', rep['clashes'][:10]); print('bad', rep['bad_fits'][:6])
    print('unc', [u for u in rep['unconnected'] if 'trk' not in u and 'frame falls apart' not in u][:10])
