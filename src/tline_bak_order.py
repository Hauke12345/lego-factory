"""Track-unit line, laid out beside the long lead-ins of track lanes A and B (side B is the mirror image).

  build lane TA (x -1790): one open build wagon per side shuttles S0 .. S4, then down to the handover beside the lead-in
  supply lane SA (x -1450, upstream of the lead-in): a five-wagon supply train, one wagon per station, parked during the
      build; after S4 it backs off to be restocked (parts warehouse: later) and comes back loaded
  stations: S0 hub pin -> build wagon's holder, S1 13L beam, S2 9L beam + pin, S3 three axles, S4 drive set
      (3 sprockets with the chain already round them, prefab from the chain station: later)
  handover (z 1520): the build wagon's own plungers slide the finished unit onto the waiting track wagon's guides.
All pushes by crank pushers (asm2 cell mechanism) on baseplates; each wagon is a rails cart pulled by its own loco.
"""
import numpy as np
import asm2, rails, tunit, lcheck
from asm2 import T, R, add, P, LBG, DBG
from rails import SEG

ZREF = 1520.0                                   # handover |z| (track wagon parked there on the lead-in)
PITCH = 300 + 2*rails.ARM + 16                  # coupled cart centres
Z4 = 2560.0
ZK = [Z4 + (4 - k)*PITCH for k in range(5)]     # station |z|, S0 .. S4
ZT0, LT = 4700.0, 11*SEG                        # build lane: start |z|, length
ZS0, LS = 6000.0, 11*SEG                        # supply lane
REFILL = 1300.0                                 # supply train backs off this far to be restocked
for side, sg in (('A', 1), ('B', -1)):
    rails.STRAIGHT['T' + side] = ((-1790.0, sg*ZT0), (0.0, -sg), LT)
    rails.STRAIGHT['S' + side] = ((-1450.0, sg*ZS0), (0.0, -sg), LS)
    rails._PATHS.pop('T' + side, None); rails._PATHS.pop('S' + side, None)

def s_lead(z): return rails.LEAD['A']*SEG + 260 + rails.RC - z             # lane A/B lead-in parameter of |z|
S_H = s_lead(ZREF)
def s_T(z): return ZT0 - z
def s_S(z): return ZS0 - z

# pushers: canonical XM offset from the cart's sx (-40) and ZM; contact faces checked in tunit tests
# long-stroke cranks (rc 60): the faces stand 64 back from the retracted yokes, so a backing-off supply train passes every pusher
SUP_PUSH = {'pin': (20, 570), 'vbeam': (-20, 570), 'hbeam': (-160, 570), 'axles': (-100, 590), 'driveset': (-140, 590)}
RCS = 60
H_PUSH = (-90, 370)                               # handover (B-type pusher, pushes +z; long-stroke crank)
def e_sup(stage, theta): return float(np.clip(asm2.face_z(theta, SUP_PUSH[stage][1], RCS) - (SUP_PUSH[stage][1] - 190), 0, tunit.TRS))
def e_hand(theta): return float(np.clip(tunit.TRH - (asm2.face_z(theta, H_PUSH[1], RCS) - (H_PUSH[1] - 190)), 0, tunit.TRH))

MIR = np.diag([1.0, 1.0, -1.0, 1.0])
def _mirror_new(n0, keep=()):
    """side B: mirror the parts added since n0 (z -> -z, proper rotation), except bodies in keep."""
    for p in P[n0:]:
        if p['body'] in keep: continue
        p['M'] = MIR @ p['M'] @ MIR

def _place_canon(n0, G):
    for p in P[n0:]: p['M'] = G @ p['M']

def cart_G(lane, s, zc):
    return rails.cart_frame(lane, s) @ np.linalg.inv(rails.frame((-40.0, zc), (1.0, 0.0)))

def _snap(G, XM, side):
    """shift the pusher along the lane so its frame bricks land on the baseplate studs (world z = 10 mod 20 for XM)."""
    w = G @ np.array([XM, 0, 0, 1.0]); z = w[2]
    zt = 20*np.round((z - 10)/20) + 10; return XM + (z - zt)*(1 if side == 'A' else -1)

def unit_parts(side, Mu, stages, body, tag):
    if side == 'A': return tunit.unit_parts(Mu, stages, tag=tag, body=body)
    # side B: the right unit, mirrored placement of a right track module
    import worker2
    Q = []; worker2.build(M0=Mu, tag=tag, body=body, P=Q, modules={'trackR'})
    out = []
    for q in Q:
        n = q['name'].split('tkR', 1)[1]
        st = next((s for s, keys in tunit._GROUPS.items() if any(n.startswith(k) for k in keys)), None)
        if n == '_pin2': st = 'hbeam'
        if st in stages: out.append(q)
    return out

def unit_M(side, dz):
    sg = 1 if side == 'A' else -1
    return T(0, 0, sg*dz) @ asm2.core_M0(0.0)                               # unit at canonical sx = -40 (core at px 0)

def build_side(side, st):
    """st: bz (build wagon |z|), eh, stages (done on the build wagon), unit (True if a unit is on the build wagon),
    cur (stage being pushed or None), ce (its supply offset), ctheta, htheta, train_dz (supply train backed off by),
    loaded (set of stages whose part is still on its supply wagon)."""
    sg = 1 if side == 'A' else -1
    # ---- build wagon + unit
    def bld(sx0):
        n0 = len(P); tunit.build_wagon(sx0, st['eh'], body=f'bwag{side}', pb=f'hplung{side}')
        if side == 'B': _mirror_new(n0)
        for p in P[n0:]: p['name'] = side + p['name']
        if st['unit']:
            for q in unit_parts(side, unit_M(side, st['eh']), st['stages'], f'bunit{side}', f'tu{side}'): P.append(q)
        if st['cur'] and st['ce'] > 0 and st['cur'] not in st['stages']:
            for q in unit_parts(side, unit_M(side, st['ce']), [st['cur']], f'part{side}', f'tp{side}'):
                q['body'] = f'part{side}'; P.append(q)
    rails.cart('T' + side, s_T(st['bz']), bld, -40.0, f'bwag{side}', f'bw{side}')
    rails.loco_at('T' + side, s_T(st['bz']), f'bloco{side}', f'bloco{side}', f'bw{side}', f'bwag{side}')
    lcheck.RESTING.update({f'bwag{side}': 'rolls on the rails', f'hplung{side}': 'slides in the build-wagon walls',
                           f'bunit{side}': 'hangs on the build stubs', f'part{side}': 'held by the supply tooling'})
    # ---- supply train
    for k, stage in enumerate(tunit.STAGES):
        z = ZK[k] + st['train_dz']; s = s_S(z)
        e = st['ce'] if st['cur'] == stage else tunit.TRS
        def sbld(sx0, stage=stage, e=e):
            n0 = len(P); tunit.SUPPLY[stage](sx0, e, body=f'swag{side}{k}', pb=f'splung{side}{k}')
            if side == 'B': _mirror_new(n0)
            for p in P[n0:]: p['name'] = side + p['name']
            if stage in st['loaded'] and not (st['cur'] == stage and st['ce'] <= 0):
                for q in unit_parts(side, unit_M(side, e) @ T(0, 0, 0), [stage], f'spart{side}{k}', f'ts{side}{k}'):
                    q['M'] = T(sx0 + 40, 0, 0) @ q['M']; q['body'] = f'spart{side}{k}'; P.append(q)
        rails.cart('S' + side, s, sbld, -40.0, f'swag{side}{k}', f'sw{side}{k}')
        lcheck.RESTING.update({f'swag{side}{k}': 'rolls on the rails', f'splung{side}{k}': 'slides in the supply-wagon walls',
                               f'spart{side}{k}': 'held by the supply tooling'})
        if k < 4:                                                            # magnet coupling to the wagon in front
            sf = s_S(ZK[k + 1] + st['train_dz'])
            front = (rails.cart_frame('S' + side, s) @ np.array([150, -84, 10, 1.0]))[:3]
            rear = rails.cart_stud('S' + side, sf)
            d = (rear - front)[[0, 2]]; d /= np.linalg.norm(d)
            rails.coupler_at(front, d, f'sc{side}{k}f', f'swag{side}{k}'); rails.coupler_at(rear, -d, f'sc{side}{k}r', f'swag{side}{k + 1}')
    rails.loco_at('S' + side, s_S(ZK[0] + st['train_dz']), f'sloco{side}', f'sloco{side}', f'sw{side}0', f'swag{side}0')
    # ---- pushers (canonical in the parked supply wagon's frame, on the stud grid)
    for k, stage in enumerate(tunit.STAGES):
        G = cart_G('S' + side, s_S(ZK[k]), 260.0)
        dx, ZM = SUP_PUSH[stage]; th = st['ctheta'] if st['cur'] == stage else np.pi
        XM = _snap(G if side == 'A' else G @ MIR, -40 + dx, side)
        n0 = len(P); asm2.pusher('A', th, XM=XM, ZM=ZM, tag=f'S{side}{k}', rc=RCS)
        if side == 'B': _mirror_new(n0)
        _place_canon(n0, G if side == 'A' else G)
    G = cart_G('T' + side, s_T(ZREF), -80.0)
    XM = _snap(G, -40 + H_PUSH[0], side)
    n0 = len(P); asm2.pusher('B', st['htheta'], XM=XM, ZM=H_PUSH[1], tag=f'H{side}', rc=RCS)
    if side == 'B': _mirror_new(n0)
    _place_canon(n0, G)

def layout(taken):
    """tracks, tile carpets and the baseplate cells this line needs (returned)."""
    cells = set()
    for side in ('A', 'B'):
        for L in ('T' + side, 'S' + side):
            rails.track(L); rails.tiles(L, taken); cells |= rails.baseplates(L)
    return cells

# ---------------------------------------------------------------- timeline (absolute seconds in the robot cycle)
T_HAND = 26.5                                   # handover push starts (track wagon back at the lead-in, during 'drive')
SEQ = []                                         # (tau0, tau1, kind, data) relative to the build wagon leaving the handover
def _seq():
    t = 0.0; out = [(0.0, 6.0, 'move', (ZREF, ZK[0]))]; t = 6.0
    for k, stage in enumerate(tunit.STAGES):
        out.append((t, t + 1.5, 'push', stage)); out.append((t + 1.5, t + 2.5, 'retract', stage)); t += 2.5
        z1 = ZK[k + 1] if k < 4 else ZREF; dur = 1.0 if k < 4 else 3.0
        out.append((t, t + dur, 'move', (ZK[k], z1))); t += dur
    return out
SEQ = _seq(); T_LEAVE = 30.0                      # build wagon leaves the handover (tau = 0)
T_HAND = 26.5                                     # handover push starts (absolute cycle time, during the robot's 'drive')

def _ez(x): return 0.5 - 0.5*np.cos(np.pi*np.clip(x, 0, 1))
def state(t, tot):
    """unit-line state at absolute cycle time t (both sides identical)."""
    ta = t % tot; tau = (ta - T_LEAVE) % tot
    st = dict(bz=ZREF, eh=0.0, stages=[], unit=False, cur=None, ce=tunit.TRS, ctheta=np.pi, htheta=np.pi, train_dz=0.0,
              loaded=set(), reloaded=False)
    done = []
    for t0, t1, kind, dat in SEQ:
        if tau < t0: break
        g = _ez((tau - t0)/(t1 - t0)); live = tau < t1
        if kind == 'move':
            za, zb = dat; st['bz'] = za + (zb - za)*g if live else zb
        elif kind == 'push':
            if live:
                th = np.pi*(1 - g); st.update(cur=dat, ctheta=th, ce=e_sup(dat, th))
            else: done.append(dat)
        elif kind == 'retract' and live:
            th = -np.pi*g; st.update(cur=dat, ctheta=th, ce=e_sup(dat, th))
    if st['cur'] in done:                      # retracting: the part is already on the unit, the tooling goes back
        st['retract'] = True
    st['stages'] = list(done); st['unit'] = bool(done) or st['cur'] is not None
    # supply train: parked with all parts before S0; after S4 it backs off, is restocked, comes back loaded
    u = (tau - SEQ[-1][0]) % tot                  # from the start of the last move (all parts used)
    started = set(done) | ({st['cur']} if st['cur'] else set())
    if u < 3.5: st['train_dz'] = REFILL*_ez(u/3.5)
    elif u < 17.5: st['train_dz'] = REFILL
    elif u < 21.0: st['train_dz'] = REFILL*(1 - _ez((u - 17.5)/3.5)); st['loaded'] = set(tunit.STAGES)
    else: st['loaded'] = set(tunit.STAGES) - started
    # handover: the finished unit slides onto the track wagon (which came back to the lead-in during 'travel'/'lift')
    if T_HAND <= ta < T_HAND + 1.5:
        th = np.pi*(1 - _ez((ta - T_HAND)/1.5)); st.update(bz=ZREF, unit=True, stages=list(tunit.STAGES), htheta=th, eh=e_hand(th))
    elif T_HAND + 1.5 <= ta < T_LEAVE:
        th = -np.pi*_ez((ta - T_HAND - 1.5)/1.0); st.update(bz=ZREF, unit=False, stages=[], htheta=th, eh=e_hand(th))
    st['reloaded'] = ta >= T_HAND + 1.5
    return st

def track_cart(t, s_station, run):
    """track wagon path parameter (A and B identical) at absolute cycle time t (line-3 phase times)."""
    if t < 11.7: return S_H
    if t < 14.7: return S_H + (s_station - S_H)*_ez((t - 11.7)/3.0)
    if t < 17.0: return s_station
    if t < 19.0: return s_station + run*_ez((t - 17.0)/2.0)
    if t < 26.0: return s_station + run + (S_H - s_station - run)*_ez((t - 19.0)/7.0)
    return S_H
