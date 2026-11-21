"""Final-assembly station 1: track units onto the core (joints J2 pin + J3 axle), robot lying on its back.

World = LDraw coords (LDU), flow +x, -y up. Three lanes: main lane z=0 carries the core pallet,
side lanes z=+-260 carry the track-unit carriers. Side A = +z (robot's left track), side B = -z.
The core lies with its side holes / motor outputs on the line y=Y0, pointing +-z; the robot's head points downstream.
A carrier holds one track unit on two guide axles; two captive plungers (joined by a yoke) push it home.
A crank pusher per side (Move Hub output -> crank -> conrod -> slide block) presses the yoke.
"""
import numpy as np
import cell, worker2, lcheck
from cell import T, R, ux, LBG, DBG, BLK, WHT, RED, YEL, BLU, ORG

XS, NL, YA, RP = 600, 162, -46, 30
YP = -84                 # belt top
Y0 = -250                # hub hole / output line
LANES = {'M': 0, 'A': 260, 'B': -260}
TR = 56                  # unit travel on the guides
LIFT = 56                # pusher mechanism lifted above its cell height
XM, ZM = -60, 550        # pusher mechanism offset from cell coordinates
P = []

def col3(a, b, c):
    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2] = a, b, c; return M
def add(name, part, col, M, body='frame'):
    P.append(dict(name=name, part=part, col=col, M=np.array(M, float), body=body))

lcheck.RESTING.update({'sledA': 'rides on tread', 'sledB': 'rides on tread', 'unitA': 'hangs on the guide axles', 'unitB': 'hangs on the guide axles',
                       'pusherA': 'slides on the ledge', 'pusherB': 'slides on the ledge', 'plungA': 'slides in the carrier', 'plungB': 'slides in the carrier'})

# ---------------------------------------------------------------- lanes
def tread_links(phase, zc, x0=0):
    L = 2*XS; per = 2*L + 2*np.pi*RP; pitch = per/NL; out = []
    B = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]], float)
    for i in range(NL):
        s = (i*pitch + phase) % per
        if s < L: p = (-XS + s, YA - RP); ang = 0.0
        elif s < L + np.pi*RP: a = (s - L)/RP; p = (XS + RP*np.sin(a), YA - RP*np.cos(a)); ang = np.degrees(a)
        elif s < 2*L + np.pi*RP: p = (XS - (s - L - np.pi*RP), YA + RP); ang = 180.0
        else: a = (s - 2*L - np.pi*RP)/RP; p = (-XS - RP*np.sin(a), YA + RP*np.cos(a)); ang = 180 + np.degrees(a)
        out.append(T(p[0] + x0, p[1], zc) @ R('z', ang) @ B)
    return out

def lane(tag, zc, belt, x0=0):
    for k, sg in ((0, -1), (1, 1)):
        zw = zc + 80*sg
        side = R('y', 180) if sg < 0 else np.eye(4)            # 87087 stud points -z by default (inner face of the +z wall)
        tl = col3((1, 0, 0), (0, 0, sg), (0, -sg, 0))          # liner tile on its edge, top face looking into the lane
        for xc in (x0 - 480, x0 - 160, x0 + 160, x0 + 480):
            s = f'{tag}{k}_{xc}'
            add(f'w1_{s}', '2465', DBG, T(xc, -24, zw))
            for dx in (-80, 80): add(f'w2_{s}{dx}', '3460', DBG, T(xc + dx, -32, zw))
            add(f'w3_{s}', '3703', BLK, T(xc, -56, zw))
            add(f'w4_{s}', '2465', DBG, T(xc, -80, zw))
            for dx in (-150, -10, 10, 150): add(f'w5_{s}s{dx}', '87087', YEL, T(xc + dx, -104, zw) @ side)
            for dx in (-80, 80):
                add(f'w5_{s}b{dx}', '3009', YEL, T(xc + dx, -104, zw))
                add(f'lin_{s}{dx}', '4162', LBG, T(xc + dx, -94, zw - 18*sg) @ tl)       # lane 124 wide for 120-wide pallets
                add(f'w6_{s}{dx}', '4162', BLK, T(xc + dx, -112, zw))
    for side, x in (('idl', x0 - XS), ('drv', x0 + XS)):
        b = f'{side}{tag}'
        add(f'ax_{b}', '3737', BLK, T(x, YA, zc) @ R('y', 90), b)                       # axle 10: ends flush with the half bushes
        for z in (zc - 95, zc + 95): add(f'bush_{b}{z}', '32123a', LBG, T(x, YA, z), b)
        add(f'spr_{b}', '3648b', LBG, T(x, YA, zc), b)
    for i, M in enumerate(tread_links(belt, zc, x0)): add(f'tread{tag}{i:03d}', '3873', BLK, M, f'link{tag}{i:03d}')
    for bx in (x0 - 400, x0 - 200, x0, x0 + 200, x0 + 400):
        add(f'bed_b{tag}{bx}', '3894', DBG, T(bx, -56, zc)); add(f'bed_p{tag}{bx}', '3666', DBG, T(bx, -64, zc))
        add(f'bed_t{tag}{bx}', '6636', LBG, T(bx, -72, zc))
        for x in (bx - 40, bx + 40): add(f'bed_ax{tag}{x}', '3737', BLK, T(x, YA, zc) @ R('y', 90))

# ---------------------------------------------------------------- core pallet
def core_M0(px):
    return T(px - 211, Y0, 0) @ col3((0, 0, -1), (-1, 0, 0), (0, 1, 0))

def pallet(px, body='pallet'):
    a = lambda n, p, c, M: add('pal_' + n, p, c, M, body)
    a('base', '3027', LBG, T(px, YP - 8, 0))
    for k in range(4):
        for dx in (-140, 0, 140): a(f'pier{k}_{dx}', '2456', DBG if k % 2 else LBG, T(px + dx, YP - 32 - 24*k, 0) @ R('y', 90))
    for dx in (-120, 0, 120): a(f'deck{dx}', '3036', LBG, T(px + dx, -196, 0) @ R('y', 90))
    a('top', '3027', LBG, T(px, -204, 0))
    for dx in (-120, -40, 40, 120):
        for dz in (-40, 0, 40): a(f'tile{dx}_{dz}', '87079', DBG, T(px + dx, -212, dz))
    for sz in (-70, 70):
        for k in range(3): a(f'lip{sz}_{k}', '3460', BLU, T(px, -204 - 8*k, sz))
    for sx in (-170, 170):
        a(f'end{sx}', '3010', BLU, T(px + sx, -220, 0) @ R('y', 90))
        a(f'endp{sx}', '3710', BLU, T(px + sx, -228, 0) @ R('y', 90))

# ---------------------------------------------------------------- carrier (sled + guides + plungers)
def carrier(side, sx, e):
    """sx: sled centre x (aligned with the core when sx = px - 40). e: plunger position, 0 = home, TR = out."""
    s = 1 if side == 'A' else -1; b = f'sled{side}'; pb = f'plung{side}'
    a = lambda n, p, c, M, body=b: add(f'pal{side}_{n}', p, c, M, body)
    a('base', '3027', LBG, T(sx, YP - 8, 260*s))
    for z in (250, 310):
        for dx in (-140, 140):
            for k in range(5): a(f'col{z}_{dx}_{k}', '3004', DBG if k % 2 else ORG, T(sx + dx, -116 - 24*k, z*s))
            for k in range(3): a(f'colp{z}_{dx}_{k}', '3023', DBG, T(sx + dx, -220 - 8*k, z*s))
        a(f'bar{z}', '3703', ORG, T(sx, -260, z*s))
    for i, dx in enumerate((0, 80)):                                    # guide axles, tips at |z| = 86
        a(f'guide{i}', '3708', LBG, T(sx + dx, Y0, 206*s) @ R('y', 90))
        for z in (265, 295): a(f'gbush{i}_{z}', '32123a', YEL, T(sx + dx, Y0, z*s))
    # plungers: pin pad (axle 16) and drive-axle pad (axle 12) joined by a beam-9 yoke
    a('pl_pin', '50451', BLK, T(sx + 60, Y0, (240 + e)*s) @ R('y', 90), pb)
    a('pl_pin_pad', '32123a', RED, T(sx + 60, Y0, (85 + e)*s), pb)
    a('pl_axl', '3708', BLK, T(sx - 100, Y0, (280 + e)*s) @ R('y', 90), pb)
    a('pl_axl_pad', '32123a', RED, T(sx - 100, Y0, (165 + e)*s), pb)
    a('pl_yoke', '40490', RED, T(sx - 20, Y0, (370 + e)*s) @ col3((0, 1, 0), (0, 0, 1), (1, 0, 0)), pb)
    for dx in (60, -100):
        for z in (355, 385): a(f'pl_b{dx}_{z}', '32123a', YEL, T(sx + dx, Y0, (z + e)*s), pb)

# ---------------------------------------------------------------- crank pusher (reuses the verified cell mechanism)
_PUSH = ('hub', 'ped', 'jump', 'bcol', 'ledge', 'rail_', 'tile_L', 'pb', 'cr_', 'lk_')
_cache = {}
def _cell_pusher(theta, rc=40):
    k = (round(float(theta), 6), rc)
    if k not in _cache:
        out = []; rc0 = cell.RC; cell.RC = rc
        for p in cell.build(dict(theta=theta)):
            n = p['name']
            if n == 'pb_tile' or not n.startswith(_PUSH) or (rc == 60 and n == 'rail_L1'): continue   # long stroke: pusher axle passes over the short left rail
            out.append(dict(p, M=p['M'].copy()))
        zc = cell.slider(theta)[0] - 20
        out.append(dict(name='pb_riser', part='3001', col=RED, M=T(0, cell.YL - 48, zc) @ R('y', 90), body='pusher'))
        for k_ in range(5): out.append(dict(name=f'fcol{k_}', part='3010', col=DBG, M=T(0, -24 - 24*k_, -110), body='frame'))
        for i, y in enumerate((-128, -136)): out.append(dict(name=f'fcol_p{i}', part='3710', col=DBG, M=T(0, y, -110), body='frame'))
        _cache[k] = out; cell.RC = rc0
    return _cache[k]

def pusher(side, theta, XM=XM, ZM=ZM, tag='', riser=True, rc=40):
    Mt = T(XM, -LIFT, ZM)
    if side == 'B': Mt = T(XM, 0, 0) @ R('y', 180) @ T(-XM, 0, 0) @ Mt
    for p in _cell_pusher(theta, rc):
        if p['name'] == 'pb_riser' and not riser: continue
        b = p['body'] if p['body'] == 'frame' else p['body'] + tag + side
        add(f"ps{tag}{side}_{p['name']}", p['part'], p['col'], Mt @ p['M'], b)
    # two brick layers + a plate under the lifted mechanism
    Mb = T(XM, 0, ZM) if side == 'A' else T(XM, 0, 0) @ R('y', 180) @ T(-XM, 0, 0) @ T(XM, 0, ZM)
    for xc in (-50, 50):
        for k in range(2): add(f'ps{tag}{side}_pf{k}_{xc}', '2465', LBG, Mb @ T(xc, -24 - 24*k, 160) @ R('y', 90))
        for dz in (-80, 80): add(f'ps{tag}{side}_pfp_{xc}{dz}', '3460', DBG, Mb @ T(xc, -56, 160 + dz) @ R('y', 90))
    for z in (-30, -110):
        for k in range(2): add(f'ps{tag}{side}_cf{k}_{z}', '3010', LBG, Mb @ T(0, -24 - 24*k, z))
        add(f'ps{tag}{side}_cfp_{z}', '3710', DBG, Mb @ T(0, -56, z))

def face_z(theta, ZM=ZM, rc=40):
    """pusher front face |z|."""
    rc0 = cell.RC; cell.RC = rc; z = cell.slider(theta)[0] - 60 + ZM; cell.RC = rc0
    return z

# ---------------------------------------------------------------- state and build
STATE = dict(px=0.0, sx=-40.0, theta=np.pi, e=float(TR), d=float(TR), seated=False, units=True, core=True, belt_m=0.0, belt_s=0.0, phase='rest')

def build(st=None):
    st = dict(STATE, **(st or {})); P.clear()
    for x, z in ((-320, 330), (320, 330), (-320, -310), (320, -310), (-320, 970), (-320, -950)):
        add(f'base{x}_{z}', '3811', LBG, T(x, 0, z))
    lane('M', 0, st['belt_m']); lane('A', 260, st['belt_s']); lane('B', -260, st['belt_s'])
    pallet(st['px'])
    M0 = core_M0(st['px'])
    if st['core']: worker2.build(M0=M0, tag='rob', body='robot', P=P, modules={'core'})
    for side, mod, s in (('A', 'trackL', 1), ('B', 'trackR', -1)):
        carrier(side, st['sx'], st['e'])
        if not st['units']: continue
        if st['seated']: worker2.build(M0=M0, tag='rob', body='robot', P=P, modules={mod})
        else: worker2.build(M0=T(st['sx'] + 40 - st['px'], 0, s*st['d']) @ M0, tag='rob', body=f'unit{side}', P=P, modules={mod})
        pusher(side, st['theta'])
    return P

PHASES = [('arrive', 2.2), ('push', 1.3), ('retract', 1.0), ('sleds', 2.0), ('robot', 2.2)]
RUN = 420.0
def cycle(t):
    tot = sum(d for _, d in PHASES); u = (t % 1.0)*tot
    for name, dur in PHASES:
        if u <= dur: f = u/dur; break
        u -= dur
    ease = lambda x: 0.5 - 0.5*np.cos(np.pi*np.clip(x, 0, 1))
    st = dict(STATE, phase=name, beta=0.0)
    if name == 'arrive':
        g = ease(f); st.update(px=-RUN*(1 - g), sx=-40 - RUN*(1 - g), belt_m=RUN*g, belt_s=RUN*g)
    elif name == 'push':
        th = np.pi*(1 - ease(f)); e = float(np.clip(face_z(th) - 380, 0, TR))
        st.update(theta=th, e=e, d=e, belt_m=RUN, belt_s=RUN)
    elif name == 'retract':
        st.update(theta=-np.pi*ease(f), e=0.0, d=0.0, seated=True, belt_m=RUN, belt_s=RUN)
    elif name == 'sleds':
        g = ease(f); st.update(e=0.0, d=0.0, seated=True, sx=-40 + RUN*g, belt_m=RUN, belt_s=RUN*(1 + g))
    else:
        g = ease(f); st.update(e=0.0, d=0.0, seated=True, sx=-40 + RUN, px=RUN*g, belt_m=RUN*(1 + g), belt_s=2*RUN)
    return st

if __name__ == '__main__':
    import sys
    st = {}
    if len(sys.argv) > 1: st = cycle(float(sys.argv[1]))
    build(st); print(len(P), 'parts', st.get('phase'))
    rep = lcheck.run(P)
    print('PASS' if not rep['clashes'] and not rep['bad_fits'] and not rep['unconnected'] else 'FAIL')
