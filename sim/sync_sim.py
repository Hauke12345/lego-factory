"""MuJoCo concept test: chain built link by link at the slot while ALL THREE sprockets turn together (coupled 1:1 like the
under-table Technic chain), the tread running in a guide channel (outer guide all round, inner guide on the straights).
Geometry: the station's sprocket centres; arcs use the pin-circle radius so the 12 pins meet every link pocket exactly.
Links: tread plate (three stacked capsules, rounded ends) + two inner blocks leaving a tooth pocket; joints switched on as
hinges (two connect constraints) the moment a link is pushed into the slot (= the snap).
X = robot z, Y = robot y, Z up; 1 LDU = 0.4 mm.   usage: python3 sync_sim.py MODE OUT.npz      MODE: sync | single"""
import sys, os, numpy as np, mujoco
sys.path.insert(0, '/home/claude/scripts')
import chainst as cs

L = 0.0004
MODE, OUT = sys.argv[1], sys.argv[2]
MU = float(os.environ.get('MU', '0.25')); DT = float(os.environ.get('DT', '0.0005')); NL = int(os.environ.get('NL', '33'))
TQ = float(os.environ.get('TQ', '0.040'))
P = cs.PITCH; RP = 12*P/(2*np.pi)
TL, TI = 0.68, 0.18
FR = f'friction="{MU} 0.005 0.0001"'
def f(v): return ' '.join(f'{x:.6f}' for x in np.asarray(v, float))
C = {'drv': np.array([cs.DRV[1], cs.DRV[0]]), 'idp': np.array([cs.IDL[0][1], cs.IDL[0][0]]), 'idm': np.array([cs.IDL[1][1], cs.IDL[1][0]])}

# ---- loop path (X, Y) with arc radius RP, running like the station: bottom run towards -X, round idm, up to drv, down to idp
def build_path():
    order = ['idm', 'drv', 'idp']                        # clockwise in X-Y with Y down? computed with outer tangents
    cen = [C[k] for k in order]; ctr = np.mean(cen, axis=0); segs = []
    def tang(a, b):                                      # outer tangent between equal circles a -> b
        d = (b - a)/np.linalg.norm(b - a); n = np.array([d[1], -d[0]])
        if np.dot(n, a - ctr) < 0: n = -n
        return a + RP*n, b + RP*n, n
    T = [tang(cen[i], cen[(i + 1) % 3]) for i in range(3)]
    pts = []
    for i in range(3):
        p0, p1, n = T[i]; pts.append(('line', p0, p1))
        c = cen[(i + 1) % 3]; nn = T[(i + 1) % 3][2]
        a0 = np.arctan2(*(p1 - c)[::-1]); a1 = np.arctan2(*(T[(i + 1) % 3][0] - c)[::-1])
        pts.append(('arc', c, a0, a1))
    return pts
PATH = build_path()
def seglen(s):
    if s[0] == 'line': return np.linalg.norm(s[2] - s[1])
    da = (s[3] - s[2]) % (2*np.pi)
    if da > np.pi: da -= 2*np.pi
    return abs(da)*RP
LENS = [seglen(s) for s in PATH]; PER = sum(LENS)
def at(u):
    u %= PER
    for s, l in zip(PATH, LENS):
        if u <= l:
            if s[0] == 'line':
                d = (s[2] - s[1])/l; return s[1] + d*u, d
            da = (s[3] - s[2]) % (2*np.pi)
            if da > np.pi: da -= 2*np.pi
            a = s[2] + np.sign(da)*u/RP; p = s[1] + RP*np.array([np.cos(a), np.sin(a)])
            t = np.sign(da)*np.array([-np.sin(a), np.cos(a)]); return p, t
        u -= l
    return at(0)
# direction check: the station's chain runs -X along the bottom run (Y = 19 side)
pb, tb = at(LENS[0]*0 + 1e-3)
# slot: on the bottom run, one pitch before the tangent point onto idm (link body spans [s - P, s])
bottom = [i for i, s in enumerate(PATH) if s[0] == 'line' and abs(s[1][1] - s[2][1]) < 1e-6][0]
s_bot0 = sum(LENS[:bottom]); line = PATH[bottom]
going_negX = (line[2] - line[1])[0] < 0
S_SLOT = s_bot0 + LENS[bottom] if going_negX else None   # end of the bottom run = tangent point onto idm
assert going_negX, 'path runs the wrong way'
def pose(u):
    p, t = at(u); return p, np.arctan2(t[1], t[0])

# ---- model
def link_xml(i):
    g = ''.join(f'<geom type="capsule" fromto="{f(np.array([-14.5, -5.5, h])*L)} {f(np.array([-1.5, -5.5, h])*L)}" size="{2.5*L}" mass="0.0001" {FR}/>'
                for h in (8, 26, 44))
    g += f'<geom type="box" pos="{f(np.array([-8, -5.5, 26])*L)}" size="{f(np.array([6.5, 2.5, 20])*L)}" mass="0.0001" {FR}/>'
    g += ''.join(f'<geom type="box" pos="{f(np.array([xc, 1, 26])*L)}" size="{f(np.array([2, 3, 24])*L)}" mass="0.0001" {FR}/>' for xc in (-2.0, -14.0))
    return f'<body name="l{i}" pos="{f(np.array([2000 + 40*i, 400, 30])*L)}"><freejoint/>{g}</body>'
# local +y must be OUTWARD: at the slot +s = -X, outward = +Y  -> left of (-X) is -Y, so flip y in the geoms
def fixy(xml): return xml   # handled by the 180 deg roll in place()
def wall(pts, h, name):
    return ''.join(f'<geom type="capsule" fromto="{f(np.array([a[0], a[1], h])*L)} {f(np.array([b[0], b[1], h])*L)}" size="{1.2*L}" '
                   f'{FR} rgba=".2 .5 .2 1" contype="2" conaffinity="1"/>' for a, b in zip(pts[:-1], pts[1:]))
chan = ''
us = np.arange(0, PER + 2, 2.0)
outer = []; inner = []
for u in us:
    p, t = at(u); n = np.array([t[1], -t[0]])                 # right of travel
    c = min(C.values(), key=lambda q: np.linalg.norm(q - p)); n = n if np.dot(n, p - c) > 0 else -n
    on_arc = abs(np.linalg.norm(p - c) - RP) < 0.3
    slot_zone = False          # the pusher pad and the mouth guides hold the link in the slot; insertion itself is not simulated
    outer.append(None if slot_zone else p + n*(8 + 2 + 1.2))
    inner.append(None if (on_arc or slot_zone) else p - n*(5 + 2 + 1.2))
def runs(lst):
    out, cur = [], []
    for q in lst:
        if q is None:
            if len(cur) > 1: out.append(cur)
            cur = []
        else: cur.append(q)
    if len(cur) > 1: out.append(cur)
    return out
for r in runs(outer):
    for h in (6, 18, 30): chan += wall(r, h, 'o')
for r in runs(inner):
    for h in (6, 18): chan += wall(r, h, 'i')
def sprocket(name, driven, th0):
    c = C[name]
    s = (f'<body name="{name}" pos="{f(np.array([c[0], c[1], 0])*L)}"><joint name="{name}j" type="hinge" axis="0 0 1" damping="1e-6" armature="1e-3"/>'
         f'<geom type="cylinder" pos="0 0 {26*L}" size="{24*L} {9*L}" mass="0.002" {FR} rgba=".7 .7 .7 1" contype="4" conaffinity="1"/>')
    if driven:
        for k in range(12):
            a = th0 + np.radians(30*k)
            s += (f'<geom type="capsule" fromto="{f(np.array([RP*np.cos(a), RP*np.sin(a), 16])*L)} {f(np.array([RP*np.cos(a), RP*np.sin(a), 36])*L)}" '
                  f'size="{1.5*L}" mass="0.0001" {FR} rgba=".6 .6 .6 1" contype="4" conaffinity="1"/>')
    return s + '</body>'
# pin phases: pockets sit 8 behind each joint; joints at S_SLOT + k*P (they move +s as the chain advances)
def pin_phase(name):
    c = C[name]
    for k in range(1, 34):                                # first pocket (time 0, virtual chain ahead of the slot) on this arc
        p, _ = at(S_SLOT + k*P - 8)
        if abs(np.linalg.norm(p - c) - RP) < 1e-3: return np.arctan2(p[1] - c[1], p[0] - c[0])
    raise ValueError(name)
drv_all = MODE == 'sync'
spr = sprocket('idm', True, pin_phase('idm')) + sprocket('drv', drv_all, pin_phase('drv')) + sprocket('idp', drv_all, pin_phase('idp'))
eqs = ''.join(f'<connect name="c{i}{k}" body1="l{i}" body2="l{i-1}" anchor="0 0 {h*L:.6f}" active="false"/>'
              for i in range(1, NL) for k, h in ((0, 10), (1, 42)))
if drv_all: eqs += '<joint joint1="drvj" joint2="idmj" polycoef="0 1 0 0 0"/><joint joint1="idpj" joint2="idmj" polycoef="0 1 0 0 0"/>'
xml = f'''<mujoco><size memory="400M"/><option timestep="{DT}" gravity="0 0 -9.81" cone="elliptic" impratio="10" integrator="implicitfast"/>
<worldbody><light pos="0 0 1" dir="0 0 -1"/><geom type="plane" size="2 2 .01" {FR} rgba=".85 .85 .85 1" contype="2" conaffinity="1"/>
{chan}{spr}{''.join(link_xml(i) for i in range(NL))}</worldbody>
<equality>{eqs}</equality>
<actuator><velocity joint="idmj" kv="1" forcelimited="true" forcerange="-{TQ} {TQ}"/></actuator></mujoco>'''
m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m)
jq = [m.jnt_qposadr[m.body(f'l{i}').jntadr[0]] for i in range(NL)]; jv = [m.jnt_dofadr[m.body(f'l{i}').jntadr[0]] for i in range(NL)]
bid = [m.body(f'l{i}').id for i in range(NL)]
def place(i, u):
    """link frame: x = travel (+s), y = left of travel (= inward on this loop), z up; plate on the right (outside)."""
    p, a = pose(u)
    d.qpos[jq[i]:jq[i] + 3] = (p[0]*L, p[1]*L, 0.0005*L); d.qpos[jq[i] + 3:jq[i] + 7] = (np.cos(a/2), 0, 0, np.sin(a/2))
    d.qvel[jv[i]:jv[i] + 6] = 0
W = (P/RP)/(TL - TI)
rec = []; k = 0
for j in range(NL + 1):
    if j < NL:
        place(j, S_SLOT)
        if j > 0:
            for kk, h in ((0, 10), (1, 42)):
                e = m.equality(f'c{j}{kk}').id; hh = h if True else h
                m.eq_data[e][0:3] = (0, 0, hh*L); m.eq_data[e][3:6] = (-P*L, 0, hh*L); d.eq_active[e] = 1
        mujoco.mj_forward(m, d)
    for phase, dur in (('ins', TI), ('adv', TL - TI)):
        for _ in range(int(dur/DT)):
            d.ctrl[0] = W if phase == 'adv' else 0.0
            mujoco.mj_step(m, d); k += 1
            if k % 200 == 0: rec.append(np.array([d.xpos[b].copy() for b in bid]))
pos = np.array(rec)/L
# expected joint positions after the last advance: link j at S_SLOT + (NL - j)*P
exp = np.array([at(S_SLOT + (NL - j)*P)[0] for j in range(NL)])
err = np.linalg.norm(pos[-1][:, :2] - exp, axis=1)
pp = np.array([at(u)[0] for u in np.linspace(0, PER, 800)])
np.savez(OUT, pos=pos, dt=200*DT, path=pp, exp=exp)
print(MODE, 'MU', MU, 'TQ', TQ, 'links', NL, '| joint position error per link [LDU]:', np.round(err, 1))
print('links within 3 LDU of their place:', int((err < 3).sum()), '/', NL, '| loop length', round(PER, 1), 'vs 34 pitches', round(34*P, 1))
