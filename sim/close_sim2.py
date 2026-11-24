"""MuJoCo closing test, head region as a hinge chain with the REAL backbend limit of 3873 links (outward bend <= BB deg, from
the LDraw meshes: plates touch at ~12 deg). Links 0..NF-1 free (tree rooted at link NF, held), links NF..32 held, slot empty.
Fork: inner bar (in the links' bottom tunnel, under link 0's front) + outer bar GAPT further out, both r 4, poke up 13 from the
table at robot z = ZT, move along robot y. Steps: lift (+LIFT), link 33 pushed in radially (kinematic) and hinged to link 32,
fork back (press). Reports head lift, collisions of link 33 with link 0, closing misalignment.
usage: python3 close_sim2.py OUT.npz   env: ZT YI0 LIFT GAPT NF BB MU"""
import sys, os, numpy as np, mujoco
sys.path.insert(0, '/home/claude/scripts')
import chainst as cs
L = 0.0004; OUT = sys.argv[1]; E = os.environ.get
MU = float(E('MU', '0.25')); ZT = float(E('ZT', '-58.5')); YI0 = float(E('YI0', '15')); LIFT = float(E('LIFT', '16'))
GAPT = float(E('GAPT', '17')); NF = int(E('NF', '6')); BB = float(E('BB', '12')); DT = 0.0002; P = cs.PITCH
FR = f'friction="{MU} 0.005 0.0001"'
def f(v): return ' '.join(f'{x:.6f}' for x in np.asarray(v, float))
def pose(u): (y, z), t = cs.AT(u % cs.PER); return np.array([z, y]), np.arctan2(t[0], t[1])
def geoms(rg, nm=''):
    g = ''.join(f'<geom {nm} type="capsule" fromto="{f(np.array([-14.5, -5.5, h])*L)} {f(np.array([-1.5, -5.5, h])*L)}" size="{2.5*L}" mass="0.0001" {FR} rgba="{rg}"/>' for h in (4, 26, 48))
    g += f'<geom type="box" pos="{f(np.array([-8, -5.5, 26])*L)}" size="{f(np.array([6.5, 2.5, 24])*L)}" mass="0.0001" {FR} rgba="{rg}"/>'
    g += ''.join(f'<geom type="box" pos="{f(np.array([xc, 1, 26])*L)}" size="{f(np.array([2, 3, 10])*L)}" mass="0.0001" {FR} rgba="{rg}"/>' for xc in (-2.0, -14.0))
    return g
def quat(a): return np.array([np.cos(a/2), 0, 0, np.sin(a/2)])
S0 = cs.S_SLOT; u_of = lambda j: S0 + (34 - j)*P
W = {j: pose(u_of(j)) for j in range(33)}
def rel(j, parent):                                  # pose of link j in its parent's frame
    (p, a), (pp, ap) = W[j], W[parent]
    c, s = np.cos(-ap), np.sin(-ap); d = p - pp; return np.array([c*d[0] - s*d[1], s*d[0] + c*d[1]]), a - ap
def tree(j):                                         # links j, j-1, ..., 0 (each the parent of the next one ahead)
    if j < 0: return ''
    r, a = rel(j, j + 1)
    rg = '.9 .3 .2 1' if j == 0 else '.3 .3 .9 1'
    return (f'<body name="l{j}" pos="{f(np.array([r[0], r[1], 0])*L)}" quat="{f(quat(a))}">'
            f'<joint name="h{j}" type="hinge" axis="0 0 1" pos="{f(np.array([-P, 0, 26])*L)}" range="{-BB} 60" limited="true" damping="2e-7" armature="1e-9"/>'
            f'{geoms(rg)}{tree(j - 1)}</body>')
bodies = ''
for j in range(NF, 33):
    p, a = W[j]
    bodies += f'<body name="l{j}" pos="{f(np.array([p[0], p[1], 0.3])*L)}" quat="{f(quat(a))}">{geoms(".25 .25 .25 1")}{tree(j - 1) if j == NF else ""}</body>'
pX, aX = pose(S0 + P)
bodies += f'<body name="lX" mocap="true" pos="{f(np.array([pX[0], pX[1] + 40, 0.3])*L)}" quat="{f(quat(aX))}">{geoms(".2 .8 .2 1")}</body>'
bodies += f'<body name="lXf" pos="{f(np.array([3000, 3000, 30])*L)}"><freejoint/>{geoms(".2 .8 .2 1")}</body>'
DROP = os.environ.get('DROP', '1') == '1'
posts = ''.join(f'<geom type="cylinder" pos="{f(np.array([z, y, 25])*L)}" size="{10*L} {25*L}" {FR} rgba=".3 .5 .3 1"/>' for y, z, k in cs.GUIDES
                if not (DROP and y > 20 and -40 < z < 75))
spr = f'<geom type="cylinder" pos="{f(np.array([-80, -11, 26])*L)}" size="{27*L} {9*L}" {FR} rgba=".6 .6 .6 1"/>'
fork = (f'<body name="fork" mocap="true" pos="{f(np.array([ZT, YI0, 0])*L)}">'
        f'<geom type="capsule" fromto="0 0 {0.5*L} 0 0 {13*L}" size="{4*L}" {FR} rgba="1 .8 0 1"/>'
        f'<geom type="capsule" fromto="0 {GAPT*L} {0.5*L} 0 {GAPT*L} {13*L}" size="{4*L}" {FR} rgba="1 .8 0 1"/></body>')
eq = ''.join(f'<connect name="cX{k}" body1="lXf" body2="l32" anchor="0 0 {h*L}" active="false"/>' for k, h in ((0, 10), (1, 42)))
xml = f'''<mujoco><size memory="200M"/><option timestep="{DT}" gravity="0 0 -9.81" cone="elliptic" impratio="10" integrator="implicitfast"/>
<worldbody><geom type="plane" size="2 2 .01" {FR} rgba=".85 .85 .85 1"/>{posts}{spr}{bodies}{fork}</worldbody><equality>{eq}</equality></mujoco>'''
m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m)
for k, h in ((0, 10), (1, 42)):
    e = m.equality(f'cX{k}').id; m.eq_data[e][0:3] = (0, 0, h*L); m.eq_data[e][3:6] = (-P*L, 0, h*L)
mujoco.mj_forward(m, d)
B = lambda n: m.body(n).id
fk = m.body('fork').mocapid[0]; xk = m.body('lX').mocapid[0]; jq = m.jnt_qposadr[m.body('lXf').jntadr[0]]
def pt(b, loc): R = d.xmat[b].reshape(3, 3); return (d.xpos[b] + R @ (np.array(loc)*L))/L
gX = set(range(m.body_geomadr[B('lX')], m.body_geomadr[B('lX')] + m.body_geomnum[B('lX')]))
g0 = set(range(m.body_geomadr[B('l0')], m.body_geomadr[B('l0')] + m.body_geomnum[B('l0')]))
t = 0.0; rec = []; hitX0 = [0.0]
def run(T, fn):
    global t
    for _ in range(int(T/DT)):
        fn(t); mujoco.mj_step(m, d); t += DT
        for c in d.contact[:d.ncon]:
            if (c.geom1 in gX and c.geom2 in g0) or (c.geom2 in gX and c.geom1 in g0): hitX0[0] = min(hitX0[0], c.dist/L)
        if int(round(t/DT)) % 100 == 0:
            rec.append(np.r_[t, d.mocap_pos[fk][1]/L, [d.qpos[m.jnt_qposadr[m.joint(f'h{j}').id]] for j in range(NF)], pt(B('l0'), (0, 0, 26))[:2]])
def ez(x): return 0.5 - 0.5*np.cos(np.pi*np.clip(x, 0, 1))
run(0.05, lambda t: None)
t1 = t; run(0.4, lambda tt: d.mocap_pos.__setitem__(fk, np.array([ZT, YI0 + LIFT*ez((tt - t1)/0.35), 0])*L))
fr = pt(B('l0'), (0, 0, 26)); bends = [np.degrees(d.qpos[m.jnt_qposadr[m.joint(f'h{j}').id]]) for j in range(NF)]
lift = fr[1] - 19
print(f'ZT {ZT} LIFT {LIFT} NF {NF} BB {BB}: after the lift link 0 front at (z {fr[0]:.1f}, y {fr[1]:.1f}) = {lift:.1f} out; joint bends {np.round(bends, 1)}')
t2 = t; run(0.25, lambda tt: d.mocap_pos.__setitem__(xk, np.array([pX[0], pX[1] + 40*(1 - ez((tt - t2)/0.2)), 0.3])*L))
print(f'  link 33 pushed in: deepest overlap with link 0 {hitX0[0]:.2f} LDU (negative = they collide)')
d.qpos[jq:jq + 3] = d.mocap_pos[xk]; d.qpos[jq + 3:jq + 7] = d.mocap_quat[xk]; d.mocap_pos[xk] = np.array([3000, 3000, 0])*L
for k in (0, 1): d.eq_active[m.equality(f'cX{k}').id] = 1
mujoco.mj_forward(m, d); run(0.05, lambda t: None)
snapped = [None]
def press(tt):
    d.mocap_pos[fk] = np.array([ZT, YI0 + LIFT*(1 - ez((tt - t3)/0.35)), 0])*L
    if snapped[0] is None:
        e_ = np.linalg.norm((pt(B('l0'), (0, 0, 26)) - pt(B('lXf'), (-P, 0, 26)))[:2])
        if e_ < 2.5: snapped[0] = (tt - t3, e_)
t3 = t; run(0.45, press)
print('  snap (front clips within 2.5 LDU of the bar) at', snapped[0])
err = pt(B('l0'), (0, 0, 26)) - pt(B('lXf'), (-P, 0, 26))
bends = [np.degrees(d.qpos[m.jnt_qposadr[m.joint(f'h{j}').id]]) for j in range(NF)]
print(f'  after the press: closing joint misalignment {np.linalg.norm(err[:2]):.2f} LDU (dz {err[0]:.1f}, dy {err[1]:.1f}); bends {np.round(bends, 1)}')
np.savez(OUT, rec=np.array(rec))
