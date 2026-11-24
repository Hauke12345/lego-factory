"""MuJoCo test of the chain-station closing step (the 34th link).
State: 33 links on the loop (links 2..32 held, links 0 and 1 free, hinged 0-1 and 1-2), slot empty, link 0's free front
end (clips) at the slot's rear. A fork with two vertical bars (r 4, poking up 13 from the table in the links' bottom tunnel
at robot z = ZT) moves along robot +y: the inner bar lifts link 0's front out (it turns about its joint with link 1); the
pusher brings link 33 in radially and it snaps onto link 32 (hinge switched on); the fork moves back and the outer bar presses
link 0's front down onto link 33's rear bar. Reports link 0's angle and the closing joint's misalignment.
Frame: X = robot z, Y = robot y, Z up (table at Z 0). usage: python3 close_sim.py OUT.npz  (env: ZT, YI0, LIFT, MU)"""
import sys, os, numpy as np, mujoco
sys.path.insert(0, '/home/claude/scripts')
import chainst as cs
L = 0.0004; OUT = sys.argv[1]
MU = float(os.environ.get('MU', '0.25')); ZT = float(os.environ.get('ZT', '-56')); YI0 = float(os.environ.get('YI0', '15'))
LIFT = float(os.environ.get('LIFT', '13')); NFREE = int(os.environ.get('NFREE', '2')); GAPT = float(os.environ.get('GAPT', '17')); DT = 0.0002
P = cs.PITCH
FR = f'friction="{MU} 0.005 0.0001"'
def f(v): return ' '.join(f'{x:.6f}' for x in np.asarray(v, float))
def pose(u):                                   # sim link at u: origin (front, clips) at u, body back to u - P
    (y, z), t = cs.AT(u % cs.PER); return np.array([z, y]), np.arctan2(t[0], t[1])
def geoms(rgba):
    g = ''.join(f'<geom type="capsule" fromto="{f(np.array([-14.5, -5.5, h])*L)} {f(np.array([-1.5, -5.5, h])*L)}" size="{2.5*L}" mass="0.0001" {FR} rgba="{rgba}"/>' for h in (4, 26, 48))
    g += f'<geom type="box" pos="{f(np.array([-8, -5.5, 26])*L)}" size="{f(np.array([6.5, 2.5, 24])*L)}" mass="0.0001" {FR} rgba="{rgba}"/>'
    g += ''.join(f'<geom type="box" pos="{f(np.array([xc, 1, 26])*L)}" size="{f(np.array([2, 3, 10])*L)}" mass="0.0001" {FR} rgba="{rgba}"/>' for xc in (-2.0, -14.0))
    return g
def quat(a): return (np.cos(a/2), 0, 0, np.sin(a/2))
S0 = cs.S_SLOT
u_of = lambda j: S0 + (34 - j)*P                # sim placement of link j (adv 33); link 33 (X) at S0 + P
bodies = ''
for j in range(0, 33):
    p, a = pose(u_of(j))
    rg = '.9 .3 .2 1' if j == 0 else ('.3 .3 .9 1' if j == 1 else '.25 .25 .25 1')
    bodies += f'<body name="l{j}" pos="{f(np.array([p[0], p[1], 0.3])*L)}" quat="{f(quat(a))}">{"<freejoint/>" if j < NFREE else ""}{geoms(rg)}</body>'
pX, aX = pose(S0 + P)
bodies += f'<body name="lX" mocap="true" pos="{f(np.array([pX[0], pX[1] + 40, 0.3])*L)}" quat="{f(quat(aX))}">{geoms(".2 .8 .2 1")}</body>'
bodies += f'<body name="lXf" pos="{f(np.array([3000, 3000, 30])*L)}"><freejoint/>{geoms(".2 .8 .2 1")}</body>'
posts = ''.join(f'<geom type="cylinder" pos="{f(np.array([z, y, 25])*L)}" size="{10*L} {25*L}" {FR} rgba=".3 .5 .3 1"/>' for y, z, k in cs.GUIDES)
fork = (f'<body name="fork" mocap="true" pos="{f(np.array([ZT, YI0, 0])*L)}">'
        f'<geom type="capsule" fromto="0 0 {0.5*L} 0 0 {13*L}" size="{4*L}" {FR} rgba="1 .8 0 1"/>'
        f'<geom type="capsule" fromto="0 {GAPT*L} {0.5*L} 0 {GAPT*L} {13*L}" size="{4*L}" {FR} rgba="1 .8 0 1"/></body>')
eq = ''.join(f'<connect name="c01{k}" body1="l1" body2="l0" anchor="0 0 {h*L}"/>' + (f'<connect name="c12{k}" body1="l2" body2="l1" anchor="0 0 {h*L}"/>' if NFREE > 1 else '') +
             f'<connect name="cX{k}" body1="lXf" body2="l32" anchor="0 0 {h*L}" active="false"/>'
             f'<connect name="c0X{k}" body1="l0" body2="lXf" anchor="0 0 {h*L}" active="false"/>' for k, h in ((0, 10), (1, 42)))
xml = f'''<mujoco><size memory="200M"/><option timestep="{DT}" gravity="0 0 -9.81" cone="elliptic" impratio="10" integrator="implicitfast"/>
<worldbody><geom type="plane" size="2 2 .01" {FR} rgba=".85 .85 .85 1"/>{posts}{bodies}{fork}</worldbody><equality>{eq}</equality></mujoco>'''
m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m)
# connect anchors: link i's front (origin) to link i-1's rear (local -P): body1 = the link behind (front at its origin)
for nm in (('c01', 'c12', 'cX', 'c0X') if NFREE > 1 else ('c01', 'cX', 'c0X')):
    for k, h in ((0, 10), (1, 42)):
        e = m.equality(f'{nm}{k}').id; m.eq_data[e][0:3] = (0, 0, h*L); m.eq_data[e][3:6] = (-P*L, 0, h*L)
# careful: c01 joins l1 (behind) front to l0 (ahead)? link 0 is AHEAD of link 1 (higher u); l1's front (origin) meets l0's rear
mujoco.mj_forward(m, d)
bid = {n: m.body(n).id for n in ('l0', 'l1', 'lXf', 'l32')}
fk = m.body('fork').mocapid[0]; xk = m.body('lX').mocapid[0]
jq = m.jnt_qposadr[m.body('lXf').jntadr[0]]
def yaw(b): q = d.xquat[b]; return np.degrees(2*np.arctan2(q[3], q[0]))
a0 = yaw(bid['l0']); a1 = yaw(bid['l1']); rec = []; t = 0.0; log = []
def run(T, fn):
    global t
    for _ in range(int(T/DT)):
        fn(t); mujoco.mj_step(m, d); t += DT
        if int(t/DT) % 100 == 0:
            rec.append(np.r_[t, d.mocap_pos[fk]/L, yaw(bid['l0']) - a0, d.xpos[bid['l0']]/L, d.xpos[bid['lXf']]/L, yaw(bid['l1']) - a1, d.xpos[bid['l1']]/L])
def ez(x): return 0.5 - 0.5*np.cos(np.pi*np.clip(x, 0, 1))
run(0.05, lambda t: None)
t1 = t; run(0.4, lambda tt: d.mocap_pos.__setitem__(fk, np.array([ZT, YI0 + LIFT*ez((tt - t1)/0.35), 0])*L))
amax = yaw(bid['l0']) - a0; log.append(f'link 0 angle after the lift: {amax:.1f} deg, link 1 {yaw(bid["l1"]) - a1:.1f}, l0 pos {np.round(d.xpos[bid["l0"]]/L,1)}, fork y {d.mocap_pos[fk][1]/L:.1f}')
# link 33 comes in radially (pusher) as a kinematic body, then becomes free and snaps onto link 32
t2 = t; run(0.25, lambda tt: d.mocap_pos.__setitem__(xk, np.array([pX[0], pX[1] + 40*(1 - ez((tt - t2)/0.2)), 0.3])*L))
# minimum distance between the moving link 33 and link 0 during that stroke is checked via contacts
d.qpos[jq:jq + 3] = d.mocap_pos[xk]; d.qpos[jq + 3:jq + 7] = d.mocap_quat[xk]; d.mocap_pos[xk] = np.array([3000, 3000, 0])*L
for k in (0, 1): d.eq_active[m.equality(f'cX{k}').id] = 1
mujoco.mj_forward(m, d)
run(0.05, lambda t: None)
t3 = t; run(0.45, lambda tt: d.mocap_pos.__setitem__(fk, np.array([ZT, YI0 + LIFT*(1 - ez((tt - t3)/0.35)), 0])*L))
# misalignment of the closing joint: link 0's front (origin) vs link 33's rear (local -P)
def pt(b, loc):
    R = d.xmat[b].reshape(3, 3); return (d.xpos[b] + R @ (np.array(loc)*L))/L
err = np.linalg.norm(pt(bid['l0'], (0, 0, 26)) - pt(bid['lXf'], (-P, 0, 26)))
af = yaw(bid['l0']) - a0
log.append(f'link 0 angle after the press: {af:.1f} deg; closing joint misalignment {err:.2f} LDU (snap needs < ~2)')
np.savez(OUT, rec=np.array(rec))
print('ZT', ZT, 'YI0', YI0, 'LIFT', LIFT, 'MU', MU); print('\n'.join(log))
