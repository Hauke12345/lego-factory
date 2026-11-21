"""Simplified physical test of the Lego bridge (MuJoCo).
Every part is a rigid body (convex hull of its shape, ABS-ish density). Real connections become constraints:
  studs + locking click hinges -> weld,  Technic pins and the pier hinge -> pivot (free to rotate, worst case).
Parts rest (unattached) on the box, book and table. Then the 107 g truck drives across.
Reports sag, settling, the most loaded connections (vs. a conservative 1 N clutch per stud) and the
static load the bridge can take at mid-span."""
import json, math, sys, numpy as np, mujoco
sys.path.insert(0, '/home/claude/scripts'); import bridge as BR
D = json.load(open('/home/claude/render_b/build_check.json'))
S, P, B = BR.S, BR.P, BR.B
TRUCK = 'notruck' not in sys.argv
LOCK = 'lock' in sys.argv          # click hinges as welds (default: free pivots = worst case for the deck)
F_CLUTCH = 1.0                         # N per stud, conservative pull-off
RHO = 550                              # kg/m3 effective (hollow ABS parts)
SC, A0 = 0.044, 9.0
RECTS = {'box': [(4.79, 6.88), (4.92, 0.46), (12.17, 0.60), (12.04, 7.02)],
         'book': [(6.27, -2.78), (6.55, -8.02), (11.89, -7.74), (11.61, -2.5)]}
parts = {n: p for n, p in D['parts'].items() if p['group'] != 'truck'}
names = list(parts)
def mquat(M):
    R = np.array(M)[:3, :3]; q = np.zeros(4); mujoco.mju_mat2Quat(q, R.flatten()); return q
assets, bodies = [], []
for n in names:
    p = parts[n]; v = np.array(p['verts'])
    assets.append(f'<mesh name="m_{n}" vertex="{" ".join(f"{x:.6f}" for x in v.flatten())}"/>')
    M = np.array(p['M']); q = mquat(M)
    bodies.append(f'''<body name="{n}" pos="{M[0,3]:.6f} {M[1,3]:.6f} {M[2,3]:.6f}" quat="{q[0]:.7f} {q[1]:.7f} {q[2]:.7f} {q[3]:.7f}">
      <freejoint/><geom type="mesh" mesh="m_{n}" density="{RHO}" contype="1" conaffinity="2" friction="0.6"/></body>''')
def cub(name, rect, z):
    vs = [(SC*(a-A0), SC*b, zz) for zz in (0, z) for a, b in rect]
    return f'<mesh name="{name}" vertex="{" ".join(f"{x:.5f} {y:.5f} {z:.5f}" for x, y, z in vs)}"/>'
assets += [cub('boxm', RECTS['box'], BR.Z_BOX), cub('bookm', RECTS['book'], BR.Z_BOOK)]
# connections
eqs, eqinfo, seen = [], [], {}
Mw = {n: np.array(parts[n]['M']) for n in names}
for c in D['conns']:
    a, b = c['a'], c['b']
    if a not in parts or b not in parts: continue
    if c['kind'] == 'stud' or (c['kind'] == 'hinge-lock' and LOCK):
        key = tuple(sorted((a, b)))
        if key in seen: eqinfo[seen[key]]['n'] += c['n']; continue
        seen[key] = len(eqinfo)
        eqs.append(f'<weld body1="{a}" body2="{b}" solref="0.0008 1" solimp="0.995 0.9999 0.0001"/>')
        eqinfo.append(dict(a=a, b=b, kind=c['kind'], n=c['n'], rows=6))
    else:                                                     # pin / pier hinge: pivot about world x
        for pt in c['pts']:
            for dx in (-0.003, 0.003):
                pw = np.array(pt) + [dx, 0, 0]
                loc = np.linalg.inv(Mw[a]) @ np.append(pw, 1)
                eqs.append(f'<connect body1="{a}" body2="{b}" anchor="{loc[0]:.6f} {loc[1]:.6f} {loc[2]:.6f}" solref="0.0008 1" solimp="0.995 0.9999 0.0001"/>')
                eqinfo.append(dict(a=a, b=b, kind=c['kind'], n=1, rows=3))
# truck (same simplified model as the drive sim)
WR, WW = 0.0145, 0.009; Z_TB = WR - 0.0058; Z_CH = Z_TB + B
AX_F, AX_R = 5*S, -4*S; WY = 3*S + 0.0015 + WW/2; ztop = Z_CH + P + 3*B + P
wheels = ''.join(f'''<body name="w{i}" pos="{x:.4f} {y:.4f} {WR}"><joint name="j{i}" type="hinge" axis="0 1 0" damping="1e-6"/>
      <geom type="cylinder" size="{WR} {WW/2}" euler="90 0 0" mass="0.005" friction="1.0 0.005 0.0001" contype="2" conaffinity="3"/></body>'''
                 for i, (x, y) in enumerate([(AX_F, WY), (AX_F, -WY), (AX_R, WY), (AX_R, -WY)]))
truck = f'''<body name="truck" pos="0 0.20 {BR.Z_BOX + 0.0003}" quat="0.70710678 0 0 -0.70710678"><freejoint/>
      <geom type="box" pos="0 0 {(Z_CH+ztop)/2:.4f}" size="{8*S:.4f} {3*S:.4f} {(ztop-Z_CH)/2:.4f}" mass="0.075" contype="2" conaffinity="3"/>
      <geom type="box" pos="0 {2.5*S:.4f} {Z_TB+B/2:.4f}" size="{7*S:.4f} {S/2:.4f} {B/2:.4f}" mass="0.006" contype="2" conaffinity="3"/>
      <geom type="box" pos="0 {-2.5*S:.4f} {Z_TB+B/2:.4f}" size="{7*S:.4f} {S/2:.4f} {B/2:.4f}" mass="0.006" contype="2" conaffinity="3"/>
      {wheels}</body>''' if TRUCK else ''
act = ''.join(f'<velocity joint="j{i}" kv="0.002" forcerange="-0.006 0.006"/>' for i in range(4)) if TRUCK else ''
xml = f'''<mujoco><option timestep="0.0001" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default><geom solref="0.002 1" solimp="0.95 0.99 0.0005" condim="3"/></default>
  <asset>{"".join(assets)}</asset>
  <worldbody>
    <geom type="plane" size="2 2 0.1" contype="2" conaffinity="1" friction="0.5"/>
    <geom type="mesh" mesh="boxm" contype="2" conaffinity="1" friction="0.8"/>
    <geom type="mesh" mesh="bookm" contype="2" conaffinity="1" friction="0.6"/>
    {"".join(bodies)}
    {truck}
  </worldbody>
  <equality>{"".join(eqs)}</equality>
  <actuator>{act}</actuator>
</mujoco>'''
m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
bid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in names}
x0 = {n: d.xpos[bid[n]].copy() for n in names}
mass = sum(m.body_mass[bid[n]] for n in names)
print(f'bridge: {len(names)} rigid parts, {mass*1000:.0f} g, {len(eqinfo)} constraints ({sum(e["kind"] in ("stud","hinge-lock") for e in eqinfo)} welds)')
tb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'truck') if TRUCK else -1
m.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
F_SHEAR = 15.0                         # N per stud in shear (studs are strong sideways)
dofadr = {n: m.jnt_dofadr[m.body_jntadr[bid[n]]] for n in names}
# stud-field centroid of each weld, in the upper part's local frame
for e in eqinfo:
    if e['rows'] == 6:
        c = [c_ for c_ in D['conns'] if {c_['a'], c_['b']} == {e['a'], e['b']} and c_['kind'] == 'stud']
        P_ = np.array([p for c_ in c for p in c_['pts']]) if c else np.array([x0[e['b']]])
        e['cw'] = np.append(P_.mean(0), 1)
        e['up'] = c[0]['b'] if c else e['b']          # part carrying the anti-studs (sits on top)
def util():
    """utilisation per weld: (tension + peel) vs clutch, shear vs stud shear"""
    J = d.efc_J.reshape(d.nefc, m.nv); u = np.zeros(len(eqinfo))
    rows = {}
    for r in range(d.nefc):
        if d.efc_type[r] == mujoco.mjtConstraint.mjCNSTR_EQUALITY: rows.setdefault(d.efc_id[r], []).append(r)
    for i, e in enumerate(eqinfo):
        if e['rows'] != 6 or i not in rows: continue
        top = e['up']; k = dofadr[top]; rr = rows[i]
        g = J[rr][:, k:k+6].T @ d.efc_force[rr]          # generalized force on the top part's free joint
        Fw = g[:3]; R = d.xmat[bid[top]].reshape(3, 3); Tw = R @ g[3:]
        c = (np.linalg.inv(np.array(parts[top]['M'])) @ e['cw'])[:3]
        cw = d.xpos[bid[top]] + R @ c                       # centroid now (world)
        Tc = Tw + np.cross(d.xipos[bid[top]] - cw, Fw)      # moment about the stud field
        upv = R[:, 2]
        ten = max(0.0, -Fw @ upv)                           # connection pulling the top part down = studs in tension
        peel = np.linalg.norm(Tc - upv*(Tc @ upv))
        shear = np.linalg.norm(Fw - upv*(Fw @ upv))
        n = e['n']; r_ = 0.5*math.sqrt(n)*S
        u[i] = max((ten + peel/r_)/(n*F_CLUTCH), shear/(n*F_SHEAR))
    return u
def eq_loads():
    F = np.zeros((len(eqinfo), 6))
    for r in range(d.nefc):
        if d.efc_type[r] == mujoco.mjtConstraint.mjCNSTR_EQUALITY:
            e = d.efc_id[r]; k = r - np.argmax((d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY) & (d.efc_id == e))
            F[e, k] = d.efc_force[r]
    return F
mid = 'road12'
zmid0 = x0[mid][2]
maxdisp = {n: 0.0 for n in names}; umax = np.zeros(len(eqinfo)); sag = 0; pin_f = 0
T = 3.2 if TRUCK else 1.0
steps = int(T/m.opt.timestep); trace = []
for k in range(steps):
    t = k*m.opt.timestep - 0.5
    if TRUCK:
        v = 0.0 if t < 0 else 0.5*min(1, t/0.3)
        if d.xpos[tb][1] < -0.235: v = 0.0
        d.ctrl[:] = v/WR
    mujoco.mj_step(m, d)
    if k % 50 == 0 and t > -0.3:
        F = eq_loads(); u = util(); umax = np.maximum(umax, u)
        pin_f = max(pin_f, max((np.linalg.norm(F[i, :3]) for i, e in enumerate(eqinfo) if e['rows'] == 3), default=0))
        for n in names: maxdisp[n] = max(maxdisp[n], np.linalg.norm(d.xpos[bid[n]] - x0[n]))
        sag = min(sag, d.xpos[bid[mid]][2] - zmid0)
        if TRUCK and k % 500 == 0: trace.append((round(t, 2), round(float(d.xpos[tb][1]), 3), round(float((d.xpos[bid[mid]][2] - zmid0)*1000), 3)))
for n in ('appA1a', 'appH1', 'dH01', 'dF10', 'pierFoot', 'pier6', 'pierTop0', 'road12', 'dH11', 'landH1', 'landA1a'):
    print(f'   {n:<10} moved', np.round((d.xpos[bid[n]] - x0[n])*1000, 2), 'mm')
print(f'\nmax movement of any part: {max(maxdisp.values())*1000:.2f} mm ({max(maxdisp, key=maxdisp.get)})')
print(f'mid-span sag (max): {-sag*1000:.3f} mm')
if TRUCK:
    print('truck y / mid-span sag mm over time:', trace)
    print(f'truck end position y = {d.xpos[tb][1]:.3f}, z = {d.xpos[tb][2]:.4f}')
order = np.argsort(-umax)[:8]
print(f'\nmost loaded connections (utilisation vs {F_CLUTCH} N/stud clutch; <1 holds):')
for i in order:
    e = eqinfo[i]; print(f'  {umax[i]:5.2f}  {e["kind"]:<10} {e["a"]:<12} <-> {e["b"]:<12} ({e["n"]} studs)')
print(f'max pin / hinge force: {pin_f:.3f} N')
