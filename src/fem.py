"""Linear 3D frame FEM of the Lego bridge.
Each part -> Euler-Bernoulli beam elements along its long axis between its connection points
(hollow-section ABS), stiff offset links to the actual connection points.
Each connection -> zero-length 6-DOF spring: studs (axial / shear / peel / twist by stud count),
Technic pins & hinges (free rotation about the axis). Parts resting on box / book / table -> supports.
Load cases: self-weight + the 107 g truck at 34 positions along the road.
Outputs render_b/fem.json: per case per part utilisation (stress vs ABS allowable, connections vs clutch),
reactions, deflection; prints the envelope and parts that carry (almost) no load."""
import json, math, sys, os, numpy as np
import scipy.sparse as sp, scipy.sparse.linalg as spl
sys.path.insert(0, '/home/claude/scripts'); import bridge as BR
D = json.load(open(sys.argv[1] if len(sys.argv) > 1 else '/home/claude/render_b/build_check.json'))
S = BR.S
E, G = 2.2e9, 0.8e9                 # ABS
SIG_ALLOW = 30e6                    # Pa, conservative allowable stress for ABS
F_CLUTCH, F_SHEAR = 1.0, 15.0       # N per stud: pull-off (conservative) / shear
K_AX, K_SH = 3e5, 6e5               # N/m per stud
RHO = 550                           # kg/m3 effective
g = 9.81
TRUCK_M = 0.107
parts = {n: p for n, p in D['parts'].items() if p['group'] != 'truck'}
names = list(parts)
M4 = {n: np.array(parts[n]['M']) for n in names}
R3 = {n: M4[n][:3, :3] for n in names}
lo = {n: np.array(parts[n]['bbox'][0]) for n in names}; hi = {n: np.array(parts[n]['bbox'][1]) for n in names}
ctr = {n: M4[n][:3, :3] @ ((lo[n] + hi[n])/2) + M4[n][:3, 3] for n in names}
ext = {n: hi[n] - lo[n] for n in names}

nodes = []                           # xyz
def node(p): nodes.append(np.asarray(p, float)); return len(nodes) - 1
elems = []                           # (type, i, j, data)
part_nodes = {n: [] for n in names}  # (t, node) along axis
conn_node = {}                       # (part, conn index) -> node

# ---- connection points per part ----
cpts = {n: [] for n in names}
conns = [c for c in D['conns'] if c['a'] in parts and c['b'] in parts]
for ci, c in enumerate(conns):
    p = np.mean(np.array(c['pts']), 0)
    for side in ('a', 'b'):
        n = c[side]; q = p.copy()
        if c['kind'] != 'stud':                     # move the pin/hinge point onto this part's mid-plane
            ax = R3[n][:, 1] if c['kind'] == 'pin' else np.array([1.0, 0, 0])
            q = q + ax*((ctr[n] - q) @ ax)
        cpts[n].append((ci, q))
# ---- beams per part ----
def section(n):
    e = sorted(range(3), key=lambda k: -ext[n][k]); a = e[0]
    b, h = ext[n][e[1]], ext[n][e[2]]
    A = 0.5*b*h; Iy = 0.35*b*h**3/12; Iz = 0.35*h*b**3/12; J = 0.3*(Iy + Iz)
    return a, e[1], A, Iy, Iz, J, b, h
sec = {}
for n in names:
    a, bdir, A, Iy, Iz, J, b, h = section(n); sec[n] = (A, Iy, Iz, J, b, h)
    ax = R3[n][:, a]; ref = R3[n][:, bdir]
    ts = [((q - ctr[n]) @ ax, ci, q) for ci, q in cpts[n]] + [(0.0, None, ctr[n])]
    ts.sort(key=lambda x: x[0])
    axis_nodes = []
    for t, ci, q in ts:
        if not axis_nodes or t - axis_nodes[-1][0] > 0.0005: axis_nodes.append((t, node(ctr[n] + ax*t)))
        an = axis_nodes[-1][1]
        if ci is None: part_nodes[n].append(('c', an)); continue
        if np.linalg.norm(q - nodes[an]) > 0.0003:
            cn = node(q); elems.append(('rigid', an, cn, (n, ref))); conn_node[(n, ci)] = cn
        else: conn_node[(n, ci)] = an
    for (t0, i), (t1, j) in zip(axis_nodes, axis_nodes[1:]):
        elems.append(('beam', i, j, (n, ref)))
    part_nodes[n] += [('a', i) for _, i in axis_nodes]
centroid_node = {n: [i for k, i in part_nodes[n] if k == 'c'][0] for n in names}
# ---- connection springs ----
for ci, c in enumerate(conns):
    i, j = conn_node[(c['a'], ci)], conn_node[(c['b'], ci)]
    if c['kind'] == 'stud':
        nst = c['n']; r = 0.5*math.sqrt(nst)*S
        Rf = R3[c['b']]                                  # z = stud axis
        k = np.array([nst*K_SH, nst*K_SH, nst*K_AX, nst*K_AX*r*r, nst*K_AX*r*r, (nst*K_SH*r*r if nst > 1 else 1e-4)])
    else:
        ax = R3[c['a']][:, 1] if c['kind'] == 'pin' else np.array([1.0, 0, 0])
        t = np.cross(ax, [0, 0, 1.0]); t /= np.linalg.norm(t); Rf = np.stack([ax, t, np.cross(ax, t)], 1)
        k = np.array([2e6, 2e6, 2e6, 1e-6, 0.3, 0.3])      # free rotation about the pin/hinge axis
    elems.append(('spring', i, j, (ci, Rf, k)))
# ---- supports: 4 corner contacts per resting part (compression-only, solved iteratively) ----
supports = []                        # (corner node, part, where)
for n, where in D['ground']:
    if n not in parts: continue
    c0 = centroid_node[n]
    for sx in (0, 1):
        for sy in (0, 1):
            pl = np.array([lo[n][0] if sx == 0 else hi[n][0], lo[n][1] if sy == 0 else hi[n][1], lo[n][2]])
            q = M4[n][:3, :3] @ pl + M4[n][:3, 3]
            cn = node(q); elems.append(('rigid', c0, cn, (n, R3[n][:, 0]))); supports.append((cn, n, where))
N = len(nodes); ndof = 6*N
def beam_k(L, A, Iy, Iz, J):
    k = np.zeros((12, 12))
    EA, GJ = E*A/L, G*J/L
    k[0, 0] = k[6, 6] = EA; k[0, 6] = k[6, 0] = -EA
    k[3, 3] = k[9, 9] = GJ; k[3, 9] = k[9, 3] = -GJ
    for (v, th, I, s) in ((1, 5, Iz, 1), (2, 4, Iy, -1)):
        a, b_, c_ = 12*E*I/L**3, 6*E*I/L**2, E*I/L
        idx = [v, th, v + 6, th + 6]
        m = np.array([[a, s*b_, -a, s*b_], [s*b_, 4*c_, -s*b_, 2*c_], [-a, -s*b_, a, -s*b_], [s*b_, 2*c_, -s*b_, 4*c_]])
        for p_ in range(4):
            for q_ in range(4): k[idx[p_], idx[q_]] += m[p_, q_]
    return k
def frame(xa, xb, ref):
    x = xb - xa; L = np.linalg.norm(x); x /= L
    z = np.cross(x, ref)
    if np.linalg.norm(z) < 1e-6: z = np.cross(x, [0, 0, 1.0]) if abs(x[2]) < 0.9 else np.cross(x, [1.0, 0, 0])
    z /= np.linalg.norm(z); y = np.cross(z, x)
    return L, np.stack([x, y, z])                       # rows = local axes
rows, cols, vals = [], [], []
einfo = []
def scatter(K, i, j):
    dofs = list(range(6*i, 6*i + 6)) + list(range(6*j, 6*j + 6))
    for a_ in range(12):
        for b_ in range(12):
            if K[a_, b_] != 0: rows.append(dofs[a_]); cols.append(dofs[b_]); vals.append(K[a_, b_])
for typ, i, j, dat in elems:
    if typ in ('beam', 'rigid'):
        n, ref = dat; A, Iy, Iz, J, b, h = sec[n]
        L, Rl = frame(nodes[i], nodes[j], ref)
        if typ == 'rigid': A, Iy, Iz, J = A*50, Iy*50, Iz*50, J*50
        kl = beam_k(L, A, Iy, Iz, J)
        T = np.kron(np.eye(4), Rl); K = T.T @ kl @ T
        einfo.append((typ, i, j, n, kl, T, A, Iy, Iz, b, h))
    else:
        ci, Rf, k = dat
        T6 = np.kron(np.eye(2), Rf.T)                     # global -> spring frame
        kd = np.diag(k); kl = np.block([[kd, -kd], [-kd, kd]])
        T = np.kron(np.eye(2), T6); K = T.T @ kl @ T
        einfo.append(('spring', i, j, ci, k, T6))
    scatter(K, i, j)
K0 = sp.csc_matrix((vals, (rows, cols)), shape=(ndof, ndof)) + sp.identity(ndof, format='csc')*1e-9
KV = 1e7
def solve_contact(F):
    """compression-only corner supports: drop corners that pull, re-solve"""
    active = np.ones(len(supports), bool)
    for it in range(12):
        r_, c_, v_ = [], [], []
        for k_, (nd, n, w) in enumerate(supports):
            for d_, kk in zip(range(3), (5e4, 5e4, KV if active[k_] else 0.0)):
                if kk: r_.append(6*nd + d_); c_.append(6*nd + d_); v_.append(kk)
        K = K0 + sp.csc_matrix((v_, (r_, c_)), shape=(ndof, ndof))
        u = spl.spsolve(K, F)
        reac = np.array([-KV*u[6*nd + 2] if active[k_] else 0.0 for k_, (nd, n, w) in enumerate(supports)])
        lift = np.array([u[6*nd + 2] > 0 for nd, n, w in supports])
        new = active & ~(reac < 0) | (active == False) & ~lift
        new = np.array([(reac[k_] >= 0) if active[k_] else (not lift[k_]) for k_ in range(len(supports))])
        if (new == active).all(): break
        active = new
    return u, reac, active
mass = {n: max(np.prod(ext[n]), 1e-9)*RHO for n in names}
print(f'FEM: {len(names)} parts, {N} nodes, {sum(e[0]=="beam" for e in elems)} beams, '
      f'{sum(e[0]=="spring" for e in elems)} connection springs, {len(supports)} support contacts, bridge mass {sum(mass.values())*1000:.0f} g')

# ---- truck wheel loads ----
road_parts = [n for n in names if parts[n]['group'] in ('road', 'appTop', 'landTop')]
def top_part_at(x, y):
    best = None
    for n in road_parts:
        inv = np.linalg.inv(M4[n]); pl = inv @ np.array([x, y, ctr[n][2], 1.0])
        if lo[n][0] - 1e-4 <= pl[0] <= hi[n][0] + 1e-4 and lo[n][1] - 1e-4 <= pl[1] <= hi[n][1] + 1e-4:
            ztop = (M4[n] @ np.array([pl[0], pl[1], hi[n][2], 1.0]))[2]
            if best is None or ztop > best[1]: best = (n, ztop)
    return best
WY = 3*S + 0.0015 + 0.0045
AXF, AXR = 5*S, -4*S
truck_y = np.round(np.arange(0.20, -0.2651, -0.005), 4)
def gravity_load():
    F = np.zeros(ndof)
    for n in names: F[6*centroid_node[n] + 2] -= mass[n]*g
    return F
def nearest_axis_node(n, p):
    cand = [i for k, i in part_nodes[n]]
    return min(cand, key=lambda i: np.linalg.norm(nodes[i][:2] - p[:2]))
def case_load(yc):
    F = gravity_load(); on = []
    if yc is None: return F, on
    for ax_ in (AXF, AXR):
        for sx in (-1, 1):
            x, y = sx*WY, yc - ax_                      # truck faces -y
            hit = top_part_at(x, y)
            if hit: F[6*nearest_axis_node(hit[0], np.array([x, y])) + 2] -= TRUCK_M*g/4; on.append(hit[0])
    return F, on
# ---- evaluate ----
def evaluate(u):
    pu = {n: 0.0 for n in names}; pf = {n: 0.0 for n in names}; cu = {}
    for e in einfo:
        if e[0] in ('beam', 'rigid'):
            typ, i, j, n, kl, T, A, Iy, Iz, b, h = e
            ue = np.concatenate([u[6*i:6*i+6], u[6*j:6*j+6]]); f = kl @ (T @ ue)
            Nx = max(abs(f[0]), abs(f[6])); My = max(abs(f[4]), abs(f[10])); Mz = max(abs(f[5]), abs(f[11]))
            if typ == 'beam':
                Wy = Iy/(h/2); Wz = Iz/(b/2)
                s = Nx/A + My/Wy + Mz/Wz
                pu[n] = max(pu[n], s/SIG_ALLOW); pf[n] = max(pf[n], Nx + max(abs(f[1]), abs(f[2])))
        else:
            _, i, j, ci, k, T6 = e
            c = conns[ci]; du = T6 @ (u[6*j:6*j+6] - u[6*i:6*i+6]); f = k*du      # on part b relative to a
            if c['kind'] == 'stud':
                nst = c['n']; r = 0.5*math.sqrt(nst)*S
                ten = max(0.0, f[2]); peel = math.hypot(f[3], f[4]); shear = math.hypot(f[0], f[1])
                ut = max((ten + peel/r)/(nst*F_CLUTCH), shear/(nst*F_SHEAR))
            else:
                ut = np.linalg.norm(f[:3])/20.0             # pin/hinge shear capacity ~20 N
            cu[ci] = ut
            for n in (c['a'], c['b']):
                pf[n] = max(pf[n], np.linalg.norm(f[:3])); pu[n] = max(pu[n], ut)
    return pu, pf, cu
cases = [None] + list(truck_y)
res = {'truck_y': [None] + truck_y.tolist(), 'cases': []}
env_u = {n: 0.0 for n in names}; env_f = {n: 0.0 for n in names}; env_c = {}
mid = BR.deck_point(9.5*S, 0, 0); mid_node = min(range(N), key=lambda i: np.linalg.norm(nodes[i] - np.array(mid)))
for yc in cases:
    F, on = case_load(yc); u, rc, act = solve_contact(F)
    pu, pf, cu = evaluate(u)
    reac = {}
    for (nd, n, where), r_ in zip(supports, rc): reac[n] = reac.get(n, 0.0) + float(r_)
    lifted = sorted({supports[k_][1] for k_ in range(len(supports)) if not act[k_]})
    res['cases'].append({'y': yc, 'util': pu, 'force': pf, 'defl_mm': float(u[6*mid_node + 2]*1000),
                         'reac': reac, 'lifted_corners': lifted, 'wheels_on': on})
    for n in names: env_u[n] = max(env_u[n], pu[n]); env_f[n] = max(env_f[n], pf[n])
    for ci, v in cu.items(): env_c[ci] = max(env_c.get(ci, 0), v)
json.dump(res, open('/home/claude/render_b/fem.json', 'w'))
# ---- report ----
d_ = [c['defl_mm'] for c in res['cases']]
print(f'mid-span deflection: self-weight {d_[0]:.4f} mm, with truck max {min(d_):.4f} mm')
sup_parts = sorted({(n, w) for nd, n, w in supports})
lost = sorted({n for c in res['cases'] for n, r in c['reac'].items() if r <= 1e-6})
part_lift = sorted({n for c in res['cases'] for n in c['lifted_corners']})
print('supports:', ', '.join(f'{n}/{w}' for n, w in sup_parts))
print('  parts that lose all contact (would lift off):', lost or 'none', '| corners lifting (rocking):', part_lift or 'none')
print('  reactions self-weight:', {n: round(r, 3) for n, r in res['cases'][0]['reac'].items()}, 'N')
top = sorted(env_u, key=lambda n: -env_u[n])[:10]
print('\nhighest utilisation (1.0 = limit; stress vs 30 MPa ABS, connections vs 1 N/stud clutch):')
for n in top: print(f'  {env_u[n]:6.3f}  {n:<12} ({parts[n]["group"]})')
fmax = max(env_f.values())
idle = sorted([n for n in names if env_f[n] < 0.01*fmax], key=lambda n: env_f[n])
print(f'\nparts carrying < 1% of the peak part force ({fmax:.3f} N): {len(idle)}')
for n in idle: print(f'  {n:<12} {env_f[n]*1000:.2f} mN  ({parts[n]["group"]})')
worst_c = sorted(env_c, key=lambda c: -env_c[c])[:5]
print('\nmost loaded connections:')
for ci in worst_c: c = conns[ci]; print(f'  {env_c[ci]:5.2f}  {c["kind"]:<10} {c["a"]} <-> {c["b"]} ({c["n"]})')

# ---------------- per-frame FEM synced to the rendered drive ----------------
if os.environ.get('FRAMES', '1') == '1':
    T_GO, FPS = 393, 30
    tr = np.load('/home/claude/render_b/traj.npz')['fine']           # t, pos(3), quat(4), wheels(4)
    def qmat(q):
        w, x, y, z = q
        return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
    WR_ = 0.0145
    out = {'frames': {}}
    for f in range(360, 642):
        s_ = max(0.0, (f - T_GO)/FPS)
        i = min(np.searchsorted(tr[:, 0], s_), len(tr) - 1)
        pos, q = tr[i, 1:4], tr[i, 4:8]; R = qmat(q)
        F = gravity_load(); on = []
        for ax_ in (AXF, AXR):
            for sy in (-1, 1):
                c = R @ np.array([ax_, sy*WY, WR_]) + pos; x, y = c[0], c[1]; zc = c[2] - WR_
                hit = top_part_at(x, y)
                if hit and abs(hit[1] - zc) < 0.004:
                    F[6*nearest_axis_node(hit[0], np.array([x, y])) + 2] -= TRUCK_M*g/4; on.append(hit[0])
        u, rc, act = solve_contact(F)
        pu, pf, cu = evaluate(u)
        out['frames'][f] = {'util': {n: round(v, 4) for n, v in pu.items()}, 'defl_mm': round(float(u[6*mid_node + 2]*1000), 4),
                            'pos': pos.tolist(), 'quat': q.tolist(), 'wheels_on': on}
    json.dump(out, open('/home/claude/render_b/fem_frames.json', 'w'))
    um = max(max(v['util'].values()) for v in out['frames'].values())
    print(f'per-frame FEM: {len(out["frames"])} frames, peak utilisation {um:.3f}')
