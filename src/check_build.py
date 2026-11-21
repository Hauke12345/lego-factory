"""Lego build checker: (1) part-part intersections, (2) connection graph (studs, Technic holes/pins,
hinges, axles), (3) exports parts + connections for the MuJoCo load test.
usage: python3 check_build.py [out.json]"""
import sys, math, json, itertools
sys.path.insert(0, '/home/claude/scripts')
import bpy, bmesh
import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree
bpy.ops.wm.read_factory_settings(use_empty=True)
import lego
lego.NOSTUDS = True                      # studs would (correctly) sit inside the part above
import bridge as BR, truck
OUT = sys.argv[1] if len(sys.argv) > 1 else '/home/claude/render_b/build_check.json'
SHRINK = 0.00012                         # 0.12 mm: touching faces are not intersections
TOL = 0.0004                             # connection position tolerance

parts = [(ob, M, g) for ob, M, g in BR.build(lego)]
tp, tw, _ = truck.build_truck()
TR = Matrix.Translation((0, 0.20, BR.Z_BOX)) @ Matrix.Rotation(-math.pi/2, 4, 'Z')
parts += [(ob, TR @ M, 'truck') for ob, M in tp]
for ob, M, g in parts: ob.matrix_world = M
bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()

def world_bm(o):
    e = o.evaluated_get(dg); me = e.to_mesh()
    bm = bmesh.new(); bm.from_mesh(me); e.to_mesh_clear()
    bm.transform(o.matrix_world)
    bm.normal_update()
    for v in bm.verts: v.co -= v.normal*SHRINK
    return bm

# ---------- 1. intersections ----------
objs = []
for ob, M, g in parts:
    for o in [ob] + [c for c in ob.children_recursive]:
        if o.type != 'MESH' or o.name.endswith('_holes'): continue
        bm = world_bm(o)
        co = np.array([v.co[:] for v in bm.verts])
        objs.append((o.name, ob.name, BVHTree.FromBMesh(bm), co.min(0), co.max(0)))
        bm.free()
EXPECT = [('axle', 'tbrick'), ('axle', 'wheel')]      # axle through Technic holes / into wheel hubs
def expected(a, b):
    return any((a.startswith(x) and b.startswith(y)) or (b.startswith(x) and a.startswith(y)) for x, y in EXPECT)
hits = []
for (na, pa, ta, lo_a, hi_a), (nb, pb, tb, lo_b, hi_b) in itertools.combinations(objs, 2):
    if pa == pb: continue
    if np.any(hi_a < lo_b) or np.any(hi_b < lo_a): continue
    ov = ta.overlap(tb)
    if ov: hits.append((na, nb, len(ov), expected(pa, pb)))
print(f'\n=== INTERSECTIONS ({len(objs)} meshes) ===')
bad = [h for h in hits if not h[3]]
for h in sorted(bad, key=lambda h: -h[2]): print(f'  CLASH  {h[0]:<16} x {h[1]:<16} {h[2]} tri pairs')
print(f'  {len(bad)} unexpected, {len(hits) - len(bad)} expected (axle in holes/hubs)')

# ---------- 2. connections ----------
def bbox(ob):
    co = []
    for o in [ob] + list(ob.children_recursive):
        if o.type == 'MESH' and not o.name.endswith('_holes'):
            co += [ob.matrix_world.inverted() @ (o.matrix_world @ Vector(c)) for c in o.bound_box]
    co = np.array([c[:] for c in co]); return co.min(0).tolist(), co.max(0).tolist()

def pts(ob, key):
    a = list(ob.get(key, [])); M = ob.matrix_world
    return [M @ Vector(a[i:i+3]) for i in range(0, len(a), 3)]
def up(ob): return (ob.matrix_world.to_3x3() @ Vector((0, 0, 1))).normalized()
def axes(ob):
    out = []
    for kind, c, d, hl in eval(ob.get('axes', '[]')):
        M = ob.matrix_world
        out.append((kind, M @ Vector(c), (M.to_3x3() @ Vector(d)).normalized(), hl))
    return out
def hole_axis(ob): return (ob.matrix_world.to_3x3() @ Vector((0, 1, 0))).normalized()
P = {ob.name: ob for ob, M, g in parts}
G = {ob.name: g for ob, M, g in parts}
conns = []          # (a, b, kind, n, points)
names = list(P)
# studs -> anti-studs
anti = {n: pts(P[n], 'anti') for n in names}
for a in names:
    sa = pts(P[a], 'studs')
    if not sa: continue
    ua = up(P[a])
    for b in names:
        if a == b or not anti[b]: continue
        if ua.dot(up(P[b])) < 0.995: continue
        hit = [s for s in sa if any((s - t).length < TOL for t in anti[b])]
        if hit: conns.append((a, b, 'stud', len(hit), [tuple(h) for h in hit]))
# Technic holes: pins (liftarm hole on brick hole / liftarm hole on liftarm hole), coaxial within TOL
holes = {n: (pts(P[n], 'holes'), hole_axis(P[n])) for n in names if P[n].get('holes')}
def coax(p, d, q, e):
    if abs(d.dot(e)) < 0.995: return False
    v = q - p; return (v - d*v.dot(d)).length < TOL and abs(v.dot(d)) < 0.02
pinned = []
for a, b in itertools.combinations(holes, 2):
    (ha, da), (hb, db) = holes[a], holes[b]
    hit = [p for p in ha if any(coax(p, da, q, db) for q in hb)]
    if hit:
        kind = 'axle-hole' if 'axle' in (a + b) else 'pin'
        conns.append((a, b, 'pin', len(hit), [tuple(h) for h in hit]))
# pin stacks: parts on one pin must touch each other (no air gap along the pin)
def yext(n):
    ys = [v.co.y for v in P[n].data.vertices]; return min(ys), max(ys)      # own mesh only (no pin heads)
gaps = []
pin_pts = {}
for a, b, k, n_, p in conns:
    if k == 'pin':
        d_ = holes[a][1]
        for q in p:
            qq = Vector(q); qq = qq - d_*qq.dot(d_)            # key = pin line (point projected along its axis)
            pin_pts.setdefault(tuple(round(x, 4) + 0.0 for x in qq) + (1 if Vector(q).dot(d_) > 0 else -1,), (Vector(q), set()))[1].update((a, b))
for key, (qv, members) in pin_pts.items():
    q = tuple(qv); ax = None; iv = []
    for n in names:
        if n not in holes: continue
        hs, d = holes[n]
        if not any(coax(qv, d, h, d) and abs((h - qv).dot(d)) < 0.02 for h in hs): continue
        ax = d if ax is None else ax
        c = P[n].matrix_world.translation; lo_, hi_ = yext(n)
        mid = c.dot(ax) + 0.5*(lo_ + hi_)*(P[n].matrix_world.to_3x3() @ Vector((0, 1, 0))).dot(ax)
        h = 0.5*(hi_ - lo_); iv.append((mid - h, mid + h, n))
    iv.sort()
    end, en = iv[0][1], iv[0][2]
    for b0, b1, nb in iv[1:]:
        if b0 - end > 0.0003: gaps.append((q, en, nb, (b0 - end)*1000))
        if b1 > end: end, en = b1, nb
print(f'\n=== PIN STACKS: {len(pin_pts)} pins, {len(gaps)} with an air gap ===')
for q, na, nb, gmm in gaps: print(f'  GAP {gmm:.1f} mm between {na} and {nb}')
# axles: axle line through Technic holes and wheel hubs
for a in names:
    for kind, c, d, hl in axes(P[a]):
        if kind != 'axle': continue
        for b in names:
            if b == a: continue
            if b in holes:
                hb, db = holes[b]
                hit = [q for q in hb if abs(d.dot(db)) > 0.995 and ((q - c) - d*(q - c).dot(d)).length < TOL and abs((q - c).dot(d)) <= hl]
                if hit: conns.append((a, b, 'axle-hole', len(hit), [tuple(h) for h in hit]))
            for kb, cb, dbb, hlb in axes(P[b]):
                if kb == 'hub' and abs(d.dot(dbb)) > 0.995 and ((cb - c) - d*(cb - c).dot(d)).length < TOL and abs((cb - c).dot(d)) <= hl + 0.001:
                    insert = hl - abs((cb - c).dot(d)) + hlb
                    conns.append((a, b, 'axle-hub', 1, [tuple(cb)]))
                    if insert < 0.004: print(f'  WARNING axle {a} only {insert*1000:.1f} mm inside {b}')
# hinges: click1<->click2 and hingeB<->hingeT on the same axis
for a, b in itertools.combinations(names, 2):
    for ka, ca, da, _ in axes(P[a]):
        for kb, cb, db, _ in axes(P[b]):
            if {ka, kb} in ({'click1', 'click2'}, {'hingeB', 'hingeT'}) and abs(da.dot(db)) > 0.995 and \
               ((cb - ca) - da*(cb - ca).dot(da)).length < TOL and abs((cb - ca).dot(da)) < 0.006:
                conns.append((a, b, 'hinge-lock' if 'click' in ka else 'hinge', 1, [tuple(ca)]))
# resting on the box / book / table
SUP = {'box': BR.Z_BOX, 'book': BR.Z_BOOK, 'table': 0.0}
ground = []
for n in names:
    if G[n] == 'truck': continue
    z = min(p.z for p in anti[n]) if anti[n] else None
    if z is not None and up(P[n]).z > 0.999:
        for k, zz in SUP.items():
            if abs(z - zz) < TOL: ground.append((n, k))
# connectivity
adj = {n: set() for n in names}
for a, b, k, n_, _ in conns: adj[a].add(b); adj[b].add(a)
def comps(sel):
    seen, out = set(), []
    for n in sel:
        if n in seen: continue
        st, c = [n], []
        while st:
            x = st.pop()
            if x in seen or x not in sel: continue
            seen.add(x); c.append(x); st += list(adj[x])
        out.append(c)
    return out
bnames = [n for n in names if G[n] != 'truck']; tnames = [n for n in names if G[n] == 'truck']
print('\n=== CONNECTIONS ===')
from collections import Counter
print('  ', dict(Counter(k for _, _, k, _, _ in conns)), ' studs total', sum(n_ for _, _, k, n_, _ in conns if k == 'stud'))
gset = {n for n, _ in ground}
for label, sel in (('bridge', bnames), ('truck', tnames)):
    cs = comps(sel)
    print(f'  {label}: {len(sel)} parts in {len(cs)} connected group(s)')
    for c in cs:
        if len(cs) > 1 or label == 'bridge':
            sup = sorted({k for n, k in ground if n in c})
            print(f'     group of {len(c):3d}: {", ".join(sorted(c)[:6])}{" ..." if len(c) > 6 else ""}   rests on: {sup or "NOTHING"}')
lone = [n for n in names if not adj[n]]
if lone: print('  UNCONNECTED parts:', lone)
# export
def shapes(ob):
    out = []
    for o in [ob] + list(ob.children_recursive):
        if o.type != 'MESH' or o.name.endswith('_holes'): continue
        try: col = list(o.data.materials[0].node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value[:3])
        except Exception: col = [0.5, 0.5, 0.5]
        out.append({'verts': [list(o.matrix_world @ v.co) for v in o.data.vertices], 'color': col, 'name': o.name})
    return out
data = {'parts': {n: {'group': G[n], 'M': [list(r) for r in P[n].matrix_world], 'bbox': bbox(P[n]),
                      'shapes': shapes(P[n]) if G[n] == 'truck' else None,
                      'verts': [list(v.co) for v in P[n].data.vertices]} for n in names},
        'conns': [dict(a=a, b=b, kind=k, n=n_, pts=p) for a, b, k, n_, p in conns],
        'ground': ground, 'clashes': [list(h) for h in bad]}
json.dump(data, open(OUT, 'w'))
print('wrote', OUT)
