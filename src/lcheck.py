"""Build checker for LDraw-part designs: clashes (shrunken-triangle BVH overlap), snap connections
(stud/anti-stud, pin/hole, axle/axle-hole, axle in round hole), gear meshes and connectivity.
Usage: python3 lcheck.py module [json_out]   (module must expose build() -> list of placements)"""
import sys, json, importlib, itertools, numpy as np
sys.path.insert(0, '/home/claude/scripts')
import ldr
from scipy.spatial import cKDTree
import bpy
from mathutils.bvhtree import BVHTree

SHRINK = 0.25          # LDU each triangle is pushed into its own solid (0.1 mm)
# connection points the shadow library lacks
def _zc(x, y, z):          # snap frame whose -Y runs along -z, at (x, y, z)
    return np.array([[1, 0, 0, x], [0, 0, -1, y], [0, 1, 0, z], [0, 0, 0, 1]], float)
EXTRA = {'38340': [dict(kind='cyl', id='wheelPin', gender='M', secs=[('R', 6, 19.5)], center=False, slide=False, scale='', M=_zc(0, 0, 27.5), src='38340')],   # bogie wheel's pin (z 8..27.5)
         '18588': [dict(kind='cyl', id='barrelPin', gender='F', secs=[('R', 8, 2), ('R', 6, 18)], center=False, slide=False, scale='', M=_zc(0, sy, 20), src='18588')
                   for sy in (-20, 20)],      # from the part file: two pegholes at (0, +-20, 20)
         '18587': [dict(kind='cyl', id='trigAxle', gender='F', secs=[('A', 6, 30)], center=False, slide=False, scale='', M=_zc(0, 0, 12), src='18587')],
         '26913': [dict(kind='cyl', id='motOut', gender='F', secs=[('A', 6, 20)], center=False, slide=False, scale='',
                        M=np.array([[1, 0, 0, 0], [0, 0, 1, 34], [0, -1, 0, -60], [0, 0, 0, 1]], float), src='motOut')],
         '26910': [dict(kind='cyl', id='hubOut', gender='F', secs=[('A', 6, 16)], center=False, slide=False, scale='',
                        M=np.array([[0, s, 0, 60*s], [-s, 0, 0, 66], [0, 0, 1, 140], [0, 0, 0, 1]], float), src='hubOut')
                   for s in (1, -1)]}
GEARS = {'3647': 8, '3648b': 24, '3648': 24, '4019': 16, '94925': 16, '3649': 40, '32270': 12, '32269': 20, '32498': 36}
BEVEL = {'32269': 30, '32270': 20}            # double bevels: gear centre to axis crossing when meshing at 90 deg
# loose by design (resting, not clutched): body -> reason
RESTING = {'pusherKA': 'slides on the ledge', 'pusherKB': 'slides on the ledge', 'kitA': 'hangs on the fork guides', 'kitB': 'hangs on the fork guides', 'plugA': 'lies in the plug shoe', 'plugB': 'lies in the plug shoe', 'ksledA': 'rides on tread', 'ksledB': 'rides on tread', 'worker': 'stands on its wheels', 'pallet': 'rides on tread', 'robot': 'sits in pallet pocket', 'pusher': 'slides on the ledge between rails, driven by the conrod',
           'head0': 'stacked in magazine', 'head1': 'stacked', 'head2': 'stacked', 'head3': 'stacked', 'head4': 'stacked'}


def _cylF(o, d, secs, gender='F', sid='x'):
    """snap whose cylinder starts at o and runs along unit d (LDCad: along the frame's -Y)."""
    d = np.asarray(d, float); Y = -d; X = np.cross(Y, [0, 0, 1]) if abs(Y[2]) < 0.9 else np.cross(Y, [1, 0, 0]); X /= np.linalg.norm(X); Z = np.cross(X, Y)
    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = X, Y, Z, o
    return dict(kind='cyl', id=sid, gender=gender, secs=secs, center=False, slide=False, scale='', M=M, src=sid)
# PF/PU train motor: hollow wheel axles (x = +-40 ends, y 42, z +-60) take the wheels' Technic axles
EXTRA['87574'] = [_cylF((sx*40, 42, z), (-sx, 0, 0), [('R', 6, 20)], sid='motAxleTube') for sx in (-1, 1) for z in (-60, 60)]

def world_tris(p):
    d = ldr.part(p['part']); V = d['tris'].astype(np.float64)
    if len(V) == 0: return V
    n = np.cross(V[:, 1] - V[:, 0], V[:, 2] - V[:, 0]); ln = np.linalg.norm(n, axis=1, keepdims=True)
    ok = ln[:, 0] > 1e-9; n[ok] /= ln[ok]
    V = V - SHRINK*n[:, None, :]          # LDraw CCW winding -> n points out of the solid
    c = V.mean(1, keepdims=True); r = np.linalg.norm(V - c, axis=2, keepdims=True)      # pull the edges in by 0.3 LDU too:
    V = c + (V - c)*np.clip(1 - 0.3/np.maximum(r, 1e-6), 0, 1)                           # faces that only share an edge line are not a clash
    M = p['M']
    return V[ok] @ M[:3, :3].T + M[:3, 3]

def _anti(d):
    """synthesised anti-studs for basic bricks/plates/tiles whose shadow file is missing."""
    import re
    if not re.match(r'^(Brick|Plate|Tile)\s', d['title']): return []
    if any(s['kind'] == 'cyl' and s['gender'] == 'F' for s in d['snaps']): return []
    V = d['tris'].reshape(-1, 3); lo, hi = V.min(0), V.max(0)
    out = []
    for x in np.arange(lo[0] + 10, hi[0], 20):
        for z in np.arange(lo[2] + 10, hi[2], 20):
            M = np.eye(4); M[:3, 3] = [x, hi[1], z]
            out.append(dict(kind='cyl', id='antiSynth', gender='F', secs=[('S', 6, 4)], center=False, slide=False, scale='', M=M, src='synth'))
    return out

def cylinders(p):
    d = ldr.part(p['part']); out = []
    snaps = d['snaps'] + EXTRA.get(p['part'], []) + _anti(d)
    for s in snaps:
        if s['kind'] != 'cyl': continue
        o, u, secs = ldr.cyl_world(s, p['M'])
        out.append(dict(o=o, u=u, secs=secs, g=s['gender'], id=s['id'] or s['src'], L=secs[-1][3]))
    # dedupe (shadow part file + primitive often describe the same cylinder)
    uniq = {}
    for c in out:
        k = (c['g'], tuple(np.round(c['o'], 1)), tuple(np.round(c['u'], 2)), round(c['L'], 1))
        uniq.setdefault(k, c)
    return list(uniq.values())

def sec_at(c, t):
    for shp, r, a, b in c['secs']:
        if a - 1e-6 <= t <= b + 1e-6: return shp, r
    return None

def fit(cm, cf):
    """classify a coaxial male/female overlap; returns (type, ok) or None if no overlap."""
    if abs(abs(np.dot(cm['u'], cf['u'])) - 1) > 2e-3: return None
    d = cf['o'] - cm['o']; perp = d - np.dot(d, cm['u'])*cm['u']
    if np.linalg.norm(perp) > 0.8: return None
    # female interval in male axis coordinates
    s = np.dot(cf['u'], cm['u']); f0 = np.dot(d, cm['u']); f1 = f0 + s*cf['L']
    lo, hi = max(0, min(f0, f1)), min(cm['L'], max(f0, f1))
    if hi - lo < 1.0: return None
    kinds = set(); ok = True
    for t in np.linspace(lo + 0.3, hi - 0.3, 5):
        a = sec_at(cm, t); tf = (t - f0)*s; b = sec_at(cf, tf)
        if a is None or b is None: continue
        (sm, rm), (sf, rf) = a, b
        if 'L' in sm or 'L' in sf: continue
        if sf == 'A' and sm == 'A': kinds.add('axle-lock')
        elif sm == 'A' and sf in 'RS' and abs(rf - 6) < 0.6: kinds.add('axle-free')
        elif sm == 'R' and sf in 'RS' and abs(rm - rf) < 0.6:
            kinds.add('stud' if hi - lo <= 5 else 'pin')
        elif sm == 'R' and sf == 'A': ok = False; kinds.add('round-in-axlehole')
        elif sm == 'R' and abs(rm - rf) >= 0.6:
            if rm > rf + 0.6: ok = False; kinds.add('oversize')
            else: kinds.add('loose')
    if not kinds: return None
    for k in ('round-in-axlehole', 'oversize', 'axle-lock', 'pin', 'axle-free', 'stud', 'loose'):
        if k in kinds: return k, ok
    return None

def run(P, verbose=True):
    P = [p for p in P if not p['part'].startswith('~') or 'ends' in p]     # cables are checked for clashes; their two ends may sit inside plug and device
    rep = {'clashes': [], 'expected': [], 'bad_fits': [], 'unconnected': [], 'bodies': {}, 'gears': []}
    # ---------- connections ----------
    cyl = [cylinders(p) for p in P]
    Ms = [(i, c) for i, cs in enumerate(cyl) for c in cs if c['g'] == 'M']
    Fs = [(i, c) for i, cs in enumerate(cyl) for c in cs if c['g'] == 'F']
    mid = lambda c: c['o'] + c['u']*c['L']/2
    tree = cKDTree(np.array([mid(c) for _, c in Fs])) if Fs else None
    Lmax = max([c['L'] for _, c in Fs] + [1])
    conns = {}; axes = {}; tyre_pairs = set()
    for i, cm in Ms:
        for j in tree.query_ball_point(mid(cm), cm['L']/2 + Lmax/2 + 1):
            jf, cf = Fs[j]
            if jf == i: continue
            r = fit(cm, cf)
            if r is None: continue
            key = tuple(sorted((i, jf)))
            conns.setdefault(key, []).append(r)
            if r[0] in ('pin', 'axle-free', 'axle-lock', 'stud'):
                axes.setdefault(key, []).append((cm['o'], cm['u'], cm['L']))
    # clips gripping bars (handles, bar parts): SNAP_CLP on one part, an R4 male cylinder on the other
    for i, p in enumerate(P):
        for s in ldr.part(p['part'])['snaps'] + EXTRA.get(p['part'], []):
            if s['kind'] != 'clp': continue
            Wc = p['M'] @ s['M']; pc, ac = Wc[:3, 3], Wc[:3, 1]/np.linalg.norm(Wc[:3, 1])
            for j, cs in enumerate(cyl):
                if j == i: continue
                for c in cs:
                    if c['g'] != 'M' or not any(abs(r - 4) < 0.3 for _, r, _, _ in c['secs']): continue
                    if abs(abs(np.dot(c['u'], ac)) - 1) > 0.02: continue
                    d_ = pc - c['o']; t_ = np.dot(d_, c['u'])
                    if np.linalg.norm(d_ - t_*c['u']) < 0.8 and -1 <= t_ <= c['L'] + 1:
                        conns.setdefault(tuple(sorted((i, j))), []).append(('clip', True))
    # tyres: stretch-fitted on a coaxial rim (no snap data for rubber)
    for i, p in enumerate(P):
        if not ldr.part(p['part'])['title'].startswith('Tyre'): continue
        for j, q in enumerate(P):
            if ldr.part(q['part'])['title'].startswith('Wheel Rim') and np.linalg.norm(p['M'][:3, 3] - q['M'][:3, 3]) < 2 \
               and abs(abs(np.dot(p['M'][:3, 2], q['M'][:3, 2])) - 1) < 1e-3:
                conns.setdefault(tuple(sorted((i, j))), []).append(('tyre-on-rim', True)); tyre_pairs.add(tuple(sorted((i, j))))
    # Powered Up plugs in sockets: the plug's snap frame must coincide with a socket's (position, insertion axis and keying)
    gens = []
    for i, p in enumerate(P):
        for s_ in ldr.part(p['part'])['snaps']:
            if s_['kind'] == 'gen': gens.append((i, s_['gender'], p['M'] @ s_['M']))
    for i, g, W in gens:
        if g != 'M': continue
        best = None
        for j, g2, W2 in gens:
            if g2 != 'F' or j == i: continue
            d_ = float(np.linalg.norm(W[:3, 3] - W2[:3, 3]))
            if best is None or d_ < best[0]: best = (d_, j, W2)
        if best is None or best[0] > 60: continue
        d_, j, W2 = best
        nz = lambda v: v/np.linalg.norm(v)
        ok = d_ < 1.5 and np.dot(nz(W[:3, 2]), nz(W2[:3, 2])) > 0.98 and np.dot(nz(W[:3, 1]), nz(W2[:3, 1])) > 0.98
        if ok: conns.setdefault(tuple(sorted((i, j))), []).append(('plug', True))
        elif d_ < 12:
            same = np.dot(nz(W[:3, 2]), nz(W2[:3, 2])) > 0.98 and np.dot(nz(W[:3, 1]), nz(W2[:3, 1])) > 0.98
            dv = W[:3, 3] - W2[:3, 3]
            along = same and min(np.linalg.norm(np.cross(dv, nz(W[:3, k]))) for k in range(3)) < 0.8
            if along: rep.setdefault('partial', []).append((P[i]['name'], P[j]['name'], 'plug part-way into its port (aligned)'))
            else: rep['bad_fits'].append((P[i]['name'], P[j]['name'], [f'plug misaligned by {d_:.1f} LDU or wrong way round']))
    # finger hinges (SNAP_FGR, e.g. train magnet in its holder): coincident hinge frames with parallel axes
    fgrs = [(i, p['M'] @ s_['M']) for i, p in enumerate(P) for s_ in ldr.part(p['part'])['snaps'] if s_['kind'] == 'fgr']
    for (i, A_), (j, B_) in itertools.combinations(fgrs, 2):
        if i != j and np.linalg.norm(A_[:3, 3] - B_[:3, 3]) < 0.5 and abs(abs(np.dot(A_[:3, 1], B_[:3, 1])) - 1) < 1e-3:
            conns.setdefault(tuple(sorted((i, j))), []).append(('hinge', True))
    # tread links: the grip plate (local -y) must face out of its closed loop; a flipped chain keeps its pins on the pitch line
    loops = {}
    for i, p in enumerate(P):
        if p['part'] == '3873': loops.setdefault(p['name'].rsplit('link', 1)[0].rstrip('0123456789'), []).append(i)
    for key, idx in loops.items():
        if len(idx) < 8: continue
        c = np.mean([P[i]['M'][:3, 3] for i in idx], axis=0); wrong = 0
        for i in idx:
            M = P[i]['M']; g = -M[:3, 1]; r_ = M[:3, 3] - c
            if np.dot(g, r_) < 0: wrong += 1
        if wrong > len(idx)//4: rep['bad_fits'].append((P[idx[0]]['name'], key, [f'tread inverted: {wrong} of {len(idx)} links have the grip plate facing into the loop']))
    adj = {i: set() for i in range(len(P))}
    for (a, b), lst in conns.items():
        kinds = [k for k, ok in lst]
        if any(not ok for k, ok in lst):
            rep['bad_fits'].append((P[a]['name'], P[b]['name'], sorted(set(kinds))))
        adj[a].add(b); adj[b].add(a)
    # ---------- gears ----------
    gi = [i for i, p in enumerate(P) if p['part'] in GEARS]
    mesh_pairs = set()
    for a, b in itertools.combinations(gi, 2):
        A, B = P[a]['M'], P[b]['M']
        ua, ub = A[:3, 2], B[:3, 2]
        if {P[a]['part'], P[b]['part']} <= set(BEVEL) and abs(np.dot(ua, ub)) < 1e-3:
            # perpendicular double bevels: axes cross, each gear centre the other's cone distance from the crossing
            n_ = np.cross(ua, ub); d = B[:3, 3] - A[:3, 3]
            if abs(np.dot(d, n_)) < 0.5:
                sa = np.dot(d, ua); sb = -np.dot(d, ub)          # crossing = A + sa*ua = B + sb*ub
                if abs(abs(sa) - BEVEL[P[a]['part']]) < 0.6 and abs(abs(sb) - BEVEL[P[b]['part']]) < 0.6:
                    mesh_pairs.add((a, b)); rep['gears'].append((P[a]['name'], P[b]['name'], 'bevel 90'))
                elif abs(sa) < 40 and abs(sb) < 40:
                    rep['bad_fits'].append((P[a]['name'], P[b]['name'], [f'bevel offsets {sa:.1f}/{sb:.1f}']))
            continue
        if abs(abs(np.dot(ua, ub)) - 1) > 1e-3: continue
        d = B[:3, 3] - A[:3, 3]; ax = np.dot(d, ua); rad = np.linalg.norm(d - ax*ua)
        cd = (GEARS[P[a]['part']] + GEARS[P[b]['part']])*1.25
        if abs(ax) < 12 and abs(rad - cd) < 0.5:
            mesh_pairs.add((a, b)); rep['gears'].append((P[a]['name'], P[b]['name'], round(cd, 1)))
        elif abs(ax) < 20 and rad < cd + 6:
            rep['bad_fits'].append((P[a]['name'], P[b]['name'], [f'gear centre distance {rad:.1f} != {cd:.1f}']))
    # ---------- clashes ----------
    W = [world_tris(p) for p in P]
    box = [(w.reshape(-1, 3).min(0), w.reshape(-1, 3).max(0)) if len(w) else None for w in W]
    bvh = {}
    def tree_of(i):
        if i not in bvh:
            w = W[i].reshape(-1, 3); bvh[i] = BVHTree.FromPolygons(w.tolist(), np.arange(len(w)).reshape(-1, 3).tolist(), all_triangles=True)
        return bvh[i]
    for a, b in itertools.combinations(range(len(P)), 2):
        if box[a] is None or box[b] is None: continue
        if np.any(box[a][1] < box[b][0]) or np.any(box[b][1] < box[a][0]): continue
        hits = tree_of(a).overlap(tree_of(b))
        if hits and (min(a, b), max(a, b)) not in axes:
            # a pin or axle part-way into a hole (insertion in progress): coaxial male/female cylinders, not yet seated
            co = []
            for (i_, j_) in ((a, b), (b, a)):
                for cm in cyl[i_]:
                    if cm['g'] != 'M': continue
                    for cf in cyl[j_]:
                        if cf['g'] != 'F' or abs(abs(np.dot(cm['u'], cf['u'])) - 1) > 2e-3: continue
                        d_ = cf['o'] - cm['o']
                        if np.linalg.norm(d_ - np.dot(d_, cm['u'])*cm['u']) < 0.5: co.append((cm['o'], cm['u'], cm['L']))
            if co:
                def near_co(t):
                    c = W[a][t].mean(0)
                    return any(np.linalg.norm(c - (o + u*np.clip(np.dot(c - o, u), -2, L + 2))) < 9.5 for o, u, L in co)
                hits = [h for h in hits if not near_co(h[0])]
                if not hits: rep['expected'].append((P[a]['name'], P[b]['name'], 'pin/axle part-way into its hole (coaxial)')); continue
        if hits and (min(a, b), max(a, b)) in axes:
            # contacts right at a shared pin/axle/stud are polygon-approximation artefacts of the hole
            ax_ = axes[(min(a, b), max(a, b))]
            def near(t):
                c = W[a][t].mean(0)
                for o, u, L in ax_:
                    s_ = np.clip(np.dot(c - o, u), -2, L + 2); d_ = np.linalg.norm(c - (o + u*s_))
                    if d_ < 9.5: return True
                return False
            hits = [h for h in hits if not near(h[0])]
        if not hits: continue
        pa, pb = P[a], P[b]
        exp = None
        for c_, o_ in ((a, b), (b, a)):                       # a cable's two ends sit inside its plug and its device: ignore contact there only
            if 'ends' in P[c_] and P[o_]['name'] in P[c_]['ends']:
                Wc = W[c_]; e0 = Wc[:8].reshape(-1, 3).mean(0); e1 = Wc[-2:].reshape(-1, 3).mean(0)
                k_ = 0 if c_ == a else 1
                hits = [h for h in hits if min(np.linalg.norm(Wc[h[k_]].mean(0) - e0), np.linalg.norm(Wc[h[k_]].mean(0) - e1)) > 16]
                if not hits: exp = 'cable end inside its plug or device'
        if exp is None and not hits: continue
        if (a, b) in mesh_pairs: exp = 'gear mesh'
        elif (a, b) in tyre_pairs: exp = 'tyre stretched over rim'
        elif any(k == 'hinge' for k, _ in conns.get((a, b), [])): exp = 'finger hinge (magnet in its holder)'
        elif {pa['part'], pb['part']} == {'23816a', '26910'}: exp = 'plug seated in the hub socket'
        elif {pa['part'], pb['part']} == {'18588', '18587'}: exp = 'six-shooter trigger inside its barrel'
        elif P[a]['body'] == P[b]['body'] and len(hits) <= 6: exp = 'minor contact within a rigid group'
        elif '3873' in (pa['part'], pb['part']) and 'pal' in (pa['name'] + pb['name']) and len(hits) < 120: exp = 'pallet rides on the tread'
        elif '3873' in (pa['part'], pb['part']) and 'railt' in (pa['name'] + pb['name']) and len(hits) < 40: exp = 'track link edge on the rail while rolling'
        elif {pa['part'], pb['part']} == {'3873', '6636'} and 'bed' in (pa['name'] + pb['name']): exp = 'tread slides on bed (1 LDU preload)'
        elif any(k == 'stud' for k, _ in conns.get((a, b), [])) and len(hits) < 60: exp = 'clutch post touching studs'
        elif {pa['part'], pb['part']} & {'3873'} and ({pa['part'], pb['part']} & set(GEARS) or pa['part'] == pb['part']):
            exp = 'tread on sprocket' if pa['part'] != pb['part'] else 'tread hinge'
        (rep['expected'] if exp else rep['clashes']).append((pa['name'], pb['name'], len(hits), exp))
    # ---------- connectivity ----------
    bodies = {}
    for i, p in enumerate(P): bodies.setdefault(p['body'], []).append(i)
    for bname, idx in bodies.items():
        if bname.startswith('link'): continue
        idx = [i for i in idx if P[i]['part'] != '3873' and 'ends' not in P[i]]      # track links: checked by pitch; cables are flexible
        S = set(idx); seen = set(); groups = 0
        for s in idx:
            if s in seen: continue
            groups += 1; st = [s]; n0 = set(seen)
            while st:
                k = st.pop()
                if k in seen: continue
                seen.add(k); st += [n for n in adj[k] if n in S]
            mem = sorted(P[k]['name'] for k in seen - n0)
            rep.setdefault('group_list', []).append((bname, P[s]['name'], len(mem)))
            rep.setdefault('group_members', []).append((bname, mem))
        external = sorted({P[n]['body'] for i in idx for n in adj[i] if P[n]['body'] != bname})
        rep['bodies'][bname] = {'parts': len(idx), 'groups': groups, 'linked_to': external}
        if not external and bname != 'frame' and bname not in RESTING:
            rep['unconnected'].append(f'body {bname} not attached to anything')
        if groups > 1:                                   # a resting body may be unattached, but it must still hold together
            rep['unconnected'].append(f'body {bname} falls apart into {groups} groups')
    for i, p in enumerate(P):
        if not adj[i] and not p['body'].startswith('link') and p['part'] != '3873' and 'ends' not in p and not (p['body'] in RESTING and len(bodies[p['body']]) == 1):
            rep['unconnected'].append(f"part {p['name']} ({p['part']}) has no connection")
    rep['n_parts'] = len(P); rep['n_conns'] = len(conns)
    if verbose:
        print(f"{len(P)} parts, {len(conns)} connected pairs, {len(rep['gears'])} gear meshes")
        print(f"CLASHES {len(rep['clashes'])}:"); [print('  ', c) for c in rep['clashes'][:60]]
        print(f"expected contacts {len(rep['expected'])}")
        print(f"BAD FITS {len(rep['bad_fits'])}:"); [print('  ', c) for c in rep['bad_fits'][:40]]
        print(f"UNCONNECTED {len(rep['unconnected'])}:"); [print('  ', c) for c in rep['unconnected'][:60]]
        for b, v in rep['bodies'].items(): print('  body', b, v)
        rep['groups_of'] = {}
    return rep

if __name__ == '__main__':
    mod = importlib.import_module(sys.argv[1] if len(sys.argv) > 1 else 'cell')
    P = mod.build()
    rep = run(P)
    if len(sys.argv) > 2: json.dump(rep, open(sys.argv[2], 'w'), indent=1, default=str)
    ok = not rep['clashes'] and not rep['bad_fits'] and not rep['unconnected']
    print('PASS' if ok else 'FAIL')
