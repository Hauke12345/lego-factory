"""LDraw part loader: geometry (triangles + colours) and LDCad-shadow snap points (connections).
Units: LDU (1 LDU = 0.4 mm), LDraw axes (-Y is up). Use to_blender() for metres / Z-up.
"""
import os, re, functools, numpy as np, pickle

ROOT = '/home/claude/ldraw/lib/complete/ldraw'
SHADOW = '/home/claude/ldraw/shadow'
CACHE = '/home/claude/ldraw/cache'; os.makedirs(CACHE, exist_ok=True)
LDU = 0.0004
# LDraw (x, y, z), -y up  ->  Blender (x, -z, -y)
C_BL = np.array([[1, 0, 0], [0, 0, -1], [0, -1, 0]], float)

def _index(dirs):
    idx = {}
    for d in dirs:
        for dp, _, fs in os.walk(d):
            for f in fs:
                rel = os.path.relpath(os.path.join(dp, f), d).replace(os.sep, '\\').lower()
                idx.setdefault(rel, os.path.join(dp, f))
    return idx
_LIB = _index([f'{ROOT}/parts', f'{ROOT}/p'])
_SHD = _index([f'{SHADOW}/parts', f'{SHADOW}/p'])

def _colours():
    col = {}
    for ln in open(f'{ROOT}/LDConfig.ldr', encoding='utf-8', errors='ignore'):
        m = re.search(r'!COLOUR\s+(\S+)\s+CODE\s+(\d+)\s+VALUE\s+#([0-9A-Fa-f]{6})', ln)
        if m:
            h = m.group(3); a = re.search(r'ALPHA\s+(\d+)', ln)
            col[int(m.group(2))] = (m.group(1), tuple(int(h[i:i+2], 16)/255 for i in (0, 2, 4)), (int(a.group(1))/255 if a else 1.0))
    return col
COLOURS = _colours()

def colour(code):
    return COLOURS.get(code, ('?', (0.5, 0.5, 0.5), 1.0))

def _lines(path):
    with open(path, encoding='utf-8', errors='ignore') as f:
        return [l.strip() for l in f]

def _kv(line):
    return {k.lower(): v.strip() for k, v in re.findall(r'\[(\w+)=([^\]]*)\]', line)}

def _mat(vals):
    x, y, z, a, b, c, d, e, f, g, h, i = map(float, vals)
    M = np.eye(4); M[:3, :3] = [[a, b, c], [d, e, f], [g, h, i]]; M[:3, 3] = [x, y, z]
    return M

def _secs(s):
    t = s.split(); out = []
    for k in range(0, len(t), 3): out.append((t[k], float(t[k+1]), float(t[k+2])))
    return out

def _grid(s):
    """LDCad grid: 'C 4 C 2 20 20' (x,z) or '1 C 2 1 0 120 0' (x,y,z) -> list of (dx, dy, dz)"""
    t = s.split(); vals = []; cen = []; i = 0
    while i < len(t):
        if t[i] == 'C': cen.append(True); vals.append(float(t[i+1])); i += 2
        else: cen.append(False); vals.append(float(t[i])); i += 1
    if len(vals) == 4: n = [vals[0], 1, vals[1]]; st = [vals[2], 0, vals[3]]; cc = [cen[0], False, cen[1]]
    else: n = vals[:3]; st = vals[3:6]; cc = cen[:3]
    o = [-(n[k]-1)*st[k]/2 if cc[k] else 0 for k in range(3)]
    return [(o[0]+i*st[0], o[1]+j*st[1], o[2]+k*st[2]) for i in range(int(n[0])) for j in range(int(n[1])) for k in range(int(n[2]))]

def _shadow_snaps(name, M, depth=0):
    """snap records from a shadow file (with SNAP_INCL), transformed by M. Returns (snaps, clear)."""
    p = _SHD.get(name.lower())
    if p is None or depth > 8: return [], False
    snaps = []; clear = False
    for l in _lines(p):
        if not l.startswith('0 !LDCAD SNAP_'): continue
        kind = l.split()[2]; kv = _kv(l)
        pos = np.array(list(map(float, kv.get('pos', '0 0 0').split())))
        R = np.array(list(map(float, kv.get('ori', '1 0 0 0 1 0 0 0 1').split()))).reshape(3, 3)
        L = np.eye(4); L[:3, :3] = R; L[:3, 3] = pos
        try: offs = _grid(kv['grid']) if 'grid' in kv else [(0, 0)]
        except Exception: offs = [(0, 0, 0)]
        if 'grid' not in kv: offs = [(0, 0, 0)]
        for dx, dy, dz in offs:
            G = np.eye(4); G[:3, 3] = [dx, dy, dz]
            W = M @ L @ G
            if kind == 'SNAP_CLEAR': clear = True
            elif kind == 'SNAP_INCL':
                s2, _ = _shadow_snaps(kv['ref'], W, depth+1); snaps += s2
            elif kind == 'SNAP_CYL':
                snaps.append({'kind': 'cyl', 'id': kv.get('id', ''), 'gender': kv.get('gender', 'M'),
                              'secs': _secs(kv.get('secs', 'R 6 4')), 'center': kv.get('center', 'false') == 'true',
                              'slide': kv.get('slide', 'false') == 'true', 'scale': kv.get('scale', ''),
                              'M': W, 'src': name})
            else:
                snaps.append({'kind': kind[5:].lower(), 'id': kv.get('id', ''), 'gender': kv.get('gender', 'M'), 'M': W, 'src': name})
    return snaps, clear

@functools.lru_cache(maxsize=None)
def _load_raw(name):
    """recursive: returns (tri verts (N,3,3), tri colour codes (N,), snaps list) in the file's own frame."""
    p = _LIB.get(name.lower())
    if p is None: return np.zeros((0, 3, 3)), np.zeros(0, int), []
    tris = []; cols = []; snaps = []
    own, clear = _shadow_snaps(name, np.eye(4))
    inv_next = False; cw = False
    for l in _lines(p):
        if l.startswith('0 BFC'):
            if 'INVERTNEXT' in l: inv_next = True
            elif ' CW' in l and 'CCW' not in l: cw = True
            elif 'CCW' in l: cw = False
            continue
        if not l or l[0] not in '134': continue
        t = l.split()
        if t[0] == '1' and len(t) >= 15:
            c = int(t[1]); M = _mat(t[2:14]); sub = ' '.join(t[14:])
            V, C, S = _load_raw(sub)
            if len(V):
                V2 = V @ M[:3, :3].T + M[:3, 3]
                if (np.linalg.det(M[:3, :3]) < 0) != inv_next: V2 = V2[:, ::-1]
                tris.append(V2); C2 = C.copy(); C2[C2 == 16] = c; cols.append(C2)
            inv_next = False
            if not clear:
                for s in S:
                    s2 = dict(s); s2['M'] = M @ s['M']; snaps.append(s2)
        elif t[0] == '3' and len(t) >= 11:
            v = np.array(list(map(float, t[2:11]))).reshape(3, 3)
            if cw: v = v[::-1]
            tris.append(v[None]); cols.append(np.array([int(t[1])]))
        elif t[0] == '4' and len(t) >= 14:
            v = np.array(list(map(float, t[2:14]))).reshape(4, 3)
            if cw: v = v[::-1]
            tris.append(np.stack([v[[0, 1, 2]], v[[0, 2, 3]]])); cols.append(np.array([int(t[1])]*2))
    snaps = own + snaps
    if not tris: return np.zeros((0, 3, 3)), np.zeros(0, int), snaps
    return np.concatenate(tris), np.concatenate(cols), snaps

def part(name):
    """cached: dict(tris (N,3,3) LDU, cols (N,), snaps, title)"""
    name = name.lower() if name.lower().endswith('.dat') else name.lower() + '.dat'
    fn = os.path.join(CACHE, name.replace('\\', '_') + '.pkl')
    if os.path.exists(fn): return pickle.load(open(fn, 'rb'))
    V, C, S = _load_raw(name)
    title = _lines(_LIB[name])[0][2:] if name in _LIB else name
    d = {'tris': V.astype(np.float32), 'cols': C, 'snaps': S, 'title': title}
    pickle.dump(d, open(fn, 'wb'))
    return d

# ---------- snap geometry helpers ----------
def cyl_world(s, Mpart):
    """world-space cylinder: (start point, unit axis, list of (shape, radius, t0, t1) along axis), LDU.
    LDCad cylinders start at pos and run along the snap frame's -Y (centred when center=true)."""
    W = Mpart @ s['M']
    A = W[:3, :3]; ay = A[:, 1]; ly = np.linalg.norm(ay)
    sy = ly if s.get('scale', '').lower() in ('yonly', 'ylen') else 1.0
    rs = np.linalg.norm(A[:, 0])
    total = sum(x[2] for x in s['secs'])*sy
    o = W[:3, 3].copy(); u = -ay/ly          # LDCad: sections run along the snap frame's -Y
    if s['center']: o = o - u*total/2
    out = []; t = 0
    for shp, r, ln in s['secs']:
        out.append((shp, r*rs, t, t + ln*sy)); t += ln*sy
    return o, u, out
