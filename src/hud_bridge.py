"""Engineering annotation overlay drawn onto composited frames."""
from PIL import Image, ImageDraw, ImageFont
import math

RED = (232, 68, 46)
WHITE = (250, 247, 240)
FONT = '/usr/share/fonts/opentype/inter/Inter-SemiBold.otf'
MONO = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'
try:
    F_LBL = ImageFont.truetype(FONT, 34); F_TTL = ImageFont.truetype(FONT, 40); F_TXT = ImageFont.truetype(MONO, 30); F_TAG = ImageFont.truetype(FONT, 30)
except OSError:
    F_LBL = F_TTL = F_TXT = F_TAG = ImageFont.load_default()

def ramp(f, a, b):
    return 0.0 if f <= a else 1.0 if f >= b else (f - a)/(b - a)

PLAN = (196, 345)          # planning overlay window
JUMP = (99999, 99999)   # predicted path during jump disabled          # predicted path during the jump

def _px(p, W, H): return (p[0]*W, p[1]*H)

def _line(d, a, b, t, col, w=4):
    if t <= 0: return
    bx = a[0] + (b[0]-a[0])*t; by = a[1] + (b[1]-a[1])*t
    d.line([a, (bx, by)], fill=(0, 0, 0, int(col[3]*0.5)), width=w+4)
    d.line([a, (bx, by)], fill=col, width=w)

def _text(d, xy, s, font, col, anchor='mm'):
    x, y = xy
    for dx, dy in ((-2, 0), (2, 0), (0, -2), (0, 2), (2, 2)):
        d.text((x+dx, y+dy), s, font=font, fill=(0, 0, 0, int(col[3]*0.7)), anchor=anchor)
    d.text(xy, s, font=font, fill=col, anchor=anchor)

def _dashed(d, pts, t, col, w=4, dash=18, gap=12):
    # draw polyline up to fraction t of its length, dashed
    L = [0]
    for i in range(1, len(pts)): L.append(L[-1] + math.dist(pts[i-1], pts[i]))
    if L[-1] == 0: return
    end = L[-1]*t; s = 0.0; on = True
    while s < end:
        e = min(s + (dash if on else gap), end)
        if on:
            seg = []
            for x in (s, e):
                i = max(1, min(len(L)-1, next((k for k in range(1, len(L)) if L[k] >= x), len(L)-1)))
                u = (x - L[i-1])/max(L[i]-L[i-1], 1e-9)
                seg.append((pts[i-1][0] + (pts[i][0]-pts[i-1][0])*u, pts[i-1][1] + (pts[i][1]-pts[i-1][1])*u))
            d.line(seg, fill=(0, 0, 0, int(col[3]*0.5)), width=w+4); d.line(seg, fill=col, width=w)
        s = e; on = not on

def _arrow(d, a, b, t, col, w=4):
    if t <= 0: return
    e = (a[0] + (b[0]-a[0])*t, a[1] + (b[1]-a[1])*t)
    _line(d, a, e, 1, col, w)
    ang = math.atan2(e[1]-a[1], e[0]-a[0])
    for s_ in (2.6, -2.6):
        p = (e[0] + 22*math.cos(ang+s_), e[1] + 22*math.sin(ang+s_)); _line(d, e, p, 1, col, w)

# ---------------- bridge-builder style stress view (live FEM, synced to the truck) ----------------
import json, sys
import numpy as np
sys.path.insert(0, '/home/claude/scripts')
import bridge as BRG
STRESS = (360, 642)                 # frames the panel is on screen
T_GO = 393; FPS = 30
_fem = _pan = _tr = _fine = None
def _load():
    global _fem, _pan, _tr, _fine
    if _fem is None:
        _fem = json.load(open('/home/claude/render_b/fem.json')); _pan = json.load(open('/home/claude/render_b/fem_panel.json'))
        _pan = _pan.get('bridge', _pan)
        globals()['_ff'] = json.load(open('/home/claude/render_b/fem_frames.json'))['frames']
        _tr = json.load(open('/home/claude/render_b/truck_parts.json'))
        _fine = np.load('/home/claude/render_b/traj.npz')['fine']
UMAX = 0.20
def _col(u, a):
    t = max(0.0, min(1.0, u/UMAX))
    stops = [(0, (64, 196, 98)), (0.5, (246, 206, 46)), (1, (232, 62, 40))]
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        if t <= t1:
            k = (t - t0)/(t1 - t0); return tuple(int(c0[i] + (c1[i]-c0[i])*k) for i in range(3)) + (a,)
    return stops[-1][1] + (a,)
def truck_pose(f):
    """root position + rotation of the truck at frame f (same MuJoCo trajectory the render uses)"""
    if f < T_GO:
        return np.array([0.0, 0.20, BRG.Z_BOX]), np.array([[0, 1.0, 0], [-1.0, 0, 0], [0, 0, 1.0]])
    t = (f - T_GO)/FPS; F = _fine
    i = int(np.clip(np.searchsorted(F[:, 0], t), 1, len(F) - 1)); u = np.clip((t - F[i-1, 0])/max(F[i, 0] - F[i-1, 0], 1e-9), 0, 1)
    pos = F[i-1, 1:4]*(1 - u) + F[i, 1:4]*u
    q = F[i-1, 4:8]*(1 - u) + F[i, 4:8]*u; q /= np.linalg.norm(q); w, x, y, z = q
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
    return pos, R
def fem_at(yc):
    cases = _fem['cases']; ys = [c['y'] for c in cases[1:]]
    if yc > ys[0] or yc < ys[-1]: return cases[0]['util'], cases[0]['defl_mm']
    j = next(k for k in range(1, len(ys)) if ys[k] <= yc); y0, y1 = ys[j-1], ys[j]; w = (y0 - yc)/(y0 - y1)
    c0, c1 = cases[j], cases[j+1]
    return {p: c0['util'][p]*(1-w) + c1['util'][p]*w for p in c0['util']}, c0['defl_mm']*(1-w) + c1['defl_mm']*w
_peak = {}
def stress_panel(d, W, H, f):
    _load()
    a = ramp(f, STRESS[0], STRESS[0] + 10)
    if a <= 0: return
    A = int(255*a)
    fr = _ff.get(str(min(f, 641)))
    if fr is None: pos, R = truck_pose(f); util, sag = fem_at(pos[1])
    else:
        pos = np.array(fr['pos']); w, x, y, z = fr['quat']
        R = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
        util, sag = fr['util'], fr['defl_mm']
    um = max(util.values())
    pk = max([um] + [max(_ff[str(g)]['util'].values()) for g in range(STRESS[0], f) if str(g) in _ff])
    px0, py0, pw, ph = 40, 1270, W - 80, 600
    d.rounded_rectangle([px0, py0, px0 + pw, py0 + ph], 22, fill=(18, 18, 22, int(205*a)))
    d.text((px0 + 30, py0 + 36), 'STRESS VIEW', font=F_TTL, fill=(*RED, A), anchor='lm')
    d.text((px0 + 340, py0 + 38), 'live FEM · truck 107 g', font=F_TXT, fill=(*WHITE, int(A*0.8)), anchor='lm')
    Y0, Y1, Z0 = 0.285, -0.335, -0.002
    gx0, gx1 = px0 + 30, px0 + pw - 30; gy1 = py0 + ph - 95; s = (gx1 - gx0)/(Y0 - Y1)
    P_ = lambda y, z: (gx0 + (Y0 - y)*s, gy1 - (z - Z0)*s)
    d.line([P_(Y0, 0), P_(Y1, 0)], fill=(150, 170, 190, int(A*0.7)), width=3)
    d.rectangle([P_(Y0, BRG.Z_BOX), P_(BRG.Y_E, 0)], fill=(120, 86, 52, int(A*0.55)))
    d.rectangle([P_(BRG.Y_K, BRG.Z_BOOK), P_(Y1, 0)], fill=(150, 40, 36, int(A*0.55)))
    from scipy.spatial import ConvexHull
    items = []                                   # (depth x, polygon, fill, outline) for bridge + truck, far first
    for nm in _pan:
        c = _col(util.get(nm, 0), A)
        if _pan[nm]['x'] > 0.03: c = tuple(int(v*0.55) for v in c[:3]) + (A,)
        items.append((_pan[nm]['x'], [P_(y, z) for y, z in _pan[nm]['poly']], c, (10, 10, 12, A)))
    for p in _tr:                                # the real truck parts, posed like in the render
        w_ = (R @ np.array(p['verts']).T).T + pos
        yz = w_[:, 1:3]
        try: yz = yz[ConvexHull(yz).vertices]
        except Exception: pass
        al = int(A*p['alpha']); col = tuple(p['col']); xm = w_[:, 0].mean()
        if xm > 0.012: col = tuple(int(v*0.6) for v in col)
        items.append((xm, [P_(y, z) for y, z in yz], col + (al,), (12, 12, 14, al)))
    for xm, poly, fc, oc in sorted(items, key=lambda t: -t[0]):
        d.polygon(poly, fill=fc, outline=oc)
    lx, ly = px0 + 30, py0 + ph - 48
    for i in range(300): d.line([lx + i, ly - 12, lx + i, ly + 12], fill=_col(UMAX*i/299, A))
    for v, lab in ((0, '0%'), (0.5, '10%'), (1, '20%')):
        d.text((lx + v*299, ly + 30), lab, font=F_TXT, fill=(*WHITE, int(A*0.8)), anchor='mm')
    d.text((lx + 320, ly), 'of capacity', font=F_TXT, fill=(*WHITE, int(A*0.8)), anchor='lm')
    d.text((px0 + pw - 30, ly - 22), f'now {um*100:4.1f}%   peak {pk*100:4.1f}%', font=F_TXT, fill=(*WHITE, A), anchor='rm')
    d.text((px0 + pw - 30, ly + 16), f'sag {abs(sag):.3f} mm', font=F_TXT, fill=(*WHITE, int(A*0.8)), anchor='rm')

def draw(img, f, h):
    """img: PIL RGB full-res frame, h: hud dict for this frame (normalized coords)."""
    if not h: return img
    W, H = img.size
    plan = ramp(f, PLAN[0], PLAN[0]+1) * (1 - ramp(f, PLAN[1]-15, PLAN[1]))
    if f >= STRESS[0]:
        ov = Image.new('RGBA', img.size, (0, 0, 0, 0)); stress_panel(ImageDraw.Draw(ov), W, H, f)
        return Image.alpha_composite(img.convert('RGBA'), ov).convert('RGB')
    if plan <= 0: return img
    ov = Image.new('RGBA', img.size, (0, 0, 0, 0)); d = ImageDraw.Draw(ov)
    vis = lambda *ks: all(h[k][2] > 0 for k in ks)
    A = int(255*plan); red = (*RED, A); wht = (*WHITE, A)
    P_ = lambda k: _px(h[k], W, H)
    if vis('gapA', 'gapB'):
        t = ramp(f, 196, 210); g0, g1 = P_('gapA'), P_('gapB')
        _line(d, g0, g1, t, red)
        for p in (g0, g1):
            if t > 0.05: _line(d, (p[0], p[1]-18), (p[0], p[1]+18), 1, red, 3)
        if t >= 1: _text(d, ((g0[0]+g1[0])/2, max(g0[1], g1[1]) + 40), 'span 14 cm', F_LBL, wht)
    if vis('dropT', 'dropB'):
        t = ramp(f, 206, 218); a, b = P_('dropT'), P_('dropB')
        _line(d, a, b, t, red, 3)
        if t >= 1: _text(d, (b[0] + 16, (a[1]+b[1])/2), 'Δh 4.0 cm', F_LBL, wht, 'lm')
    if vis('slope0', 'slope1', 'slopeH'):
        t = ramp(f, 212, 224); a, b, c = P_('slope0'), P_('slope1'), P_('slopeH')
        _line(d, a, b, t, red, 3); _line(d, a, c, t, (*RED, int(A*0.6)), 2)
        if t >= 1: _text(d, (a[0] + 0.75*(b[0]-a[0]), min(a[1], b[1]) - 150), 'deck 15.3°', F_LBL, wht)
    if vis('pierT', 'pierB'):
        t = ramp(f, 218, 230); a, b = P_('pierT'), P_('pierB')
        _arrow(d, a, b, t, red, 4)
        if t >= 1: _text(d, (b[0] - 16, b[1] + 6), 'pier 0.5 N', F_LBL, wht, 'rm')
    pts = [_px(p, W, H) for p in h['path'] if p[2] > 0]
    if len(pts) > 2: _dashed(d, pts, ramp(f, 222, 252), red, 5)
    lines = [('BRIDGE PLAN', None), ('span', '14.0 cm'), ('drop', '4.0 cm'), ('deck', '19 studs'),
             ('slope', '15.3°'), ('pier', '7 + hinge'), ('truck', '107 g'), ('speed', '0.5 m/s')]
    x0, y0 = 60, 1460
    n_show = int(ramp(f, 226, 262)*len(lines) + 0.999)
    if n_show:
        hgt = 30 + 46*n_show
        d.rounded_rectangle([x0-24, y0-30, x0+430, y0-30+hgt+20], 18, fill=(20, 18, 16, int(150*plan)))
        for i, (k, v) in enumerate(lines[:n_show]):
            y = y0 + 46*i
            if v is None: d.text((x0, y), k, font=F_TTL, fill=(*RED, A), anchor='lm')
            else:
                d.text((x0, y), k, font=F_TXT, fill=(*WHITE, int(A*0.8)), anchor='lm')
                d.text((x0+390, y), v, font=F_TXT, fill=wht, anchor='rm')
    return Image.alpha_composite(img.convert('RGBA'), ov).convert('RGB')
