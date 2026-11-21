"""Refine box/book cuboids and the table-frame rim line against image edges across many frames."""
import numpy as np, pickle, cv2, json
from scipy.optimize import minimize
F = np.load('/home/claude/sfm/frame.npy'); w, e2, up = F
R, C = pickle.load(open('/home/claude/sfm/allcams_ref.pkl', 'rb'))
f, cx, cy, k = 633.637*1.5, 540, 960, -0.00369
K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.]]); D = np.array([k, 0, 0, 0.])
T0 = -6.55
frames = list(range(4, 642, 40))
G = {}; GX = {}; GY = {}; RID = {}
for fr in frames:
    g = cv2.cvtColor(cv2.imread(f'/home/claude/plate/f{fr:04d}.jpg'), cv2.COLOR_BGR2LAB).astype(np.float32)
    g = cv2.GaussianBlur(g, (0, 0), 1.2)
    gx = sum(cv2.Sobel(g[..., c], cv2.CV_32F, 1, 0, ksize=3)**2 for c in range(3))**0.5
    gy = sum(cv2.Sobel(g[..., c], cv2.CV_32F, 0, 1, ksize=3)**2 for c in range(3))**0.5
    G[fr] = np.hypot(gx, gy)
    L = g[..., 0]
    RID[fr] = L - 0.5*(np.roll(L, 6, 0) + np.roll(L, -6, 0))     # bright thin horizontal ridge
rv_t = {fr: (cv2.Rodrigues(R[fr])[0], -R[fr] @ C[fr]) for fr in frames}

def proj(fr, P):
    rv, t = rv_t[fr]
    p, _ = cv2.projectPoints(np.asarray(P, np.float64), rv, t, K, D); return p.reshape(-1, 2)

def sample(img, p):
    ok = (p[:, 0] > 1) & (p[:, 0] < 1078) & (p[:, 1] > 1) & (p[:, 1] < 1918)
    if ok.sum() < 5: return None
    return cv2.remap(img, p[ok, 0].astype(np.float32).reshape(-1, 1), p[ok, 1].astype(np.float32).reshape(-1, 1), cv2.INTER_LINEAR).ravel()

def corners(prm):
    ca, cb, L, W, yaw, h = prm
    c, s = np.cos(yaw), np.sin(yaw)
    out = []
    for z in (T0, h):
        for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            a = ca + c*sx*L/2 - s*sy*W/2; b = cb + s*sx*L/2 + c*sy*W/2
            out.append(a*w + b*e2 + z*up)
    return np.array(out)

def cuboid_score(prm):
    P = corners(prm); tot = []
    for fr in frames:
        p = proj(fr, P)
        hull = cv2.convexHull(p.astype(np.float32)).reshape(-1, 2)
        # also top-face outline (inner edges are visible too)
        polys = [hull, p[4:8]]
        vals = []
        for poly in polys:
            for i in range(len(poly)):
                a, b = poly[i], poly[(i+1) % len(poly)]
                n = max(int(np.linalg.norm(b-a)/3), 2)
                pts = a + (b-a)*np.linspace(0, 1, n)[:, None]
                v = sample(G[fr], pts)
                if v is not None: vals.append(v)
        if vals: tot.append(np.median(np.concatenate(vals)))
    return -np.mean(tot)

def rect_to_prm(rect, h):
    c = np.mean(rect, 0)
    ax = np.array(rect[2]) - np.array(rect[1]); ay = np.array(rect[0]) - np.array(rect[1])
    return [c[0], c[1], np.linalg.norm(ax), np.linalg.norm(ay), np.arctan2(ax[1], ax[0]), h]

RECTS = {'box': [(4.79, 6.88), (4.92, 0.46), (12.17, 0.60), (12.04, 7.02)],
         'book': [(6.27, -2.78), (6.55, -8.02), (11.89, -7.74), (11.61, -2.5)]}
res = {}
for name, h in (('box', -4.348), ('book', -5.262)):
    p0 = rect_to_prm(RECTS[name], h)
    s0 = cuboid_score(p0)
    r = minimize(cuboid_score, p0, method='Powell', options=dict(xtol=1e-3, ftol=1e-4, maxfev=1500))
    print(name, 'before', -s0, 'after', -r.fun, 'params', np.round(p0, 3), '->', np.round(r.x, 3))
    res[name] = r.x.tolist()

def rim_score(prm):
    a0, s, dz = prm; bs = np.linspace(-14, 10, 200); tot = []
    P = np.array([(a0 + s*b)*w + b*e2 + (T0+dz)*up for b in bs])
    for fr in frames:
        v = sample(RID[fr], proj(fr, P))
        if v is not None and len(v) > 20: tot.append(np.percentile(v, 60))
    return -np.mean(tot)
best = None
for a0 in np.arange(13.6, 16.6, 0.1):
    for s in np.arange(-0.12, 0.121, 0.02):
        sc = rim_score([a0, s, 0.1])
        if best is None or sc < best[0]: best = (sc, a0, s)
r = minimize(rim_score, [best[1], best[2], 0.1], method='Powell', options=dict(xtol=1e-3))
print('rim', best, '->', np.round(r.x, 3), -r.fun)
res['rim'] = r.x.tolist()
json.dump(res, open('/home/claude/sfm/fitted.json', 'w'))
