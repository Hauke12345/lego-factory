import numpy as np, json, cv2
from scipy.optimize import minimize
exec(open('/home/claude/scripts/fitedges.py').read().split("def corners(prm):")[0])
def corners_e(prm):
    af, ab, bl, br, h = prm      # a front/back, b left/right (b larger = left in plate), top height
    out = []
    for z in (T0, h):
        for a, b in ((af, br), (ab, br), (ab, bl), (af, bl)):
            out.append(a*w + b*e2 + z*up)
    return np.array(out)
def sil_score(prm):
    P = corners_e(prm); tot = []
    for fr in frames:
        p = proj(fr, P)
        hull = cv2.convexHull(p.astype(np.float32)).reshape(-1, 2); vals = []
        for i in range(len(hull)):
            a, b = hull[i], hull[(i+1) % len(hull)]
            n = max(int(np.linalg.norm(b-a)/3), 2)
            v = sample(G[fr], a + (b-a)*np.linspace(0.05, 0.95, n)[:, None])
            if v is not None: vals.append(v)
        if vals: tot.append(np.median(np.concatenate(vals)))
    return -np.mean(tot)
book0 = [6.41, 11.75, -2.64, -7.88, -5.262]
box0 = [4.86, 12.1, 6.95, 0.53, -4.348]
res = json.load(open('/home/claude/sfm/fitted.json'))
for name, p0 in (('book_e', book0), ('box_e', box0)):
    s0 = sil_score(p0)
    r = minimize(sil_score, p0, method='Powell', options=dict(xtol=5e-3, ftol=1e-4, maxfev=2000))
    print(name, -s0, '->', -r.fun, np.round(p0, 2), '->', np.round(r.x, 2))
    res[name] = r.x.tolist()
json.dump(res, open('/home/claude/sfm/fitted.json', 'w'))
