"""Per-frame camera refinement at full resolution.
Every frame gets its own PnP solve from 2D-3D correspondences:
 even frames: SIFT observations from the SfM (scaled to 1080p) refined with LK,
 odd frames : those observations tracked with LK from both neighbouring keyframes.
Writes sfm/allcams_ref.pkl in the same format as sfm/allcams.pkl."""
import numpy as np, pickle, cv2, pycolmap
r = pycolmap.Reconstruction('/home/claude/sfm/sparse/0')
Rint, Cint = pickle.load(open('/home/claude/sfm/allcams.pkl', 'rb'))
f, cx, cy, k = 633.637*1.5, 540.0, 960.0, -0.00368988843
K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]]); D = np.array([k, 0, 0, 0.0])
N = 642
P3 = {pid: p.xyz for pid, p in r.points3D.items()}
obs = {}
for im in r.images.values():
    i = int(im.name[1:5]); o = []
    for p2 in im.points2D:
        if p2.has_point3D(): o.append((p2.point3D_id, p2.xy*1.5 - 0.5))  # -> OpenCV pixel centres
    obs[i] = o
gray = {}
def G(i):
    if i not in gray:
        gray[i] = cv2.imread(f'/home/claude/plate/f{i:04d}.jpg', 0)
        if len(gray) > 6: gray.pop(min(gray))
    return gray[i]
lk = dict(winSize=(25, 25), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, 0.01))
def track(a, b, pts):
    nb, s1, _ = cv2.calcOpticalFlowPyrLK(G(a), G(b), pts, None, **lk)
    bk, s2, _ = cv2.calcOpticalFlowPyrLK(G(b), G(a), nb, None, **lk)
    ok = (s1[:, 0] == 1) & (s2[:, 0] == 1) & (np.linalg.norm(bk - pts, axis=1) < 0.3)
    return nb, ok

def residuals(R, C, X, x):
    rv, _ = cv2.Rodrigues(R); t = -R @ C
    p, _ = cv2.projectPoints(X, rv, t, K, D)
    return p.reshape(-1, 2) - x

Rout, Cout, stats = [], [], []
LOC = []
UP = np.load('/home/claude/sfm/up.npy')
keys = sorted(obs)
for i in range(N):
    corr = {}
    if i in obs:
        ids = np.array([o[0] for o in obs[i]]); xy = np.array([o[1] for o in obs[i]], np.float32)
        for pid, p in zip(ids, xy): corr[pid] = [p]
    for kf in (i-1, i+1):
        if kf in obs and kf != i:
            ids = np.array([o[0] for o in obs[kf]]); xy = np.array([o[1] for o in obs[kf]], np.float32)
            nb, ok = track(kf, i, xy)
            for pid, p, g in zip(ids, nb, ok):
                if g: corr.setdefault(pid, []).append(p)
    ids = [pid for pid, v in corr.items() if len(v) >= (1 if i in obs else 2)]
    xs = []
    keep = []
    for pid in ids:
        v = np.array(corr[pid])
        if len(v) > 1 and np.linalg.norm(v.max(0) - v.min(0)) > 1.0: continue   # disagreeing tracks
        xs.append(v.mean(0)); keep.append(pid)
    X = np.array([P3[p] for p in keep], np.float64); x = np.array(xs, np.float64)
    R0, C0 = Rint[i], Cint[i]
    e0 = np.linalg.norm(residuals(R0, C0, X, x), axis=1)
    inl = e0 < max(4.0, np.percentile(e0, 70))
    rv, _ = cv2.Rodrigues(R0); tv = -R0 @ C0
    for it in range(3):
        rv, tv = cv2.solvePnPRefineLM(X[inl], x[inl], K, D, rv, tv)
        R = cv2.Rodrigues(rv)[0]; C = -R.T @ tv.ravel()
        e = np.linalg.norm(residuals(R, C, X, x), axis=1)
        inl = e < 2.0
        if inl.sum() < 30: inl = e < np.percentile(e, 60)
    Rout.append(R); Cout.append(C)
    hh = X @ UP; loc = (np.abs(hh+4.35) < 0.15)
    if loc.sum() < 6: LOC.append(([np.nan]*2, [np.nan]*2)); continue
    r0 = residuals(R0, C0, X[loc], x[loc]); r1 = residuals(R, C, X[loc], x[loc])
    ok0 = np.linalg.norm(r0, axis=1) < 4
    LOC.append((np.median(r0[ok0], 0) if ok0.sum() > 5 else [np.nan]*2, np.median(r1[ok0], 0) if ok0.sum() > 5 else [np.nan]*2))
    stats.append((i, len(X), inl.sum(), np.median(e0), np.median(e[inl])))
    if i % 40 == 0: print(stats[-1], flush=True)
pickle.dump((np.array(Rout), np.array(Cout)), open('/home/claude/sfm/allcams_ref.pkl', 'wb'))
st = np.array(stats)
print('median err before: even %.2f odd %.2f | after: even %.2f odd %.2f' % (
    np.median(st[0::2, 3]), np.median(st[1::2, 3]), np.median(st[0::2, 4]), np.median(st[1::2, 4])))
np.save('/home/claude/sfm/refine_stats.npy', st)
np.save('/home/claude/sfm/loc_res.npy', np.array(LOC, float))
