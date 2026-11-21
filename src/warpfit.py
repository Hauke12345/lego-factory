import numpy as np, pickle, cv2
CORR = pickle.load(open('/home/claude/sfm/corr.pkl', 'rb'))
R, C = pickle.load(open('/home/claude/sfm/allcams_ref.pkl', 'rb'))
up = np.load('/home/claude/sfm/up.npy')
f = 633.637*1.5; K = np.array([[f, 0, 539.5], [0, f, 959.5], [0, 0, 1.]])
def basis(p):
    u = (p[:, 0]-540)/540; v = (p[:, 1]-960)/960
    return np.stack([np.ones_like(u), u, v, u*u, u*v, v*v, u*v*v, v*v*v], 1)
import os
LAM = float(os.environ.get('LAM', '1'))
coef = np.zeros((642, 8, 2)); st = []
for i, (X, x) in enumerate(CORR):
    rv, _ = cv2.Rodrigues(R[i]); t = -R[i] @ C[i]
    p, _ = cv2.projectPoints(X, rv, t, K, None); p = p.reshape(-1, 2)
    d = x - p; A = basis(p)
    w = np.ones(len(d)); good = np.linalg.norm(d, axis=1) < 6
    for it in range(6):
        W = (w*good)[:, None]
        AW = A*W; lam = np.diag([0, 2, 2, 30, 30, 30, 80, 80]).astype(float)*LAM
        c = np.linalg.solve(AW.T@AW + lam, AW.T@(d*W))
        r = np.linalg.norm(d - A@c, axis=1)
        w = 1/np.maximum(1, r/0.8)            # Huber
    coef[i] = c
    loc = np.abs(X@up + 4.35) < 0.15
    st.append((np.median(np.linalg.norm(d[good], axis=1)), np.median(r[good]),
               np.median(np.linalg.norm(d[good & loc], axis=1)) if (good & loc).sum() > 5 else np.nan,
               np.median(r[good & loc]) if (good & loc).sum() > 5 else np.nan))
st = np.array(st)
print('all points   median px: rigid %.2f -> warped %.2f' % tuple(np.nanmedian(st[:, :2], 0)))
print('box-top pts  median px: rigid %.2f -> warped %.2f' % tuple(np.nanmedian(st[:, 2:], 0)))
print('worst frames rigid p95 %.2f -> warped %.2f' % (np.nanpercentile(st[:, 0], 95), np.nanpercentile(st[:, 1], 95)))
np.save('/home/claude/sfm/warp.npy', coef)

def dd(u,v): B=np.array([1,u,v,u*u,u*v,v*v,u*v*v,v*v*v]); return np.einsum('k,fkj->fj',B,coef)
for name,(u,v) in {'centre':(0,0),'top-left':(-0.9,-0.9),'bottom-right':(0.9,0.9)}.items():
    x=dd(u,v); j=np.linalg.norm(np.diff(x,axis=0),axis=1)
    print(f'  {name:12s} frame-to-frame change median {np.median(j):.2f} p95 {np.percentile(j,95):.2f}')
