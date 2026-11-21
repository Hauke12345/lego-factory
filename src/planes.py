import numpy as np
P=np.load('sfm/pts.npy'); C=np.load('sfm/cols.npy')
rng=np.random.default_rng(0)
def ransac(X,th,it=3000):
    best=None
    for _ in range(it):
        s=X[rng.choice(len(X),3,replace=False)]
        n=np.cross(s[1]-s[0],s[2]-s[0]); nn=np.linalg.norm(n)
        if nn<1e-9: continue
        n/=nn; d=-n@s[0]; inl=np.abs(X@n+d)<th
        if best is None or inl.sum()>best[2].sum(): best=(n,d,inl)
    n,d,inl=best
    Y=X[inl]; c=Y.mean(0); u,s,vt=np.linalg.svd(Y-c); n=vt[2]; d=-n@c
    inl=np.abs(X@n+d)<th
    return n,d,inl
X=P.copy(); idx=np.arange(len(P)); out=[]
for k in range(6):
    n,d,inl=ransac(X,0.05)
    print(k,'n=',np.round(n,3),'d=%.3f'%d,'cnt',inl.sum(),'meancol',C[idx[inl]].mean(0).astype(int), 'ctr',np.round(X[inl].mean(0),2))
    out.append((n,d,idx[inl])); X=X[~inl]; idx=idx[~inl]
import pickle; pickle.dump(out,open('sfm/planes.pkl','wb'))
