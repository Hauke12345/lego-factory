"""Fit the white table-frame back edge as a 3D line a(b)=a0+s*b at height T0+dz by maximizing whiteness along its projection."""
import numpy as np, pickle, cv2, itertools
F=np.load('sfm/frame.npy'); w,e2,up=F
R,C=pickle.load(open('sfm/allcams_ref.pkl','rb'))
f,cx,cy,k=633.637*1.5,540,960,-0.00369
K=np.array([[f,0,cx],[0,f,cy],[0,0,1.]]); D=np.array([k,0,0,0.])
frames=list(range(0,642,40))
imgs={}
for fr in frames:
    im=cv2.imread(f'plate/f{fr:04d}.jpg').astype(np.float32)/255
    mn=im.min(2); mx=im.max(2)
    white=np.clip((mn-0.55)*4,0,1)*np.clip(1-(mx-mn)*4,0,1)       # bright & unsaturated
    imgs[fr]=cv2.GaussianBlur(white,(0,0),1.5)
bs=np.linspace(-13,9,120)
def score(a0,s,dz,detail=False):
    tot=[];
    for fr in frames:
        rv,_=cv2.Rodrigues(R[fr]); t=-R[fr]@C[fr]
        P=np.array([(a0+s*b)*w+b*e2+(-6.55+dz)*up for b in bs])
        p,_=cv2.projectPoints(P,rv,t,K,D); p=p.reshape(-1,2)
        ok=(p[:,0]>2)&(p[:,0]<1077)&(p[:,1]>2)&(p[:,1]<1917)
        if ok.sum()<10: continue
        v=imgs[fr][p[ok,1].astype(int),p[ok,0].astype(int)]
        tot.append(np.percentile(v,60))
    return np.mean(tot)
best=None
for a0 in np.arange(13.6,16.6,0.1):
    for s in np.arange(-0.12,0.121,0.02):
        for dz in (0.0,0.15,0.3):
            sc=score(a0,s,dz)
            if best is None or sc>best[0]: best=(sc,a0,s,dz)
print('coarse',best)
sc0,a0,s0,dz0=best
for a in np.arange(a0-0.12,a0+0.121,0.03):
    for s in np.arange(s0-0.02,s0+0.021,0.005):
        for dz in np.arange(max(-0.1,dz0-0.12),dz0+0.121,0.04):
            sc=score(a,s,dz)
            if sc>best[0]: best=(sc,a,s,dz)
print('fine',best)
np.save('sfm/rim.npy',np.array(best[1:]))
