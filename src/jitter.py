"""Measure 2D misalignment: KLT-track real features on box/book tops through all full-res frames,
compare with projection of their 3D points using a given camera set."""
import numpy as np, pickle, cv2, pycolmap, sys
F=np.load('sfm/frame.npy'); w,e2,up=F
r=pycolmap.Reconstruction('sfm/sparse/0')
camfile=sys.argv[1]
Rall,Call=pickle.load(open(camfile,'rb'))
f,cx,cy,k=633.637*1.5,540,960,-0.00368988843
K=np.array([[f,0,cx],[0,f,cy],[0,0,1]]); D=np.array([k,0,0,0])
def proj(i,X):
    R=Rall[i]; t=-R@Call[i]; rv,_=cv2.Rodrigues(R)
    p,_=cv2.projectPoints(X,rv,t,K,D); return p.reshape(-1,2)
# 3D points on box top / book top with long tracks
P=[];ids=[]
for pid,p in r.points3D.items():
    h=p.xyz@up
    if (abs(h+4.35)<0.1 or abs(h+5.26)<0.1) and p.track.length()>=12: P.append(p.xyz)
P=np.array(P); print('pts',len(P))
gray=lambda i: cv2.imread(f'plate/f{i:04d}.jpg',0)
N=642; res=np.full((N,len(P),2),np.nan)
prev=gray(0); pts=proj(0,P).astype(np.float32); alive=np.ones(len(P),bool)
# start positions: refine projection in frame 0 is unnecessary; we measure relative motion residual
track=np.full((N,len(P),2),np.nan); track[0]=pts
for i in range(1,N):
    cur=gray(i)
    nxt,st,_=cv2.calcOpticalFlowPyrLK(prev,cur,pts,None,winSize=(21,21),maxLevel=3)
    back,st2,_=cv2.calcOpticalFlowPyrLK(cur,prev,nxt,None,winSize=(21,21),maxLevel=3)
    ok=(st[:,0]==1)&(st2[:,0]==1)&(np.linalg.norm(back-pts,axis=1)<0.5)
    alive&=ok; track[i]=np.where(alive[:,None],nxt,np.nan)
    pts=nxt; prev=cur
    # re-seed dead points from projection
    dead=~alive
    if dead.any():
        pr=proj(i,P[dead]).astype(np.float32); pts[dead]=pr; alive[dead]=True; track[i][dead]=np.nan
        # mark: offsets measured only from consistent tracks
pickle.dump((P,track),open('sfm/klt.pkl','wb'))
