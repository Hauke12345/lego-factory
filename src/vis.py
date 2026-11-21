import numpy as np, pickle
from scipy.spatial.transform import Rotation as Rot, Slerp
F=np.load('sfm/frame.npy'); w,e2,up=F
cams=pickle.load(open('sfm/cams.pkl','rb'))
idx=sorted(int(k[1:5]) for k in cams)
Rs=Rot.from_matrix([cams[f'f{i:04d}.jpg'][0] for i in idx])
C=np.array([-cams[f'f{i:04d}.jpg'][0].T@cams[f'f{i:04d}.jpg'][1] for i in idx])
sl=Slerp(idx,Rs); N=642; fr=np.clip(np.arange(N),idx[0],idx[-1])
Rall=sl(fr).as_matrix(); Call=np.stack([np.interp(fr,idx,C[:,k]) for k in range(3)],1)
pickle.dump((Rall,Call),open('sfm/allcams.pkl','wb'))
f,cx,cy=633.637,360,640
pts={'build':(8.5,4.5,-4.35),'edge':(8.5,0.1,-4.35),'gap':(8.5,-1.2,-5.3),'land':(8.5,-4.0,-5.26),'stop':(8.5,-6.0,-5.26)}
print('frame '+' '.join(pts))
for i in range(0,N,15):
    s=[]
    for k,(a,b,h) in pts.items():
        X=a*w+b*e2+h*up; x=Rall[i]@(X-Call[i]); u=f*x[0]/x[2]+cx; v=f*x[1]/x[2]+cy
        s.append('Y' if x[2]>0 and 20<u<700 and 20<v<1260 else '.')
    print(i,' '.join(s))
