import numpy as np, cv2, pickle, sys
F=np.load('sfm/frame.npy'); w,e2,up=F
cams=pickle.load(open('sfm/cams.pkl','rb')); R_=np.load('sfm/rects.npy',allow_pickle=True).item()
f,cx,cy=633.637,360,640
T=float(sys.argv[1])
def corners(r):
    c=r['center']; s=r['size']/2; R=r['R']
    return [c+R@np.array([sx*s[0],sy*s[1]]) for sx,sy in [(-1,-1),(1,-1),(1,1),(-1,1)]]
def W(ab,hh): return ab[0]*w+ab[1]*e2+hh*up
outs=[]
for name in sys.argv[2:]:
    Rm,t=cams[name]; img=cv2.imread('sfm/images/'+name)
    def pj(X):
        x=Rm@X+t; return (int(f*x[0]/x[2]+cx),int(f*x[1]/x[2]+cy))
    for key,col in [('box',(0,255,0)),('book',(255,0,255))]:
        r=R_[key]; cs=corners(r)
        for i in range(4):
            cv2.line(img,pj(W(cs[i],r['h'])),pj(W(cs[(i+1)%4],r['h'])),col,2)
            cv2.line(img,pj(W(cs[i],T)),pj(W(cs[(i+1)%4],T)),(0,255,255),1)
            cv2.line(img,pj(W(cs[i],r['h'])),pj(W(cs[i],T)),col,1)
    outs.append(cv2.resize(img,(360,640)))
cv2.imwrite('test/overlay.jpg',np.hstack(outs))
