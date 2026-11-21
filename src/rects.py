import numpy as np, cv2
P=np.load('sfm/pts.npy'); F=np.load('sfm/frame.npy'); w,e2,up=F
a,b,h=P@w,P@e2,P@up
def rect(b0,b1,a0,a1,h0,h1):
    s=(b>b0)&(b<b1)&(a>a0)&(a<a1)&(h>h0)&(h<h1)
    pts=np.stack([a[s],b[s]],1).astype(np.float32)
    # robust: drop outliers far from median
    (cx,cy),(W,H),ang=cv2.minAreaRect(pts)
    # trim 1% extremes by projecting onto rect axes
    th=np.deg2rad(ang); R=np.array([[np.cos(th),-np.sin(th)],[np.sin(th),np.cos(th)]])
    q=(pts-[cx,cy])@R
    lo=np.percentile(q,0.7,0); hi=np.percentile(q,99.3,0)
    c=(lo+hi)/2; cen=np.array([cx,cy])+R@c
    return dict(center=cen,size=hi-lo,angle=ang,R=R,h=np.median(h[s]),n=s.sum())
box=rect(0.2,7.6,4.4,12.2,-4.55,-4.25)
book=rect(-8.8,-2.3,4.8,11.8,-5.42,-5.18)
for k,v in [('box',box),('book',book)]: print(k,{x:(np.round(y,3) if not isinstance(y,int) else y) for x,y in v.items()})
np.save('sfm/rects.npy',dict(box=box,book=book),allow_pickle=True)
