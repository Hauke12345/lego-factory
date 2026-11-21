import numpy as np, pickle
from PIL import Image, ImageDraw
P=np.load('sfm/pts.npy'); C=np.load('sfm/cols.npy'); cams=pickle.load(open('sfm/cams.pkl','rb'))
up=np.load('sfm/up.npy')
w=np.array([0.433,-0.309,0.847]); w-=up*(w@up); w/=np.linalg.norm(w)   # toward wall?
e2=np.cross(up,w)
h=P@up; a=P@w; b=P@e2
ctr=np.array([-R.T@t for R,t in cams.values()])
print('wall dot', w@up)
import sys; sel=(h>float(sys.argv[1]))&(h<float(sys.argv[2]))
print('a range',np.percentile(a[sel],[2,98]),'b',np.percentile(b[sel],[2,98]))
print('cam a,b',np.percentile(ctr@w,[0,100]),np.percentile(ctr@e2,[0,100]))
lo=np.array([-7,-11]); sc=30
img=Image.new("RGB",(26*sc,26*sc),'white'); d=ImageDraw.Draw(img)
def px(aa,bb): return ((bb-lo[1])*sc,(aa-lo[0])*sc)
for i in np.where(sel)[0]:
    x,y=px(a[i],b[i]); t=(h[i]+5.25)/1.35
    col=tuple(int(v) for v in C[i])
    d.rectangle([x-1,y-1,x+1,y+1],fill=col)
for c in ctr: x,y=px(c@w,c@e2); d.point((x,y),fill=(0,160,0))
for k in range(-14,10,2):
    x,y=px(k,lo[1]); d.text((2,y),str(k),fill='black')
for k in range(-10,14,2):
    x,y=px(lo[0],k); d.text((x,2),str(k),fill='black')
img.save('test/topdown.png')
np.save('sfm/frame.npy',np.stack([w,e2,up]))
