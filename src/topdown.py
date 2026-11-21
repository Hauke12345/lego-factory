import numpy as np, pickle
from PIL import Image, ImageDraw
P=np.load('sfm/pts.npy'); C=np.load('sfm/cols.npy'); cams=pickle.load(open('sfm/cams.pkl','rb'))
n=np.array([-0.162,-0.947,-0.276]); n/=np.linalg.norm(n)
camup=np.mean([R.T@np.array([0,-1,0]) for R,t in cams.values()],0)
if camup@n<0: n=-n
print('up',n)
h=P@n
ctr=np.array([-R.T@t for R,t in cams.values()])
print('cam heights',np.percentile(ctr@n,[0,50,100]))
hist,edges=np.histogram(h,bins=80,range=(np.percentile(h,2),np.percentile(h,98)))
for c,e in zip(hist,edges): print('%.2f %s'%(e,'#'*(c//20)))
np.save('sfm/up.npy',n)
