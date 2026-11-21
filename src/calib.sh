cd /home/claude
MB=0 CALIB=1 ENVSTR=$1 python3 scripts/scene.py 300 301 prev >/dev/null 2>&1
python3 - <<'P'
import OpenEXR, numpy as np, cv2
ex=OpenEXR.File('render/prev/r0300.exr'); ch={}
for p in ex.parts:
    for k,v in p.channels.items(): ch[k]=v.pixels
r=ch['ViewLayer.Combined'][...,:3].astype(np.float32)
pl=cv2.imread('plate/f0300.jpg')[:,:,::-1].astype(np.float32)/255
pl=cv2.resize(pl,(r.shape[1],r.shape[0]),interpolation=cv2.INTER_AREA); lin=np.where(pl<=0.04045,pl/12.92,((pl+0.055)/1.055)**2.4)
for n,(x0,x1,y0,y1) in {'boxtop':(30,120,262,285),'boxfront':(40,110,340,430),'bookfront':(300,350,330,400)}.items():
    a=r[y0:y1,x0:x1].reshape(-1,3).mean(0); b=lin[y0:y1,x0:x1].reshape(-1,3).mean(0)
    print(n,'ratio plate/render',np.round(b/a,2))
P
