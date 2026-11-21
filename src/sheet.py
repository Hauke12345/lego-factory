import cv2, numpy as np, OpenEXR, sys
frames=[int(x) for x in sys.argv[2:]]; out=sys.argv[1]
rows=[]
for fr in frames:
    ex=OpenEXR.File(f'render/wall/r{fr:04d}.exr')
    for p in ex.parts:
        for k,v in p.channels.items(): a=v.pixels[...,3]
    a=cv2.resize(a.astype(np.float32),(1080,1920)); col=(a>0.5).sum(0); yb=int(np.median(col[col>0]))   # wall bottom row
    y0=max(0,yb-170); y1=min(1920,y0+340)
    ims=[cv2.imread(f'plate/f{fr:04d}.jpg')[y0:y1], cv2.imread(f'render/ctest_r1/c{fr:04d}.jpg')[y0:y1]]
    r=np.hstack([cv2.resize(i,(540,170)) for i in ims])
    cv2.putText(r,str(fr),(8,24),0,0.8,(0,0,255),2); rows.append(r)
cv2.imwrite(out,np.vstack(rows))
