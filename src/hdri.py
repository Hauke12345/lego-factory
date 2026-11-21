import numpy as np, imageio.v3 as iio
from PIL import Image
p=np.asarray(Image.open('/home/claude/hdri/pano.jpg').convert('RGB')).astype(np.float32)/255
lin=p**2.2
H,W=1024,2048
ph,pw=lin.shape[:2]
top,bot=50.0,-55.0           # elevation span of strip (deg)
v0=int((90-top)/180*H); v1=int((90-bot)/180*H)
strip=np.asarray(Image.fromarray((p*255).astype(np.uint8)).resize((W,v1-v0),Image.LANCZOS)).astype(np.float32)/255
strip=strip**2.2
# HDR boost for window (near white pixels)
lum=strip.mean(2,keepdims=True)
boost=1+ 40*np.clip((lum-0.75)/0.25,0,1)**2
strip=strip*boost
env=np.zeros((H,W,3),np.float32)
env[v0:v1]=strip
ceil=np.array([0.20,0.03,0.025])
floor=np.array([0.80,0.62,0.42])**2.2*0.9
rt=np.minimum(strip[:8].mean(0),0.25); rb=np.minimum(strip[-8:].mean(0),0.5)
for y in range(v0):
    t=(y/max(v0,1))**3; env[y]=ceil*(1-t)+rt*t
for y in range(v1,H):
    t=(y-v1)/max(H-v1,1); env[y]=rb*(1-t)+floor*t
np.save('/home/claude/hdri/env.npy',env)
