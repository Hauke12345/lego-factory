"""Cut die-cut stickers out of a magenta sheet -> individual RGBA PNGs (soft, despilled edges)."""
import sys, numpy as np, cv2
from PIL import Image
src = sys.argv[1]; out = sys.argv[2]
im = np.array(Image.open(src).convert('RGB')).astype(np.float32)
r, g, b = im[..., 0], im[..., 1], im[..., 2]
mag = np.clip((np.minimum(r, b) - g)/255.0, 0, 1)            # magenta-ness
alpha = np.clip(1 - (mag - 0.35)/0.35, 0, 1)                    # >0.7 magenta -> transparent
fg = (alpha > 0.5).astype(np.uint8)
fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
n, lab, st, _ = cv2.connectedComponentsWithStats(fg)
# despill: pull residual magenta out of edge pixels
spill = np.clip(np.minimum(r, b) - g, 0, None)*(1 - alpha)
rgb = im.copy(); rgb[..., 0] -= spill; rgb[..., 2] -= spill
rgb = np.clip(rgb, 0, 255)
order = sorted([i for i in range(1, n) if st[i, 4] > 2000], key=lambda i: (round(st[i, 1]/250), st[i, 0]))
for k, i in enumerate(order):
    x, y, w, h = st[i, :4]; pad = 4
    x0, y0, x1, y1 = max(0, x-pad), max(0, y-pad), min(im.shape[1], x+w+pad), min(im.shape[0], y+h+pad)
    m = (lab[y0:y1, x0:x1] == i).astype(np.float32)
    m = cv2.dilate(m, np.ones((3, 3), np.uint8))
    a = np.clip(alpha[y0:y1, x0:x1]*m, 0, 1)
    rgba = np.dstack([rgb[y0:y1, x0:x1], a*255]).astype(np.uint8)
    Image.fromarray(rgba, 'RGBA').save(f'{out}/s{k}.png'); print(k, (x, y, w, h), round(w/h, 2))
