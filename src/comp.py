import sys, json, glob, os
import numpy as np, OpenEXR, cv2
sys.path.insert(0, '/home/claude/scripts')
import importlib; HUD = importlib.import_module(os.environ.get('HUDMOD', 'hud'))
from PIL import Image
src = sys.argv[1]; dst = sys.argv[2]; frames = sys.argv[3:]  # empty = all with exr
os.makedirs(dst, exist_ok=True)
meta = {}
for m in glob.glob(f'{src}/meta_*.json'):
    meta.update({int(k): v for k, v in json.load(open(m)).items()})
huds = {}
for m in glob.glob(f'{src}/hud_*.json'):
    huds.update({int(k): v for k, v in json.load(open(m)).items()})
def s2l(x): return np.where(x <= 0.04045, x/12.92, ((x+0.055)/1.055)**2.4)
def l2s(x): x = np.clip(x, 0, 1); return np.where(x <= 0.0031308, x*12.92, 1.055*x**(1/2.4)-0.055)
GAIN = float(os.environ.get('GAIN', '1.0'))
BAND = int(os.environ.get('BAND', '22'))
def refine_alpha(img, wa):
    """img: HxWx3 sRGB float, wa: HxW proxy wall alpha. Re-decide pixels in a band around the proxy edge by colour."""
    m = (wa > 0.5).astype(np.uint8)
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*BAND+1, 2*BAND+1))
    sure_w = cv2.erode(m, ker); sure_n = 1 - cv2.dilate(m, ker)
    ring = cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (6*BAND+1, 6*BAND+1))) & sure_n.astype(np.uint8)
    near_w = sure_w & (1 - cv2.erode(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (6*BAND+1, 6*BAND+1))))
    band = (1 - sure_w) & (1 - sure_n)
    if band.sum() == 0 or near_w.sum() < 200 or ring.sum() < 200: return wa
    lab = cv2.cvtColor((img*255).astype(np.uint8), cv2.COLOR_RGB2LAB)
    q = (lab // 8).astype(np.int32); idx = q[..., 0]*1024 + q[..., 1]*32 + q[..., 2]
    hw = np.bincount(idx[near_w > 0], minlength=32768).astype(np.float32)
    hn = np.bincount(idx[ring > 0], minlength=32768).astype(np.float32)
    # smooth histograms in colour space a little
    hw = cv2.GaussianBlur(hw.reshape(32, 32, 32).reshape(32, 1024), (3, 3), 0.8).reshape(-1) + 1e-3
    hn = cv2.GaussianBlur(hn.reshape(32, 32, 32).reshape(32, 1024), (3, 3), 0.8).reshape(-1) + 1e-3
    hw /= hw.sum(); hn /= hn.sum()
    p = hw[idx] / (hw[idx] + hn[idx])
    p = cv2.GaussianBlur(p.astype(np.float32), (0, 0), 1.5)
    # spatial prior: stay close to the proxy edge
    dist_in = cv2.distanceTransform(m, cv2.DIST_L2, 5); dist_out = cv2.distanceTransform(1 - m, cv2.DIST_L2, 5)
    sd = np.where(m > 0, dist_in, -dist_out) / BAND
    prior = 1/(1 + np.exp(-2.0*sd))
    post = p*prior / (p*prior + (1-p)*(1-prior) + 1e-6)
    a = np.clip((post - 0.5)*4 + 0.5, 0, 1)
    a = cv2.GaussianBlur(a, (0, 0), 0.8)
    out = np.where(band > 0, a, (sure_w > 0).astype(np.float32))
    return out
todo = [int(f) for f in frames] if frames else range(642)
WARP = np.load('/home/claude/sfm/warp.npy') if os.environ.get('WARP', '1') == '1' else None
def _basis(u, v):
    return [np.ones_like(u), u, v, u*u, u*v, v*v, u*v*v, v*v*v]
_gy, _gx = np.mgrid[0:1920, 0:1080].astype(np.float32)
_B = _basis((_gx-540)/540, (_gy-960)/960)
def disp_field(f):
    c = WARP[f]
    dx = sum(b*c[k, 0] for k, b in enumerate(_B)); dy = sum(b*c[k, 1] for k, b in enumerate(_B))
    return dx.astype(np.float32), dy.astype(np.float32)
def disp_pts(f, x, y):
    c = WARP[f]; B = _basis((x-540)/540, (y-960)/960)
    return sum(b*c[k, 0] for k, b in enumerate(B)), sum(b*c[k, 1] for k, b in enumerate(B))
for f in todo:
    plate = cv2.imread(f'/home/claude/plate/f{f:04d}.jpg')[:, :, ::-1].astype(np.float32)/255
    H, W = plate.shape[:2]
    out = plate
    WALL = os.environ.get('WALL')
    wp = f'{WALL}/r{f:04d}.exr' if WALL else None
    if wp and os.path.exists(wp):
        ex = OpenEXR.File(wp); wc = None
        for part in ex.parts:
            for k, v in part.channels.items():
                if k == 'ViewLayer.Combined': wc = v.pixels.astype(np.float32)
        wc = cv2.resize(wc, (W, H), interpolation=cv2.INTER_LINEAR)
        if WARP is not None:
            dx, dy = disp_field(f); wc = cv2.remap(wc, _gx - dx, _gy - dy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        wa = np.clip(wc[..., 3:4], 0, 1)
        alb = wc[..., :3]/np.maximum(wa, 1e-4)
        if os.environ.get('REFINE', '1') == '1': wa = refine_alpha(plate, wa[..., 0])[..., None]
        pl = s2l(plate)
        lum = (pl*[0.2126, 0.7152, 0.0722]).sum(-1)
        sm = lambda x: cv2.resize(cv2.GaussianBlur(cv2.resize(x, (W//8, H//8), interpolation=cv2.INTER_AREA), (0, 0), 9), (W, H))
        m = (wa[..., 0] > 0.5).astype(np.float32)
        illum = sm(lum*m)/np.maximum(sm(m), 1e-3)
        illum = illum[..., None]/float(os.environ.get('PLY', '0.42'))
        out = l2s(wa*alb*illum + (1-wa)*pl)
        plate = out
    bd = meta.get(f)
    p = f'{src}/r{f:04d}.exr'
    if bd and os.path.exists(p):
        ex = OpenEXR.File(p)
        ch = {}
        for part in ex.parts:
            for k, v in part.channels.items(): ch[k] = v.pixels
        comb = ch['ViewLayer.Combined'].astype(np.float32)
        shc = ch['ViewLayer.Shadow Catcher'].astype(np.float32)[..., :3] if 'ViewLayer.Shadow Catcher' in ch else np.ones_like(comb[..., :3])
        if comb.shape[-1] == 3: comb = np.concatenate([comb, np.ones_like(comb[..., :1])], -1)
        x0, x1 = int(round(bd[0]*W)), int(round(bd[1]*W))
        y0, y1 = H - int(round(bd[3]*H)), H - int(round(bd[2]*H))
        w, h = x1-x0, y1-y0
        if comb.shape[:2] != (h, w):
            comb = cv2.resize(comb, (w, h), interpolation=cv2.INTER_CUBIC)
            shc = cv2.resize(shc, (w, h), interpolation=cv2.INTER_LINEAR)
        CB = np.zeros((H, W, 4), np.float32); SH = np.ones((H, W, 3), np.float32)
        CB[y0:y1, x0:x1] = comb; SH[y0:y1, x0:x1] = shc
        if WARP is not None:
            dx, dy = disp_field(f)
            CB = cv2.remap(CB, _gx - dx, _gy - dy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            SH = cv2.remap(SH, _gx - dx, _gy - dy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(1, 1, 1))
        lin = s2l(plate)
        a = np.clip(CB[..., 3:4], 0, 1)
        out = l2s(CB[..., :3]*GAIN + (1-a)*lin*np.clip(SH, 0, 1.2))
    im = Image.fromarray((np.clip(out, 0, 1)*255+0.5).astype(np.uint8))
    hd = huds.get(f)
    if hd and WARP is not None:
        hd2 = {}
        for kk, vv in hd.items():
            pts = vv if kk == 'path' else [vv]
            xs = np.array([p[0]*W for p in pts]); ys = np.array([p[1]*H for p in pts])
            ddx, ddy = disp_pts(f, xs, ys)
            np2 = [((x+a_)/W, (y+b_)/H, p[2]) for x, y, a_, b_, p in zip(xs, ys, np.atleast_1d(ddx), np.atleast_1d(ddy), pts)]
            hd2[kk] = np2 if kk == 'path' else np2[0]
        hd = hd2
    im = HUD.draw(im, f, hd)
    im.save(f'{dst}/c{f:04d}.jpg', quality=94)
