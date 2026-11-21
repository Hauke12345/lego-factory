"""Combine bridge video + bonus jump into a 9:16 (YT Short) and a 1:1 (LinkedIn) cut with title cards.
Usage: python3 formats.py [test]"""
import sys, subprocess, numpy as np, cv2
from PIL import Image, ImageDraw, ImageFont

B = '/home/claude/render_b/lego_bridge_final.mp4'
C = '/home/claude/render/lego_car_final_sfx.mp4'
OUT = '/home/claude/render_b/formats'
XF = 9                                   # crossfade frames
BOLD = '/usr/share/fonts/opentype/inter/Inter-Bold.otf'
SEMI = '/usr/share/fonts/opentype/inter/Inter-SemiBold.otf'
RED = (232, 68, 46); WHITE = (250, 247, 240)
Y0 = int(sys.argv[2]) if len(sys.argv) > 2 else 190    # square crop top (keeps the original panels below the crop)
PS = 0.68                                                # panel scale in square

class Clip:
    """sequential frame reader with seek fallback; holds only the last frame."""
    def __init__(s, fn):
        s.cap = cv2.VideoCapture(fn); s.n = int(s.cap.get(cv2.CAP_PROP_FRAME_COUNT)); s.i = -1; s.f = None
    def __getitem__(s, k):
        k = min(k, s.n - 1)
        if k != s.i:
            if k != s.i + 1: s.cap.set(cv2.CAP_PROP_POS_FRAMES, k)
            ok, f = s.cap.read()
            if ok: s.f = f
            s.i = k
        return s.f
fb, fc = Clip(B), Clip(C)
nb, nc = fb.n, fc.n
N = nb + nc - XF

def ramp(x, a, b): return float(np.clip((x - a)/(b - a), 0, 1))

def rmask(w, h, r):
    m = Image.new('L', (w, h), 0); ImageDraw.Draw(m).rounded_rectangle([0, 0, w-1, h-1], r, fill=255)
    return np.asarray(m, np.float32)[..., None]/255

def card(img, lines, y, width, a, scale=1.0):
    """lines: [(text, font, size, color)], centred dark rounded card starting at y."""
    if a <= 0: return img
    W = img.width
    fonts = [ImageFont.truetype(f, int(s*scale)) for _, f, s, _ in lines]
    d0 = ImageDraw.Draw(img)
    hs = [d0.textbbox((0, 0), t, font=fo)[3] for (t, *_), fo in zip(lines, fonts)]
    pad = int(34*scale); gap = int(18*scale)
    H = sum(hs) + gap*(len(lines)-1) + 2*pad
    ov = Image.new('RGBA', img.size, (0, 0, 0, 0)); d = ImageDraw.Draw(ov)
    x0 = (W - width)//2
    d.rounded_rectangle([x0, y, x0 + width, y + H], int(26*scale), fill=(20, 18, 16, int(185*a)))
    yy = y + pad
    for (t, _, _, col), fo, h in zip(lines, fonts, hs):
        d.text((W//2, yy), t, font=fo, fill=(*col, int(255*a)), anchor='mt'); yy += h + gap
    return Image.alpha_composite(img.convert('RGBA'), ov).convert('RGB')

TITLE = [('Can AI do Civil Engineering?', BOLD, 70, WHITE)]
BONUS = [('BONUS', BOLD, 44, RED), ('Same gap. No bridge. Jump it.', SEMI, 46, WHITE)]

def base(i):
    """returns (frame BGR, clip, local frame index)"""
    if i < nb - XF: return fb[i], 'b', i
    if i < nb:
        k = i - (nb - XF); t = (k + 1)/(XF + 1)
        return cv2.addWeighted(fb[i], 1 - t, fc[k], t, 0), ('b' if t < 0.5 else 'c'), (i if t < 0.5 else k)
    return fc[i - nb + XF], 'c', i - nb + XF

def panels_square(sq, fr, clip, f):
    """paste the bottom-of-frame HUD panels, scaled, into the square crop."""
    jobs = []
    # plan panel (both clips): box x 36..490, y 1430..(1430+50+46n), grows 226-262, fades 330-345
    n_show = int(ramp(f, 226, 262)*8 + 0.999) if 226 <= f < 345 else 0
    if n_show:
        a = 1 - ramp(f, 330, 345)
        jobs.append((36, 1430, 454, 50 + 46*n_show, 18, a))
    if clip == 'b' and f >= 360:
        jobs.append((40, 1270, 1000, 600, 22, ramp(f, 360, 370)))
    for x, y, w, h, r, a in jobs:
        if a <= 0: continue
        crop = fr[y:y+h, x:x+w].astype(np.float32)
        sw, sh = int(w*PS), int(h*PS)
        crop = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_AREA)
        m = rmask(sw, sh, int(r*PS))*a
        px, py = (20 if w < 600 else (1080 - sw)//2), 20     # over the wall, top of the square
        reg = sq[py:py+sh, px:px+sw].astype(np.float32)
        sq[py:py+sh, px:px+sw] = (reg*(1 - m) + crop*m).astype(np.uint8)
    return sq

def compose(i):
    fr, clip, f = base(i)
    # panels are taken from the un-faded source frame
    src = fb[f] if clip == 'b' else fc[f]
    tall = fr.copy()
    sq = panels_square(fr[Y0:Y0+1080].copy(), src, clip, f)
    ta = ramp(i, 4, 14)*(1 - ramp(i, 100, 114))
    ba = ramp(i, nb - XF, nb + 4)*(1 - ramp(i, nb + 80, nb + 92))
    T = Image.fromarray(cv2.cvtColor(tall, cv2.COLOR_BGR2RGB))
    S = Image.fromarray(cv2.cvtColor(sq, cv2.COLOR_BGR2RGB))
    if ta > 0:
        T = card(T, TITLE, 110, 960, ta); S = card(S, TITLE, 40, 900, ta, 0.92)
    if ba > 0:
        T = card(T, BONUS, 110, 860, ba); S = card(S, BONUS, 40, 820, ba, 0.92)
    return np.asarray(T), np.asarray(S)

def enc(path, w, h):
    return subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{w}x{h}',
                             '-r', '30', '-i', '-', '-c:v', 'libx264', '-crf', '18', '-preset', 'medium', '-pix_fmt', 'yuv420p',
                             path], stdin=subprocess.PIPE)

if __name__ == '__main__':
    import os; os.makedirs(OUT, exist_ok=True)
    if len(sys.argv) > 1 and sys.argv[1] == 'test':
        for i in (40, 250, 420, 600, nb + 30, nb + 250):
            T, S = compose(i)
            cv2.imwrite(f'{OUT}/t{i}.jpg', cv2.cvtColor(T, cv2.COLOR_RGB2BGR)); cv2.imwrite(f'{OUT}/s{i}.jpg', cv2.cvtColor(S, cv2.COLOR_RGB2BGR))
        sys.exit()
    pt, ps = enc(f'{OUT}/v916.mp4', 1080, 1920), enc(f'{OUT}/v11.mp4', 1080, 1080)
    for i in range(N):
        T, S = compose(i); pt.stdin.write(T.tobytes()); ps.stdin.write(S.tobytes())
    pt.stdin.close(); ps.stdin.close(); pt.wait(); ps.wait()
    tb = nb/30
    for v, o in (('v916', 'lego_bridge_short_9x16.mp4'), ('v11', 'lego_bridge_linkedin_1x1.mp4')):
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', f'{OUT}/{v}.mp4', '-i', B, '-i', C, '-filter_complex',
                        f'[1:a]atrim=0:{tb},asetpts=N/SR/TB[a1];[2:a]atrim=0:{nc/30},asetpts=N/SR/TB[a2];[a1][a2]acrossfade=d={XF/30}[a]',
                        '-map', '0:v', '-map', '[a]', '-c:v', 'copy', '-c:a', 'aac', '-b:a', '192k', '-shortest', f'{OUT}/{o}'], check=True)
    print('done', N)
