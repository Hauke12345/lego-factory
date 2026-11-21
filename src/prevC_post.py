"""Caption the chain-station preview frames and encode. usage: python3 prevT_post.py SRC OUT.mp4 FPS"""
import sys, os, glob, subprocess
from PIL import Image, ImageDraw, ImageFont
src, out, fps = sys.argv[1], sys.argv[2], int(sys.argv[3])
fb = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 19)
fr = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 16)
BG = (16, 20, 26); YEL = (255, 206, 64); WHITE = (235, 238, 242)
tmp = '/tmp/prevC_post'; os.makedirs(tmp, exist_ok=True)
for f in glob.glob(f'{tmp}/*.png'): os.remove(f)
for n, p in enumerate(sorted(glob.glob(f'{src}/f*.png'))):
    cap = open(p[:-4] + '.txt').read().strip()
    im = Image.open(p).convert('RGBA'); ov = Image.new('RGBA', im.size, (0, 0, 0, 0)); d = ImageDraw.Draw(ov)
    head = 'Chain station · drive set from 3 sprockets + 34 single links (side A)'
    w = max(d.textlength(head, font=fb), d.textlength(cap, font=fr)) + 32
    d.rounded_rectangle((18, 460, 18 + w, 526), 8, fill=BG + (200,))
    d.text((34, 466), head, font=fb, fill=YEL + (255,)); d.text((34, 496), cap, font=fr, fill=WHITE + (255,))
    Image.alpha_composite(im, ov).convert('RGB').save(f'{tmp}/f{n:04d}.png')
subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(fps), '-i', f'{tmp}/f%04d.png', '-c:v', 'libx264', '-crf', '20',
                '-pix_fmt', 'yuv420p', '-movflags', '+faststart', out], check=True)
print('wrote', out)
