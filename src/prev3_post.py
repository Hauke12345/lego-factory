"""Captions over the line-3 preview frames and encode. usage: python3 prev3_post.py OUT.mp4 FPS DIR1 [DIR2 ...]"""
import sys, os, glob, subprocess
from PIL import Image, ImageDraw, ImageFont
out, fps, dirs = sys.argv[1], int(sys.argv[2]), sys.argv[3:]
FB = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'; FR = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
fb, fr = ImageFont.truetype(FB, 19), ImageFont.truetype(FR, 16)
BG = (16, 20, 26); YEL = (255, 206, 64); WHITE = (235, 238, 242)
CAP = {
 'builder': {'karrive': 'Body (hub + head) arrives lying on its flap pallet', 'kfeed': 'Kit trains: motor kit (left), sensor kit (right)',
             'kpush': 'Crank pushers: kits onto the shoulder pins, plugs into ports C and D', 'kretract': 'Pushers back, kit wagons leave',
             'travel1': 'To the track station', 'feed': 'Track-unit trains: plain units', 'push': 'Units onto the side pins and motor outputs',
             'retract': 'Plungers back', 'sleds': 'Empty wagons leave', 'travel': 'To the stand-up flap', 'lift': 'Flap lifts the robot upright',
             'drive': 'Robot drives off on its own tracks', 'lower': 'Builder needs no shoulder module: drives through',
             'ufeed': 'Shoulder station idle for this variant', 'upush': 'Shoulder station idle for this variant', 'uretract': 'Builder done',
             'uleave': 'Builder done: two motors, sensor, plain tracks'},
 'tug':     {'karrive': 'Next body arrives: this one becomes a tug', 'kfeed': 'Port station idle for this variant', 'kpush': 'Port station idle for this variant',
             'kretract': 'Port station idle for this variant', 'travel1': 'To the track station',
             'feed': 'Right unit comes with the belly sensor; plug D rides in a shoe', 'push': 'One push: units home, plug D into its port',
             'retract': 'Plungers back', 'sleds': 'Empty wagons leave', 'travel': 'To the stand-up flap', 'lift': 'Flap lifts the robot upright',
             'drive': 'Robot drives to the shoulder station (motor encoders stop it)', 'lower': 'Flap lowers for the next pallet',
             'ufeed': 'Train brings the hook module, plug C and cable on a tall carrier', 'upush': 'Crank pusher: shoulder pins, plug C, all in one push',
             'uretract': 'Plungers back: module stays on the robot', 'uleave': 'Wagon leaves: tug complete (belly sensor + hook motor)'}}
os.makedirs('/tmp/prev3_post', exist_ok=True)
for f in glob.glob('/tmp/prev3_post/*.png'): os.remove(f)
n = 0
for d in dirs:
    cfg = 'tug' if 'tug' in d else 'builder'
    for src in sorted(glob.glob(f'{d}/f*.png')):
        ph = open(src[:-4] + '.txt').read().strip()
        im = Image.open(src).convert('RGBA'); ov = Image.new('RGBA', im.size, (0, 0, 0, 0)); dr = ImageDraw.Draw(ov)
        head = ('BUILDER' if cfg == 'builder' else 'TUG') + ' configuration'; tail = CAP[cfg].get(ph, ph)
        w = max(dr.textlength(head, font=fb), dr.textlength(tail, font=fr)) + 32
        dr.rounded_rectangle((18, 460, 18 + w, 526), 8, fill=BG + (200,))
        dr.text((34, 466), head, font=fb, fill=YEL + (255,)); dr.text((34, 496), tail, font=fr, fill=WHITE + (255,))
        Image.alpha_composite(im, ov).convert('RGB').save(f'/tmp/prev3_post/f{n:04d}.png'); n += 1
subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(fps), '-i', '/tmp/prev3_post/f%04d.png', '-c:v', 'libx264', '-crf', '20',
                '-pix_fmt', 'yuv420p', '-movflags', '+faststart', out], check=True)
print('wrote', out, n)
