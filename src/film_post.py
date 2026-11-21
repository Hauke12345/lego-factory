"""Captions, station-program panels and the tug's line-follower panel over the rendered frames, then encode.
usage: python3 film_post.py SRC_DIR [last_frame]"""
import sys, os, subprocess, numpy as np
sys.path.insert(0, '/home/claude/scripts')
from PIL import Image, ImageDraw, ImageFont
import film, tugsim

SRC = sys.argv[1] if len(sys.argv) > 1 else '/home/claude/factory/film/cy'
DST = SRC.rstrip('/') + '_post'
os.makedirs(DST, exist_ok=True)
F = film.frames()
last = int(sys.argv[2]) if len(sys.argv) > 2 else len(F) - 1
FB = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'; FR = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FM = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'
fb, fr_, fs, fm, fmb = ImageFont.truetype(FB, 24), ImageFont.truetype(FR, 22), ImageFont.truetype(FR, 17), ImageFont.truetype(FM, 16), ImageFont.truetype(FB, 17)
BG = (16, 20, 26); YEL = (255, 206, 64); WHITE = (235, 238, 242); DIM = (140, 148, 160); RED = (255, 96, 86); BLUE = (110, 170, 255)

# what each station's hub does, step by step (the step lit up follows the line phase)
PROG = {
    1: ('Station 1 hub · ports C and D', [('body docked at the stop', ('karrive',)),
                                         ('call the kit trains', ('kfeed',)),
                                         ('crank ½ turn: plugs into C, D', ('kpush',)),
                                         ('robot hub sees: C motor, D sensor', ('kretract',)),
                                         ('crank back, send wagons home', ('travel1',))]),
    2: ('Station 2 hub · tracks', [('body docked at the stop', ('travel1',)),
                                  ('call the track-unit trains', ('feed',)),
                                  ('crank ½ turn: units onto pin, output', ('push',)),
                                  ('crank back, wagons leave', ('retract', 'sleds'))]),
    3: ('Station 3 hubs · stand-up', [('flap pallet docked', ('travel',)),
                                     ('both arms lift the deck upright', ('lift',)),
                                     ("robot's own program: drive off", ('drive',)),
                                     ('lower the flap, pallet returns', ('lower',))]),
}
SHOT_STATION = {'establish': 1, 'kfeed': 1, 'plug': 1, 'kit_out': 1, 'feed': 2, 'tracks': 2, 'travel': 3, 'lift': 3, 'drive': 3, 'next': 1}

starts = {}
for k, f in enumerate(F): starts.setdefault(f['shot'], k)
n_of = {s: sum(1 for f in F if f['shot'] == s) for s in starts}

def panel_program(d, station, phase, a):
    title, steps = PROG[station]
    x0, y0, w = 862, 26, 396; h = 44 + 26*len(steps)
    d.rounded_rectangle((x0, y0, x0 + w, y0 + h), 10, fill=BG + (int(195*a),))
    d.text((x0 + 14, y0 + 10), title, font=fmb, fill=YEL + (int(255*a),))
    for i, (txt, phs) in enumerate(steps):
        on = phase in phs; y = y0 + 40 + 26*i
        if on: d.rounded_rectangle((x0 + 8, y - 3, x0 + w - 8, y + 21), 6, fill=(60, 52, 20, int(220*a)))
        d.text((x0 + 16, y), f'{i + 1}  {txt}', font=fs, fill=(WHITE if on else DIM) + (int(255*a),))

def panel_tug(d, st):
    """reading -> error -> steering: the one number the program uses and what it does with it."""
    v = float(st['v']); thr = tugsim.THRESH; err = v - thr
    cl, cr = st.get('cl', 0.0)/tugsim.DEG, st.get('cr', 0.0)/tugsim.DEG
    x0, y0, w, h = 862, 22, 396, 232
    d.rounded_rectangle((x0, y0, x0 + w, y0 + h), 10, fill=BG + (200,))
    d.text((x0 + 14, y0 + 10), 'Tug program · edge follower', font=fmb, fill=YEL + (255,))
    d.text((x0 + 14, y0 + 34), f'state: {st["state"]}', font=fs, fill=DIM + (255,))
    # reflection bar with the target
    bx, by, bw = x0 + 14, y0 + 66, w - 28
    d.text((bx, by - 2), 'light sensor (looking down)', font=fs, fill=WHITE + (255,))
    by += 22
    d.rectangle((bx, by, bx + bw, by + 14), fill=(50, 56, 66, 255))
    # gradient hint: dark line (left) ... white floor (right)
    d.rectangle((bx, by, bx + int(bw*v/100), by + 14), fill=YEL + (255,))
    tx = bx + int(bw*thr/100); d.line((tx, by - 4, tx, by + 18), fill=RED + (255,), width=3)
    d.text((bx, by + 18), f'reflection {v:3.0f} %', font=fm, fill=WHITE + (255,))
    d.text((bx + 186, by + 18), f'target {thr:.1f}', font=fm, fill=RED + (255,))
    col = st.get('col') or ''
    if col: d.text((bx + 262, by - 22), f'colour: {col}', font=fm, fill=(RED if col == 'RED' else BLUE) + (255,))
    # error and the decision
    y = by + 44
    tow = 'tow' in str(st['state'])
    if 'follow' in str(st['state']):
        if abs(err) < 3: msg, c = 'on the edge: straight on', WHITE
        else:
            bright = err > 0
            turn_right = cl > cr                            # read back from the commanded track speeds (the tow leg follows the edge mirrored)
            msg = ('too bright (on white)' if bright else 'too dark (on the line)') + ' -> ' + ('steer right' if turn_right else 'steer left')
            c = YEL
        d.text((bx, y), f'error = {v:3.0f} - {thr:.1f} = {err:+5.1f}', font=fm, fill=WHITE + (255,))
        d.text((bx, y + 22), msg, font=fs, fill=c + (255,))
    else:
        d.text((bx, y), 'not steering by the sensor in this step', font=fs, fill=DIM + (255,))
    # the two track motors
    y += 52; mx = 360.0
    for i, (lab, sp) in enumerate((('left track', cl), ('right track', cr))):
        yy = y + 20*i; cx = bx + 110 + (bw - 110)//2
        d.text((bx, yy - 2), lab, font=fs, fill=WHITE + (255,))
        d.rectangle((bx + 110, yy, bx + bw, yy + 12), fill=(50, 56, 66, 255))
        e = cx + int((bw - 110)/2*np.clip(sp/mx, -1, 1))
        d.rectangle((min(cx, e), yy, max(cx, e), yy + 12), fill=(120, 200, 140, 255))
        d.line((cx, yy - 2, cx, yy + 14), fill=DIM + (255,), width=1)

last_done = -1
for k in range(last + 1):
    src = f'{SRC}/f{k:04d}.png'
    if not os.path.exists(src):
        if os.environ.get('SPARSE'): continue
        break
    fr = F[k]; sh = film.SHOTS[fr['shot']]; cap = sh['cap']
    im = Image.open(src).convert('RGBA'); ov = Image.new('RGBA', im.size, (0, 0, 0, 0)); d = ImageDraw.Draw(ov)
    j = k - starts[fr['shot']]; n = n_of[fr['shot']]
    a = max(0.0, min(1.0, j/8.0, (n - 1 - j)/8.0))                             # panels fade in and out with their shot
    if cap and a > 0:
        head, _, tail = cap.partition(' · ')
        w = max(d.textlength(head, font=fb), d.textlength(tail, font=fr_)) + 40
        d.rounded_rectangle((28, 610, 28 + w, 692), 10, fill=BG + (int(190*a),))
        d.text((48, 618), head, font=fb, fill=YEL + (int(255*a),))
        d.text((48, 654), tail, font=fr_, fill=WHITE + (int(255*a),))
    if fr['seg'] == 'tug': panel_tug(d, fr['st'])
    elif sh['name'] in SHOT_STATION and a > 0: panel_program(d, SHOT_STATION[sh['name']], fr['st']['phase'], a)
    Image.alpha_composite(im, ov).convert('RGB').save(f'{DST}/f{k:04d}.png')
    last_done = k

if os.environ.get('SPARSE'): sys.exit(0)
done = last_done + 1
out = '/home/claude/factory/film/lego_factory_film2.mp4' if done == len(F) else f'/home/claude/factory/film/lego_factory_film2_part_{done}f.mp4'
subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(film.FPS), '-i', f'{DST}/f%04d.png', '-frames:v', str(done),
                '-c:v', 'libx264', '-preset', 'slow', '-crf', '17', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', out], check=True)
print('wrote', out, done, 'frames')
