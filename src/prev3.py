"""Workbench preview of line 3 (one robot cycle). usage: blender-less python3 prev3.py CFG OUTDIR N [K0 K1]
Renders frames K0..K1-1 of N evenly spaced cycle samples."""
import sys, os, numpy as np
sys.path.insert(0, '/home/claude/scripts')
import bpy, mathutils
import line3, shoulder, lblend, cprev

cfg, out, N = sys.argv[1], sys.argv[2], int(sys.argv[3])
k0, k1 = (int(sys.argv[4]), int(sys.argv[5])) if len(sys.argv) > 5 else (0, N)
os.makedirs(out, exist_ok=True)
LDU = 0.0004
def bl(p): return np.array([p[0], -p[2], -p[1]])*LDU

TS = [(k + 0.5)/N for k in range(N)]
# camera keys per phase: (target LDU, offset from target LDU, lens)
RX = shoulder.RX
CAM = {'port':   ((1280, -260, 0), (-760, -760, 1250), 40),
       'track':  ((0, -260, 0), (-760, -760, 1250), 40),
       'stand':  ((-1700, -260, 0), (-700, -700, 1250), 38),
       'drive':  (None, (-600, -650, 1150), 36),
       'shoul':  ((RX + 20, -360, 180), (760, -640, 1080), 40)}
PH_CAM = {'karrive': 'port', 'kfeed': 'port', 'kpush': 'port', 'kretract': 'port', 'travel1': 'track', 'feed': 'track', 'push': 'track',
          'retract': 'track', 'sleds': 'track', 'travel': 'stand', 'lift': 'stand', 'drive': 'drive', 'lower': 'shoul',
          'ufeed': 'shoul', 'upush': 'shoul', 'uretract': 'shoul', 'uleave': 'shoul'}

def cam_for(st, t):
    key = PH_CAM[st['phase']]; tgt, off, lens = CAM[key]
    if key == 'drive':
        x = (line3.robot_M(st) @ np.array([0, -200, 0, 1.0]))[0]; tgt = (x, -260, 60)
    if st['phase'] in ('travel1', 'travel'):                 # pan along with the pallet
        x = st['px'] - 40; tgt = (x, -260, 0)
    tgt = np.array(tgt, float); pos = tgt + np.array(off, float)
    return bl(pos), bl(tgt), lens

# union of objects over the cycle (sampled)
names = {}
for t in np.linspace(0, 0.999, 40):
    for p in line3.build(line3.cycle(t, cfg)): names.setdefault(p['name'], dict(p))
for k in range(k0, k1):
    for p in line3.build(line3.cycle(TS[k], cfg)): names.setdefault(p['name'], dict(p))
sc, cam, objs = cprev.setup(list(names.values()), (960, 540))
sc.display.render_aa = 'FXAA'
cam.data.clip_start = 0.005
for k in range(k0, k1):
    st = line3.cycle(TS[k], cfg); P = {p['name']: p for p in line3.build(st)}
    pos, tgt, lens = cam_for(st, TS[k])
    for n, ob in objs.items():
        p = P.get(n); vis = p is not None
        if vis:
            c = bl(p['M'][:3, 3]); vis = np.linalg.norm(c - tgt) < 1.6
        ob.hide_render = not vis
        if vis: ob.matrix_world = lblend.M_bl(p['M'])
    cam.location = mathutils.Vector(pos); cam.rotation_euler = (mathutils.Vector(tgt) - mathutils.Vector(pos)).to_track_quat('-Z', 'Y').to_euler()
    cam.data.lens = lens
    sc.render.filepath = f'{out}/f{k:04d}.png'; bpy.ops.render.render(write_still=True)
    with open(f'{out}/f{k:04d}.txt', 'w') as fh: fh.write(st['phase'])
    print('FRAME', k, st['phase'], flush=True)
