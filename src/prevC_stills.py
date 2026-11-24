"""Still views of the chain-station table (side A canonical): top (guide channel), underside (coupling chain), oblique.
usage: python3 prevC_stills.py OUT.jpg [u]"""
import sys, os, numpy as np
sys.path.insert(0, '/home/claude/scripts')
import bpy, mathutils
import asm2, cstation, lblend, cprev
from PIL import Image, ImageDraw, ImageFont
out = sys.argv[1]; u = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
LDU = 0.0004
def bl(p): return np.array([p[0], -p[2], -p[1]])*LDU
asm2.P.clear(); st = cstation.state(u); cstation.build_canon(st, {})
P = [p for p in asm2.P if not p['name'].startswith(('cs_belt', 'cs_bc', 'cs_rq', 'cs_bh', 'cs_q', 'cs_bax', 'cs_bspr'))]
sc, cam, objs = cprev.setup(P, (900, 640)); sc.display.render_aa = 'FXAA'; cam.data.clip_start = 0.002
for p in P: objs[p['name']].matrix_world = lblend.M_bl(p['M'])
G = cstation.table_G(90.0)
def W(q): return (G @ np.array([*q, 1.0]))[:3]
views = [('top: guide channel (round posts), open chain', W((-136, -40, 0)), np.array([0.0, -900.0, 1.0]), 50),
         ('from below: 24T gears + 34-link Technic chain couple the three stubs', W((-40, -40, 0)), np.array([-260.0, 700.0, 380.0]), 40),
         ('oblique', W((-136, -40, 0)), np.array([-520.0, -600.0, 480.0]), 42)]
tiles = []
for i, (lab, tgt, off, lens) in enumerate(views):
    pos = tgt + off
    cam.location = mathutils.Vector(bl(pos)); cam.rotation_euler = (mathutils.Vector(bl(tgt)) - mathutils.Vector(bl(pos))).to_track_quat('-Z', 'Y').to_euler()
    cam.data.lens = lens; f = f'/tmp/_cst{i}.png'; sc.render.filepath = f; bpy.ops.render.render(write_still=True)
    im = Image.open(f).convert('RGB'); d = ImageDraw.Draw(im)
    try: font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 20)
    except Exception: font = None
    d.rectangle([0, 0, 900, 30], fill=(255, 255, 255)); d.text((8, 4), lab, fill=(0, 0, 0), font=font); tiles.append(im)
sheet = Image.new('RGB', (1800, 1280), 'white')
for i, im in enumerate(tiles): sheet.paste(im, ((i % 2)*900, (i // 2)*640))
sheet.save(out, quality=90); print('saved', out)
