"""Render the combined film. usage: python3 film_render.py MODE OUTDIR K0 K1 [STEP]
MODE: 'wb' (Workbench framing check) or 'cy' (Cycles final). Frames K0..K1-1 of film.frames()."""
import sys, os, math, numpy as np
sys.path.insert(0, '/home/claude/scripts')
import bpy, mathutils
import film, line2, tug, tugscene, lblend, ldr, cprev

mode, out, k0, k1 = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]); step = int(sys.argv[5]) if len(sys.argv) > 5 else 1
os.makedirs(out, exist_ok=True)
F = film.frames()
L0 = film.log()[0]
REST = line2.cycle(0.0)                                    # line in its starting state while the tug works

DUP = ('rbase', 'rail1_', 'rail-1_', 'railp1_', 'railp-1_', 'railt1_', 'railt-1_')
def yard_static():
    P = []
    for p in tugscene.static():
        n = p['name']
        if n.startswith(DUP) and not n.endswith('_lip'): continue      # the stand-up station already has these rails and baseplates
        P.append(dict(p, name='Y_' + n))
    return P
YARD = yard_static()

def scene_parts(fr):
    if fr['seg'] == 'line':
        P = [dict(p) for p in line2.build(fr['st'])]
        Q = []; tug.trailer(tug.trailer_M(*L0['trl']), Q)
    else:
        P = [dict(p) for p in line2.build(REST)]
        Q = tugscene.dynamic(fr['st'])
    return P + YARD + Q

# ---------------------------------------------------------------- union of all objects over the film
names = {}
for i in range(0, len(F), 12):
    for p in scene_parts(F[i]): names.setdefault(p['name'], p)
for p in tugscene.dynamic([f for f in F if f['seg'] == 'tug'][-1]['st']): names.setdefault(p['name'], p)
sc, cam, objs = cprev.setup(list(names.values()), (1280, 720))
cam.data.clip_start = 0.005

def setup_cycles():
    sc.render.engine = 'CYCLES'
    c = sc.cycles; c.device = 'CPU'; c.samples = 8; c.use_adaptive_sampling = True; c.adaptive_threshold = 0.05
    c.use_denoising = True; c.denoiser = 'OPENIMAGEDENOISE'
    c.max_bounces = 3; c.diffuse_bounces = 1; c.glossy_bounces = 1; c.transmission_bounces = 2; c.transparent_max_bounces = 2
    c.caustics_reflective = False; c.caustics_refractive = False; c.blur_glossy = 1.0
    sc.render.use_persistent_data = True
    sc.view_settings.view_transform = 'AgX'; sc.view_settings.look = 'AgX - Medium High Contrast'; sc.view_settings.exposure = -0.95
    # world: soft blue-grey sky light
    w = bpy.data.worlds.new('world'); sc.world = w; w.use_nodes = True
    bg = w.node_tree.nodes['Background']; bg.inputs[0].default_value = (0.50, 0.56, 0.66, 1); bg.inputs[1].default_value = 0.6
    # key light: warm, slightly soft sun; fill: large cool area light; rim: from behind
    sun = bpy.data.objects.new('key', bpy.data.lights.new('key', 'SUN')); sun.data.energy = 3.2; sun.data.angle = math.radians(3)
    sun.data.color = (1.0, 0.95, 0.88); sun.rotation_euler = (math.radians(48), 0, math.radians(35)); sc.collection.objects.link(sun)
    fill = bpy.data.objects.new('fill', bpy.data.lights.new('fill', 'AREA')); fill.data.energy = 900; fill.data.size = 3.0
    fill.data.color = (0.85, 0.9, 1.0); fill.location = (-1.5, 2.5, 2.0)
    fill.rotation_euler = (mathutils.Vector((0, 0, 0)) - fill.location).to_track_quat('-Z', 'Y').to_euler(); sc.collection.objects.link(fill)
    # table under the baseplates
    me = bpy.data.meshes.new('table'); s = 12.0; z = -0.0016
    me.from_pydata([(-s, -s, z), (s, -s, z), (s, s, z), (-s, s, z)], [], [(0, 1, 2, 3)])
    tb = bpy.data.objects.new('table', me); sc.collection.objects.link(tb)
    m = bpy.data.materials.new('table'); m.use_nodes = True; b = m.node_tree.nodes['Principled BSDF']
    b.inputs['Base Color'].default_value = (0.20, 0.19, 0.18, 1); b.inputs['Roughness'].default_value = 0.55; me.materials.append(m)
    # ABS plastic: glossy, smooth shading with sharp edges kept
    for mat in bpy.data.materials:
        if not mat.name.startswith('ld_'): continue
        b = mat.node_tree.nodes.get('Principled BSDF')
        if b is None: continue
        alpha = b.inputs['Alpha'].default_value
        b.inputs['Roughness'].default_value = 0.24
        try: b.inputs['Specular IOR Level'].default_value = 0.55
        except KeyError: pass
        if alpha < 1:
            b.inputs['Alpha'].default_value = 1.0; b.inputs['Transmission Weight'].default_value = 0.92; b.inputs['Roughness'].default_value = 0.08
    for me in bpy.data.meshes:
        if me.name == 'table': continue
        for p in me.polygons: p.use_smooth = True
        try: me.set_sharp_from_angle(angle=math.radians(38))
        except Exception as e: print('sharp', e)
    cam.data.dof.use_dof = False

if mode == 'cy': setup_cycles()
else:
    sc.display.render_aa = 'FXAA'; sc.render.resolution_x, sc.render.resolution_y = 640, 360

FOCUS = {}
KS = [int(x) for x in open('/home/claude/factory/film/' + ('keyframes.txt' if k0 == -1 else 'testframes.txt')).read().split()] if k0 < 0 else range(k0, min(k1, len(F)), step)
for k in KS:
    fr = F[k]
    if mode == 'cy' and os.path.exists(f'{out}/f{k:04d}.png'): continue
    import time; _t0 = time.time()
    P = {p['name']: p for p in scene_parts(fr)}
    _t1 = time.time()
    pos, tgt, lens, fst = film.camera(fr)
    for n, ob in objs.items():
        p = P.get(n)
        vis = p is not None
        if vis and film.SHOTS[fr['shot']]['name'] not in ('establish', 'end', 'travel', 'follow', 'tow'):
            c = p['M'][:3, 3]*film.LDU; d = np.linalg.norm(np.array([c[0], -c[2], -c[1]]) - tgt)
            vis = d < 1.2                                     # skip what is far outside the frame
        ob.hide_render = not vis
        if vis: ob.matrix_world = lblend.M_bl(p['M'])
    cam.location = mathutils.Vector(pos); cam.rotation_euler = (mathutils.Vector(tgt) - mathutils.Vector(pos)).to_track_quat('-Z', 'Y').to_euler()
    cam.data.lens = lens
    if mode == 'cy':
        cam.data.dof.use_dof = fst > 0
        if fst > 0: cam.data.dof.focus_distance = float(np.linalg.norm(np.array(tgt) - np.array(pos))); cam.data.dof.aperture_fstop = fst
    _t2 = time.time(); sc.render.filepath = f'{out}/f{k:04d}.png'; bpy.ops.render.render(write_still=True)
    print('FRAME', k, 'build %.1f set %.1f render %.1f' % (_t1 - _t0, _t2 - _t1, time.time() - _t2), sum(not o.hide_render for o in objs.values()), 'visible', flush=True)
