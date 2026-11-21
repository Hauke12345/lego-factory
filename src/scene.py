"""Build the full scene and render a frame range.
usage: python3 scene.py START END [preview]"""
import sys, os, json, math, pickle, random
sys.path.insert(0, '/home/claude/scripts')
import numpy as np
import bpy
from mathutils import Matrix, Vector, Quaternion, Euler

START, END = int(sys.argv[1]), int(sys.argv[2])
MODE = sys.argv[3] if len(sys.argv) > 3 else 'final'
OUT = '/home/claude/render/' + {'final': 'exr', 'wb': 'wb', 'wall': 'wall'}.get(MODE, 'prev')
os.makedirs(OUT, exist_ok=True)

bpy.ops.wm.read_factory_settings(use_empty=True)
import lego

# ---------------- world mapping (COLMAP -> metres) ----------------
F = np.load('/home/claude/sfm/frame.npy'); wv, e2, up = F
SCALE, A0, T0 = 0.044, 9.0, -6.55
def to_bl(a, b, h): return Vector((SCALE*(a-A0), SCALE*b, SCALE*(h-T0)))
Z_BOX = SCALE*(-4.348-T0); Z_BOOK = SCALE*(-5.262-T0)
RECTS = {'box': [(4.79, 6.88), (4.92, 0.46), (12.17, 0.60), (12.04, 7.02)],
         'book': [(6.27, -2.78), (6.55, -8.02), (11.89, -7.74), (11.61, -2.5)]}
Y_BOXEDGE = SCALE*0.539
Y_BOOKEDGE = SCALE*-2.637

sc = bpy.context.scene
sc.render.engine = 'CYCLES'
sc.cycles.device = 'CPU'
sc.cycles.samples = int(os.environ.get('SPP', 64 if MODE == 'final' else 16))
sc.cycles.use_adaptive_sampling = True
sc.cycles.adaptive_threshold = 0.02
sc.cycles.use_denoising = True
sc.cycles.max_bounces = 6
sc.cycles.caustics_reflective = sc.cycles.caustics_refractive = False
sc.render.film_transparent = True
sc.render.resolution_x, sc.render.resolution_y = 1080, 1920
sc.render.resolution_percentage = int(os.environ.get('RES', 100 if MODE == 'final' else 50))
sc.render.use_persistent_data = True
sc.view_settings.view_transform = 'Standard'
vl = sc.view_layers[0]
vl.cycles.use_pass_shadow_catcher = True
ENGINE = os.environ.get('ENGINE', 'CYCLES')

# ---------------- environment light ----------------
wd = bpy.data.worlds.new('w'); sc.world = wd; wd.use_nodes = True
nt = wd.node_tree
env = nt.nodes.new('ShaderNodeTexEnvironment'); env.image = bpy.data.images.load('/home/claude/hdri/env.exr')
mp = nt.nodes.new('ShaderNodeMapping'); tc = nt.nodes.new('ShaderNodeTexCoord')
mp.inputs['Rotation'].default_value = (0, 0, math.radians(-6.6))
nt.links.new(tc.outputs['Generated'], mp.inputs['Vector']); nt.links.new(mp.outputs[0], env.inputs[0])
bg = nt.nodes['Background']; nt.links.new(env.outputs[0], bg.inputs[0])
bg.inputs['Strength'].default_value = float(os.environ.get('ENVSTR', '2.0'))

# ---------------- shadow-catcher proxies ----------------
def catcher_mat(name, rgb, rough):
    m = bpy.data.materials.new(name); m.use_nodes = True
    b = m.node_tree.nodes['Principled BSDF']
    b.inputs['Base Color'].default_value = (*rgb, 1); b.inputs['Roughness'].default_value = rough
    return m

def cuboid(name, corners_ab, z0, z1, m):
    import bmesh
    bm = bmesh.new()
    vs = [bm.verts.new(to_bl(a, b, T0) + Vector((0, 0, z))) for z in (z0, z1) for a, b in corners_ab]
    bm.faces.new(vs[:4][::-1]); bm.faces.new(vs[4:])
    for i in range(4):
        j = (i+1) % 4; bm.faces.new((vs[i], vs[j], vs[4+j], vs[4+i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name); bm.to_mesh(me)
    ob = bpy.data.objects.new(name, me); sc.collection.objects.link(ob)
    me.materials.append(m); ob.is_shadow_catcher = True
    return ob

table_m = catcher_mat('tableM', (0.35, 0.45, 0.55), 0.2)
bpy.ops.mesh.primitive_plane_add(size=2.0, location=(0, 0, 0))
table = bpy.context.object; table.data.materials.append(table_m); table.is_shadow_catcher = True
bpy.ops.mesh.primitive_plane_add(size=1.0, location=(SCALE*(16.375-A0), 0, 0.6), rotation=(0, math.pi/2, 0))
wall = bpy.context.object; wall.scale = (1.2, 2.0, 1); wall.is_shadow_catcher = True
wall.data.materials.append(catcher_mat('wallM', (0.42, 0.28, 0.16), 0.7))
cuboid('box', RECTS['box'], 0, Z_BOX, catcher_mat('boxM', (0.38, 0.24, 0.13), 0.8))
cuboid('book', RECTS['book'], 0, Z_BOOK, catcher_mat('bookM', (0.45, 0.1, 0.08), 0.5))

# ---------------- camera ----------------
Rall, Call = pickle.load(open(os.environ.get('CAMS', '/home/claude/sfm/allcams_ref.pkl'), 'rb'))
camd = bpy.data.cameras.new('cam'); cam = bpy.data.objects.new('cam', camd)
sc.collection.objects.link(cam); sc.camera = cam
camd.sensor_fit = 'HORIZONTAL'; camd.sensor_width = 36.0
camd.lens = 633.637/720*36.0
camd.clip_start = 0.005; camd.clip_end = 20
W2B = np.stack([wv, e2, up])
def cam_matrix(i):
    Rcw = Rall[i].T                                  # cam->world (colmap)
    Rb = W2B @ Rcw @ np.diag([1, -1, -1])
    c = Call[i]; loc = to_bl(c@wv, c@e2, c@up)
    M = Matrix(Rb.tolist()).to_4x4(); M.translation = loc
    return M

# ---------------- car + ramp ----------------
parts, wheels = lego.build_car()
ramp_parts = []
for k, x in enumerate((-2.25*lego.S, 2.25*lego.S)):   # two 3-wide wedges side by side across travel dir
    ob = lego.slope(f'ramp{k}', 4, 4, lego.DGRAY(), h=lego.B + lego.P, flat=0.0)
    ramp_parts.append(ob)
RAMP_LEN = 4*lego.S; RAMP_H = lego.B + lego.P
# ramp: wedge slopes down towards its local +X; we want low end away from the edge (towards +Y),
# high end at box edge (towards -Y). local -X -> world -Y => rotate +90deg about Z.
ramp_final = []
for k, ob in enumerate(ramp_parts):
    x = (-2*lego.S, 2*lego.S)[k]
    M = Matrix.Translation((x, Y_BOXEDGE + RAMP_LEN/2 + 0.001, Z_BOX)) @ Matrix.Rotation(math.pi/2, 4, 'Z')
    ramp_final.append(M)

def ramp_height(y):
    """height of ramp surface above box top at world y (high end at edge)."""
    y0 = Y_BOXEDGE + 0.001; y1 = y0 + RAMP_LEN
    if y < y0 or y > y1: return 0.0
    flatpart = 0.0
    lo = 0.25*RAMP_H
    t = (y - y0)/RAMP_LEN      # 0 at edge (high), 1 far
    return RAMP_H - (RAMP_H - lo)*t if t < 1 else 0.0

def ground(y):
    if y > Y_BOXEDGE: return Z_BOX + ramp_height(y)
    if y < Y_BOOKEDGE: return Z_BOOK
    return None

WR = 0.0125; WB = 9*lego.S/2       # wheel radius, half wheelbase
BUILD_Y = 0.20
YAW = -math.pi/2                    # car +X -> world -Y

# lowest local z of each part (for resting on the box)
def low_z(ob, M):
    zs = []
    for o in [ob] + list(ob.children):
        Mo = M if o is ob else M @ o.matrix_parent_inverse @ o.matrix_basis
        zs += [(Mo @ Vector(c)).z for c in o.bound_box]
    return min(zs)

all_parts = [(ob, M, 'car') for ob, M in parts] + [(ob, M, 'ramp') for ob, M in zip(ramp_parts, ramp_final)]
rng = random.Random(7)
starts = []
for i, (ob, M, kind) in enumerate(all_parts):
    for _ in range(200):
        ang = rng.uniform(0, 2*math.pi); r = rng.uniform(0.05, 0.11)
        x = r*math.cos(ang)*0.9; y = BUILD_Y + r*math.sin(ang)
        if -0.15 < x < 0.12 and Y_BOXEDGE + 0.04 < y < SCALE*6.8: break
    yaw = rng.uniform(-math.pi, math.pi)
    R = Matrix.Rotation(yaw, 4, 'Z') @ M.to_3x3().to_4x4()
    R.translation = (0, 0, 0)
    z = Z_BOX - low_z(ob, R) + 0.0002
    S0 = Matrix.Translation((x, y, z)) @ R
    starts.append(S0)

def car_root_static():
    return Matrix.Translation((0, BUILD_Y, Z_BOX)) @ Matrix.Rotation(YAW, 4, 'Z')

T_ASM0, DT, DUR = 15, 5, 24
def ease(t): return t*t*(3-2*t)

def part_matrix(i, f, car_world):
    ob, M, kind = all_parts[i]
    final = (car_world @ M) if kind == 'car' else M
    t0 = T_ASM0 + i*DT
    if f <= t0: return starts[i], False
    if f >= t0 + DUR: return final, True
    t = (f - t0)/DUR
    S0 = starts[i]
    above = final.copy(); above.translation = final.translation + Vector((0, 0, 0.012))
    q0 = S0.to_quaternion(); q1 = final.to_quaternion()
    if t < 0.8:
        u = ease(t/0.8)
        p = S0.translation.lerp(above.translation, u) + Vector((0, 0, 0.06*math.sin(math.pi*u)))
        q = q0.slerp(q1, u)
        # extra tumble while flying
        q = q @ Quaternion((1, 0, 0), 2*math.pi*u*(1 if i % 2 else -1)) if kind == 'car' and i % 3 == 0 else q
    else:
        u = (t-0.8)/0.2; u = u*u
        p = above.translation.lerp(final.translation, u); q = q1
    Mx = q.to_matrix().to_4x4(); Mx.translation = p
    return Mx, False

# ---------------- drive / jump ----------------
T_GO = 406            # start accelerating
TRAJ = np.load('/home/claude/render/traj.npz')['traj']
FINE = np.load('/home/claude/render/traj.npz')['fine']      # t, qpos(7), wheels(4) every 2 ms
# slow-motion time warp: playback rate k(s) (1 = real time)
SLOW = float(os.environ.get('SLOW', '0'))
def _k(s):
    def sm(a, b, x): x = min(1, max(0, (x-a)/(b-a))); return x*x*(3-2*x)
    return 1 - SLOW*sm(0.15, 0.185, s) + SLOW*sm(0.40, 0.50, s)
_ss = np.arange(0, FINE[-1, 0], 0.0005)
_ff = np.concatenate([[0], np.cumsum(30*0.0005/np.array([_k(x) for x in _ss[:-1]]))])
def sim_time(fr):           # frames since T_GO -> sim seconds
    return float(np.interp(fr, _ff, _ss))
def sim_state(s):
    i = np.searchsorted(FINE[:, 0], s); i = min(max(i, 1), len(FINE)-1)
    a, b = FINE[i-1], FINE[i]; u = (s-a[0])/max(b[0]-a[0], 1e-9); u = min(max(u, 0), 1)
    pos = a[1:4]*(1-u) + b[1:4]*u
    qa, qb = Quaternion(a[4:8]), Quaternion(b[4:8]); q = qa.slerp(qb, u)
    wh = a[8:12]*(1-u) + b[8:12]*u
    return pos, q, wh
print('TAKEOFF frame ~', T_GO + np.interp(0.1876, _ss, _ff), 'LAND ~', T_GO + np.interp(0.40, _ss, _ff))
V_LAUNCH = 0.6        # m/s
FPS = 30
def drive_state(f):
    """returns (y_center, z_center_offset_matrix stuff): car root matrix and wheel spin distance."""
    y0 = BUILD_Y
    # wiggle/rev before go
    if f < T_GO:
        M = car_root_static()
        if T_GO-40 < f < T_GO:
            k = math.sin((f)*2.1)*0.0006
            M = M @ Matrix.Translation((0, 0, abs(k))) @ Matrix.Rotation(k*3, 4, 'X')
        return M, 0.0, False
    # ---- MuJoCo rigid-body trajectory ----
    pos, q, wh = sim_state(sim_time(f - T_GO))
    M = q.to_matrix().to_4x4(); M.translation = Vector(pos)
    return M, list(wh), True
    yl = Y_BOXEDGE + 0.001                     # ramp lip
    acc = V_LAUNCH**2/(2*(y0 - (yl - WB)))     # rear wheel reaches lip at V_LAUNCH... approx
    t = (f - T_GO)/FPS
    d = 0.5*acc*t*t
    yc = y0 - d
    y_rear = yc + WB
    if y_rear > yl:
        yf = yc - WB
        zr = Z_BOX + ramp_height(y_rear)
        if yf > yl: zf = Z_BOX + ramp_height(yf)
        else:  # front beyond lip: continue along ramp slope line
            slope = (RAMP_H*0.75)/RAMP_LEN
            zf = Z_BOX + RAMP_H + (yl - yf)*slope
        pitch = math.atan2(zf - zr, 2*WB)
        zc = (zf + zr)/2 + WR
        M = Matrix.Translation((0, yc, zc - WR)) @ Matrix.Rotation(YAW, 4, 'Z') @ Matrix.Rotation(-pitch, 4, 'Y')
        return M, d, False
    # ---- ballistic ----
    t_launch = math.sqrt(2*(y0 + WB - yl)/acc) if False else None
    d_l = y0 - (yl - WB); tl = math.sqrt(2*d_l/acc); v = acc*tl
    th = math.atan2(RAMP_H*0.75, RAMP_LEN)
    vy, vz = v*math.cos(th), v*math.sin(th)
    G = 3.0
    yc_l = yl - WB; zc_l = Z_BOX + RAMP_H + WB*math.tan(th)*0 + WR - 0.0
    zc_l = Z_BOX + (RAMP_H + (RAMP_H + 2*WB*math.tan(th)))/2 + WR
    def flight(tt):
        return yc_l - vy*tt, zc_l + vz*tt - 0.5*G*tt*tt, th - 0.9*tt   # pitch decays nose-down
    # find landing time
    tt, land = 0.0, None
    while tt < 2:
        yy, zz, pp = flight(tt)
        if yy < Y_BOOKEDGE:
            zlow = min(zz - WR + WB*math.sin(-pp), zz - WR - WB*math.sin(-pp))
            if zlow <= Z_BOOK: land = tt; break
        tt += 0.002
    tf = t - tl
    if tf < land:
        yy, zz, pp = flight(tf)
        M = Matrix.Translation((0, yy, zz - WR)) @ Matrix.Rotation(YAW, 4, 'Z') @ Matrix.Rotation(-pp, 4, 'Y')
        return M, d_l + vy*tf*1.0, False
    # ---- landed: bounce, brake and drift ----
    yy, zz, pp = flight(land)
    ta = tf - land
    dec = 2.6
    tstop = vy/dec
    ts = min(ta, tstop)
    dist = vy*ts - 0.5*dec*ts*ts
    bounce = 0.006*math.exp(-ta*9)*abs(math.sin(ta*22))
    pitch = pp*math.exp(-ta*14)*math.cos(ta*20)
    yaw_extra = math.radians(35)*(1 - math.exp(-ta*5))
    y_c = yy - dist*math.cos(yaw_extra*0.5)
    x_c = -dist*math.sin(yaw_extra*0.5)*0.6
    M = (Matrix.Translation((x_c, y_c, Z_BOOK + bounce)) @ Matrix.Rotation(YAW + yaw_extra, 4, 'Z')
         @ Matrix.Rotation(-pitch, 4, 'Y'))
    return M, d_l + vy*land + dist, True

def wheel_local_spin(M, dist):
    return M @ Matrix.Rotation(dist/WR, 4, 'Y')

# headlight emission animation
def set_lights(f):
    L0 = 192
    e = 0.0 if f < L0 else min(1.0, (f-L0)/6)
    if L0 <= f < L0+12: e *= (f//3) % 2
    lego.TYELLOW().node_tree.nodes['Principled BSDF'].inputs['Emission Strength'].default_value = 0.3 + 6*e
    lego.TRED().node_tree.nodes['Principled BSDF'].inputs['Emission Strength'].default_value = 0.3 + 3*e

wheel_set = set(wheels)
def pose(f):
    root, dist, _ = drive_state(f)
    done_all = True
    for i, (ob, M, kind) in enumerate(all_parts):
        if kind == 'car' and ob in wheel_set and f >= T_GO:
            ob.matrix_world = root @ M @ Matrix.Rotation(dist[wheels.index(ob)], 4, 'Y'); continue
        Mx, done = part_matrix(i, f, root)
        ob.matrix_world = Mx
    set_lights(f)
    cam.matrix_world = cam_matrix(f)

# ---------------- render loop ----------------
from bpy_extras.object_utils import world_to_camera_view
def border(f):
    dg = bpy.context.evaluated_depsgraph_get()
    xs, ys = [], []
    for ob, _, _ in all_parts:
        for o in [ob] + list(ob.children):
            for c in o.bound_box:
                p = world_to_camera_view(sc, cam, o.matrix_world @ Vector(c))
                if p.z > 0: xs.append(p.x); ys.append(p.y)
    if not xs: return None
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    if x1 < 0 or x0 > 1 or y1 < 0 or y0 > 1: return None
    mx, my = (x1-x0)*0.45 + 0.04, (y1-y0)*0.45 + 0.03
    return max(0, x0-mx), min(1, x1+mx), max(0, y0-my), min(1, y1+my)

sc.render.use_border = True; sc.render.use_crop_to_border = True
sc.render.image_settings.media_type = 'MULTI_LAYER_IMAGE'
sc.render.image_settings.file_format = 'OPEN_EXR_MULTILAYER'
sc.render.image_settings.color_depth = '16'
# ---- bake every pose into keyframes so Cycles can render motion blur ----
def bake(fr0, fr1):
    objs = [ob for ob, _, _ in all_parts] + [cam]
    for o in objs: o.rotation_mode = 'QUATERNION'
    prevq = {}
    for f in range(fr0, fr1):
        pose(f)
        for o in objs:
            loc, q, s_ = o.matrix_world.decompose()
            if o.name in prevq and prevq[o.name].dot(q) < 0: q = -q
            prevq[o.name] = q
            o.location = loc; o.rotation_quaternion = q; o.scale = s_
            for p in ('location', 'rotation_quaternion', 'scale'): o.keyframe_insert(p, frame=f)
    for o in objs:
        for fc in o.animation_data.action.fcurves if hasattr(o.animation_data.action, 'fcurves') else []:
            for kp in fc.keyframe_points: kp.interpolation = 'LINEAR'
MB = float(os.environ.get('MB', '0.5'))
if ENGINE == 'WORKBENCH':
    sc.render.engine = 'BLENDER_WORKBENCH'; MB = 0
    sc.display.shading.light = 'STUDIO'; sc.display.shading.color_type = 'MATERIAL'
    sc.display.shading.show_shadows = False; sc.display.shading.show_cavity = True
    for o in bpy.data.objects:
        if o.is_shadow_catcher: o.hide_render = True
    for m_ in bpy.data.materials:
        try: m_.diffuse_color = (*m_.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value[:3], 1)
        except Exception: pass
if MB > 0:
    bake(max(0, START-1), min(642, END+1))
    sc.render.use_motion_blur = True; sc.render.motion_blur_shutter = MB
if MODE == 'wall':
    sc.render.engine = 'CYCLES'
    sc.cycles.samples = int(os.environ.get('SPP', 6)); sc.cycles.use_denoising = False
    sc.cycles.max_bounces = 0; sc.cycles.use_adaptive_sampling = False
    sc.render.use_motion_blur = MB > 0
    vl.cycles.use_pass_shadow_catcher = False
    for ob, _, _ in all_parts:
        for o in [ob] + list(ob.children): o.hide_render = True
    for o in list(bpy.data.objects):
        if o.is_shadow_catcher:
            o.is_shadow_catcher = False; o.is_holdout = True
    wall.hide_render = True
    # table top + white frame as holdouts, back edge along the fitted frame line a(b) = a0 + s*b
    a0r, sr, dzr = json.load(open('/home/claude/sfm/fitted.json'))['rim']
    YR = SCALE*float(os.environ.get('RIGHT_B', '-8.8'))
    import bmesh
    def quad(name, z, inset):
        bm = bmesh.new(); vs = []
        for b in (YR/SCALE, 30):
            vs.append(bm.verts.new((SCALE*(a0r + sr*b - inset - A0), SCALE*b, z)))
        for b in (30, YR/SCALE):
            vs.append(bm.verts.new((-1.5, SCALE*b, z)))
        bm.faces.new(vs); me = bpy.data.meshes.new(name); bm.to_mesh(me)
        o = bpy.data.objects.new(name, me); sc.collection.objects.link(o); o.is_holdout = True; return o
    table.hide_render = True
    bpy.data.objects['book'].hide_render = True
    af, ab, bl, br = 6.41, 12.7, -2.55, -7.9          # book outline tuned against footage
    hf, hb = float(os.environ.get('BH_F', '-5.26')), float(os.environ.get('BH_B', '-5.06'))   # cover slopes up towards the back
    import bmesh as _bm
    bm = _bm.new()
    abr = float(os.environ.get('AB_R', '13.3'))
    base = [(af, bl), (af, br), (abr, br), (ab, bl)]; tops = [hf, hf, hb, hb]
    vb = [bm.verts.new(to_bl(a, b, T0)) for a, b in base]
    vt = [bm.verts.new(to_bl(a, b, h)) for (a, b), h in zip(base, tops)]
    bm.faces.new(vb[::-1]); bm.faces.new(vt)
    for i in range(4):
        j = (i+1) % 4; bm.faces.new((vb[i], vb[j], vt[j], vt[i]))
    me = bpy.data.meshes.new('bookMatte'); bm.to_mesh(me)
    bk = bpy.data.objects.new('bookMatte', me); sc.collection.objects.link(bk); bk.is_holdout = True
    quad('tableTop', 0.0, 0.0)
    quad('frameTop', SCALE*dzr, 0.0)
    if os.environ.get('DEBUGVIS'):
        cols = [(1,0,0),(0,1,0),(0,0,1),(1,1,0),(1,0,1)]
        for i, o in enumerate([o for o in bpy.data.objects if o.is_holdout]):
            o.is_holdout = False; mm = bpy.data.materials.new('dbg%d' % i); mm.use_nodes = True
            mm.node_tree.nodes['Principled BSDF'].inputs['Emission Color'].default_value = (*cols[i % 5], 1)
            mm.node_tree.nodes['Principled BSDF'].inputs['Emission Strength'].default_value = 1
            o.data.materials.clear(); o.data.materials.append(mm); print('DBGCOL', o.name, cols[i % 5], o.hide_render)
    print('DBG wall hidden', wall.hide_render, 'cam z', cam.matrix_world.translation)
    WT = float(os.environ.get('WALLTILE', '0.55'))       # wallpaper image width on the wall, metres
    img = bpy.data.images.load('/home/claude/hdri/wallpaper.jpg')
    ar = img.size[1]/img.size[0]
    bpy.ops.mesh.primitive_plane_add(size=1)
    wp = bpy.context.object; wp.name = 'wallpaper'
    wp.rotation_euler = (math.pi/2, 0, math.pi/2)          # plane in YZ, facing -X
    WZ0 = float(os.environ.get('WALL_Z0', '0.13')); WH = 1.3 - WZ0
    wp.location = (SCALE*(16.375-A0) - 0.001, 0, WZ0 + WH/2); wp.scale = (2.4, WH, 1)
    m = bpy.data.materials.new('wpM'); m.use_nodes = True; nt2 = m.node_tree
    for n in list(nt2.nodes): nt2.nodes.remove(n)
    tc2 = nt2.nodes.new('ShaderNodeTexCoord'); mp2 = nt2.nodes.new('ShaderNodeMapping')
    tx = nt2.nodes.new('ShaderNodeTexImage'); tx.image = img; tx.extension = 'REPEAT'; tx.interpolation = 'Cubic'
    em = nt2.nodes.new('ShaderNodeEmission'); outn = nt2.nodes.new('ShaderNodeOutputMaterial')
    mp2.inputs['Scale'].default_value = (2.4/WT, WH/(WT*ar), 1)
    nt2.links.new(tc2.outputs['UV'], mp2.inputs['Vector']); nt2.links.new(mp2.outputs[0], tx.inputs[0])
    nt2.links.new(tx.outputs['Color'], em.inputs['Color']); nt2.links.new(em.outputs[0], outn.inputs[0])
    wp.data.materials.append(m)
    # ---- framed assembly drawing on the wall ----
    def emis_mat(name, rgb=None, img_path=None, alpha_mix=False):
        mm = bpy.data.materials.new(name); mm.use_nodes = True; t = mm.node_tree
        for n in list(t.nodes): t.nodes.remove(n)
        out = t.nodes.new('ShaderNodeOutputMaterial'); e = t.nodes.new('ShaderNodeEmission')
        if img_path:
            tx2 = t.nodes.new('ShaderNodeTexImage'); tx2.image = bpy.data.images.load(img_path); tx2.interpolation = 'Cubic'
            t.links.new(tx2.outputs['Color'], e.inputs['Color'])
        else: e.inputs['Color'].default_value = (*rgb, 1)
        if alpha_mix:
            tr = t.nodes.new('ShaderNodeBsdfTransparent'); mx = t.nodes.new('ShaderNodeMixShader')
            t.links.new(tx2.outputs['Alpha'], mx.inputs[0]); t.links.new(tr.outputs[0], mx.inputs[1]); t.links.new(e.outputs[0], mx.inputs[2])
            e.inputs['Color'].default_value = (0, 0, 0, 1); t.links.remove(e.inputs['Color'].links[0])
            t.links.new(mx.outputs[0], out.inputs[0])
        else: t.links.new(e.outputs[0], out.inputs[0])
        return mm
    XW = SCALE*(16.375-A0) - 0.001
    PW = 0.24*float(os.environ.get('PIC_SCALE', '1.3')); PH = PW*1700/2400
    PY, PZ = float(os.environ.get('PIC_Y', '-0.02')), float(os.environ.get('PIC_Z', '0.41'))
    FB, FD = 0.018, 0.02
    def wall_plane(name, w_, h_, y_, z_, x_, mat_):
        bpy.ops.mesh.primitive_plane_add(size=1); o = bpy.context.object; o.name = name
        o.rotation_euler = (math.pi/2, 0, -math.pi/2); o.location = (x_, y_, z_); o.scale = (w_, h_, 1)
        o.data.materials.append(mat_); return o
    wall_plane('picShadow', PW+2*FB+0.05, PH+2*FB+0.05, PY+0.006, PZ-0.008, XW-0.0005, emis_mat('shM', img_path='/home/claude/drawing/shadow.png', alpha_mix=True))
    wall_plane('picture', PW, PH, PY, PZ, XW-FD+0.004, emis_mat('picM', img_path='/home/claude/drawing/assembly_drawing.png'))
    fm = emis_mat('frameM', rgb=(0.035, 0.033, 0.03))
    for nm, (dy, dz, sy, sz) in {'fT': (0, PH/2+FB/2, PW+2*FB, FB), 'fB': (0, -PH/2-FB/2, PW+2*FB, FB),
                                 'fL': (-PW/2-FB/2, 0, FB, PH), 'fR': (PW/2+FB/2, 0, FB, PH)}.items():
        bpy.ops.mesh.primitive_cube_add(size=1); o = bpy.context.object; o.name = nm
        o.location = (XW - FD/2, PY+dy, PZ+dz); o.scale = (FD, sy, sz); o.data.materials.append(fm)
    sc.world.node_tree.nodes['Background'].inputs['Strength'].default_value = 0
    bpy.context.view_layer.update()
    for o in bpy.data.objects:
        if o.type == 'MESH' and not o.hide_render and o.name in ('Plane', 'wallpaper', 'box', 'book') or o.is_holdout:
            bb = [o.matrix_world @ Vector(c) for c in o.bound_box]
            print('DBG', o.name, o.is_holdout, [round(min(v[i] for v in bb), 3) for i in range(3)], [round(max(v[i] for v in bb), 3) for i in range(3)])
if os.environ.get('CALIB'):
    for o in bpy.data.objects:
        if o.is_shadow_catcher: o.is_shadow_catcher = False
    for ob, _, _ in all_parts:
        for o in [ob] + list(ob.children): o.hide_render = True
meta = {}
step = int(os.environ.get('STEP', '1'))
# 3D annotation geometry (world metres)
XH = -0.055
HUD3D = {'gapA': (XH, Y_BOXEDGE, Z_BOX), 'gapB': (XH, Y_BOOKEDGE, Z_BOX),
         'dropT': (XH, Y_BOOKEDGE, Z_BOX), 'dropB': (XH, Y_BOOKEDGE, Z_BOOK),
         'ramp0': (0.04, Y_BOXEDGE+0.001, Z_BOX+RAMP_H), 'ramp1': (0.04, Y_BOXEDGE+0.001+RAMP_LEN, Z_BOX+0.25*RAMP_H)}
_th = math.atan2(0.75*RAMP_H, RAMP_LEN); _v = 1.6
_y0 = Y_BOXEDGE + 0.001 - WB; _z0 = Z_BOX + RAMP_H + WB*math.tan(_th) + WR
path = []
for k in range(60):
    tt = k*0.003
    yy = _y0 - _v*math.cos(_th)*tt; zz = _z0 + _v*math.sin(_th)*tt - 0.5*9.81*tt*tt
    path.append((0.0, yy, zz))
    if zz < Z_BOOK + WR: break
HUD3D['path'] = path
def hud_proj():
    out = {}
    for k, v in HUD3D.items():
        pts = v if k == 'path' else [v]
        pp = []
        for p in pts:
            c = world_to_camera_view(sc, cam, Vector(p)); pp.append((c.x, 1-c.y, c.z))
        out[k] = pp if k == 'path' else pp[0]
    return out
hud = {}
for f in range(START, END, step):
    if MB > 0: sc.frame_set(f); set_lights(f)
    else: pose(f)
    bpy.context.view_layer.update()
    hud[f] = hud_proj(); json.dump(hud, open(f'{OUT}/hud_{START}_{END}.json', 'w'))
    bd = border(f) if not (os.environ.get('CALIB') or MODE == 'wall') else (0, 1, 0, 1)
    if bd is None:
        meta[f] = None; continue
    sc.render.border_min_x, sc.render.border_max_x, sc.render.border_min_y, sc.render.border_max_y = bd
    meta[f] = bd
    json.dump(meta, open(f'{OUT}/meta_{START}_{END}.json', 'w'))
    if os.environ.get('NORENDER'): continue
    sc.render.filepath = f'{OUT}/r{f:04d}.exr'
    bpy.ops.render.render(write_still=True)
    meta[f] = bd
    print('FRAME', f, flush=True)
json.dump(meta, open(f'{OUT}/meta_{START}_{END}.json', 'w'))
