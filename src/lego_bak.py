"""Procedural Lego car. All coords in metres, car-local: X forward, Y left, Z up, ground at z=0."""
import bpy, bmesh, math
from mathutils import Vector, Matrix

S = 0.008      # stud pitch
P = 0.0032     # plate height
B = 0.0096     # brick height
GAP = 0.0001
STUD_R, STUD_H = 0.0024, 0.0017

_mats = {}
def mat(name, rgb, rough=0.25, trans=0.0, emit=0.0):
    if name in _mats: return _mats[name]
    m = bpy.data.materials.new(name); m.use_nodes = True
    bs = m.node_tree.nodes['Principled BSDF']
    bs.inputs['Base Color'].default_value = (*rgb, 1)
    bs.inputs['Roughness'].default_value = rough
    bs.inputs['Coat Weight'].default_value = 0.3
    bs.inputs['Coat Roughness'].default_value = 0.15
    if trans:
        bs.inputs['Transmission Weight'].default_value = trans
        bs.inputs['IOR'].default_value = 1.49
        bs.inputs['Roughness'].default_value = 0.05
    if emit:
        bs.inputs['Emission Color'].default_value = (*rgb, 1)
        bs.inputs['Emission Strength'].default_value = emit
    _mats[name] = m
    return m

def srgb(h):
    c = [int(h[i:i+2], 16)/255 for i in (0, 2, 4)]
    return tuple(x/12.92 if x <= 0.04045 else ((x+0.055)/1.055)**2.4 for x in c)

RED = lambda: mat('red', srgb('C91A09'))
DGRAY = lambda: mat('dgray', srgb('6C6E68'))
LGRAY = lambda: mat('lgray', srgb('A0A5A9'))
BLACK = lambda: mat('black', (0.012, 0.014, 0.016), 0.45)
YELLOW = lambda: mat('yellow', srgb('F2CD37'))
BLUE = lambda: mat('blue', srgb('0055BF'))
WHITE = lambda: mat('white', srgb('F4F4F4'))
TCLEAR = lambda: mat('tclear', (0.85, 0.92, 0.95), trans=1.0)
TYELLOW = lambda: mat('tyellow', srgb('F8F184'), trans=0.6, emit=0.5)
TRED = lambda: mat('tred', srgb('C91A09'), trans=0.6, emit=0.4)


def _add_cyl(bm, center, r, h, segs=16, axis='Z'):
    rot = Matrix.Identity(4)
    if axis == 'Y': rot = Matrix.Rotation(math.pi/2, 4, 'X')
    if axis == 'X': rot = Matrix.Rotation(math.pi/2, 4, 'Y')
    m = Matrix.Translation(center) @ rot
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segs, radius1=r, radius2=r, depth=h, matrix=m)


def _studs(bm, nx, ny, ztop, skip=()):
    for i in range(nx):
        for j in range(ny):
            if (i, j) in skip: continue
            x = (i - (nx-1)/2) * S; y = (j - (ny-1)/2) * S
            _add_cyl(bm, Vector((x, y, ztop + STUD_H/2)), STUD_R, STUD_H, 12)


def _finish(bm, name, material, bevel=0.0004):
    me = bpy.data.meshes.new(name); bm.to_mesh(me); bm.free()
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    me.materials.append(material)
    for p in me.polygons: p.use_smooth = True
    if bevel:
        bv = ob.modifiers.new('bev', 'BEVEL'); bv.width = bevel; bv.segments = 2
        bv.limit_method = 'ANGLE'; bv.harden_normals = False
    return ob


def brick(name, nx, ny, h, material, studs=True):
    """nx studs along X, ny along Y, origin at bottom centre."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Translation((0, 0, h/2)) @
                          Matrix.Diagonal((nx*S-2*GAP, ny*S-2*GAP, h, 1)))
    if studs: _studs(bm, nx, ny, h)
    return _finish(bm, name, material)


def slope(name, nx, ny, material, h=B, flat=S*0.5):
    """Wedge sloping down towards +X; flat top strip of width `flat` at -X end."""
    bm = bmesh.new()
    x0, x1 = -nx*S/2+GAP, nx*S/2-GAP; y0, y1 = -ny*S/2+GAP, ny*S/2-GAP
    lo = 0.25*h
    pts = [(x0, y0, 0), (x1, y0, 0), (x1, y0, lo), (x0+flat, y0, h), (x0, y0, h)]
    v0 = [bm.verts.new((x, y0, z)) for x, _, z in pts]
    v1 = [bm.verts.new((x, y1, z)) for x, _, z in pts]
    bm.faces.new(v0[::-1]); bm.faces.new(v1)
    n = len(pts)
    for i in range(n):
        bm.faces.new((v0[i], v0[(i+1) % n], v1[(i+1) % n], v1[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    for j in range(ny):  # studs on flat strip
        _add_cyl(bm, Vector((x0+S/2-GAP if flat >= S else x0+flat/2, (j-(ny-1)/2)*S, h+STUD_H/2)), STUD_R, STUD_H, 12) if flat >= S else None
    return _finish(bm, name, material)


def panel(name, ny, h, thick, material):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Translation((0, 0, h/2)) @
                          Matrix.Diagonal((thick, ny*S-2*GAP, h, 1)))
    return _finish(bm, name, material, 0.0003)


def round1x1(name, material, h=P):
    bm = bmesh.new()
    _add_cyl(bm, Vector((0, 0, h/2)), S/2-GAP, h, 20)
    _add_cyl(bm, Vector((0, 0, h+STUD_H/2)), STUD_R, STUD_H, 12)
    return _finish(bm, name, material, 0.0002)


def wheel(name):
    bm = bmesh.new()
    _add_cyl(bm, Vector((0, 0, 0)), 0.0125, 0.010, 32, 'Y')
    ob = _finish(bm, name, BLACK(), 0.0015)
    bm = bmesh.new()
    _add_cyl(bm, Vector((0, 0, 0)), 0.0085, 0.0104, 24, 'Y')
    rim = _finish(bm, name+'_rim', LGRAY(), 0.0003)
    rim.parent = ob
    return ob


def axle(name):
    bm = bmesh.new()
    _add_cyl(bm, Vector((0, 0, 0)), 0.0016, 8*S, 10, 'Y')
    return _finish(bm, name, DGRAY(), 0)


def minifig(name):
    bm = bmesh.new()  # torso
    bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Translation((0, 0, 0.0064)) @ Matrix.Diagonal((0.008, 0.0155, 0.0128, 1)))
    torso = _finish(bm, name, BLUE(), 0.0005)
    bm = bmesh.new()
    _add_cyl(bm, Vector((0, 0, 0.0128+0.0046)), 0.0048, 0.0092, 20)
    _add_cyl(bm, Vector((0, 0, 0.0128+0.0092+0.001)), 0.0018, 0.002, 12)
    head = _finish(bm, name+'_head', YELLOW(), 0.0008); head.parent = torso
    bm = bmesh.new()  # helmet
    bmesh.ops.create_uvsphere(bm, u_segments=20, v_segments=10, radius=0.0056,
                              matrix=Matrix.Translation((0, 0, 0.0128+0.0075)))
    helm = _finish(bm, name+'_helmet', WHITE(), 0); helm.parent = torso
    return torso


def build_car():
    """Returns list of (obj, final_local_matrix) in assembly order, plus wheel list."""
    parts = []
    def add(ob, x, y, z, rz=0.0, rx=0.0):
        M = Matrix.Translation((x, y, z)) @ Matrix.Rotation(rz, 4, 'Z') @ Matrix.Rotation(rx, 4, 'Y')
        parts.append((ob, M)); return ob

    wz = 0.0125                     # wheel centre height
    zc = 0.0105                     # chassis bottom
    add(brick('chassis', 14, 6, P, DGRAY()), 0, 0, zc)
    wheels = []
    for k, x in enumerate((4.5*S, -4.5*S)):
        add(axle(f'axle{k}'), x, 0, wz - 0.0005)
    for k, (x, y) in enumerate([(4.5*S, 1), (4.5*S, -1), (-4.5*S, 1), (-4.5*S, -1)]):
        w = add(wheel(f'wheel{k}'), x, y*(3*S+0.0058), wz)
        wheels.append(w)
    z1 = zc + P
    add(brick('bumperF', 1, 6, B, LGRAY()), 6.5*S, 0, z1)
    add(brick('bumperR', 1, 6, B, LGRAY()), -6.5*S, 0, z1)
    add(brick('front', 2, 6, B, RED()), 5*S, 0, z1)
    add(brick('rear', 2, 6, B, RED()), -5*S, 0, z1)
    add(brick('sideL', 8, 1, B, RED()), 0, 2.5*S, z1)
    add(brick('sideR', 8, 1, B, RED()), 0, -2.5*S, z1)
    add(brick('seatL', 2, 2, P, BLACK()), -1.5*S, 1*S, z1)
    add(brick('seatR', 2, 2, P, BLACK()), -1.5*S, -1*S, z1)
    add(brick('seatbackL', 1, 2, B, BLACK()), -3*S, 1*S, z1)
    add(brick('seatbackR', 1, 2, B, BLACK()), -3*S, -1*S, z1)
    add(minifig('driver'), -1.5*S, 1*S, z1 + P)
    z2 = z1 + B
    add(slope('hood', 2, 6, RED(), flat=S*0.3), 5*S, 0, z2)
    add(brick('dash', 1, 6, P, BLACK()), 3.5*S, 0, z2)
    bm = bmesh.new(); _add_cyl(bm, Vector((0, 0, 0)), 0.0035, 0.0012, 16, 'X')
    add(_finish(bm, 'steer', BLACK(), 0), 2.3*S, 1*S, z2 + 0.004, rx=-0.5)
    add(panel('windshield', 6, 1.6*B, 0.0016, TCLEAR()), 3.2*S, 0, z2 + P, rx=-0.45)
    add(brick('rearDeck', 2, 6, P, RED()), -5*S, 0, z2)
    add(round1x1('post1', LGRAY(), B), -6*S, 2*S, z2 + P)
    add(round1x1('post2', LGRAY(), B), -6*S, -2*S, z2 + P)
    add(brick('spoiler', 2, 6, P, BLACK()), -6*S, 0, z2 + P + B)
    for k, y in enumerate((2*S, -2*S)):
        add(round1x1(f'head{k}', TYELLOW()), 6.5*S, y, z2)
        add(round1x1(f'tail{k}', TRED()), -6.5*S, y, z2)
    return parts, wheels
