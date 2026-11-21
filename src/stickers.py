"""Sticker decals: thin image planes parented to the parts (they move with them)."""
import bpy, math
from mathutils import Matrix
S, P, B = 0.008, 0.0032, 0.0096
D = '/home/claude/stickers/'
EPS = 0.00008
_mats = {}
def mat(img):
    if img in _mats: return _mats[img]
    m = bpy.data.materials.new('stk_' + img); m.use_nodes = True; nt = m.node_tree
    bs = nt.nodes['Principled BSDF']; tx = nt.nodes.new('ShaderNodeTexImage')
    tx.image = bpy.data.images.load(D + img + '.png'); tx.interpolation = 'Cubic'; tx.extension = 'CLIP'
    nt.links.new(tx.outputs['Color'], bs.inputs['Base Color']); nt.links.new(tx.outputs['Alpha'], bs.inputs['Alpha'])
    bs.inputs['Roughness'].default_value = 0.22                         # shiny vinyl with clear coat
    bs.inputs['Coat Weight'].default_value = 1.0; bs.inputs['Coat Roughness'].default_value = 0.03
    bs.inputs['Specular IOR Level'].default_value = 0.6
    m.diffuse_color = (0.9, 0.9, 0.9, 1)
    _mats[img] = m; return m
def decal(parent, img, w, h, M):
    me = bpy.data.meshes.new('stk')
    me.from_pydata([(-0.5, -0.5, 0), (0.5, -0.5, 0), (0.5, 0.5, 0), (-0.5, 0.5, 0)], [], [(0, 1, 2, 3)])
    uv = me.uv_layers.new()
    for i, (u, v) in enumerate(((0, 0), (1, 0), (1, 1), (0, 1))): uv.data[i].uv = (u, v)
    me.materials.append(mat(img))
    o = bpy.data.objects.new(parent.name + '_stk_' + img, me); bpy.context.scene.collection.objects.link(o)
    o.parent = parent; o.matrix_parent_inverse = Matrix.Identity(4)
    o.matrix_basis = M @ Matrix.Diagonal((w, h, 1, 1))
    return o
Rneg_y = Matrix.Rotation(math.pi/2, 4, 'X')                                      # faces local -Y, reads along +X
Rpos_y = Matrix.Rotation(math.pi, 4, 'Z') @ Matrix.Rotation(math.pi/2, 4, 'X')  # faces local +Y, reads along -X
Rneg_x = Matrix.Rotation(-math.pi/2, 4, 'Y') @ Matrix.Rotation(-math.pi/2, 4, 'Z')  # faces local -X
# sticker sheet cut-outs: name -> (file, aspect w/h)
K = {'name': ('sheet/s0', 3.82), 'badge': ('sheet/s1', 1.0), 'channel': ('sheet/s2', 4.8), 'load': ('sheet/s3', 1.0),
     'stripe': ('sheet/s4', 3.07), 'hazard': ('sheet/s5', 3.97), 'fem': ('sheet/s6', 0.96), 'built': ('sheet/s7', 2.5),
     'bridge': ('sheet/s8', 1.0)}
def put(objs, part, key, h, M):
    if part not in objs: return None
    f, ar = K[key]; return decal(objs[part], f, h*ar, h, M)
def add(objs):
    T = Matrix.Translation
    out = []
    # truck, camera side (local -Y)
    out.append(put(objs, 'bed1_1', 'channel', 0.0088, T((0, -S/2 - EPS, B/2)) @ Rneg_y))
    out.append(put(objs, 'bed1_0', 'stripe', 0.0072, T((0, -S/2 - EPS, B/2)) @ Rneg_y))
    out.append(put(objs, 'cabside1', 'name', 0.0058, T((0, -S/2 - EPS, B/2)) @ Rneg_y))
    out.append(put(objs, 'engine', 'badge', 0.0086, T((0, -3*S - EPS, B/2)) @ Rneg_y))
    # truck, far side (local +Y) and rear
    out.append(put(objs, 'bed0_1', 'name', 0.0088, T((0, S/2 + EPS, B/2)) @ Rpos_y))
    out.append(put(objs, 'bed0_0', 'stripe', 0.0072, T((0, S/2 + EPS, B/2)) @ Rpos_y))
    out.append(put(objs, 'cabside0', 'built', 0.0080, T((0, S/2 + EPS, B/2)) @ Rpos_y))
    out.append(put(objs, 'engine', 'badge', 0.0086, T((0, 3*S + EPS, B/2)) @ Rpos_y))
    out.append(put(objs, 'tailgate1', 'bridge', 0.0086, T((-S/2 - EPS, 0, B/2)) @ Rneg_x))
    # bridge pier, end face towards the camera (local -X)
    out.append(put(objs, 'pier6', 'hazard', 0.0038, T((-4*S - EPS, 0, B/2)) @ Rneg_x))
    out.append(put(objs, 'pier0', 'hazard', 0.0038, T((-4*S - EPS, 0, B/2)) @ Rneg_x))
    out.append(put(objs, 'pier4', 'load', 0.0086, T((-4*S - EPS, 0, B/2)) @ Rneg_x))
    out.append(put(objs, 'pier2', 'fem', 0.0086, T((-4*S - EPS, 0, B/2)) @ Rneg_x))
    return [o for o in out if o]
