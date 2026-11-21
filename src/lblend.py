"""Blender helpers for LDraw parts: mesh creation (one mesh datablock per part+colour), workbench preview setup."""
import bpy, numpy as np, ldr, mathutils

_MESH = {}; _MAT = {}
SRGB = lambda c: tuple((x/12.92 if x <= 0.04045 else ((x+0.055)/1.055)**2.4) for x in c)

def material(code):
    if code in _MAT: return _MAT[code]
    name, rgb, a = ldr.colour(code)
    m = bpy.data.materials.new(f'ld_{code}_{name}')
    lin = SRGB(rgb)
    m.diffuse_color = (*lin, a)
    m.use_nodes = True
    b = m.node_tree.nodes.get('Principled BSDF')
    if b:
        b.inputs['Base Color'].default_value = (*lin, 1)
        b.inputs['Roughness'].default_value = 0.35
        if a < 1: b.inputs['Alpha'].default_value = a
    _MAT[code] = m
    return m

def mesh(part, colour):
    key = (part, colour)
    if key in _MESH: return _MESH[key]
    d = ldr.part(part)
    V = d['tris'].reshape(-1, 3).astype(np.float64) @ ldr.C_BL.T * ldr.LDU
    # weld identical verts
    q = np.round(V/1e-6).astype(np.int64)
    uq, inv = np.unique(q, axis=0, return_inverse=True)
    verts = np.zeros((len(uq), 3)); verts[inv] = V
    faces = inv.reshape(-1, 3)
    cols = d['cols'].copy(); cols[cols == 16] = colour; cols[cols == 24] = colour
    me = bpy.data.meshes.new(f'{part}_{colour}')
    me.from_pydata(verts.tolist(), [], faces.tolist())
    codes = sorted(set(cols.tolist()))
    for c in codes: me.materials.append(material(c))
    lut = {c: i for i, c in enumerate(codes)}
    me.polygons.foreach_set('material_index', [lut[c] for c in cols.tolist()])
    me.validate(); me.update()
    for p in me.polygons: p.use_smooth = False
    _MESH[key] = me
    return me

def M_bl(M_ld):
    """4x4 LDraw world (LDU) -> Blender world matrix (m)."""
    C = np.eye(4); C[:3, :3] = ldr.C_BL
    S = np.eye(4); S[:3, :3] *= ldr.LDU
    # x_bl = C S M x_ld ; object built from verts already converted (C*LDU), so object matrix = C S M S^-1 C^-1
    Mb = C @ S @ M_ld @ np.linalg.inv(S) @ np.linalg.inv(C)
    return mathutils.Matrix(Mb.tolist())

def add(name, part, colour, M_ld, coll=None):
    ob = bpy.data.objects.new(name, mesh(part, colour))
    ob.matrix_world = M_bl(M_ld)
    (coll or bpy.context.scene.collection).objects.link(ob)
    return ob

def workbench(scene, res=(1280, 720), cavity=True):
    scene.render.engine = 'BLENDER_WORKBENCH'
    sh = scene.display.shading
    sh.light = 'STUDIO'; sh.color_type = 'MATERIAL'
    sh.show_cavity = cavity; sh.cavity_type = 'BOTH'
    sh.show_shadows = True; sh.shadow_intensity = 0.35
    sh.show_specular_highlight = True
    scene.display.render_aa = '8'
    scene.render.resolution_x, scene.render.resolution_y = res
    scene.render.film_transparent = False
    try: scene.view_settings.view_transform = 'Standard'; scene.view_settings.exposure = 0.55
    except Exception: pass
