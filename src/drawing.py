"""Exploded CAD-style assembly drawing of the Lego car (EEVEE + Freestyle line art)."""
import sys, math, json
sys.path.insert(0, '/home/claude/scripts')
import bpy
from mathutils import Matrix, Vector
bpy.ops.wm.read_factory_settings(use_empty=True)
import lego
from bpy_extras.object_utils import world_to_camera_view
parts, wheels = lego.build_car()
S = lego.S
def level(n):
    if n.startswith('axle'): return (0, 0, -0.022)
    if n.startswith('wheel'): return (0, 0.045 if n in ('wheel0', 'wheel2') else -0.045, -0.022)
    if n == 'chassis': return (0, 0, 0)
    if n.startswith(('bumper', 'front', 'rear', 'side')) and n != 'rearDeck': return (0, 0, 0.03)
    if n.startswith('seat'): return (0, 0, 0.045)
    if n == 'driver': return (0, 0, 0.085)
    if n in ('hood', 'dash', 'rearDeck', 'steer', 'windshield'): return (0, 0, 0.075)
    if n.startswith('post'): return (0, 0, 0.1)
    if n == 'spoiler': return (0, 0, 0.125)
    if n.startswith('head'): return (0.03, 0, 0.09)
    if n.startswith('tail'): return (-0.03, 0, 0.09)
    return (0, 0, 0.05)
sc = bpy.context.scene
info = []
white = bpy.data.materials.new('w'); white.use_nodes = True
nt = white.node_tree; [nt.nodes.remove(n) for n in list(nt.nodes)]
em = nt.nodes.new('ShaderNodeEmission'); em.inputs[0].default_value = (0.70, 0.70, 0.70, 1)   # light grey fill (linear ~ sRGB 0.86)
o = nt.nodes.new('ShaderNodeOutputMaterial'); nt.links.new(em.outputs[0], o.inputs[0])
for ob, M in parts:
    off = Vector(level(ob.name))
    ob.matrix_world = Matrix.Translation(off) @ M
    for x in [ob] + list(ob.children):
        x.data.materials.clear(); x.data.materials.append(white)
        for m in x.modifiers: pass
    info.append((ob.name, M.translation.copy(), (Matrix.Translation(off) @ M).translation.copy()))
cam = bpy.data.objects.new('cam', bpy.data.cameras.new('cam')); sc.collection.objects.link(cam); sc.camera = cam
cam.data.type = 'ORTHO'; cam.data.ortho_scale = 0.30
d = Vector((1, -1.15, 0.85)).normalized()
cam.location = Vector((0, 0, 0.05)) + d*1.0
cam.rotation_euler = (-d).to_track_quat('-Z', 'Y').to_euler()
sc.render.engine = 'BLENDER_EEVEE'
sc.render.resolution_x, sc.render.resolution_y = 1700, 1500
sc.render.film_transparent = True
sc.render.use_freestyle = True
sc.render.line_thickness_mode = 'ABSOLUTE'; sc.render.line_thickness = 1.0
vl = sc.view_layers[0]; fs = vl.freestyle_settings
ls = fs.linesets[0] if fs.linesets else fs.linesets.new('L')
ls.select_by_visibility = True; ls.select_by_edge_types = True
ls.select_silhouette = ls.select_border = True; ls.select_crease = False
ls2 = fs.linesets.new('inner'); ls2.select_by_visibility = True; ls2.select_by_edge_types = True
ls2.select_silhouette = ls2.select_border = False; ls2.select_crease = True
ls2.linestyle = bpy.data.linestyles.new('LS2'); ls2.linestyle.color = (0, 0, 0); ls2.linestyle.thickness = 1.5
if ls.linestyle is None: ls.linestyle = bpy.data.linestyles.new('LS')
ls.linestyle.color = (0, 0, 0); ls.linestyle.thickness = 2.0
fs.crease_angle = math.radians(140)
w = bpy.data.worlds.new('w'); sc.world = w
sc.view_settings.view_transform = 'Standard'
sc.render.filepath = '/home/claude/drawing/lines.png'
bpy.ops.render.render(write_still=True)
pts = []
for n, fin, ex in info:
    a = world_to_camera_view(sc, cam, fin); b = world_to_camera_view(sc, cam, ex)
    pts.append((n, (a.x*1700, (1-a.y)*1500), (b.x*1700, (1-b.y)*1500)))
json.dump(pts, open('/home/claude/drawing/pts.json', 'w'))
print('OK')
