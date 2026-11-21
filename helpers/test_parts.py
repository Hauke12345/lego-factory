import sys; sys.path.insert(0, '/home/claude/scripts')
import bpy, numpy as np, ldr, lblend, mathutils
bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
def T(x, y, z):
    M = np.eye(4); M[:3, 3] = [x, y, z]; return M
items = [('26910', 71, T(0, 0, 0)), ('26913c01', 71, T(200, 0, 0)), ('26912', 71, T(-150, 0, 0)),
         ('32525', 0, T(0, -40, 260)), ('3648b', 7, T(-150, -40, 160)), ('3001', 4, T(-150, 0, -120)), ('2780', 0, T(-100, -40, 160))]
for i, (p, c, M) in enumerate(items): lblend.add(f'{p}_{i}', p, c, M)
lblend.workbench(sc, (1280, 800))
cam = bpy.data.objects.new('cam', bpy.data.cameras.new('cam')); sc.collection.objects.link(cam); sc.camera = cam
cam.data.lens = 50
loc = mathutils.Vector((0.25, -0.30, 0.22)); tgt = mathutils.Vector((0.02, 0.0, 0.0))
cam.location = loc; cam.rotation_euler = (tgt - loc).to_track_quat('-Z', 'Y').to_euler()
sc.world = bpy.data.worlds.new('w'); sc.display.shading.background_type = 'VIEWPORT'
sc.display.shading.background_color = (0.9, 0.9, 0.92)
sc.render.filepath = '/home/claude/factory/prev/parts_test.png'
bpy.ops.render.render(write_still=True)
