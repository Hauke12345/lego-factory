"""Workbench preview of an LDraw design module. Usage: python3 cprev.py module out.png [view]"""
import sys, importlib, math
sys.path.insert(0, '/home/claude/scripts')
import bpy, numpy as np, mathutils, lblend, ldr

VIEWS = {  # metres, Blender Z-up. Conveyor runs along +x at y=+0.076 (front), hub behind at y=0
    'iso':   ((0.30, 0.40, 0.26), (0.02, 0.04, 0.03), 40),
    'front': ((0.04, 0.46, 0.10), (0.02, 0.05, 0.035), 42),
    'top':   ((0.0, 0.04, 0.60), (0.0, 0.04, 0.0), 40),
    'close': ((0.10, 0.22, 0.12), (0.0, 0.076, 0.045), 45),
    'station': ((-0.20, 0.30, 0.26), (0.01, 0.05, 0.06), 38),
    'hero':  ((0.26, 0.34, 0.22), (0.0, 0.045, 0.05), 30),
    'back':  ((-0.32, -0.34, 0.28), (0.0, 0.03, 0.03), 40),
}

def setup(P, res=(1600, 1000)):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    objs = {}
    for p in P: objs[p['name']] = lblend.add(p['name'], p['part'], p['col'], p['M'])
    lblend.workbench(sc, res)
    sc.display.shading.background_type = 'VIEWPORT'; sc.display.shading.background_color = (0.86, 0.87, 0.9)
    cam = bpy.data.objects.new('cam', bpy.data.cameras.new('cam')); sc.collection.objects.link(cam); sc.camera = cam
    cam.data.clip_start = 0.002; cam.data.clip_end = 10
    return sc, cam, objs

def aim(cam, view):
    loc, tgt, lens = VIEWS[view] if isinstance(view, str) else view
    loc, tgt = mathutils.Vector(loc), mathutils.Vector(tgt)
    cam.location = loc; cam.rotation_euler = (tgt - loc).to_track_quat('-Z', 'Y').to_euler(); cam.data.lens = lens

if __name__ == '__main__':
    mod = importlib.import_module(sys.argv[1]); P = mod.build()
    sc, cam, _ = setup(P)
    for v in (sys.argv[3].split(',') if len(sys.argv) > 3 else ['iso']):
        aim(cam, v)
        sc.render.filepath = sys.argv[2].replace('.png', f'_{v}.png')
        bpy.ops.render.render(write_still=True)
