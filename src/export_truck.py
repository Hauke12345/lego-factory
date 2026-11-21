"""Truck part silhouettes (root-local verts + colour) for the stress-view panel."""
import sys, json, math
sys.path.insert(0, '/home/claude/scripts')
import bpy
bpy.ops.wm.read_factory_settings(use_empty=True)
import lego, truck
tp, tw, _ = truck.build_truck()
for ob, M in tp: ob.matrix_world = M
bpy.context.view_layer.update()
def srgb(c): return [int(255*(x*12.92 if x <= 0.0031308 else 1.055*x**(1/2.4) - 0.055)) for x in c[:3]]
out = []
for ob, M in tp:
    for o in [ob] + list(ob.children_recursive):
        if o.type != 'MESH' or o.name.endswith('_holes'): continue
        m = o.data.materials[0]; b = m.node_tree.nodes['Principled BSDF']
        col = srgb(b.inputs['Base Color'].default_value)
        tr = b.inputs['Transmission Weight'].default_value
        vs = [list(o.matrix_world @ v.co) for v in o.data.vertices]
        out.append({'name': o.name, 'col': col, 'alpha': 0.35 if tr > 0.5 else 1.0, 'verts': vs})
json.dump(out, open('/home/claude/render_b/truck_parts.json', 'w')); print(len(out))
