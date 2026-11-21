"""Export one film shot as an animated .blend for WarpKernel (Cycles scene as in film_render 'cy', no depth of field).
usage: python3 film_wk.py SHOT_INDEX OUT.blend [dof]  (dof: keep the shot's depth of field, for plain Cycles)"""
import sys, time, numpy as np
si, out = int(sys.argv[1]), sys.argv[2]; KEEP_DOF = len(sys.argv) > 3 and sys.argv[3] == 'dof'
sys.argv = ['film_render.py', 'cy', '/tmp/claude-0/-home-claude/5109fde9-05e6-5615-9b0c-31028b28323a/scratchpad/wk_dummy', '0', '0']
sys.path.insert(0, '/home/claude/scripts')
import bpy, mathutils
import film_render as FR, film, lblend

F = FR.F; sc, cam, objs = FR.sc, FR.cam, FR.objs
idx = [k for k, f in enumerate(F) if f['shot'] == si]
t0 = time.time()
PS = [{p['name']: p for p in FR.scene_parts(F[k])} for k in idx]
for P in PS:                                    # parts the film-wide union missed (it samples every 12th frame)
    for n, p in P.items():
        if n not in objs:
            objs[n] = lblend.add(n, p['part'], p['col'], p['M'])
            for me in [objs[n].data] if objs[n].type == 'MESH' else []:
                for pl in me.polygons: pl.use_smooth = True
import math
for mat in bpy.data.materials:                  # same plastic look as film_render.setup_cycles, also for parts added here
    if not mat.name.startswith('ld_') or not mat.node_tree: continue
    b = mat.node_tree.nodes.get('Principled BSDF')
    if b is None: continue
    if b.inputs['Transmission Weight'].default_value > 0.5 or b.inputs['Alpha'].default_value < 1:
        b.inputs['Alpha'].default_value = 1.0; b.inputs['Transmission Weight'].default_value = 0.92; b.inputs['Roughness'].default_value = 0.08
    else: b.inputs['Roughness'].default_value = 0.24
    try: b.inputs['Specular IOR Level'].default_value = 0.55
    except KeyError: pass
for me in bpy.data.meshes:
    if me.name == 'table': continue
    for pl in me.polygons: pl.use_smooth = True
    try: me.set_sharp_from_angle(angle=math.radians(38))
    except Exception: pass
names = list(objs.keys())
MAT = np.zeros((len(idx), len(names), 4, 4)); VIS = np.zeros((len(idx), len(names)), bool)
for j, k in enumerate(idx):
    P = PS[j]
    for i, n in enumerate(names):
        p = P.get(n)
        if p is not None: VIS[j, i] = True; MAT[j, i] = np.array(lblend.M_bl(p['M']))
print('states', time.time() - t0, flush=True)
moving = []
for i, n in enumerate(names):
    ob = objs[n]; v = VIS[:, i]
    if not v.any(): ob.hide_render = True; ob.hide_viewport = True; continue
    first = MAT[np.argmax(v), i]
    ob.matrix_world = mathutils.Matrix(first.tolist()); ob.hide_render = False
    if v.all() and np.abs(MAT[:, i] - first).max() < 1e-7: continue
    moving.append(i)
print('moving objects', len(moving), 'of', len(names), flush=True)
sc.frame_start, sc.frame_end = 1, len(idx)
for i in moving:
    ob = objs[names[i]]; ob.rotation_mode = 'QUATERNION'
    for j in range(len(idx)):
        f = j + 1
        if VIS[j, i]:
            ob.matrix_world = mathutils.Matrix(MAT[j, i].tolist())
            ob.keyframe_insert('location', frame=f); ob.keyframe_insert('rotation_quaternion', frame=f)
        vis = bool(VIS[j, i])
        if not VIS[:, i].all():
            ob.hide_render = not vis; ob.keyframe_insert('hide_render', frame=f)
cam.rotation_mode = 'QUATERNION'
for j, k in enumerate(idx):
    pos, tgt, lens, fst = film.camera(F[k])
    cam.location = mathutils.Vector(pos); cam.rotation_quaternion = (mathutils.Vector(tgt) - mathutils.Vector(pos)).to_track_quat('-Z', 'Y')
    cam.data.lens = lens
    cam.keyframe_insert('location', frame=j + 1); cam.keyframe_insert('rotation_quaternion', frame=j + 1); cam.data.keyframe_insert('lens', frame=j + 1)
    if KEEP_DOF and fst > 0:
        cam.data.dof.use_dof = True; cam.data.dof.focus_distance = float(np.linalg.norm(np.array(tgt) - np.array(pos))); cam.data.dof.aperture_fstop = fst
        cam.data.dof.keyframe_insert('focus_distance', frame=j + 1); cam.data.dof.keyframe_insert('aperture_fstop', frame=j + 1)
cam.data.dof.use_dof = bool(KEEP_DOF and any(film.camera(F[k])[3] > 0 for k in idx))
# constant interpolation: every frame is keyed, no easing between keys
for a in bpy.data.actions:
    try:
        for fc in a.fcurves:
            for kp in fc.keyframe_points: kp.interpolation = 'LINEAR'
    except AttributeError: pass
sc.render.filepath = '//frame_'
bpy.ops.wm.save_as_mainfile(filepath=out)
print('saved', out, len(idx), 'frames', time.time() - t0, 's', flush=True)
