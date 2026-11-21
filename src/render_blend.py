"""Plain Cycles render of an exported shot .blend into the global frame sequence (resumable).
usage: python3 render_blend.py SHOT.blend OUTDIR FIRST_GLOBAL_FRAME"""
import sys, os, time, bpy
blend, out, g0 = sys.argv[1], sys.argv[2], int(sys.argv[3])
bpy.ops.wm.open_mainfile(filepath=blend); sc = bpy.context.scene
sc.render.use_persistent_data = True
for f in range(sc.frame_start, sc.frame_end + 1):
    path = f'{out}/f{g0 + f - 1:04d}.png'
    if os.path.exists(path): continue
    t = time.time(); sc.frame_set(f); sc.render.filepath = path; bpy.ops.render.render(write_still=True)
    print('FRAME', g0 + f - 1, round(time.time() - t, 1), flush=True)
