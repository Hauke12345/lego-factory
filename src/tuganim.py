import sys, os, pickle, numpy as np
sys.path.insert(0,'/home/claude/scripts')
import bpy, tug, tugscene, cprev, lblend
out=sys.argv[1]; k0=int(sys.argv[2]); k1=int(sys.argv[3]); step=int(sys.argv[4]) if len(sys.argv)>4 else 1
os.makedirs(out,exist_ok=True)
L,park=pickle.load(open('/home/claude/factory/prev/tug_log2.pkl','rb'))
P0=tugscene.static()+tugscene.dynamic(L[0])
sc,cam,objs=cprev.setup(P0,(1280,720)); sc.display.render_aa='FXAA'
# smoothed camera target
tx=np.array([s['x'] for s in L]); tz=np.array([s['z'] for s in L])
ker=np.ones(41)/41; pad=lambda a: np.convolve(np.pad(a,20,mode='edge'),ker,mode='valid')
tx,tz=pad(tx),pad(tz)
for k in range(k0,min(k1,len(L)),step):
    s=L[k]
    for p in tugscene.dynamic(s):
        ob=objs.get(p['name'])
        if ob is not None: ob.matrix_world=lblend.M_bl(p['M'])
    fx,fy=tx[k]*0.0004,-tz[k]*0.0004
    cprev.aim(cam,((fx+0.16,fy+0.50,0.36),(fx-0.01,fy,0.04),30))
    sc.render.filepath=f'{out}/f{k:04d}.png'; bpy.ops.render.render(write_still=True); print('FRAME',k,flush=True)
