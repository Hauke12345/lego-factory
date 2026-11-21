import sys, numpy as np
sys.path.insert(0,'/home/claude/scripts')
import bpy, line2, cprev
out='/home/claude/factory/prev/'
t=float(sys.argv[1]); P=line2.build(dict(line2.cycle(t), s1=False, s2=False))
sc,cam,_=cprev.setup(P,(1400,900)); sc.display.render_aa='FXAA'
views={'iso':((0.512+0.30,0.46,0.40),(0.512+0.03,0.0,0.09),32),'close':((0.512+0.16,0.06,0.30),(0.512+0.02,-0.03,0.11),40),'top':((0.512+0.04,0.001,0.6),(0.512+0.04,0.0,0.0),45)}
for v in sys.argv[3].split(','):
    cprev.aim(cam,views[v]); sc.render.filepath=f'{out}l2_{sys.argv[2]}_{v}.png'; bpy.ops.render.render(write_still=True)
