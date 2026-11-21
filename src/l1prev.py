import sys, numpy as np
sys.path.insert(0,'/home/claude/scripts')
import bpy, line1, cprev
out='/home/claude/factory/prev/'
P=line1.build(dict(line1.cycle(float(sys.argv[1])), s1=False))
sc,cam,_=cprev.setup(P,(1400,900)); sc.display.render_aa='FXAA'
cprev.aim(cam,((-0.50,0.50,0.38),(-0.76,0.0,0.08),32)); sc.render.filepath=f'{out}l1_{sys.argv[2]}.png'; bpy.ops.render.render(write_still=True)
