"""Render the whole film shot by shot from exported animated .blends (resumable).
Shots listed in WK go through WarpKernel (no depth of field), the rest through plain Cycles with their depth of field.
usage: python3 render_film2.py"""
import os, sys, subprocess, json, shutil, glob
sys.path.insert(0, '/home/claude/scripts')
import film
OUT = '/home/claude/factory/film/cy2'; WKD = '/home/claude/factory/film/wk'
os.makedirs(OUT, exist_ok=True); os.makedirs(WKD, exist_ok=True)
WK = set(json.loads(os.environ.get('WK_SHOTS', '[]')))
F = film.frames(); first = {}
for k, f in enumerate(F): first.setdefault(f['shot'], k)
n_of = {s: sum(1 for f in F if f['shot'] == s) for s in first}
for si in range(len(film.SHOTS)):
    g0, n = first[si], n_of[si]
    if all(os.path.exists(f'{OUT}/f{g0 + j:04d}.png') for j in range(n)): continue
    use_wk = si in WK
    blend = f'{WKD}/film_shot{si:02d}{"" if use_wk else "_dof"}.blend'
    if not os.path.exists(blend):
        subprocess.run(['python3', '/home/claude/scripts/film_wk.py', str(si), blend] + ([] if use_wk else ['dof']), check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print('SHOT', si, film.SHOTS[si]['name'], 'WK' if use_wk else 'cycles', flush=True)
    if use_wk:
        wo = f'{WKD}/run_shot{si:02d}'
        subprocess.run(['python3', 'warpkernel/render.py', '--blend', blend, '--out', wo], cwd='/home/claude/WarpKernel', check=True,
                       stdout=open(f'{wo}.log', 'w'), stderr=subprocess.STDOUT)
        fr = sorted(glob.glob(f'{wo}/frames/*.png'))
        for j, p in enumerate(fr[:n]): shutil.copy(p, f'{OUT}/f{g0 + j:04d}.png')
    else:
        subprocess.run(['python3', '/home/claude/scripts/render_blend.py', blend, OUT, str(g0)], check=True,
                       stdout=open(f'{WKD}/plain_shot{si:02d}.log', 'a'), stderr=subprocess.STDOUT)
print('ALL DONE', flush=True)
