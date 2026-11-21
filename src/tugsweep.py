import sys, pickle, numpy as np
sys.path.insert(0,'/home/claude/scripts')
import tug, tugsim, tugscene, lcheck
r=tugsim.run(); L=r.log; pickle.dump((L, r.park), open('/home/claude/factory/prev/tug_log_check.pkl','wb'))
print('park', r.park, 'duration', L[-1]['t'], flush=True)
issues={}; n=0
for k in range(0, len(L), 8):
    s=L[k]; rep=lcheck.run(tugscene.static()+tugscene.dynamic(s), verbose=False); n+=1
    for a,b,h,_ in rep['clashes']: issues.setdefault(('CLASH',a,b),[]).append(round(s['t'],1))
    for a,b,kk in rep['bad_fits']: issues.setdefault(('BADFIT',a,b,str(kk)),[]).append(round(s['t'],1))
    for u in rep['unconnected']: issues.setdefault(('UNCONNECTED',u),[]).append(round(s['t'],1))
print('states checked', n, 'distinct issues', len(issues))
for k,v in issues.items(): print(' ', k, 'at t =', v[:12], '...' if len(v)>12 else '')
