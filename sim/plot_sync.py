import sys, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
D=np.load(sys.argv[1]); pos=D['pos']; pp=D['path']; out=sys.argv[2]; title=sys.argv[3]; exp=D['exp']
n=len(pos); idx=[int(n*f)-1 for f in (0.25,0.5,0.75,1.0)]
cents=[(0,-71),(80,-11),(-80,-11)]
fig,axs=plt.subplots(1,4,figsize=(16,4.6))
for ax,k in zip(axs,idx):
    ax.plot(pp[:,0],pp[:,1],'g-',lw=4,alpha=.25)
    for c in cents: ax.add_patch(plt.Circle(c,24,color='.75'))
    p=pos[k]; m=(abs(p[:,0])<400)&(abs(p[:,1])<300)
    ax.plot(p[m,0],p[m,1],'k.-',ms=4,lw=.8)
    if m[0]: ax.plot(p[0,0],p[0,1],'ro',ms=6)
    ax.set_xlim(-150,150); ax.set_ylim(-140,60); ax.set_aspect('equal'); ax.invert_yaxis(); ax.set_title(f'{int(m.sum())} links in', fontsize=10)
axs[-1].plot(exp[:,0],exp[:,1],'bx',ms=4)
fig.suptitle(title); plt.tight_layout(); plt.savefig(out,dpi=80)
