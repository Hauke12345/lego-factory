import numpy as np, cv2, sys, json
exec(open('/home/claude/scripts/fitedges.py').read().split("def corners(prm):")[0])
prm = [float(x) for x in sys.argv[1:6]]; out = sys.argv[6]
def corners_e(prm):
    af, ab, bl, br, h = prm; o = []
    for z in (T0, h):
        for a, b in ((af, br), (ab, br), (ab, bl), (af, bl)): o.append(a*w + b*e2 + z*up)
    return np.array(o)
E2 = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
a0r, sr, dzr = json.load(open('/home/claude/sfm/fitted.json'))['rim']
rows = []
for fr in (300, 444, 524, 604):
    rv_t[fr] = (cv2.Rodrigues(R[fr])[0], -R[fr] @ C[fr])
    im = cv2.imread(f'/home/claude/plate/f{fr:04d}.jpg'); p = proj(fr, corners_e(prm))
    for i, j in E2: cv2.line(im, tuple(p[i].astype(int)), tuple(p[j].astype(int)), (0, 255, 0), 1, cv2.LINE_AA)
    bs = np.linspace(-14, 10, 100)
    q = proj(fr, np.array([(a0r+sr*b)*w + b*e2 + (T0+dzr)*up for b in bs]))
    cv2.polylines(im, [q.astype(np.int32)], False, (255, 0, 255), 1, cv2.LINE_AA)
    x0, x1 = int(np.clip(p[:, 0].min()-60, 0, 1080)), int(np.clip(p[:, 0].max()+60, 0, 1080)); y0 = int(np.clip(p[4:, 1].min()-70, 0, 1900)); y1 = y0 + int(0.45*(x1-x0))
    c = im[y0:y1, x0:x1]; rows.append(cv2.resize(c, (720, int(720*c.shape[0]/c.shape[1]))))
cv2.imwrite(out, np.vstack(rows))
