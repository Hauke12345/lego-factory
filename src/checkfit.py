import numpy as np, json, cv2, sys
sys.argv=['x']; exec(open('/home/claude/scripts/fitedges.py').read().split("RECTS = ")[0])
RECTS = {'box': [(4.79, 6.88), (4.92, 0.46), (12.17, 0.60), (12.04, 7.02)],
         'book': [(6.27, -2.78), (6.55, -8.02), (11.89, -7.74), (11.61, -2.5)]}
def rect_to_prm(rect, h):
    c = np.mean(rect, 0); ax = np.array(rect[2]) - np.array(rect[1]); ay = np.array(rect[0]) - np.array(rect[1])
    return [c[0], c[1], np.linalg.norm(ax), np.linalg.norm(ay), np.arctan2(ax[1], ax[0]), h]
fit = json.load(open('/home/claude/sfm/fitted.json'))
E2 = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
outs = []
for fr in (44, 124, 204, 284, 364, 444, 524, 604):
    rv_t[fr] = (cv2.Rodrigues(R[fr])[0], -R[fr] @ C[fr])
    im = cv2.imread(f'/home/claude/plate/f{fr:04d}.jpg')
    for name, h in (('box', -4.348), ('book', -5.262)):
        for prm, col in ((rect_to_prm(RECTS[name], h), (255, 80, 0)), (fit[name], (0, 255, 0))):
            p = proj(fr, corners(prm))
            for i, j in E2: cv2.line(im, tuple(p[i].astype(int)), tuple(p[j].astype(int)), col, 2)
    a0, s, dz = fit['rim']
    bs = np.linspace(-14, 10, 100)
    p = proj(fr, np.array([(a0+s*b)*w + b*e2 + (T0+dz)*up for b in bs]))
    cv2.polylines(im, [p.astype(np.int32)], False, (255, 255, 255), 2)
    outs.append(cv2.resize(im, (270, 480)))
cv2.imwrite('/home/claude/test/fitcheck.jpg', np.vstack([np.hstack(outs[:4]), np.hstack(outs[4:])]))
