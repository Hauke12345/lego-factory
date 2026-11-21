"""Flat ribbon cable (Powered Up / Boost), routed only the ways a flat cable can physically go:
  S(L, twist)  straight run, optionally twisting about its own axis (<= 90 deg per 16 LDU)
  B(axis, ang, r)  bend about the cable's WIDE axis only (axis must be +-W), radius >= R_MIN
An edgewise (in-plane) bend raises an error. Cross-section 17.6 x 3 LDU (measured from LDraw u9218c01: chord ribbon +-8.8, +-1.5).
Real length: 25 cm = 625 LDU (LDraw 'Coiled Cable 25cm'); surplus is stored in flat S-folds.
"""
import numpy as np
import ldr

HW, HT, R_MIN, L_REAL, FOLD_R = 8.8, 1.5, 3.4, 625.0, 3.5
_SYN = {}
_orig = ldr.part
def _part(name): return _SYN[name] if name in _SYN else _orig(name)
if getattr(ldr.part, '__name__', '') != '_part_cable':
    _part.__name__ = '_part_cable'; ldr.part = _part

def rot(a, th):
    a = np.asarray(a, float); a = a/np.linalg.norm(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th)*K + (1 - np.cos(th))*K @ K

class Route:
    def __init__(self, p, T, W):
        self.p = np.array(p, float); self.T = np.array(T, float)/np.linalg.norm(T)
        W = np.array(W, float); W = W - np.dot(W, self.T)*self.T; self.W = W/np.linalg.norm(W)
        self.P = [self.p.copy()]; self.Ws = [self.W.copy()]; self.L = 0.0
    def _rec(self): self.P.append(self.p.copy()); self.Ws.append(self.W.copy())
    def S(self, L, twist=0.0, step=2.0):
        if L <= 1e-9: return self
        if abs(twist) > 1e-9 and abs(twist)/L > 90/16 + 1e-6: raise ValueError(f'twist {twist} deg over {L:.1f} LDU is too tight')
        n = max(1, int(np.ceil(L/step))); ds = L/n; dth = np.radians(twist)/n
        for _ in range(n):
            self.p = self.p + self.T*ds; self.W = rot(self.T, dth) @ self.W; self.L += ds; self._rec()
        return self
    def B(self, axis, ang, r, step=1.5):
        axis = np.array(axis, float); axis /= np.linalg.norm(axis)
        if abs(abs(np.dot(axis, self.W)) - 1) > 1e-3: raise ValueError('edgewise bend: a flat cable only bends about its wide axis')
        if r < R_MIN - 1e-9: raise ValueError(f'bend radius {r} below minimum {R_MIN}')
        th = np.radians(ang); n = max(2, int(np.ceil(abs(th)*r/step))); dth = th/n; ds = r*abs(dth)
        for _ in range(n):
            Rh = rot(axis, dth/2); self.T = Rh @ self.T; self.W = Rh @ self.W
            self.p = self.p + self.T*ds; self.L += ds
            self.T = Rh @ self.T; self.W = Rh @ self.W; self._rec()
        return self
    def to(self, target, twist=0.0):
        """straight run to a point that must lie ahead on the current heading."""
        d = np.asarray(target, float) - self.p; L = float(np.dot(d, self.T))
        if np.linalg.norm(d - L*self.T) > 0.5 or L < -0.5: raise ValueError(f'target not straight ahead: off by {np.linalg.norm(d - L*self.T):.2f}, L={L:.1f}')
        return self.S(L, twist)
    def fold_pair(self, side, length):
        """two flat 180-degree folds (an S-fold): back along the run and forward again, layers stacked towards 'side'."""
        side = np.asarray(side, float); ax = np.cross(self.T, side); ax /= np.linalg.norm(ax)      # bend axis turning T towards 'side'
        self.B(ax, 180, FOLD_R).S(length)
        ax = np.cross(self.T, side); ax /= np.linalg.norm(ax)
        self.B(ax, 180, FOLD_R).S(length)
        return self
    def mesh(self):
        C = np.array(self.P); Wd = np.array(self.Ws)
        t = np.gradient(C, axis=0); t /= np.linalg.norm(t, axis=1, keepdims=True)
        N = np.cross(t, Wd); N /= np.linalg.norm(N, axis=1, keepdims=True)
        ring = [C + a*HW*Wd + b*HT*N for a, b in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
        tris = []
        for i in range(len(C) - 1):
            for k in range(4):
                a, b = ring[k], ring[(k + 1) % 4]
                tris += [[a[i], b[i], b[i + 1]], [a[i], b[i + 1], a[i + 1]]]
        for e, idx in ((0, 0), (1, -1)):                             # end caps
            q = [r[idx] for r in ring]
            tris += ([[q[0], q[2], q[1]], [q[0], q[3], q[2]]] if e == 0 else [[q[0], q[1], q[2]], [q[0], q[2], q[3]]])
        return np.array(tris)

def register(name, route, col=0):
    tris = route.mesh()
    _SYN[name] = dict(tris=tris, cols=np.full(len(tris), col), snaps=[], title='~cable')
    return name

def solve_fold(build, lo=0.0, hi=400.0, target=L_REAL):
    """build(fold_length) -> Route; finds the fold length that gives the real cable length."""
    for _ in range(40):
        mid = 0.5*(lo + hi)
        if build(mid).L < target: lo = mid
        else: hi = mid
    r = build(0.5*(lo + hi)); return r, 0.5*(lo + hi)
