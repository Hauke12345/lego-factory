"""Plan-view simulation of the tug on the yard. The control program below mirrors tug_pybricks.py line by line;
here it runs against simulated sensor readings (tile colours under the sensor spot) and simple track kinematics.
Units: LDU, seconds. Heading psi: forward = (-sin psi, -cos psi) in (x, z); psi grows when turning left.
"""
import numpy as np
import tug

DT = 0.01
EYE_OFF = 1.0          # trailer origin ahead of the hook point so the peg just touches the eye's front bar
SLIP = 0.75            # skid steering: only this share of the track speed difference turns the robot (assumed)
TAU = 0.08             # motor speed lag
DEG = np.pi*30/180     # LDU of track travel per degree of the drive sprocket (pitch radius 30)

class Sim:
    def __init__(self):
        self.x, self.z, self.psi = -2380.0, 0.0, np.pi/2
        self.vl = self.vr = 0.0; self.cl = self.cr = 0.0          # actual / commanded track speeds (LDU/s)
        self.dl = self.dr = 0.0                                  # track travel
        self.hook = tug.HOOK_UP; self.hook_cmd = tug.HOOK_UP
        self.t = 0.0; self.state = 'start'; self.log = []
        self.trl = None; self.hitched = False                    # trailer: (contact x, z, heading)
    # ---- what the program can see and do
    def fwd(self): return np.array([-np.sin(self.psi), -np.cos(self.psi)])
    def sensor_xy(self): return np.array([self.x, self.z]) + tug.SENS*self.fwd()
    def brightness(self):
        p = self.sensor_xy(); pts = [p] + [p + 9*np.array([np.cos(a), np.sin(a)]) for a in np.linspace(0, 2*np.pi, 8, endpoint=False)]
        return float(np.mean([tug.REFL.get(tug.cell_at(*q), 30.0) for q in pts]))
    def colour(self):
        return {2: 'RED', 3: 'BLUE'}.get(tug.cell_at(*self.sensor_xy()), None)
    def run(self, left_deg_s, right_deg_s): self.cl, self.cr = left_deg_s*DEG, right_deg_s*DEG
    def angles(self): return self.dl/DEG, self.dr/DEG
    # ---- physics step
    def step(self):
        a = DT/TAU; self.vl += a*(self.cl - self.vl); self.vr += a*(self.cr - self.vr)
        v = (self.vl + self.vr)/2; w = SLIP*(self.vr - self.vl)/tug.BTRK
        self.x, self.z = np.array([self.x, self.z]) + v*DT*self.fwd(); self.psi += w*DT
        self.dl += self.vl*DT; self.dr += self.vr*DT
        self.hook += np.clip(self.hook_cmd - self.hook, -90*DT, 90*DT)
        if self.hitched and self.trl:                              # trailer dragged by the peg once the eye's front edge is taut
            H = np.array([self.x, self.z]) - tug.DH*self.fwd()
            xo, zo, phi = self.trl; f = np.array([-np.sin(phi), -np.cos(phi)]); C = np.array([xo, zo]) - tug.LT*f
            d = H - C; n = np.linalg.norm(d)
            if n > tug.LT - EYE_OFF:                                # the front bar is EYE_OFF ahead of the hook point when the peg touches it
                u = d/n; C = H - (tug.LT - EYE_OFF)*u; O = C + tug.LT*u
                self.trl = (O[0], O[1], float(np.arctan2(-u[0], -u[1])))
        self.t += DT
        if len(self.log) == 0 or self.t - self.log[-1]['t'] >= 0.05 - 1e-9:
            self.log.append(dict(t=self.t, x=self.x, z=self.z, psi=self.psi, hook=self.hook, dist=(self.dl + self.dr)/2, dl=self.dl, dr=self.dr, cl=self.cl, cr=self.cr, trl=self.trl,
                                 state=self.state, v=self.brightness(), col=self.colour(), hitched=self.hitched))

# ---------------------------------------------------------------- the program (generator: each yield = wait(10))
BLACK, WHITE = 6, 85
THRESH = (BLACK + WHITE)/2
KP = 9.0                 # deg/s of track speed per % of brightness error
SPEED, TOW_SPEED = 200, 170
TURN_DEG = 180/0.75*(tug.BTRK/2)/30      # sprocket degrees per track for a half turn on the spot (includes measured slip)
BACK_DEG = 158/DEG

def follow(r, speed, side=1):
    err = side*(r.brightness() - THRESH)               # side=1: floor on the left of the edge (too bright -> steer right); -1: mirrored
    r.run(speed + KP*err, speed - KP*err)

def program(r):
    r.state = 'leave the line'; a0 = r.angles()[0]
    while r.angles()[0] - a0 < 560/DEG: r.run(190, 190); yield
    r.state = 'follow line'
    n = 0
    while n < 3:
        follow(r, SPEED); n = n + 1 if r.colour() == 'RED' else 0; yield
    r.run(0, 0); r.state = 'red marker: stop'
    for _ in range(40): yield
    r.state = 'turn round'; a0 = r.angles()[1]
    while r.angles()[1] - a0 < TURN_DEG: r.run(-200, 200); yield
    r.run(0, 0)
    for _ in range(30): yield
    r.state = 'back up to the trailer'; a0 = r.angles()[0]
    while a0 - r.angles()[0] < BACK_DEG: r.run(-120, -120); yield
    r.run(0, 0); r.state = 'hook down'; r.hook_cmd = tug.HOOK_DOWN
    for _ in range(90): yield
    r.hitched = True; r.state = 'tow: follow line'
    n = 0
    while n < 3:
        follow(r, TOW_SPEED, -1); n = n + 1 if r.colour() == 'BLUE' else 0; yield      # same physical edge on the way back
    r.run(0, 0); r.state = 'blue marker: stop'
    for _ in range(40): yield
    r.state = 'hook up'; r.hook_cmd = tug.HOOK_UP
    for _ in range(90): yield
    r.hitched = False; r.state = 'drive clear'; a0 = r.angles()[0]
    while r.angles()[0] - a0 < 90/DEG: r.run(150, 150); yield
    r.run(0, 0); r.state = 'done'
    for _ in range(60): yield

def simulate(trailer=None):
    r = Sim(); r.trl = trailer
    for _ in program(r):
        r.step()
        if r.t > 150: raise RuntimeError('program did not finish: lost at state ' + r.state)
    return r

def run():
    """first pass finds where the hook ends up after backing; the trailer is parked so that its eye centre is there."""
    r = simulate()
    k = next(i for i, s in enumerate(r.log) if s['state'] == 'hook down')
    s = r.log[k]; f = np.array([-np.sin(s['psi']), -np.cos(s['psi'])]); H = np.array([s['x'], s['z']]) - tug.DH*f
    phi = float(np.round(s['psi']/(np.pi/2))*np.pi/2)             # trailer parked square to the grid
    ft = np.array([-np.sin(phi), -np.cos(phi)])
    eye = np.round(H/10)*10                                       # parked on the stud grid
    contact = eye + 42*ft
    r2 = simulate((float(contact[0]), float(contact[1]), phi))
    r2.park = dict(eye=eye, hook=H, heading_err_deg=float(np.degrees(s['psi'] - phi)))
    return r2

if __name__ == '__main__':
    r = run(); L = r.log
    print('duration %.1f s, %d samples' % (L[-1]['t'], len(L)))
    print('parking: eye centre', r.park['eye'], 'hook arrives at', r.park['hook'].round(1), 'heading error %.1f deg' % r.park['heading_err_deg'])
    st = None
    for s in L:
        if s['state'] != st: st = s['state']; print('%6.1f s  %-24s x=%7.0f z=%6.0f psi=%6.1f' % (s['t'], st, s['x'], s['z'], np.degrees(s['psi'])))
    pp = tug.path_points()
    for name in ('follow line', 'tow: follow line'):
        d = [np.linalg.norm(pp - np.array([s['x'], s['z']]) - tug.SENS*np.array([-np.sin(s['psi']), -np.cos(s['psi'])]), axis=1).min() for s in L if s['state'] == name]
        print(name, 'sensor distance from line centre: mean %.1f max %.1f LDU' % (np.mean(d), np.max(d)))
    print('trailer end pose', np.round(L[-1]['trl'], 2))
