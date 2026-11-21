"""Sound design for the Lego bridge video: 127 parts snapping together, plan/stress UI blips,
truck motor following the MuJoCo speed, wheel clacks at every joint/lip/hinge, settle + brake.
Usage: python3 sfx_bridge.py room.wav out.wav [room_gain]"""
import sys, json, math, numpy as np, wave
sys.path.insert(0, '/home/claude/scripts')
from scipy.signal import butter, sosfilt
import bridge as BR

SR = 48000; FPS = 30; NFR = 642
rng = np.random.default_rng(5)
N = int(NFR/FPS*SR) + SR
L = np.zeros(N); R = np.zeros(N)
def bp(x, lo, hi, o=2): return sosfilt(butter(o, [lo, hi], 'band', fs=SR, output='sos'), x)
def lp(x, hi, o=2): return sosfilt(butter(o, hi, 'low', fs=SR, output='sos'), x)
def hp(x, lo, o=2): return sosfilt(butter(o, lo, 'high', fs=SR, output='sos'), x)
def add(sig, t, gain=1.0, pan=0.0):
    i = int(t*SR); n = min(len(sig), N - i)
    if n <= 0 or i < 0: return
    L[i:i+n] += sig[:n]*gain*np.sqrt(0.5*(1-pan)); R[i:i+n] += sig[:n]*gain*np.sqrt(0.5*(1+pan))
def env(n, a, d):
    t = np.arange(n)/SR; return np.minimum(1, t/max(a, 1e-4))*np.exp(-t/d)
def snap(pitch=1.0, body=1.0):
    n = int(0.07*SR); t = np.arange(n)/SR; out = np.zeros(n)
    for k, dt in enumerate((0.0, rng.uniform(0.005, 0.010))):
        i = int(dt*SR); m = n - i
        nz = bp(rng.standard_normal(m), 1500*pitch, 9000, 2)*env(m, 0.0002, 0.0025)
        ring = sum(np.sin(2*np.pi*f*pitch*t[:m])*env(m, 0.0003, d) for f, d in ((2900, 0.008), (4700, 0.005), (1350*body, 0.014)))
        out[i:] += (nz*0.9 + ring*0.35)*(1.0 if k else 0.55)
    return out
def pin_click(pitch=1.0):          # Technic pin pushed in: soft pop + higher tick
    n = int(0.05*SR); t = np.arange(n)/SR
    pop = lp(rng.standard_normal(n), 2500)*env(n, 0.0005, 0.004)
    tick = np.sin(2*np.pi*3800*pitch*t)*env(n, 0.0002, 0.003)
    return pop*0.8 + tick*0.6
def ratchet(k=3):                  # click hinge ratchet
    out = np.zeros(int(0.12*SR))
    for j in range(k):
        s = snap(1.4 + 0.1*j)*0.5; i = int(j*0.025*SR); out[i:i+len(s)] += s[:len(out)-i]
    return out
def whoosh(dur):
    n = max(int(dur*SR), 10); x = rng.standard_normal(n); u = np.linspace(0, 1, n)
    return bp(x, 500, 3500)*np.sin(np.pi*u)**2
def blip(f0, dur=0.07):
    n = int(dur*SR); t = np.arange(n)/SR
    return (np.sin(2*np.pi*f0*t) + 0.25*np.sin(2*np.pi*2*f0*t))*env(n, 0.003, dur/3.5)
def thud(f0=150, d=0.06):
    n = int(0.25*SR); t = np.arange(n)/SR
    return np.sin(2*np.pi*f0*t*(1 - 0.3*t))*env(n, 0.001, d) + lp(rng.standard_normal(n), 900)*env(n, 0.0005, 0.02)*1.5

# ---- assembly: same order/timing as bscene.py ----
D = json.load(open('/home/claude/render_b/build_check.json'))
order = ['pier', 'app', 'land', 'deck', 'road', 'curb', 'post', 'diag', 'chord', 'appTop', 'landTop']
bp_ = [(n, p['group']) for n, p in D['parts'].items() if p['group'] != 'truck']
bp_.sort(key=lambda x: order.index(x[1]))
T_ASM0, DT, DUR = 15, 1.6, 22
for j, (n, gname) in enumerate(bp_):
    t0 = (T_ASM0 + j*DT)/FPS; t1 = (T_ASM0 + j*DT + DUR)/FPS
    pan = float(np.clip(-np.array(D['parts'][n]['M'])[1][3]*4, -0.7, 0.7))        # box left, book right
    add(whoosh(0.8*DUR/FPS), t0, 0.012, pan)
    if gname in ('post', 'diag'):
        add(pin_click(rng.uniform(0.9, 1.15)), t1, 0.45, pan); add(pin_click(rng.uniform(0.9, 1.15)), t1 + 0.03, 0.35, pan)
    elif n.startswith(('appH', 'landH', 'dH')): add(ratchet(2), t1, 0.4, pan)
    elif gname == 'pier' or n.startswith(('curb', 'chord')): add(snap(rng.uniform(0.7, 0.85), 0.8), t1, 0.5, pan)
    else: add(snap(rng.uniform(0.95, 1.25)), t1, 0.32, pan)
# ---- plan HUD + stress view ----
for f, fr in ((196, 1500), (206, 1320), (212, 1660), (218, 1250), (222, 1180)): add(blip(fr), f/FPS, 0.10)
for k in range(8): add(blip(2100 + 90*k, 0.04), (226 + 36*k/8)/FPS, 0.05)
add(blip(880, 0.12), 360/FPS, 0.10); add(blip(1320, 0.12), 364/FPS, 0.08)
# ---- truck: headlights blink, motor, wheel clacks ----
for f in (352, 358): add(blip(2600, 0.03), f/FPS, 0.04)
T_GO = 393
fine = np.load('/home/claude/render_b/traj.npz')['fine']
ts, ys = fine[:, 0], fine[:, 2]
vs = np.abs(np.gradient(ys, ts)); vs = np.convolve(vs, np.ones(9)/9, 'same')
tt = np.arange(int((T_GO/FPS - 1.3)*SR), int((T_GO/FPS + 1.6)*SR))/SR
st = tt - T_GO/FPS
v = np.interp(st, ts, vs, left=0, right=0)
rev = 0.10 + 0.25*np.exp(-((st + 0.9)/0.12)**2) + 0.35*np.exp(-((st + 0.45)/0.12)**2)
load = np.where(st < 0, rev, v/0.5)
f0 = lp(200 + 900*np.clip(load, 0, 1.3), 30, 1)
ph = 2*np.cumsum(f0)/SR*np.pi
mot = bp(sum(np.sin(k*ph)/k for k in range(1, 9)) + 0.4*np.sin(3.5*ph), 150, 7000)
amp = np.where(st < 0, 0.3*(st > -1.25)*np.minimum(1, (st + 1.25)/0.1), 0.35 + 0.65*np.clip(load, 0, 1))
tstop = ts[np.argmax((ts > 0.3) & (vs < 0.02))]
amp *= np.where(st > tstop - 0.05, np.exp(-(st - tstop + 0.05)/0.08), 1)
add(mot*amp, tt[0], 0.09, -0.1)
# rolling rumble: tiles smooth, studded hinge rows rattle
yc = np.interp(st, ts, ys, left=ys[0], right=ys[-1])
rum = lp(rng.standard_normal(len(tt)), 500)*np.clip(v/0.5, 0, 1)*(st > 0)
add(rum, tt[0], 0.14)
feat = {'lip': [BR.Y_E + 10*BR.S, BR.Y_LE - 10*BR.S], 'edge': [BR.Y_E + 8*BR.S, BR.Y_LE - 8*BR.S],
        'hinge': [BR.YA, BR.YL], 'pier': [BR.Q_PIER[1]], 'studs': [BR.Y_E + 2*BR.S, BR.Y_LE - 2*BR.S]}
for name, yl in feat.items():
    for yf in yl:
        for ax in (5*BR.S, -4*BR.S):                      # front, rear axle (truck faces -y)
            wy = ys - ax
            idx = np.where((wy[:-1] > yf) & (wy[1:] <= yf))[0]
            for i in idx:
                t_ = T_GO/FPS + ts[i]
                if name == 'hinge': add(thud(240, 0.025), t_, 0.35); add(snap(0.75), t_ + 0.004, 0.3)
                elif name == 'pier': add(thud(110, 0.08), t_, 0.18)                                 # deck flexes over the pier
                elif name == 'studs':
                    for k in range(5): add(snap(rng.uniform(1.2, 1.5))*0.6, t_ + k*0.012, 0.18)
                else: add(snap(0.65, 0.7), t_, 0.35); add(thud(190, 0.03), t_, 0.25)
# drop off the landing lip onto the book + brake skid + settle
lt = T_GO/FPS + tstop
n = int(0.25*SR); add(bp(rng.standard_normal(n), 1500, 6000)*env(n, 0.01, 0.06), lt - 0.12, 0.10)
add(thud(150, 0.05), lt, 0.25)
# ---- mix with room tone ----
w = wave.open(sys.argv[1]); ch = w.getnchannels(); assert w.getframerate() == SR
raw = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float32).reshape(-1, ch)/32768
n = min(len(raw), N); room = raw[:n] if ch == 2 else np.repeat(raw[:n], 2, 1)
ev = np.abs(room).max(1); k = int(0.02*SR); ev = np.convolve(ev, np.ones(k)/k, 'same')
ev = np.maximum.reduce([np.roll(ev, s_) for s_ in range(-k, k+1, k//4)])
room = room*np.minimum(1, 0.0012/np.maximum(ev, 1e-9))[:, None]
mix = np.stack([L, R], 1)[:n] + room*float(sys.argv[3] if len(sys.argv) > 3 else 6.0)
mix = np.tanh(mix*1.6)/1.6; mix *= 0.89/np.abs(mix).max()
out = wave.open(sys.argv[2], 'w'); out.setnchannels(2); out.setsampwidth(2); out.setframerate(SR)
out.writeframes((np.clip(mix, -1, 1)*32767).astype(np.int16).tobytes()); out.close()
print('ok, stop at', round(T_GO/FPS + tstop, 2), 's')
