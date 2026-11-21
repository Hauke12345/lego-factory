"""Synthesized sound design for the Lego car video, mixed over the boosted room tone.
Usage: python3 sfx.py room.wav out.wav"""
import sys, numpy as np, wave
from scipy.signal import butter, sosfilt

SR = 48000; FPS = 30; NFR = 642
rng = np.random.default_rng(3)
N = int(NFR/FPS*SR) + SR
L = np.zeros(N); R = np.zeros(N)

def bp(x, lo, hi, o=2): return sosfilt(butter(o, [lo, hi], 'band', fs=SR, output='sos'), x)
def lp(x, hi, o=2): return sosfilt(butter(o, hi, 'low', fs=SR, output='sos'), x)
def hp(x, lo, o=2): return sosfilt(butter(o, lo, 'high', fs=SR, output='sos'), x)
def add(sig, t, gain=1.0, pan=0.0):
    i = int(t*SR); n = min(len(sig), N - i)
    if n <= 0: return
    L[i:i+n] += sig[:n]*gain*np.sqrt(0.5*(1-pan)); R[i:i+n] += sig[:n]*gain*np.sqrt(0.5*(1+pan))
def env(n, a, d):
    t = np.arange(n)/SR; return np.minimum(1, t/max(a, 1e-4))*np.exp(-t/d)

def snap(pitch=1.0):
    """plastic stud click-in: two quick transients + a short resonant ring"""
    n = int(0.06*SR); t = np.arange(n)/SR; out = np.zeros(n)
    for k, dt in enumerate((0.0, rng.uniform(0.006, 0.011))):
        i = int(dt*SR); m = n - i
        nz = bp(rng.standard_normal(m), 1800*pitch, 9000, 2)*env(m, 0.0002, 0.0025)
        ring = sum(np.sin(2*np.pi*f*pitch*t[:m])*env(m, 0.0003, d) for f, d in ((2900, 0.008), (4700, 0.005), (1350, 0.012)))
        out[i:] += (nz*0.9 + ring*0.35)*(1.0 if k else 0.55)
    return out

def whoosh(dur):
    n = int(dur*SR); x = rng.standard_normal(n)
    u = np.linspace(0, 1, n); e = np.sin(np.pi*u)**2
    return bp(x, 500, 3500)*e

def blip(f0, dur=0.07):
    n = int(dur*SR); t = np.arange(n)/SR
    return (np.sin(2*np.pi*f0*t) + 0.25*np.sin(2*np.pi*2*f0*t))*env(n, 0.003, dur/3.5)

def thud(f0=150, d=0.06):
    n = int(0.25*SR); t = np.arange(n)/SR
    return np.sin(2*np.pi*f0*t*(1 - 0.3*t))*env(n, 0.001, d) + lp(rng.standard_normal(n), 900)*env(n, 0.0005, 0.02)*1.5

# ---- assembly: 32 parts fly in and click ----
T_ASM0, DT, DUR = 15, 5, 24
for i in range(32):
    t0 = (T_ASM0 + i*DT)/FPS; pan = rng.uniform(-0.5, 0.5)
    add(whoosh(0.8*DUR/FPS), t0, 0.035, pan)
    p = rng.uniform(0.85, 1.2) * (0.8 if i >= 30 else 1.0)       # ramp wedges a bit deeper
    add(snap(p), t0 + DUR/FPS, 0.9, pan*0.6)

# ---- planning HUD blips ----
for f, fr in ((196, 1500), (206, 1320), (212, 1660), (220, 1180)):
    add(blip(fr), f/FPS, 0.12)
for k in range(8):
    add(blip(2100 + 90*k, 0.04), (226 + 36*k/8)/FPS, 0.06)

# ---- motor: speed from MuJoCo trajectory ----
T_GO = 406
fine = np.load('/home/claude/render/traj.npz')['fine']
ts, ys = fine[:, 0], fine[:, 2]
vs = np.abs(np.gradient(ys, ts)); vs = np.convolve(vs, np.ones(9)/9, 'same')
z = fine[:, 3]
t_end = 1.0
tt = np.arange(int((T_GO/FPS - 1.3)*SR), int((T_GO/FPS + t_end)*SR))/SR
st = tt - T_GO/FPS
v = np.interp(st, ts, vs, left=0, right=0)
# pre-go: idle rev (two blips of throttle)
rev = 0.10 + 0.25*np.exp(-((st + 0.9)/0.12)**2) + 0.35*np.exp(-((st + 0.45)/0.12)**2)
air = np.interp(st, ts, (z > 0.1 + 0.0) .astype(float), left=0, right=0)      # z>box top + small = airborne
load = np.where(st < 0, rev, v/1.45)
f0 = 220 + 1100*load
f0[(st > 0) & (air > 0.5)] *= 1.18          # unloaded wheels spin up in the air
f0 = lp(f0, 30, 1)
ph = 2*np.cumsum(f0)/SR*np.pi
saw = sum(np.sin(k*ph)/k for k in range(1, 9))
gear = np.sin(3.5*ph)*0.4
amp = np.clip(np.where(st < 0, 0.3*(st > -1.25)*np.minimum(1, (st + 1.25)/0.1), 0.35 + 0.65*load), 0, 1)
amp *= np.where(st > 0.47, np.exp(-(st - 0.47)/0.06), 1)
mot = bp(saw + gear, 150, 7000)*amp
mot += hp(rng.standard_normal(len(tt)), 3000)*0.08*amp*(st > 0)
add(mot, tt[0], 0.10, -0.1)

# ---- wheel rumble on cardboard (while on box, not in air) ----
on = (st > 0) & (st < 0.19)
rum = lp(rng.standard_normal(len(tt)), 400)*on*np.clip(v/1.45, 0, 1)
add(rum, tt[0], 0.25)

# ---- ramp clack, takeoff, landing, bounce, skid ----
s_lip = 0.175; s_land = 0.40
add(snap(0.7), (T_GO/FPS) + 0.15, 0.4)                       # front wheels hit the ramp
add(thud(220, 0.03), (T_GO/FPS) + s_lip, 0.25)
lt = T_GO/FPS + s_land
add(thud(140, 0.07), lt, 0.6, 0.2)
for k in range(6):
    add(snap(rng.uniform(0.6, 1.0)), lt + 0.008 + k*rng.uniform(0.008, 0.02), 0.35*(0.75**k), 0.2)
add(thud(170, 0.04), lt + 0.09, 0.35, 0.2)                   # small bounce
n = int(0.22*SR); sk = bp(rng.standard_normal(n), 1500, 6000)*env(n, 0.01, 0.07)
add(sk, lt + 0.05, 0.15, 0.2)

# ---- mix with room tone ----
w = wave.open(sys.argv[1]); ch = w.getnchannels(); rate = w.getframerate()
raw = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float32)/32768
raw = raw.reshape(-1, ch); assert rate == SR
room = raw[:, 0] if ch == 1 else raw
n = min(len(room), N)
mix = np.stack([L, R], 1)[:n]
room = (room[:n, None] if room.ndim == 1 else room[:n]).copy()
# tame handling bumps in the phone audio: gain-ride its envelope down to a ceiling
ev = np.abs(room).max(1); k = int(0.02*SR)
ev = np.convolve(np.maximum.accumulate(ev.reshape(-1)) * 0 + ev, np.ones(k)/k, 'same')
ev = np.maximum.reduce([np.roll(ev, s_) for s_ in range(-k, k+1, k//4)])
room *= np.minimum(1, 0.0012/np.maximum(ev, 1e-9))[:, None]
mix += room*float(sys.argv[3] if len(sys.argv) > 3 else 6.0)
# soft limiter
mix = np.tanh(mix*1.6)/1.6
peak = np.abs(mix).max(); mix *= 0.89/peak
print('peak', peak)
out = wave.open(sys.argv[2], 'w'); out.setnchannels(2); out.setsampwidth(2); out.setframerate(SR)
out.writeframes((np.clip(mix, -1, 1)*32767).astype(np.int16).tobytes()); out.close()
