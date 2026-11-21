"""MuJoCo rigid-body sim of the Lego car run-up, ramp jump and landing.
Writes render/traj.npz with per-frame chassis pose + wheel angles (Blender world coords)."""
import sys, math, numpy as np, mujoco

S, P, B = 0.008, 0.0032, 0.0096
SC, A0, T0 = 0.044, 9.0, -6.55
Z_BOX = SC*(-4.348-T0); Z_BOOK = SC*(-5.262-T0)
Y_BOXEDGE = SC*0.539
RECTS = {'box': [(4.79, 6.88), (4.92, 0.46), (12.17, 0.60), (12.04, 7.02)],
         'book': [(6.27, -2.78), (6.55, -8.02), (11.89, -7.74), (11.61, -2.5)]}
def bl(a, b): return (SC*(a-A0), SC*b)
RAMP_LEN, RAMP_H = 4*S, B+P
BUILD_Y = 0.20
WR, WB, TRACK = 0.0125, 4.5*S, 3*S+0.0058

V_MAX = float(sys.argv[1]) if len(sys.argv) > 1 else 1.3      # target top speed m/s
ACC_T = float(sys.argv[2]) if len(sys.argv) > 2 else 0.35     # time to reach top speed

def cuboid_mesh(name, rect, z):
    vs = [(*bl(a, b), zz) for zz in (0, z) for a, b in rect]
    return f'<mesh name="{name}" vertex="{" ".join(f"{x:.5f} {y:.5f} {z:.5f}" for x, y, z in vs)}"/>'

y0 = Y_BOXEDGE + 0.001; y1 = y0 + RAMP_LEN; lo = 0.25*RAMP_H
ramp = []
for x in (-4*S, 4*S):
    xa, xb = min(0, x), max(0, x)
    for xx in (xa, xb):
        ramp += [(xx, y0, Z_BOX), (xx, y1, Z_BOX), (xx, y1, Z_BOX+lo), (xx, y0, Z_BOX+RAMP_H)]
ramp_v = " ".join(f"{x:.5f} {y:.5f} {z:.5f}" for x, y, z in ramp)

zc = 0.0105; ztop = zc + P + B + P
wheels = ''.join(f'''
      <body name="w{i}" pos="{x:.4f} {y:.4f} {WR}">
        <joint name="j{i}" type="hinge" axis="0 1 0" damping="1e-6"/>
        <geom type="cylinder" size="{WR} 0.005" euler="90 0 0" mass="0.004" friction="0.9 0.005 0.0001"/>
      </body>''' for i, (x, y) in enumerate([(WB, TRACK), (WB, -TRACK), (-WB, TRACK), (-WB, -TRACK)]))

xml = f'''<mujoco>
  <option timestep="0.0004" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default><geom solref="0.003 1" solimp="0.95 0.99 0.0005" condim="3"/></default>
  <asset>{cuboid_mesh('boxm', RECTS['box'], Z_BOX)}{cuboid_mesh('bookm', RECTS['book'], Z_BOOK)}
    <mesh name="rampm" vertex="{ramp_v}"/></asset>
  <worldbody>
    <geom type="plane" size="2 2 0.1" friction="0.5"/>
    <geom type="mesh" mesh="boxm" friction="0.9"/>
    <geom type="mesh" mesh="bookm" friction="0.7"/>
    <geom type="mesh" mesh="rampm" friction="0.9"/>
    <body name="car" pos="0 {BUILD_Y} {Z_BOX+0.0003}" quat="0.70710678 0 0 -0.70710678">
      <freejoint/>
      <geom type="box" pos="0 0 {(zc+ztop)/2:.4f}" size="{7*S:.4f} {3*S:.4f} {(ztop-zc)/2:.4f}" mass="0.045"/>
      <geom type="box" pos="{-1.5*S:.4f} 0 {ztop+0.008:.4f}" size="0.005 0.008 0.008" mass="0.004"/>
      {wheels}
    </body>
  </worldbody>
  <actuator>{''.join(f'<velocity joint="j{i}" kv="0.0015" forcerange="-0.004 0.004"/>' for i in range(4))}</actuator>
</mujoco>'''

m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m)
FPS = 30; T_SIM = 66/FPS + 0.5      # settle 0.5 s before go
steps_per_frame = int(round(1/FPS/m.opt.timestep))
fine = []; frames = []; braking = False; land_t = None
nsteps = int(T_SIM/m.opt.timestep)
for k in range(nsteps):
    t = k*m.opt.timestep - 0.5
    pos = d.qpos[:3]
    if t < 0: v = 0.0
    else: v = V_MAX*min(1, t/ACC_T)
    # simple "driver": once fully over the book and wheels near book height, brake hard
    if not braking and pos[1] < SC*-2.637 - 0.03 and pos[2] < Z_BOOK + 0.004:
        braking = True; land_t = t
    d.ctrl[:] = 0.0 if braking else v/WR
    mujoco.mj_step(m, d)
    if t >= 0 and (k+1) % 5 == 0:
        fine.append(np.concatenate([[t], d.qpos[:7], d.qpos[7:11]]))
    if t >= 0 and (k+1) % steps_per_frame == 0:
        frames.append(np.concatenate([d.qpos[:7], d.qpos[7:11]]))
fr = np.array(frames)
np.savez('/home/claude/render/traj.npz', traj=fr, build_y=BUILD_Y, fine=np.array(fine))
# report
ys, zs = fr[:, 1], fr[:, 2]
air = [i for i in range(len(fr)) if zs[i] > Z_BOX + 0.02 or (ys[i] < Y_BOXEDGE and zs[i] > Z_BOOK + 0.003)]
print(f'V={V_MAX} frames={len(fr)} final y={ys[-1]:.3f} z={zs[-1]:.3f} (book z {Z_BOOK:.3f}, far edge y {SC*-8.0:.3f})')
print('air frames', air[:1], air[-1:] if air else None, 'brake t', land_t)
up = [2*(q[1]*q[3]+q[0]*q[2]) for q in fr[:, 3:7]]  # not used
for i in range(0, len(fr), 3): print(i, np.round(fr[i, :3], 3))
