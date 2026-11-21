"""MuJoCo rigid-body sim of the Lego truck driving over the Lego bridge (render_b/traj.npz).
args: V_MAX ACC_T T_SIM STOP_Y"""
import sys, math, numpy as np, mujoco
sys.path.insert(0, '/home/claude/scripts'); import bridge as BR
S, P, B = BR.S, BR.P, BR.B
SC, A0, T0 = 0.044, 9.0, -6.55
Z_BOX, Z_BOOK = BR.Z_BOX, BR.Z_BOOK
RECTS = {'box': [(4.79, 6.88), (4.92, 0.46), (12.17, 0.60), (12.04, 7.02)],
         'book': [(6.27, -2.78), (6.55, -8.02), (11.89, -7.74), (11.61, -2.5)]}
def bl(a, b): return (SC*(a-A0), SC*b)
BUILD_Y = 0.20
# truck dims (truck.py)
WR, WW = 0.0145, 0.009; Z_TB = WR - 0.0058; Z_CH = Z_TB + B
AX_F, AX_R = 5*S, -4*S; WY = 3*S + 0.0015 + WW/2
V_MAX = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
ACC_T = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
T_SIM = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
STOP_Y = float(sys.argv[4]) if len(sys.argv) > 4 else -0.235

def cuboid_mesh(name, rect, z):
    vs = [(*bl(a, b), zz) for zz in (0, z) for a, b in rect]
    return f'<mesh name="{name}" vertex="{" ".join(f"{x:.5f} {y:.5f} {z:.5f}" for x, y, z in vs)}"/>'
def prism(name, xs, prof):
    vs = [(x, y, z) for x in xs for y, z in prof]
    return f'<mesh name="{name}" vertex="{" ".join(f"{x:.5f} {y:.5f} {z:.5f}" for x, y, z in vs)}"/>'
def lip(name, y_edge, z0, sgn):
    # curved lip (2 studs) from road height (2 plates above A1 top) down to A1 top
    prof = [(y_edge + sgn*8*S, z0 + P)]
    for k in range(9):
        t = k/8; prof.append((y_edge + sgn*(8 + 2*t)*S, z0 + P + 0.0006 + (2*P - 0.0006)*(0.5 + 0.5*math.cos(math.pi*t))))
    prof.append((y_edge + sgn*10*S, z0 + P))
    if sgn < 0: prof = prof[::-1]
    return prism(name, (-6*S, 6*S), prof)
def box(pos, size, quat=None, fr=0.9):
    q = f' quat="{quat}"' if quat else ''
    return f'<geom type="box" pos="{pos[0]:.5f} {pos[1]:.5f} {pos[2]:.5f}" size="{size[0]:.5f} {size[1]:.5f} {size[2]:.5f}"{q} friction="{fr}"/>'
g = []
for y_edge, z0, sgn in ((BR.Y_E, Z_BOX, 1), (BR.Y_LE, Z_BOOK, -1)):
    g.append(box((0, y_edge + sgn*4*S, z0 + 1.5*P), (6*S, 4*S, 1.5*P)))       # A1+A2+tiles (8 studs)
    g.append(box((0, y_edge + sgn*9*S, z0 + 0.5*P), (6*S, 1*S, 0.5*P)))       # A1 under the lip
q = f"{math.cos(BR.TH/2):.6f} {math.sin(BR.TH/2):.6f} 0 0"
dc = BR.deck_point(BR.LH/2, 0, 0.5*P)
g.append(box(dc, (6*S, BR.LH/2, P), q))
for v in (-BR.CURB_V, BR.CURB_V):
    c = BR.deck_point(9.5*S, v, P/2 + B/2); g.append(box(c, (S/2, 8*S, B/2), q, 0.3))
meshes = cuboid_mesh('boxm', RECTS['box'], Z_BOX) + cuboid_mesh('bookm', RECTS['book'], Z_BOOK) + lip('lipA', BR.Y_E, Z_BOX, 1) + lip('lipL', BR.Y_LE, Z_BOOK, -1)
wheels = ''.join(f'''
      <body name="w{i}" pos="{x:.4f} {y:.4f} {WR}">
        <joint name="j{i}" type="hinge" axis="0 1 0" damping="1e-6"/>
        <geom type="cylinder" size="{WR} {WW/2}" euler="90 0 0" mass="0.005" friction="1.0 0.005 0.0001"/>
      </body>''' for i, (x, y) in enumerate([(AX_F, WY), (AX_F, -WY), (AX_R, WY), (AX_R, -WY)]))
ztop = Z_CH + P + 3*B + P
xml = f'''<mujoco>
  <option timestep="0.0004" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default><geom solref="0.003 1" solimp="0.95 0.99 0.0005" condim="3"/></default>
  <asset>{meshes}</asset>
  <worldbody>
    <geom type="plane" size="2 2 0.1" friction="0.5"/>
    <geom type="mesh" mesh="boxm" friction="0.9"/><geom type="mesh" mesh="bookm" friction="0.7"/>
    <geom type="mesh" mesh="lipA" friction="0.9"/><geom type="mesh" mesh="lipL" friction="0.9"/>
    {"".join(g)}
    <body name="car" pos="0 {BUILD_Y} {Z_BOX+0.0003}" quat="0.70710678 0 0 -0.70710678">
      <freejoint/>
      <geom type="box" pos="0 0 {(Z_CH+ztop)/2:.4f}" size="{8*S:.4f} {3*S:.4f} {(ztop-Z_CH)/2:.4f}" mass="0.075"/>
      <geom type="box" pos="0 {2.5*S:.4f} {Z_TB+B/2:.4f}" size="{7*S:.4f} {S/2:.4f} {B/2:.4f}" mass="0.006"/>
      <geom type="box" pos="0 {-2.5*S:.4f} {Z_TB+B/2:.4f}" size="{7*S:.4f} {S/2:.4f} {B/2:.4f}" mass="0.006"/>
      {wheels}
    </body>
  </worldbody>
  <actuator>{''.join(f'<velocity joint="j{i}" kv="0.002" forcerange="-0.006 0.006"/>' for i in range(4))}</actuator>
</mujoco>'''
m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m)
FPS = 30
steps_per_frame = int(round(1/FPS/m.opt.timestep))
fine = []; frames = []; braking = False
nsteps = int((T_SIM + 0.5)/m.opt.timestep)
for k in range(nsteps):
    t = k*m.opt.timestep - 0.5
    pos = d.qpos[:3]
    v = 0.0 if t < 0 else V_MAX*min(1, t/ACC_T)
    if not braking and pos[1] < STOP_Y: braking = True
    d.ctrl[:] = 0.0 if braking else v/WR
    mujoco.mj_step(m, d)
    if t >= 0 and (k+1) % 5 == 0: fine.append(np.concatenate([[t], d.qpos[:7], d.qpos[7:11]]))
    if t >= 0 and (k+1) % steps_per_frame == 0: frames.append(np.concatenate([d.qpos[:7], d.qpos[7:11]]))
fr = np.array(frames)
np.savez('/home/claude/render_b/traj.npz', traj=fr, build_y=BUILD_Y, fine=np.array(fine))
for i in range(0, len(fr), 2):
    q = fr[i, 3:7]; pitch = math.degrees(math.asin(max(-1, min(1, 2*(q[0]*q[2] - q[3]*q[1])))))
    print(i, np.round(fr[i, :3], 4), 'pitch', round(pitch, 1))
