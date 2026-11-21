# Instructions for an AI picking up this project

Read this first if the original workspace is gone. Everything (robots, stations, lines, previews) is generated from the
Python in `src/`; nothing important lives outside this repo except the public LDraw library.

## 1. Rebuild the environment
1. Run `./setup.sh` (Linux). It installs `bpy==5.2.3` (Blender as a Python module), numpy, scipy, Pillow, matplotlib,
   OpenCV, imageio, ffmpeg; downloads the LDraw `complete.zip` to `/home/claude/ldraw/lib/complete/ldraw`; copies
   `ldcad-shadow/` to `/home/claude/ldraw/shadow`; links `src/` to `/home/claude/scripts`.
2. Paths are hard-coded (`ldr.py`: ROOT, SHADOW, CACHE; preview scripts insert `/home/claude/scripts`). Keep that
   layout rather than refactoring, or change the constants in `ldr.py` first.
3. Smoke test (takes ~3 min, builds ~8300 parts):
   ```
   cd /home/claude/scripts && python3 -c "import line3, lcheck; lcheck.run(line3.build(line3.cycle(14/41, 'builder')))"
   ```
   Expected: `CLASHES 0`, `BAD FITS 0`. The only acceptable "unconnected" lines are bodies of loose parts held by
   tooling (`spart*`, `part*`, `unit*` falling apart into separate pieces) and `frame falls apart` (ground stacks).
4. If the LDraw library version differs and a part is missing, look for the renamed part number in the library
   (e.g. `3794b` vs `3794`) — do not substitute a different-shaped part.

## 2. What the project is (user's rules — keep them)
- Video of a self-replicating LEGO world: robots build the factory, the factory builds robots. No humans.
- Real LDraw parts only, physically correct interactions, every motor/sensor powered by a hub (Boost/Move Hub 26910,
  Boost motor 26913), cables plugged by the machines.
- ALWAYS check for intersecting parts and broken connections (`lcheck.py`) at many time points before showing anything.
- All inputs arrive by transport (rail or belt). Guess interaction values; don't ask the user for them.
- Previews in Blender Workbench (fast); final film in Cycles.
- Never put the user's private email in anything public; commit as `Hauke12345 <Hauke12345@users.noreply.github.com>`.
- Work step by step, build stations rather than theorise, show results as preview videos + still sheets.

## 3. Conventions
- Units: LDraw LDU, y points DOWN (-y is up). Blender coords = (x, -z, -y) * 0.0004 (`lblend.M_bl`).
- Baseplates 3811 at y 0; 640-LDU cells, centres x ≡ 320, z ≡ 330 (mod 640) → world studs at x ≡ 10, z ≡ 0 (mod 20).
  Check stud parity for every ground-standing part (this bit us: off-by-10 columns clash with baseplate studs).
- Technic hole heights above the baseplate: ≡ 2 (mod 8) (-14 - 8n); from a wagon deck top (-92): ≡ 6 (mod 8).
- Part frames: bricks/plates origin at the top face, extend +y; axles/pins along local x, centred; gears/bushes axis local z;
  liftarms: length local z, holes along local y.
- Robot frame (worker2): x right, y down, z back. `asm2.core_M0(px)` maps robot → lane-canonical
  (x_c = px - 211 - y_r, y_c = -250 + z_r, z_c = -x_r).
- Lanes/carts: `rails.py`. A cart is built in its canonical frame (x along the lane) and placed with
  `rails.cart(lane, s, build, sx0, body, tag)`; `tline.cart_G(lane, s, zc)` gives canonical → world.
- Side B of every line is the mirror of side A: build A in canonical, conjugate with `tline.MIR` (z → -z), place with
  `cart_G(..., zc * sg)` (sg = -1 on side B — using A's zc on B was a real bug). Parts lopsided along local z
  (tread links, motors, hubs) need an extra `R('y', 180)` after conjugation (`tline._bflip`).
- Bodies: every part has a `body`; moving groups get their own body and an entry in `lcheck.RESTING` saying why they
  may be unattached (resting, sliding, hanging).

## 4. Code map (`src/`)
- `ldr.py` part loader + LDCad snaps · `lcheck.py` clash/fit/gear checker (`run(P)`) · `lsweep.py` sweeps a cycle.
- Robots: `worker2.py` (core, track units trackL/trackR, head; `track_path()` gives the 34-link chain loop),
  `mods.py`, `lowmods.py`, `periph.py`, `tug.py`.
- Final assembly line: `line3.py` (`PHASES`, `cycle(t, cfg)`, `build(st)`; configs `builder` and `tug`), `asm2.py`
  (track-unit cell, crank pusher `asm2.pusher`), `cell.py`, `shoulder.py`, `rails.py`.
- Track-unit line: `tline.py` (build wagon visits S0..S4, supply train S0..S5 + loco, timeline `state()`, refill table
  `REF`), `tunit.py` (build/supply wagons and their tooling).
- Chain station: `chainst.py` (tilting table in robot frame: stubs, Boost chain drive on the idler at z -80,
  transfer rods + crank, guide posts, `chain_links(n, adv)`), `cstation.py` (hinge, tilt hub, link belt, queue,
  side rails, `state(u)`, `build_canon()`); placed from `tline.build_side`.
- Previews: `prev3.py` (final line), `prevT.py` (unit line), `prevC.py` (chain station) + `*_post.py` (captions, ffmpeg).
  Example: `python3 prevC.py OUT 246 0 123 & python3 prevC.py OUT 246 123 246; python3 prevC_post.py OUT out.mp4 6`.
- Other files (`bridge*`, `bscene.py`, `sfm.py`, `film*.py`, `fem*.py`, …) belong to the user's earlier bridge/film
  projects and to older factory iterations (`line1/line2`, `worker.py`); not needed for the current line.

## 5. Status (2026-11-21)
Done and clash-checked over the cycle (both sides): final assembly line (builder + tug), track-unit line with five
stations and handover, chain station building the drive set from 3 sprockets + 34 single 3873 links and handing it
to supply wagon S4; S5 brings loose sprockets. Previews in `previews/`.
Open: parts warehouse restocking the supply wagons, link belt and S5; cable from the tilt hub to the chain-station
motors; eye-bracket station for the tug's B unit; final Cycles film.

## 6. Working method that worked
1. Design a station in its own frame, test it alone with a small script (build parts list → `lcheck.run`), at several
   motion states. 2. Place it in the line, then sweep ~20 time points of the 41 s cycle in parallel (2 processes).
3. Fix every clash/bad fit (never hide them by renaming bodies). 4. Render a Workbench preview with captions,
   send the video + a still sheet, summarise in plain words.
