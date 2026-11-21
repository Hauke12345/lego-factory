from PIL import Image, ImageDraw, ImageFont
import json, math
W, H = 2400, 1700
PAPER = (246, 242, 231); INK = (35, 35, 38); RED = (201, 26, 9)
sheet = Image.new('RGB', (W, H), PAPER); d = ImageDraw.Draw(sheet)
F = lambda s, b=False: ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf' % ('-Bold' if b else ''), s)
# border with zone ticks
d.rectangle([30, 30, W-30, H-30], outline=INK, width=4); d.rectangle([60, 60, W-60, H-60], outline=INK, width=2)
for i in range(1, 8):
    x = 60 + i*(W-120)/8; d.line([x, 30, x, 60], fill=INK, width=2); d.line([x, H-60, x, H-30], fill=INK, width=2)
    d.text((x-(W-120)/16, 45), str(i), font=F(22), fill=INK, anchor='mm'); d.text((x-(W-120)/16, H-45), str(i), font=F(22), fill=INK, anchor='mm')
for i, L in enumerate('ABCDE'):
    y = 60 + (i+0.5)*(H-120)/5; d.text((45, y), L, font=F(22), fill=INK, anchor='mm'); d.text((W-45, y), L, font=F(22), fill=INK, anchor='mm')
# line art
art = Image.open('/home/claude/drawing/b_lines.png').convert('RGBA')
OX, OY = 90, 120
sheet.paste(art, (OX, OY), art)
pts = json.load(open('/home/claude/drawing/b_pts.json'))
P = {n: ((ex[0]+OX, ex[1]+OY), (fi[0]+OX, fi[1]+OY)) for n, fi, ex in pts}
# dash-dot assembly trace lines
def dashdot(a, b):
    L = math.dist(a, b); n = int(L/34)
    for k in range(n):
        t0, t1, t2 = k/n, (k+0.55)/n, (k+0.75)/n
        p = lambda t: (a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t)
        d.line([p(t0), p(t1)], fill=(120, 120, 120), width=2); d.ellipse([p(t2)[0]-2, p(t2)[1]-2, p(t2)[0]+2, p(t2)[1]+2], fill=(120, 120, 120))
for n in ('pier0', 'chord0', 'curb1', 'appT2', 'landT3', 'road12', 'post21'):
    ex, fi = P[n]
    if math.dist(ex, fi) > 40: dashdot(ex, fi)
BOM = [('pierFoot', 1, 'Plate 4×8', 'light grey'), ('pier3', 7, 'Brick 2×8', 'light grey'),
       ('pierHinge1', 2, 'Hinge brick 1×2', 'light grey'), ('appA2', 4, 'Plate 6×12', 'dark grey'),
       ('landA1b', 2, 'Plate 4×12', 'dark grey'), ('appH5', 16, 'Click hinge 2-fing.', 'light grey'),
       ('dH110', 16, 'Click hinge 1-fing.', 'light grey'), ('landF1', 16, 'Plate 1×2', 'grey'),
       ('deck1', 2, 'Plate 6×14', 'light grey'), ('road21', 19, 'Tile 2×6', 'dark grey'),
       ('landT2', 4, 'Tile 6×6', 'dark grey'), ('appL2', 12, 'Slope curved 2×2', 'dark grey'),
       ('curb0', 2, 'Technic brick 1×16', 'light grey'), ('chord1', 2, 'Technic brick 1×14', 'light grey'),
       ('post30', 10, 'Liftarm thin 1×5', 'yellow'), ('diag20', 8, 'Liftarm thin 1×6', 'yellow'),
       ('post11', 20, 'Technic pin 3L', 'blue')]
cx = OX + 850; cy = OY + 750
used = []
for i, (n, q, desc, col) in enumerate(BOM, 1):
    ex = P[n][0]
    vx, vy = ex[0]-cx, ex[1]-cy; L = math.hypot(vx, vy) or 1
    r = 150 + (i % 3)*30
    bx, by = ex[0] + vx/L*r, ex[1] + vy/L*r
    bx = min(max(bx, 120), OX+1650); by = min(max(by, 120), H-110)
    for (ux, uy) in used:
        if math.dist((bx, by), (ux, uy)) < 64: by += 70
    used.append((bx, by))
    d.line([ex, (bx, by)], fill=INK, width=2); d.ellipse([ex[0]-5, ex[1]-5, ex[0]+5, ex[1]+5], fill=INK)
    d.ellipse([bx-28, by-28, bx+28, by+28], fill=PAPER, outline=INK, width=3)
    d.text((bx, by), str(i), font=F(26, True), fill=INK, anchor='mm')
# parts list
TX = 1830; TY = 120; RH = 46
d.text((TX, TY), 'PARTS LIST', font=F(34, True), fill=INK)
TY += 60
cols = [0, 64, 124, 365]
d.rectangle([TX-10, TY-8, W-80, TY+RH*(len(BOM)+1)], outline=INK, width=2)
for j, h in enumerate(('ITEM', 'QTY', 'PART', 'COLOUR')): d.text((TX+cols[j], TY), h, font=F(22, True), fill=INK)
d.line([TX-10, TY+RH-12, W-80, TY+RH-12], fill=INK, width=2)
for i, (n, q, desc, col) in enumerate(BOM, 1):
    y = TY + RH*i
    for j, v in enumerate((str(i), str(q), desc, col)): d.text((TX+cols[j], y), v, font=F(22), fill=INK)
    d.line([TX-10, y+RH-12, W-80, y+RH-12], fill=(170, 170, 170), width=1)
# title block
by0 = H-60-300
d.rectangle([TX-10, by0, W-60, H-60], outline=INK, width=3)
rows = [('TITLE', 'LEGO TRUSS BRIDGE'), ('PROJECT', 'Hauke | AI Experiments'), ('DRAWN', 'Claude   2026-11-08'),
        ('PARTS', '145 incl. pins'), ('SCALE', '1:1      SHEET 1/1'), ('DWG NO.', 'HAE-LEGO-002')]
for i, (k, v) in enumerate(rows):
    y = by0 + 12 + i*48
    d.text((TX, y), k, font=F(18), fill=(110, 110, 110)); d.text((TX+130, y-2), v, font=F(24, i == 0), fill=RED if i == 0 else INK)
    if i: d.line([TX-10, y-8, W-60, y-8], fill=INK, width=1)
sheet.save('/home/claude/drawing/bridge_drawing.png')
sheet.resize((1200, 850)).save('/home/claude/test/sheet_bridge.jpg')
