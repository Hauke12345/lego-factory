"""Sticker artwork (PNG with alpha), printed-Lego-sticker look."""
from PIL import Image, ImageDraw, ImageFont
import math
B = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
INTER = '/usr/share/fonts/opentype/inter/Inter-SemiBold.otf'
def font(p, s):
    try: return ImageFont.truetype(p, s)
    except OSError: return ImageFont.truetype(B, s)
O = '/home/claude/stickers/'
# 1) "Hauke AI" side sticker (5 studs x 0.8 brick -> 40 x 7.7 mm)
W, H = 1040, 200
im = Image.new('RGBA', (W, H), (0, 0, 0, 0)); d = ImageDraw.Draw(im)
d.rounded_rectangle([4, 4, W-4, H-4], 46, fill=(250, 248, 242, 255), outline=(20, 22, 28, 255), width=8)
d.rounded_rectangle([4, 4, 230, H-4], 46, fill=(201, 26, 9, 255))
d.rectangle([150, 4, 230, H-4], fill=(201, 26, 9, 255))
# little white 2x1 brick icon
d.rounded_rectangle([52, 92, 182, 150], 8, fill=(255, 255, 255, 255))
for cx in (85, 149): d.rounded_rectangle([cx - 20, 64, cx + 20, 94], 6, fill=(255, 255, 255, 255))
d.text((630, H/2 + 4), 'Hauke AI', font=font(B, 128), fill=(20, 22, 28, 255), anchor='mm')
im.save(O + 'hauke_ai.png')
# 2) speed stripe for the cab side (3 studs x 0.6 brick)
W, H = 600, 150
im = Image.new('RGBA', (W, H), (0, 0, 0, 0)); d = ImageDraw.Draw(im)
for k, c in enumerate(((242, 205, 55), (240, 143, 28), (201, 26, 9))):
    y0 = 10 + k*44
    d.polygon([(0, y0), (W - 60 - k*40, y0), (W - 20 - k*40, y0 + 34), (0, y0 + 34)], fill=c + (255,))
im.save(O + 'stripe.png')
# 3) hazard band (yellow/black chevrons) for the pier top
W, H = 400, 160
im = Image.new('RGBA', (W, H), (242, 205, 55, 255)); d = ImageDraw.Draw(im)
for x in range(-H, W + H, 80):
    d.polygon([(x, H), (x + 40, H), (x + 40 + H, 0), (x + H, 0)], fill=(20, 20, 22, 255))
d.rectangle([0, 0, W-1, H-1], outline=(20, 20, 22, 255), width=6)
im.save(O + 'hazard.png')
# 4) round load-limit sign "max 0.5 kg"
S = 400
im = Image.new('RGBA', (S, S), (0, 0, 0, 0)); d = ImageDraw.Draw(im)
d.ellipse([4, 4, S-4, S-4], fill=(201, 26, 9, 255)); d.ellipse([52, 52, S-52, S-52], fill=(250, 248, 242, 255))
d.text((S/2, S/2 - 42), '0.5', font=font(B, 128), fill=(20, 22, 28, 255), anchor='mm')
d.text((S/2, S/2 + 70), 'kg', font=font(B, 92), fill=(20, 22, 28, 255), anchor='mm')
im.save(O + 'maxload.png')
# 5) small round "Hauke AI" badge for the truck door/roof
S = 400
im = Image.new('RGBA', (S, S), (0, 0, 0, 0)); d = ImageDraw.Draw(im)
d.ellipse([4, 4, S-4, S-4], fill=(20, 22, 28, 255)); d.ellipse([28, 28, S-28, S-28], outline=(242, 205, 55, 255), width=14)
d.text((S/2, S/2 - 50), 'HAUKE', font=font(B, 78), fill=(250, 248, 242, 255), anchor='mm')
d.text((S/2, S/2 + 48), 'AI', font=font(B, 120), fill=(242, 205, 55, 255), anchor='mm')
im.save(O + 'badge.png')
print('ok')
