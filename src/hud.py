import pickle, sys
from PIL import Image, ImageDraw, ImageFont
L,park=pickle.load(open('/home/claude/factory/prev/tug_log2.pkl','rb'))
f1=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',26); f2=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf',20)
import os; os.makedirs('/home/claude/factory/prev/tug_hud',exist_ok=True)
for k,s in enumerate(L):
    im=Image.open(f'/home/claude/factory/prev/tug_anim/f{k:04d}.png').convert('RGB'); d=ImageDraw.Draw(im,'RGBA')
    d.rectangle((24,596,560,696),fill=(20,24,30,215))
    d.text((40,606),s['state'],font=f1,fill=(255,255,255))
    col=s['col'] or '-'
    d.text((40,648),'reflection %3d %%   marker %-4s   hook %s'%(round(s['v']),col,'down' if s['hook']<0 else 'up'),font=f2,fill=(200,210,220))
    d.rectangle((40,676,40+int(4.8*s['v']),684),fill=(240,200,60,255))
    im.save(f'/home/claude/factory/prev/tug_hud/f{k:04d}.png')
