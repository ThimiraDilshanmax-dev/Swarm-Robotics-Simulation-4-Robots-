"""Measured trails with smooth age fading; visual geometry never drives robots."""
import json
import math
from robot_config import NAMES,COLORS

TRAIL_CAPACITY=128
TRAIL_LIFETIME=12.0
TRAIL_OPACITY=0.24
TRAIL_SPACING=0.025


def fade_opacity(age):
    t=max(0.0,min(1.0,age/TRAIL_LIFETIME))
    return TRAIL_OPACITY*(1.0-t*t*(3.0-2.0*t))


def make_trail_nodes():
    result=[]
    for i,color in enumerate(COLORS):
        rgb=' '.join(str(0.42*c+0.58*b) for c,b in zip(color,(0.055,0.082,0.12)))
        for j in range(TRAIL_CAPACITY):
            result.append(f'''Shape {{
  appearance Appearance {{ material DEF FADE_MAT_{i}_{j} Material {{
    diffuseColor {rgb} emissiveColor {rgb} transparency 1
  }} }}
  geometry IndexedLineSet {{
    coord DEF FADE_COORD_{i}_{j} Coordinate {{ point [ 0 0 0.004 0 0 0.004 ] }}
    coordIndex [ 0 1 -1 ]
  }}
}}
''')
    return ''.join(result)


class PathTrails:
    def __init__(self,supervisor):
        self.paths=[[] for _ in NAMES]
        self.times=[[] for _ in NAMES]
        self.fields=[]
        self.active=[{} for _ in NAMES]
        self.last_visual=[None]*4
        self.cursor=[0]*4
        self.now=0.0
        for i in range(4):
            self.fields.append([
                (supervisor.getFromDef(f'FADE_COORD_{i}_{j}').getField('point'),
                 supervisor.getFromDef(f'FADE_MAT_{i}_{j}').getField('transparency'))
                for j in range(TRAIL_CAPACITY)])

    def update(self,positions,now):
        self.now=now
        for i,position in enumerate(positions):
            x,y=position[:2]
            path=self.paths[i]
            if not path or math.hypot(x-path[-1][0],y-path[-1][1])>=0.012:
                path.append([x,y])
                self.times[i].append(now)
            previous=self.last_visual[i]
            if previous is None:
                self.last_visual[i]=[x,y]
            elif math.hypot(x-previous[0],y-previous[1])>=TRAIL_SPACING:
                slot=self.cursor[i]
                coords,material=self.fields[i][slot]
                coords.setMFVec3f(0,[previous[0],previous[1],0.004])
                coords.setMFVec3f(1,[x,y,0.004])
                material.setSFFloat(1-TRAIL_OPACITY)
                self.active[i][slot]=[now,TRAIL_OPACITY]
                self.cursor[i]=(slot+1)%TRAIL_CAPACITY
                self.last_visual[i]=[x,y]
            for slot,(born,old_opacity) in list(self.active[i].items()):
                opacity=fade_opacity(now-born)
                if opacity==0 or abs(opacity-old_opacity)>0.0005:
                    self.fields[i][slot][1].setSFFloat(1-opacity)
                    if opacity==0:
                        del self.active[i][slot]
                    else:
                        self.active[i][slot][1]=opacity

    def export(self,directory):
        data=dict(source='Measured physical robot centre positions',units='metres',
                  paths=dict(zip(NAMES,self.paths)),sample_times_s=dict(zip(NAMES,self.times)),
                  fade_lifetime_s=TRAIL_LIFETIME)
        (directory/'paths.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
        all_points=[p for path in self.paths for p in path]
        xmin=min(p[0] for p in all_points)-0.3
        xmax=max(p[0] for p in all_points)+0.3
        ymin=min(p[1] for p in all_points)-0.3
        ymax=max(p[1] for p in all_points)+0.3
        scale=800/max(xmax-xmin,ymax-ymin,1)
        def pixel(p):
            return 100+(p[0]-xmin)*scale,870-(p[1]-ymin)*scale
        svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="1000" viewBox="0 0 1000 1000">',
             '<rect width="1000" height="1000" fill="#101923"/>',
             '<g font-family="Arial,sans-serif"><text x="45" y="48" fill="#edf4ff" font-size="26">Fading robot paths</text>',
             '<text x="45" y="78" fill="#a8bdd1" font-size="15">Recent movement fades smoothly over 12 seconds | JSON retains the full route</text>']
        for k in range(math.ceil(xmin*2),math.floor(xmax*2)+1):
            a,b=pixel([k/2,ymin]),pixel([k/2,ymax])
            svg.append(f'<path d="M {a[0]:.2f} {a[1]:.2f} L {b[0]:.2f} {b[1]:.2f}" stroke="#223343" stroke-width="1" stroke-opacity="0.15"/>')
        for k in range(math.ceil(ymin*2),math.floor(ymax*2)+1):
            a,b=pixel([xmin,k/2]),pixel([xmax,k/2])
            svg.append(f'<path d="M {a[0]:.2f} {a[1]:.2f} L {b[0]:.2f} {b[1]:.2f}" stroke="#223343" stroke-width="1" stroke-opacity="0.15"/>')
        for i,path in enumerate(self.paths):
            color='#'+''.join(f'{int(c*255):02x}' for c in COLORS[i])
            for j in range(1,len(path)):
                opacity=fade_opacity(self.now-self.times[i][j])
                if opacity<=0:
                    continue
                a,b=pixel(path[j-1]),pixel(path[j])
                svg.append(f'<path d="M {a[0]:.2f} {a[1]:.2f} L {b[0]:.2f} {b[1]:.2f}" fill="none" stroke="{color}" stroke-width="1" stroke-opacity="{opacity:.4f}" stroke-linecap="round"/>')
            end=pixel(path[-1])
            svg.append(f'<circle cx="{end[0]:.2f}" cy="{end[1]:.2f}" r="4" fill="{color}" fill-opacity="0.4"/>')
            svg.append(f'<text x="{45+i*240}" y="956" fill="{color}" font-size="19">R{i+1} '+('master' if i==0 else 'follower')+'</text>')
        svg.append('</g></svg>')
        (directory/'paths.svg').write_text(''.join(svg),encoding='utf-8')

