"""The two reference triangles, drawn from real radio freshness and body positions."""
import math
from robot_config import NAMES

# R1=master, R2=blue, R3=orange, R4=purple.
TRIANGLES=(
    ('RED',(0.60,0.22,0.18),((0,2),(2,3),(3,0))),
    ('GREEN',(0.22,0.56,0.32),((0,1),(1,3),(3,0))),
)


def make_communication_nodes():
    result=[]
    for name,color,edges in TRIANGLES:
        rgb=' '.join(map(str,color))
        for j,_ in enumerate(edges):
            result.append(f'''Shape {{
  appearance Appearance {{ material DEF LINK_MAT_{name}_{j} Material {{
    diffuseColor {rgb} emissiveColor {rgb} transparency 0.98
  }} }}
  geometry IndexedLineSet {{
    coord DEF LINK_COORD_{name}_{j} Coordinate {{ point [ 0 0 0.009 0 0 0.009 ] }}
    coordIndex [ 0 1 -1 ]
  }}
}}
''')
    return ''.join(result)


class CommunicationOverlay:
    def __init__(self,supervisor):
        self.edges=[]
        for name,_,edges in TRIANGLES:
            for j,(a,b) in enumerate(edges):
                coords=supervisor.getFromDef(f'LINK_COORD_{name}_{j}').getField('point')
                material=supervisor.getFromDef(f'LINK_MAT_{name}_{j}').getField('transparency')
                self.edges.append(dict(a=a,b=b,name=name,coords=coords,material=material,opacity=0.02))
        self.fresh_links={}

    def update(self,positions,master,followers,now):
        for edge in self.edges:
            a,b=edge['a'],edge['b']
            if a==0 or b==0:
                follower=NAMES[b if a==0 else a]
                data=followers.get(follower,{})
                ack=None if master is None else master.get('follower_ack',{}).get(follower)
                fresh=(not data.get('stale',True) and now-data.get('timestamp',-100)<0.4
                       and ack is not None and now-ack<0.4)
            else:
                first=followers.get(NAMES[a],{})
                second=followers.get(NAMES[b],{})
                fresh=(now-first.get('timestamp',-100)<0.4 and now-second.get('timestamp',-100)<0.4
                       and first.get('peer_links',{}).get(NAMES[b],{}).get('fresh',False)
                       and second.get('peer_links',{}).get(NAMES[a],{}).get('fresh',False))
            key='--'.join(sorted((NAMES[a],NAMES[b])))
            self.fresh_links[key]=bool(fresh)
            target=0.32 if fresh else 0.025
            edge['opacity']+=0.20*(target-edge['opacity'])
            edge['material'].setSFFloat(1-edge['opacity'])
            x1,y1=positions[a][:2]
            x2,y2=positions[b][:2]
            # Offset the shared master-purple diagonal slightly so both colours
            # remain visible, as in the reference, without a bright thick stroke.
            offset=0.004 if edge['name']=='RED' else -0.004
            if set((a,b))=={0,3}:
                length=max(0.01,math.hypot(x2-x1,y2-y1))
                nx,ny=-(y2-y1)/length*offset,(x2-x1)/length*offset
                x1,y1,x2,y2=x1+nx,y1+ny,x2+nx,y2+ny
            edge['coords'].setMFVec3f(0,[x1,y1,0.009])
            edge['coords'].setMFVec3f(1,[x2,y2,0.009])

