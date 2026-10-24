"""Local indoor lab scenery; static furniture stays outside the tested route."""
def material(color, roughness=0.8):
    rgb=' '.join(map(str,color))
    return f'PBRAppearance {{ baseColor {rgb} roughness {roughness} metalness 0 }}'


def assembly(name,parts):
    visuals=[]
    collisions=[]
    for position,size,color in parts:
        xyz=' '.join(map(str,position))
        dimensions=' '.join(map(str,size))
        visuals.append(f'Pose {{ translation {xyz} children [ Shape {{ appearance {material(color)} geometry Box {{ size {dimensions} }} }} ] }}')
        collisions.append(f'Pose {{ translation {xyz} children [ Box {{ size {dimensions} }} ] }}')
    return f'''Solid {{
  name "{name}"
  children [ {' '.join(visuals)} ]
  boundingObject Group {{ children [ {' '.join(collisions)} ] }}
}}
'''


def make_environment():
    wall=(0.29,0.33,0.38)
    trim=(0.13,0.17,0.22)
    cabinet=(0.17,0.23,0.29)
    wood=(0.37,0.28,0.19)
    carton=(0.44,0.36,0.26)
    parts=[]
    # Low walls let the camera see into the room.
    for x in (-3.25,4.05):
        parts.extend([((x,0.5,0.32),(0.12,7.65,0.64),wall),
                      ((x,0.5,0.065),(0.14,7.65,0.13),trim)])
    for y in (-3.28,4.28):
        parts.extend([((0.4,y,0.32),(7.42,0.12,0.64),wall),
                      ((0.4,y,0.065),(7.42,0.14,0.13),trim)])
    result=assembly('Lab perimeter walls and skirting',parts)
    # South electronics bench: table, storage drawers, instrument and bins.
    parts=[((0.15,-2.98,0.64),(3.0,0.40,0.08),wood),
           ((-0.88,-2.98,0.30),(0.66,0.36,0.60),cabinet),
           ((1.18,-2.98,0.30),(0.66,0.36,0.60),cabinet),
           ((0.10,-2.98,0.76),(0.43,0.26,0.15),(0.25,0.29,0.34)),
           ((0.60,-2.98,0.77),(0.24,0.25,0.18),carton)]
    for x in (-0.88,1.18):
        for z in (0.18,0.38,0.56):
            parts.append(((x,-2.788,z),(0.30,0.014,0.016),(0.40,0.44,0.47)))
    result+=assembly('Electronics workbench',parts)
    # West work area and two small equipment cases.
    result+=assembly('West storage and assembly bench',[
        ((-2.98,-0.65,0.37),(0.38,2.5,0.74),cabinet),
        ((-2.98,-0.65,0.76),(0.42,2.55,0.06),wood),
        ((-2.98,-1.38,0.88),(0.28,0.40,0.18),carton),
        ((-2.98,0.08,0.86),(0.26,0.48,0.14),(0.25,0.31,0.36))])
    # East metal shelving, shelves and muted storage boxes.
    parts=[]
    for y in (-0.30,2.35):
        for x in (3.54,3.84):
            parts.append(((x,y,0.65),(0.035,0.035,1.30),trim))
    for z in (0.10,0.55,1.02):
        parts.append(((3.69,1.025,z),(0.36,2.70,0.045),cabinet))
    for y,z,size in ((0.12,0.28,(0.24,0.43,0.31)),(0.89,0.29,(0.25,0.51,0.33)),
                     (1.70,0.70,(0.25,0.42,0.26)),(0.45,1.18,(0.23,0.47,0.28))):
        parts.append(((3.69,y,z),size,carton))
    result+=assembly('East shelving and parts boxes',parts)
    # North packing counter and a cabinet near the rear corner.
    result+=assembly('North packing counter',[
        ((1.25,3.91,0.34),(2.60,0.42,0.68),cabinet),
        ((1.25,3.91,0.71),(2.70,0.46,0.06),wood),
        ((0.40,3.91,0.88),(0.43,0.32,0.28),carton),
        ((1.95,3.91,0.85),(0.50,0.31,0.23),carton),
        ((-2.75,3.85,0.48),(0.65,0.60,0.96),cabinet)])
    # Visual-only floor zones, set below trail height, with no contact geometry.
    result+='''Pose { translation 0.35 0.45 0.0002 children [
      Shape { appearance PBRAppearance { baseColor 0.058 0.078 0.10 roughness 1 metalness 0 }
              geometry Box { size 6.0 6.05 0.0002 } }
    ] }
'''
    return result

