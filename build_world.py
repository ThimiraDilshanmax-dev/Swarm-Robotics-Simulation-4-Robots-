"""Regenerate this project's local geometry. No downloads or external assets."""
from pathlib import Path
import math
from mission import COLORS, NAMES, OFFSETS, START
from environment import make_environment
from path_trails import make_trail_nodes
from communication_visuals import make_communication_nodes

ROOT = Path(__file__).resolve().parent


def appearance(color):
    return 'PBRAppearance { baseColor ' + ' '.join(map(str, color)) + ' roughness 0.65 metalness 0.05 }'


def wheel(side, y):
    return f'''
      HingeJoint {{
        jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0 {y} 0.045 dampingConstant 0.001 }}
        device [ RotationalMotor {{ name "{side}_motor" maxVelocity 8 maxTorque 0.30 acceleration 7 sound "" }} PositionSensor {{ name "{side}_encoder" }} ]
        endPoint Solid {{
          translation 0 {y} 0.045
          rotation 1 0 0 1.57079632679
          name "{side}_wheel"
          contactMaterial "rubber"
          children [
            Shape {{ appearance {appearance((0.025,0.032,0.045))} geometry Cylinder {{ radius 0.045 height 0.026 subdivision 32 }} }}
            Pose {{ translation 0 0 0.014 children [ Shape {{ appearance {appearance((0.65,0.72,0.8))} geometry Cylinder {{ radius 0.022 height 0.002 }} }} ] }}
          ]
          boundingObject Cylinder {{ radius 0.045 height 0.026 subdivision 32 }}
          physics Physics {{ density -1 mass 0.065 }}
        }}
      }}'''


def caster(x, name):
    return f'''
      BallJoint {{
        jointParameters BallJointParameters {{ anchor {x} 0 0.012 }}
        endPoint Solid {{
          translation {x} 0 0.012
          name "{name}_support"
          contactMaterial "caster"
          children [ Shape {{ appearance {appearance((0.42,0.48,0.55))} geometry Sphere {{ radius 0.012 subdivision 2 }} }} ]
          boundingObject Sphere {{ radius 0.012 subdivision 2 }}
          physics Physics {{ density -1 mass 0.015 }}
        }}
      }}'''


proto = '''#VRML_SIM R2025a utf8
PROTO StableRobot [
  field SFVec3f translation 0 0 0
  field SFRotation rotation 0 0 1 0
  field SFString name "R1_Green"
  field SFString controller "stable_robot"
  field SFColor bodyColor 0.1 0.72 0.39
] {
  Robot {
    translation IS translation
    rotation IS rotation
    name IS name
    controller IS controller
    children [
      Pose { translation 0 0 0.052 children [
        Shape { appearance PBRAppearance { baseColor IS bodyColor roughness 0.4 metalness 0.1 }
                geometry Box { size 0.23 0.19 0.054 } }
      ] }
      Pose { translation 0.084 0 0.083 rotation 0 1 0 1.57079632679 children [
        Shape { appearance PBRAppearance { baseColor 0.95 0.98 1 roughness 0.5 }
                geometry Cone { bottomRadius 0.023 height 0.05 subdivision 3 } }
      ] }
      Pose { translation -0.035 0 0.084 children [
        Shape { appearance PBRAppearance { baseColor 0.04 0.065 0.09 roughness 0.4 }
                geometry Box { size 0.065 0.09 0.01 } }
      ] }
''' + wheel('left', 0.12) + wheel('right', -0.12) + caster(-0.095, 'rear') + caster(0.095, 'front') + '''
      Gyro { name "gyro" }
      Receiver { name "state_receiver" channel 42 signalStrengthNoise 0.01 }
      Receiver { name "feedback_receiver" channel 43 }
      Emitter { name "state_sender" channel 42 range 20 }
      Emitter { name "feedback_sender" channel 43 range 20 }
      Receiver { name "peer_receiver" channel 44 }
      Emitter { name "peer_sender" channel 44 range 20 }
    ]
    boundingObject Pose { translation 0 0 0.052 children [ Box { size 0.23 0.19 0.054 } ] }
    physics Physics {
      density -1
      mass 0.85
      centerOfMass [ 0 0 0.035 ]
    }
  }
}
'''

world = '''#VRML_SIM R2025a utf8
EXTERNPROTO "../protos/StableRobot.proto"
WorldInfo {
  title "Master and Three Followers - Moving Turns"
  info [ "Four physical differential-drive robots; 8 measured left/right turns."
         "Master measured motion over radio; follower gyro/encoder odometry. Read-only observer." ]
  basicTimeStep 10
  randomSeed 12
  contactProperties [
    ContactProperties { material1 "rubber" material2 "floor" coulombFriction [ 1.1 ] bounce 0 forceDependentSlip [ 0.001 ] bumpSound "" rollSound "" slideSound "" }
    ContactProperties { material1 "caster" material2 "floor" coulombFriction [ 0.025 ] bounce 0 softCFM 0.0001 bumpSound "" rollSound "" slideSound "" }
  ]
}
Viewpoint {
  orientation -0.57735026919 0.57735026919 0.57735026919 2.09439510239
  position -2.2 -2.0 6.5
  follow "R1_Green"
  followType "Tracking Shot"
  followSmoothness 0.1
  fieldOfView 0.78
}
Background { skyColor [ 0.025 0.04 0.065 ] }
DirectionalLight { direction -0.4 -0.6 -1 intensity 1.5 castShadows TRUE }
DirectionalLight { direction 0.6 0.2 -1 intensity 0.5 }
Solid {
  translation 0 0 -0.03
  name "12 metre arena"
  contactMaterial "floor"
  children [ Shape { appearance PBRAppearance { baseColor 0.055 0.082 0.12 roughness 1 metalness 0 }
                     geometry DEF FLOOR Box { size 12 12 0.06 } } ]
  boundingObject USE FLOOR
}
'''

# Grid and actual-path trails are visual only, with no collision objects.
grid_points, grid_indices = [], []
for k in range(-12, 13):
    coordinate = k * 0.5
    for a, b in [((-6,coordinate,0.001),(6,coordinate,0.001)), ((coordinate,-6,0.001),(coordinate,6,0.001))]:
        index = len(grid_points)
        grid_points.extend([a, b])
        grid_indices.extend([index, index + 1, -1])


def lines(points, indices, color):
    coords = ', '.join(' '.join(f'{v:.5f}' for v in p) for p in points)
    return f'''Shape {{ appearance Appearance {{ material Material {{ diffuseColor {color} emissiveColor {color} transparency 0.94 }} }}
      geometry IndexedLineSet {{ coord Coordinate {{ point [ {coords} ] }} coordIndex [ {' '.join(map(str, indices))} ] }} }}\n'''


world += lines(grid_points, grid_indices, '0.065 0.085 0.11')
world += make_environment()
world += make_trail_nodes()
world += make_communication_nodes()
for i, (name, offset, color) in enumerate(zip(NAMES, OFFSETS, COLORS)):
    x, y = START[0] + offset[0], START[1] + offset[1]
    world += f'''DEF ROBOT_{i} StableRobot {{ translation {x} {y} 0 name "{name}" controller "{'master_robot' if i == 0 else 'stable_robot'}" bodyColor {' '.join(map(str, color))} }}\n'''
world += '''Robot {
  name "Mission display and measurement"
  supervisor TRUE
  controller "demo_supervisor"
  children [ Receiver { name "master_monitor" channel 42 } Receiver { name "follower_monitor" channel 43 } ]
}
'''

if __name__ == '__main__':
    (ROOT / 'protos').mkdir(exist_ok=True)
    (ROOT / 'worlds').mkdir(exist_ok=True)
    (ROOT / 'protos/StableRobot.proto').write_text(proto, encoding='utf-8')
    (ROOT / 'worlds/four_robots_multi_angle.wbt').write_text(world, encoding='utf-8')
    print('Created local robot prototype and four-robot world.')
