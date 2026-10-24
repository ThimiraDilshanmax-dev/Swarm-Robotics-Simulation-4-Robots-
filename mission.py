"""Master-only manoeuvre plan. Followers never import this module."""
import math
from robot_config import NAMES, OFFSETS, COLORS, WHEEL_RADIUS, TRACK

TURN_DEGREES = (30, 45, -60, 90, -120, 135, 180, -90)
START = (-1.80, -2.00)
SPEED = 0.13
TURN_RATE = 0.28


def make_plan(stationary=False):
    if stationary:
        return [dict(kind='turn', angle_deg=90, label='Stationary master: 90 deg left', turn_id=1)]
    plan = [dict(kind='drive', distance=0.40, label='Master straight departure')]
    for i, angle in enumerate(TURN_DEGREES, 1):
        plan.append(dict(kind='turn', angle_deg=angle, turn_id=i,
                         label=f'Moving turn {i}: {angle:+d} deg'))
        plan.append(dict(kind='drive', distance=0.30, label=f'Straight after turn {i}'))
    return plan


def reference_path():
    """Approximate visual guide only; never supplied to a robot controller."""
    x, y, yaw = START[0], START[1], 0.0
    points = [(x, y)]
    for phase in make_plan():
        if phase['kind'] == 'drive':
            x += phase['distance'] * math.cos(yaw)
            y += phase['distance'] * math.sin(yaw)
            points.append((x, y))
        else:
            angle = math.radians(phase['angle_deg'])
            rate = math.copysign(TURN_RATE, angle)
            count = max(2, int(abs(angle) / 0.07))
            for _ in range(count):
                increment = angle / count
                x += SPEED / rate * (math.sin(yaw + increment) - math.sin(yaw))
                y -= SPEED / rate * (math.cos(yaw + increment) - math.cos(yaw))
                yaw += increment
                points.append((x, y))
    return points

