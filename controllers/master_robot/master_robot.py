"""Only this robot knows the manoeuvre plan. Radio carries measured motion."""
import math
import os
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from controller import Robot
from robot_io import RobotIO,clip
from mission import make_plan,SPEED,TURN_RATE

robot = Robot()
io = RobotIO(robot)
io.feedback_rx.enable(io.step_ms)
follower_ack = {}
stationary = os.environ.get('SWARM_SCENARIO') == 'stationary'
plan = make_plan(stationary)
phase_id = -1
start_heading = start_distance = target_heading = phase_start = 0.0
seq = 0
turns = []
while robot.step(io.step_ms) != -1:
    io.sense()
    now = robot.getTime()
    for feedback,_ in io.receive(io.feedback_rx):
        if feedback.get('role')=='follower':
            follower_ack[feedback['name']]=feedback['timestamp']
    if now >= 2.0 and phase_id == -1:
        phase_id,phase_start = 0,now
        start_heading,start_distance = io.heading,io.distance
        target_heading = start_heading+math.radians(plan[0].get('angle_deg',0))
    phase = plan[phase_id] if 0 <= phase_id < len(plan) else dict(
        kind='finish' if phase_id >= len(plan) else 'calibrate',
        label='Hold position' if phase_id >= len(plan) else 'Gyro calibration / aligned start')
    v,w = 0.0,0.0
    if phase['kind'] in ('turn','drive'):
        v = 0.0 if stationary else SPEED
        if phase['kind'] == 'turn':
            error = target_heading-io.heading
            w = clip(1.5*error,-TURN_RATE,TURN_RATE)
            reached = abs(error)<math.radians(0.25) and abs(io.command_w)<0.035
        else:
            w = clip(2.0*(target_heading-io.heading),-0.28,0.28)
            reached = io.distance-start_distance >= phase['distance']
        if reached:
            if phase['kind'] == 'turn':
                turns.append(dict(turn_id=phase['turn_id'],target_deg=phase['angle_deg'],
                                  estimated_deg=math.degrees(io.heading-start_heading)))
            phase_id += 1
            phase_start = now
            start_heading,start_distance = io.heading,io.distance
            if phase_id < len(plan) and plan[phase_id]['kind']=='turn':
                target_heading = io.heading+math.radians(plan[phase_id]['angle_deg'])
        if now-phase_start > 40:
            raise RuntimeError('Master manoeuvre timed out')
    io.drive(v,w)
    if io.tick % 5 == 0:
        seq += 1
        # No future headings, target angles, or waypoints are transmitted.
        io.send(io.state_tx,dict(role='master',name=robot.getName(),seq=seq,timestamp=now,
            pose=io.pose(),phase_id=phase_id,kind=phase['kind'],
            turn_id=phase.get('turn_id'),done=phase['kind']=='finish',
            phase_elapsed_s=now-phase_start,turns_completed=len(turns),follower_ack=follower_ack))

