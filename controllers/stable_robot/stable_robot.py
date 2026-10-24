"""Follower: no mission, GPS, absolute yaw, or supervisor commands."""
import math
import os
from pathlib import Path
import random
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from controller import Robot
from robot_config import OFFSETS,RADIO_DELAY,RADIO_TIMEOUT,PACKET_LOSS,NAMES,PEER_NEIGHBOURS
from robot_io import RobotIO,clip

robot = Robot()
io = RobotIO(robot)
io.state_rx.enable(io.step_ms)
io.peer_rx.enable(io.step_ms)
peer_rng=random.Random(300+io.index)
peer_pending=[]
peer_received={}
peer_seq=0
neighbours={NAMES[i] for i in PEER_NEIGHBOURS[io.index]}
rng = random.Random(80+io.index)
scenario = os.environ.get('SWARM_SCENARIO','nominal')
pending = []
master = None
last_seq = -1
received = dropped = timeout_count = 0
range_m = None
blackout_start = None
was_stale = True
while robot.step(io.step_ms) != -1:
    io.sense()
    now = robot.getTime()
    for message,peer_range in io.receive(io.peer_rx):
        if message.get('role')=='peer' and message.get('name') in neighbours and peer_rng.random()>=PACKET_LOSS:
            peer_pending.append((now+RADIO_DELAY,message,peer_range))
    while peer_pending and peer_pending[0][0]<=now:
        _,peer,peer_range=peer_pending.pop(0)
        previous=peer_received.get(peer['name'])
        if now-peer['timestamp']<=RADIO_TIMEOUT and (previous is None or peer['seq']>previous['seq']):
            peer_received[peer['name']]=dict(seq=peer['seq'],timestamp=peer['timestamp'],range_m=peer_range)
    if io.tick%10==0:
        peer_seq+=1
        io.send(io.peer_tx,dict(role='peer',name=robot.getName(),seq=peer_seq,timestamp=now,pose=io.pose()))
    for message,distance in io.receive(io.state_rx):
        if message.get('role') != 'master':
            continue
        if scenario=='radio_loss' and io.index==1 and message.get('turn_id')==1 and blackout_start is None:
            blackout_start = now
        blackout = blackout_start is not None and blackout_start <= now < blackout_start+1.2
        if blackout or rng.random()<PACKET_LOSS:
            dropped += 1
            continue
        pending.append((now+RADIO_DELAY,message,distance))
    while pending and pending[0][0] <= now:
        _,message,distance = pending.pop(0)
        if message['seq'] <= last_seq or now-message['timestamp'] > RADIO_TIMEOUT:
            continue
        master,last_seq = message,message['seq']
        received += 1
        if distance is not None:
            range_m = distance if range_m is None else range_m+0.25*(distance-range_m)
    stale = master is None or now-master['timestamp']>RADIO_TIMEOUT
    if stale and not was_stale:
        timeout_count += 1
    was_stale = stale
    v,w = 0.0,0.0
    if not stale and now>1.5:
        pose = master['pose']
        # Known aligned starts define a common odometry frame. Each robot follows
        # a translated copy of the measured master trajectory.
        dx,dy = pose['x']-io.x,pose['y']-io.y
        ex = math.cos(io.heading)*dx+math.sin(io.heading)*dy
        ey = -math.sin(io.heading)*dx+math.cos(io.heading)*dy
        eh = pose['heading']-io.heading
        v = clip(pose['v']*math.cos(eh)+1.8*ex,-0.10,0.25)
        w = clip(pose['w']+2.8*eh+3.5*pose['v']*ey,-0.65,0.65)
        # Scalar distance corrects spacing weakly along the odometry-estimated
        # relative direction. No radio direction/bearing API is used.
        rx = OFFSETS[0][0]-OFFSETS[io.index][0]+dx
        ry = OFFSETS[0][1]-OFFSETS[io.index][1]+dy
        predicted = math.hypot(rx,ry)
        if range_m is not None and predicted>0.1 and master['kind']=='drive':
            error = clip(range_m-predicted,-0.05,0.05)
            v += 0.25*error*(rx*math.cos(io.heading)+ry*math.sin(io.heading))/predicted
        if master['kind'] in ('calibrate','finish') and math.hypot(dx,dy)<0.01:
            v = 0.0
            if abs(eh)<math.radians(0.2):
                w = 0.0
    io.drive(v,w)
    if io.tick % 10 == 0:
        io.send(io.feedback_tx,dict(role='follower',name=robot.getName(),timestamp=now,
            pose=io.pose(),last_master_seq=last_seq,
            master_age_s=None if master is None else now-master['timestamp'],
            stale=stale,received=received,dropped=dropped,timeouts=timeout_count,
            range_m=range_m,command_v=io.command_v,command_w=io.command_w,
            peer_links={name:dict(seq=entry['seq'],age_s=now-entry['timestamp'],fresh=now-entry['timestamp']<=RADIO_TIMEOUT,range_m=entry['range_m']) for name,entry in peer_received.items()},
            blackout=blackout_start is not None and blackout_start <= now < blackout_start+1.2))

