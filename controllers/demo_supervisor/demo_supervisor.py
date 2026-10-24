"""Read-only observer: radio listening, physical measurements, labels and reports."""
import json
import math
import os
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from controller import Supervisor
from robot_config import NAMES,OFFSETS,COLORS,RADIO_TIMEOUT
from robot_io import RobotIO
from path_trails import PathTrails
from communication_visuals import CommunicationOverlay

root = Path(__file__).resolve().parents[2]
runtime = root/'runtime'
runtime.mkdir(exist_ok=True)
supervisor = Supervisor()
step = int(supervisor.getBasicTimeStep())
nodes = [supervisor.getFromDef(f'ROBOT_{i}') for i in range(4)]
trails = PathTrails(supervisor)
communication = CommunicationOverlay(supervisor)
fade_snapshots=set()
master_rx = supervisor.getDevice('master_monitor')
feedback_rx = supervisor.getDevice('follower_monitor')
master_rx.enable(step)
feedback_rx.enable(step)
scenario = os.environ.get('SWARM_SCENARIO','nominal')
check = os.environ.get('FOUR_ROBOT_CHECK')=='1'
screenshots = os.environ.get('FOUR_ROBOT_SCREENSHOTS')=='1'
state = None
followers = {}
samples,turns = [],[]
active_turn = None
minimum_upright = 1.0
minimum_separation = 100.0
max_formation_error = 0.0
distance = [0.0]*4
headings = [0.0]*4
previous_yaw = [0.0]*4
previous_positions = None
done_at = None
report_written = False
tick = 0
failed = None
blackout_evidence = dict(stale_observed=False, stopped_observed=False, resumed_observed=False, max_master_turn_while_stale_deg=0.0)
stale_heading_start = None
late_blackout_end = None


def save_report():
    trails.export(runtime)
    if screenshots:
        supervisor.exportImage(str(runtime/'paths_preview.jpg'),95)
    expected = 1 if scenario=='stationary' else 8
    checks = dict(
        master_finished=state is not None and state['done'],
        all_turns_measured=len(turns)==expected,
        all_turns_within_2_degrees=len(turns)==expected and all(abs(v-t['target_deg'])<2.0 for t in turns for v in t['actual_deg'].values()),
        no_flips=minimum_upright>math.cos(math.radians(10)),
        no_robot_collisions=minimum_separation>0.40,
        formation_error_bounded=max_formation_error<(0.20 if scenario=='radio_loss' else 0.10),
        final_formation_error_bounded=formation_error<0.06,
        all_robots_inside_arena=all(abs(p[0])<5.8 and abs(p[1])<5.8 and abs(p[2])<0.08 for s in samples for p in s['positions']),
        fresh_master_messages_at_finish=len(followers)==3 and all(not s['stale'] and s['received']>20 for s in followers.values()),
        no_controller_timeout=failed is None,
        reference_triangle_links_fresh=len(communication.fresh_links)==5 and all(communication.fresh_links.values()),
        no_future_targets_in_master_packets=not any(k in state for k in ('targets','angle_deg','target_heading','label','waypoints','plan')))
    if scenario=='stationary':
        checks['master_turned_without_translation'] = len(turns)==1 and turns[0]['master_translation_m']<0.01
        checks['followers_detected_stationary_turn'] = len(turns)==1 and all(abs(v-90)<2 for v in turns[0]['actual_deg'].values())
    elif scenario=='nominal':
        checks['all_robots_kept_moving_through_every_turn'] = len(turns)==8 and all(min(t['minimum_speed_m_s'])>0.04 for t in turns)
        checks['all_robots_travelled'] = min(distance)>5
    else:
        checks['blackout_triggered_watchdog_stop_and_recovery'] = all(blackout_evidence[k] for k in ('stale_observed','stopped_observed','resumed_observed'))
        checks['master_continued_turning_during_blackout'] = blackout_evidence['max_master_turn_while_stale_deg']>5
    report = dict(passed=all(checks.values()),scenario=scenario,duration_s=supervisor.getTime(),
        checks=checks,failure=failed,completed_turns=turns,minimum_upright_z=minimum_upright,
        maximum_tilt_deg=math.degrees(math.acos(min(1,max(-1,minimum_upright)))),
        minimum_center_separation_m=minimum_separation,maximum_formation_error_m=max_formation_error,
        final_formation_error_m=formation_error,travel_m=dict(zip(NAMES,distance)),
        final_heading_deg=dict(zip(NAMES,[math.degrees(x) for x in headings])),
        follower_radio=followers,blackout_evidence=blackout_evidence,
        controller_sensors=['wheel encoders','calibrated noisy Z gyroscope','simulated scalar radio range'],
        known_start_geometry=True,supervisor_is_read_only=True,
        trail_point_counts=dict(zip(NAMES,[len(p) for p in trails.paths])),
        communication_links=communication.fresh_links,
        trail_fade_lifetime_s=12.0)
    (runtime/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    (runtime/'samples.json').write_text(json.dumps(samples,indent=2),encoding='utf-8')
    print('[Verification] '+('PASS' if report['passed'] else 'FAIL')+' '+scenario,flush=True)
    return report


while supervisor.step(step)!=-1:
    now = supervisor.getTime()
    for message,_ in RobotIO.receive(master_rx):
        if message.get('role')=='master':
            state = message
    for message,_ in RobotIO.receive(feedback_rx):
        if message.get('role')=='follower':
            followers[message['name']] = message
    positions = [n.getPosition() for n in nodes]
    if tick%5==0:
        trails.update(positions,now)
        communication.update(positions,state,followers,now)
    orientation = [n.getOrientation() for n in nodes]
    speed = [math.hypot(*n.getVelocity()[:2]) for n in nodes]
    for i,r in enumerate(orientation):
        yaw = math.atan2(r[3],r[0])
        headings[i] += math.atan2(math.sin(yaw-previous_yaw[i]),math.cos(yaw-previous_yaw[i]))
        previous_yaw[i] = yaw
    minimum_upright = min(minimum_upright,*(r[8] for r in orientation))
    minimum_separation = min(minimum_separation,*(math.dist(positions[i][:2],positions[j][:2]) for i in range(4) for j in range(i)))
    if previous_positions:
        for i in range(4):
            distance[i] += math.dist(positions[i][:2],previous_positions[i][:2])
    previous_positions = positions
    formation_error = max(math.hypot(
        positions[i][0]-positions[0][0]-(OFFSETS[i][0]-OFFSETS[0][0]),
        positions[i][1]-positions[0][1]-(OFFSETS[i][1]-OFFSETS[0][1])) for i in range(1,4))
    max_formation_error = max(max_formation_error,formation_error)
    if state:
        if active_turn is None and state['kind']=='turn':
            active_turn = dict(turn_id=state['turn_id'],initial_heading=list(headings),
                initial_position=list(positions[0]),min_speed=list(speed),start_s=now,
                phase_id=state['phase_id'])
        if active_turn:
            for i in range(4):
                active_turn['min_speed'][i] = min(active_turn['min_speed'][i],speed[i])
            if state['kind']!='turn' or state['turn_id']!=active_turn['turn_id']:
                # Only the observer reads the desired plan, after measurement.
                from mission import make_plan
                target = [p['angle_deg'] for p in make_plan(scenario=='stationary') if p['kind']=='turn'][active_turn['turn_id']-1]
                result = dict(turn_id=active_turn['turn_id'],target_deg=target,
                    actual_deg=dict(zip(NAMES,[math.degrees(a-b) for a,b in zip(headings,active_turn['initial_heading'])])),
                    minimum_speed_m_s=active_turn['min_speed'],
                    master_translation_m=math.dist(positions[0][:2],active_turn['initial_position'][:2]),time_s=now)
                turns.append(result)
                print('[Measured turn] '+json.dumps(result),flush=True)
                active_turn = None
        if state['done'] and done_at is None:
            done_at = now
    blue = followers.get(NAMES[1])
    if blue and blue['stale'] and blue['timeouts']>0:
        blackout_evidence['stale_observed']=True
        if stale_heading_start is None:
            stale_heading_start=headings[0]
        blackout_evidence['max_master_turn_while_stale_deg']=max(
            blackout_evidence['max_master_turn_while_stale_deg'],abs(math.degrees(headings[0]-stale_heading_start)))
        if abs(blue['command_v'])<0.005 and abs(blue['command_w'])<0.01 and speed[1]<0.01:
            blackout_evidence['stopped_observed']=True
    if blue and blackout_evidence['stopped_observed'] and not blue['stale'] and speed[1]>0.04:
        blackout_evidence['resumed_observed']=True
    if minimum_upright<math.cos(math.radians(20)):
        failed='Excessive tilt'
    if now>200:
        failed='Mission timeout'
    if tick%10==0 and not report_written:
        samples.append(dict(time_s=now,phase_id=None if state is None else state['phase_id'],
            kind=None if state is None else state['kind'],positions=positions,
            heading_deg=[math.degrees(h) for h in headings],speed_m_s=speed,formation_error_m=formation_error,
            master_seq=None if state is None else state['seq'],
            follower_stale={k:v['stale'] for k,v in followers.items()}))
    if tick%10==0:
        supervisor.setLabel(0,'MASTER + 3 FOLLOWERS / MOVING TURNS',0.025,0.025,0.058,0xEAF1FF,0,'Arial')
        label='Waiting for master radio'
        if state is not None:
            from mission import make_plan
            observer_plan=make_plan(scenario=='stationary')
            label=observer_plan[state['phase_id']]['label'] if 0 <= state['phase_id'] < len(observer_plan) else ('Gyro calibration' if state['kind']=='calibrate' else 'Master finished / followers settling')
        supervisor.setLabel(1,f'{label}\nMaster radio: measured gyro + encoder motion\nTime {now:5.1f} s | turns {len(turns)} | master speed {speed[0]:.3f} m/s\nFormation error {formation_error*100:.1f} cm | max tilt {math.degrees(math.acos(min(1,minimum_upright))):.2f} deg',0.025,0.10,0.039,0xB8C9DF,0,'Arial')
        for i,name in enumerate(NAMES):
            color=sum(int(c*255)<<s for c,s in zip(COLORS[i],(16,8,0)))
            suffix='MASTER' if i==0 else ('RADIO STALE / STOP' if followers.get(name,{}).get('stale',True) else 'following master radio')
            supervisor.setLabel(2+i,f'R{i+1}: {math.degrees(headings[i]):7.1f} deg | {suffix}',0.025,0.30+i*0.045,0.033,color,0,'Arial')
    if tick%10==0:
        supervisor.setLabel(6,'Trails fade over 12 s | Red / green triangles show live radio links',0.025,0.51,0.03,0xB8C9DF,0,'Arial')
    if screenshots and active_turn and now-active_turn['start_s']>1.0 and not active_turn.get('screenshot'):
        supervisor.exportImage(str(runtime/f"turn_{active_turn['turn_id']:02d}.jpg"),95)
        active_turn['screenshot']=True
    if screenshots and done_at is not None:
        for age,name in ((1.0,'fade_early'),(13.0,'fade_clear')):
            if now-done_at>=age and name not in fade_snapshots:
                supervisor.exportImage(str(runtime/(name+'.jpg')),95)
                fade_snapshots.add(name)
    hold_seconds=15 if screenshots else 7
    if not report_written and (failed or (done_at is not None and now-done_at>hold_seconds)):
        report=save_report()
        report_written=True
        if check:
            supervisor.simulationQuit(0 if report['passed'] else 1)
            break
    tick+=1

