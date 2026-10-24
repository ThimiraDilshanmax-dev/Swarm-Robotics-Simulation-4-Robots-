"""Launch the master/follower world, or verify it in isolated project copies."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parent


def find_webots(explicit):
    candidates=[explicit,os.environ.get('WEBOTS_EXECUTABLE'),shutil.which('webots')]
    for home in [os.environ.get('WEBOTS_HOME'),r'C:\Program Files\Webots','/usr/local/webots','/Applications/Webots.app']:
        if home:
            candidates.extend(str(Path(home)/suffix) for suffix in ['msys64/mingw64/bin/webots.exe','webots','Contents/MacOS/webots'])
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return str(Path(candidate).resolve())
    raise RuntimeError('Webots was not found. Use --webots PATH.')


def run_check(executable,scenario,environment,screenshots):
    project=ROOT/'runtime/check_projects'/scenario
    output=ROOT/'runtime/verification'/scenario
    output.mkdir(parents=True,exist_ok=True)
    for folder in ('controllers','protos','worlds'):
        shutil.copytree(ROOT/folder,project/folder,dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns('__pycache__','.*'))
    for name in ('robot_config.py','robot_io.py','mission.py','path_trails.py','communication_visuals.py'):
        shutil.copy2(ROOT/name,project/name)
    (project/'runtime').mkdir(exist_ok=True)
    (project/'runtime/report.json').unlink(missing_ok=True)
    environment=dict(environment,FOUR_ROBOT_CHECK='1',
                     FOUR_ROBOT_SCREENSHOTS='1' if screenshots else '0',SWARM_SCENARIO=scenario)
    command=[executable,'--batch','--mode=fast','--stdout','--stderr']
    command+=['--fullscreen'] if screenshots else ['--minimize','--no-rendering']
    command+=[str(project/'worlds/four_robots_multi_angle.wbt')]
    print('Checking '+scenario+'...',flush=True)
    with (output/'stdout.log').open('w',encoding='utf-8') as stdout,(output/'stderr.log').open('w',encoding='utf-8') as stderr:
        result=subprocess.run(command,env=environment,cwd=project,stdout=stdout,stderr=stderr,timeout=240)
    errors=(output/'stderr.log').read_text(encoding='utf-8')
    if 'ERROR:' in errors or 'Traceback (most recent call last)' in errors:
        raise RuntimeError(f'World/controller error. See {output/"stderr.log"}')
    report_path=project/'runtime/report.json'
    if not report_path.is_file():
        raise RuntimeError(f'No completed measurement report. See {output}')
    report=json.loads(report_path.read_text(encoding='utf-8'))
    for name in ('report.json','samples.json'):
        shutil.copy2(project/'runtime'/name,output/name)
    for name in ('paths.json','paths.svg','paths_preview.jpg','fade_early.jpg','fade_clear.jpg'):
        if (project/'runtime'/name).is_file():
            shutil.copy2(project/'runtime'/name,output/name)
    for image in (project/'runtime').glob('turn_*.jpg'):
        shutil.copy2(image,output/image.name)
    report['process_exit_code']=result.returncode
    report['passed']=report['passed'] and result.returncode==0
    print(scenario+': '+('PASS' if report['passed'] else 'FAIL')+' '+json.dumps(report['checks']),flush=True)
    return report


def audit_architecture():
    import ast
    paths=['controllers/stable_robot/stable_robot.py','robot_io.py']
    trees=[ast.parse((ROOT/p).read_text(encoding='utf-8-sig')) for p in paths]
    attrs={n.attr for t in trees for n in ast.walk(t) if isinstance(n,ast.Attribute)}
    calls={n.func.id for t in trees for n in ast.walk(t) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)}
    imports={n.module for t in trees for n in ast.walk(t) if isinstance(n,ast.ImportFrom)}
    world=(ROOT/'worlds/four_robots_multi_angle.wbt').read_text()
    proto=(ROOT/'protos/StableRobot.proto').read_text()
    observer=(ROOT/'controllers/demo_supervisor/demo_supervisor.py').read_text(encoding='utf-8-sig')
    audit=dict(
        follower_does_not_import_mission='mission' not in imports,
        follower_and_common_do_not_read_global_pose=not ({'getPosition','getRollPitchYaw','getEmitterDirection'} & attrs or {'Supervisor'} & calls),
        physical_robot_has_no_gps_or_absolute_yaw_sensor=all(s not in proto for s in ('GPS {','InertialUnit {','Compass {')),
        world_observer_has_no_transmitter='Emitter' not in world[world.index('name "Mission display and measurement"'):],
        observer_does_not_move_physical_bodies=all(s not in observer for s in ('setVelocity','setSFVec3f','setSFRotation','resetPhysics','setForce','addForce')))
    output=ROOT/'runtime/verification'
    output.mkdir(parents=True,exist_ok=True)
    (output/'architecture_audit.json').write_text(json.dumps(audit,indent=2))
    if not all(audit.values()):
        raise RuntimeError('Controller architecture audit failed: '+str(audit))
    return audit


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--scenario',choices=['all','nominal','radio_loss','stationary'])
    parser.add_argument('--screenshots',action='store_true')
    parser.add_argument('--webots')
    args=parser.parse_args()
    executable=find_webots(args.webots)
    environment=dict(os.environ)
    environment['PATH']=str(Path(sys.executable).parent)+os.pathsep+environment.get('PATH','')
    environment['PYTHONIOENCODING']='utf-8'
    if args.check:
        audit_architecture()
        scenarios=['nominal','radio_loss','stationary'] if args.scenario in (None,'all') else [args.scenario]
        reports={name:run_check(executable,name,environment,args.screenshots) for name in scenarios}
        summary=dict(passed=all(r['passed'] for r in reports.values()),scenarios=reports)
        (ROOT/'runtime/verification/summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        return 0 if summary['passed'] else 1
    environment.update(FOUR_ROBOT_CHECK='0',FOUR_ROBOT_SCREENSHOTS='0',
                       SWARM_SCENARIO=args.scenario if args.scenario not in (None,'all') else 'nominal')
    return subprocess.call([executable,'--mode=realtime',str(ROOT/'worlds/four_robots_multi_angle.wbt')],env=environment,cwd=ROOT)


if __name__=='__main__':
    try:
        sys.exit(main())
    except (RuntimeError,OSError,ValueError,subprocess.TimeoutExpired) as error:
        print(str(error),file=sys.stderr)
        sys.exit(1)

