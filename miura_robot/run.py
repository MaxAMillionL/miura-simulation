"""Motor-only driving/folding, independent speed sliders, and telemetry."""
import argparse
import csv
import json
import math
import queue
import time
from pathlib import Path
from .control import Simulation, MotorCommand
from .motion import RobotMotion
from .speed_view import MotorSpeedView
from .build import ROOT


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer',action='store_true')
    parser.add_argument('--paused',action='store_true')
    parser.add_argument('--seconds',type=float,default=10)
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--motor',help='One motor in manual mode')
    modes.add_argument('--motors',nargs='+',metavar='NAME=RAD_S')
    modes.add_argument('--demo',action='store_true',help='Exercise each motor in sequence')
    modes.add_argument('--drive',action='store_true',help='Coordinated wheel/fold control (viewer default)')
    parser.add_argument('--speed',type=float,default=8,help='Manual motor target in rad/s')
    parser.add_argument('--forward',type=float,default=0,help='Drive request in m/s')
    parser.add_argument('--turn',type=float,default=0,help='Yaw request in rad/s')
    parser.add_argument('--fold',type=float,help='Fold target, 0 to 1.2 rad; drives motors, does not reset pose')
    parser.add_argument('--log',type=Path,default=ROOT/'output/telemetry.csv')
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<=0: parser.error('--seconds must be positive and finite')
    if args.paused and not args.viewer: parser.error('--paused requires --viewer')
    if args.fold is not None and (args.forward or args.turn): parser.error('--fold is separate from driving, which unfolds first')
    sim=Simulation(); motion=RobotMotion(sim)
    if abs(args.speed)>sim.config['motor_speed_limit_rad_s']: parser.error('--speed exceeds configured motor speed limit')
    if (args.motor or args.motors or args.demo) and (args.forward or args.turn or args.fold is not None):
        parser.error('Use --forward/--turn/--fold with coordinated drive mode')
    mode='manual' if args.motor or args.motors or args.demo else 'drive'
    manual={}; selected=0; keys=queue.SimpleQueue(); paused=args.paused
    try:
        if not all(math.isfinite(x) for x in [args.forward,args.turn,args.speed]): raise ValueError('Speeds must be finite')
        if args.motor: manual[args.motor]=args.speed
        for entry in args.motors or []:
            n,v=entry.split('=',1)
            if n in manual: raise ValueError('Duplicate motor: '+n)
            manual[n]=float(v)
        sim.command({n:MotorCommand('velocity',v) for n,v in manual.items()}); sim.stop()
        if manual: selected=sim.names.index(next(iter(manual)))
        if args.fold is not None: motion.set_fold(args.fold)
        if args.forward or args.turn: motion.drive(args.forward,args.turn)
    except (ValueError,KeyError) as error: parser.error(str(error))
    viewer=None; display=None
    if args.viewer:
        import mujoco.viewer
        display=MotorSpeedView(sim)
        viewer=mujoco.viewer.launch_passive(display.model,display.data,key_callback=keys.put)
        with viewer.lock():
            viewer.cam.lookat[:]=[0,0,.06]; viewer.cam.distance=.9; viewer.cam.azimuth=130; viewer.cam.elevation=-35
            viewer.opt.geomgroup[3]=False
        print(f'Assumed simulation motor torque limit: {sim.config["motor_torque_limit_nm"]:g} N m; real motor specs remain unknown.',flush=True)
        print('Tap arrows: drive/turn; C fold; O unfold; Space brake/hold; X coast; P pause; M manual; [ ] motor; +/- speed; R/F inspection resets. Sliders: motor rad/s.',flush=True)
    args.log.parent.mkdir(parents=True,exist_ok=True)
    max_error=0.; elapsed=0.; original_colors=sim.model.geom_rgba.copy()
    targets={}; linear=sim.config['drive_linear_speed_m_s']; yaw=sim.config['drive_yaw_speed_rad_s']
    try:
        with args.log.open('w',newline='') as stream:
            writer=csv.writer(stream)
            writer.writerow(['time','motor','target_rad_s','speed_rad_s','wheel_speed_rad_s','commanded_torque_nm','closure_error_m','fold_1_rad','body_x_m','body_y_m','mode'])
            while elapsed < args.seconds-1e-9 and (viewer is None or viewer.is_running()):
                start=time.monotonic()
                # Only speed-slider edits cross from the display into physics.
                if viewer:
                    with viewer.lock(): changes=display.edits()
                    if changes:
                        if mode!='manual': manual=dict(targets)
                        mode='manual'; args.demo=False; manual.update(changes)
                while not keys.empty():
                    code=keys.get(); name=sim.names[selected]
                    if code in (262,263,264,265):
                        mode='drive'; args.demo=False
                        motion.drive({265:linear,264:-linear}.get(code,0.),{263:yaw,262:-yaw}.get(code,0.))
                    elif code in (ord('C'),ord('O')):
                        mode='drive'; args.demo=False; motion.forward_m_s=motion.yaw_rad_s=0.
                        motion.set_fold(1.0 if code==ord('C') else 0.)
                    elif code==32:
                        if mode=='drive': motion.stop()
                        else: manual={n:0. for n in sim.names}; mode='manual'; args.demo=False
                    elif code==ord('X'): mode='coast'; args.demo=False
                    elif code==ord('M'): mode='manual'; manual={n:0. for n in sim.names}; args.demo=False
                    elif code==ord(']'): selected=(selected+1)%len(sim.names)
                    elif code==ord('['): selected=(selected-1)%len(sim.names)
                    elif code in (ord('='),ord('+'),ord('-')):
                        if mode!='manual': manual=dict(targets)
                        mode='manual'; args.demo=False
                        limit=sim.config['motor_speed_limit_rad_s']
                        manual[name]=max(-limit,min(limit,manual.get(name,0.)+(-1 if code==ord('-') else 1)))
                    elif code==ord('P'): paused=not paused
                    elif code in (ord('R'),ord('F')):
                        sim.reset(flat=code==ord('F')); motion=RobotMotion(sim)
                        manual={}; mode='drive'; args.demo=False; paused=True
                    print(f'{mode}, selected {sim.names[selected]}, fold target {motion.fold_target_rad:.2f} rad, paused={paused}',flush=True)
                if mode=='drive': commands=motion.targets(dt=0. if paused else .01)
                elif mode=='coast': commands={}
                else:
                    if args.demo:
                        selected=min(15,int(elapsed/(args.seconds/16)))
                        manual={sim.names[selected]:args.speed}
                    commands={n:MotorCommand('velocity',v) for n,v in manual.items()}
                targets={n:c.value for n,c in commands.items()}
                sim.stop()
                if not paused: sim.command(commands)
                state=sim.read() if paused else sim.step(.01)
                max_error=max(max_error,state['closure_error_m'])
                if not paused:
                    elapsed+=.01
                    body=sim.data.body('panel_0').xpos
                    for n,values in state['motors'].items():
                        writer.writerow([state['time'],n,targets.get(n,0.),values['velocity_rad_s'],values['wheel_velocity_rad_s'],values['commanded_torque_nm'],state['closure_error_m'],state['fold_rad'][0],body[0],body[1],mode])
                if viewer:
                    name=sim.names[selected]
                    with viewer.lock():
                        display.publish(targets)
                        display.model.geom_rgba[:]=original_colors
                        for part in ('_wheel','_motor_housing','_gear_visual','_idler_gear_visual','_output_gear_visual'):
                            display.model.geom(name+part).rgba[:]=[1,.85,.1,1]
                    viewer.set_texts((None,mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                        f'{"PAUSED" if paused else "RUNNING"} | {mode.upper()} | {name}\nTarget / actual motor speed (rad/s)\nFold target / actual (rad)\nALL right control sliders: +/-{sim.config["motor_speed_limit_rad_s"]:g} rad/s\nTap arrows drive/turn | C fold | O unfold\nSpace brake/hold | X coast | P pause\nM manual | [ ] select | +/- motor speed\nR folded / F flat: inspection resets',
                        f'\n{targets.get(name,0.):.2f} / {state["motors"][name]["velocity_rad_s"]:.2f}\n{motion.fold_target_rad:.2f} / {state["fold_rad"][0]:.2f}'))
                    viewer.sync(); time.sleep(max(0,.01-(time.monotonic()-start)))
    finally:
        sim.stop()
        if viewer: viewer.close()
    summary={'simulation_seconds':elapsed,'motors':len(sim.names),'max_closure_error_m':max_error,
             'final_fold_rad':sim.read()['fold_rad'],'warnings':sim.data.warning.number.tolist(),'log':str(args.log)}
    args.log.with_suffix('.summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))

if __name__=='__main__': main()
