"""Run from the project root: .venv/bin/python examples/individual_control.py"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from miura_robot import Simulation, MotorCommand

robot=Simulation()
try:
    for _ in range(500):
        robot.command({
            'p0_m0':MotorCommand('velocity',8.0),
            'p0_m1':MotorCommand('velocity',-4.0),
            'p2_m3':MotorCommand('torque',0.002),
        })
        state=robot.step(.01)
    print(state)
finally:
    robot.stop()
