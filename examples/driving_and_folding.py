"""Run with .venv/bin/python examples/driving_and_folding.py after installation."""
from miura_robot import Simulation
from miura_robot.motion import RobotMotion

robot=Simulation()
motion=RobotMotion(robot)
try:
    motion.set_fold(1.)
    for _ in range(600):
        robot.command(motion.targets(.01))
        state=robot.step(.01)
    print('Folded:',state['fold_rad'])
    motion.drive(forward_m_s=.04)
    for _ in range(1000):
        robot.command(motion.targets(.01))
        state=robot.step(.01)
    print('After unfolding and driving:',robot.data.body('panel_0').xpos)
finally:
    robot.stop()
