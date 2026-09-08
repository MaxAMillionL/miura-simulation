import numpy as np
import pytest
from miura_robot.control import Simulation
from miura_robot.motion import RobotMotion
from miura_robot.speed_view import MotorSpeedView


def test_speed_sliders_preserve_physics_and_use_motor_units():
    sim=Simulation(); display=MotorSpeedView(sim)
    assert np.all(display.model.actuator_ctrlrange==[-40,40])
    assert np.all(sim.model.actuator_ctrlrange==[-.08,.08])
    display.publish({'p0_m2':8.})
    assert display.edits()=={}
    display.data.ctrl[sim._ids['p0_m3']]=-12
    assert display.edits()=={'p0_m3':-12.}
    before=sim.data.qpos.copy()
    display.data.qpos[:]=7; display.data.xfrc_applied[:]=9
    display.publish({'p0_m3':-12.})
    np.testing.assert_array_equal(sim.data.qpos,before)
    np.testing.assert_array_equal(display.data.qpos,before)
    assert not np.any(sim.data.ctrl)
    assert not np.any(sim.data.xfrc_applied)
    assert not np.any(display.data.xfrc_applied)


def test_motor_driven_folding_and_unfolding():
    sim=Simulation(); motion=RobotMotion(sim)
    assert len(sim.fold_couplings)==8
    assert len(motion.wheels.names)==8
    maximum=0.
    for target in [1.,.1,1.]:
        motion.set_fold(target)
        before=sim.data.qpos.copy(); commands=motion.targets()
        np.testing.assert_array_equal(sim.data.qpos,before)
        assert set(commands)==set(sim.names)
        for _ in range(600):
            sim.command(motion.targets()); state=sim.step(.01)
            maximum=max(maximum,state['closure_error_m'])
        assert abs(state['fold_rad'][0]-target)<.12
    assert maximum<.001
    assert not np.any(sim.data.warning.number)
    assert not np.any(sim.data.qfrc_applied)
    assert not np.any(sim.data.xfrc_applied)
    motion.stop()
    assert motion.forward_m_s==motion.yaw_rad_s==0.
    with pytest.raises(ValueError):motion.set_fold(2.)


@pytest.mark.parametrize('forward,yaw',[(.04,0),(-.04,0),(0,.4),(0,-.4)])
def test_drive_and_turn_directions(forward,yaw):
    sim=Simulation(); motion=RobotMotion(sim)
    motion.set_fold(0.)
    for _ in range(400):sim.command(motion.targets());sim.step(.01)
    before=sim.data.body('panel_0').xpos.copy()
    r=sim.data.body('panel_0').xmat.reshape(3,3).copy()
    initial_yaw=np.arctan2(r[1,0],r[0,0])
    motion.drive(forward,yaw)
    maximum=0.
    for _ in range(300):
        sim.command(motion.targets());state=sim.step(.01)
        maximum=max(maximum,state['closure_error_m'])
    displacement=sim.data.body('panel_0').xpos-before
    if forward:
        assert (displacement@r[:,0])*np.sign(forward)>.002
    else:
        r=sim.data.body('panel_0').xmat.reshape(3,3)
        delta=(np.arctan2(r[1,0],r[0,0])-initial_yaw+np.pi)%(2*np.pi)-np.pi
        assert delta*np.sign(yaw)>.02
    assert maximum<.001
    assert not np.any(sim.data.warning.number)
