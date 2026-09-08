import numpy as np
import pytest
from miura_robot import Simulation, MotorCommand, HardwareBridge

@pytest.fixture
def sim(): return Simulation()

def test_geometry_and_closed_loop(sim):
    assert len(sim.names)==sim.model.nu==16
    assert len(set(sim._ids.values()))==16
    assert sim.read()['closure_error_m'] < 1e-8
    # Each wheel spindle is tangent to its inferred rhombus edge plane.
    for n in sim.names: assert abs(sim.model.joint(n).axis[2]) < 1e-5

def test_each_motor_controls_only_its_actuator(sim):
    sim.model.opt.gravity[:]=0
    for name in sim.names:
        sim.reset()
        sim.set_torque(name,.002)
        sim.step(.001)
        ids=np.flatnonzero(sim.data.ctrl)
        assert ids.tolist()==[sim.model.actuator(name).id]
        assert sim.data.joint(name).qvel[0] > 0
        assert sim.read()['motors'][name]['commanded_torque_nm']==pytest.approx(.002)

def test_differential_commands_and_watchdog(sim):
    sim.command({'p0_m0':MotorCommand('velocity',8),'p3_m3':MotorCommand('torque',-.003)})
    sim.step(.01)
    assert sim.data.ctrl[sim.model.actuator('p0_m0').id]>0
    assert sim.data.ctrl[sim.model.actuator('p3_m3').id]<0
    sim.step(.30)
    assert np.all(sim.data.ctrl==0)

def test_invalid_batch_is_atomic(sim):
    with pytest.raises(ValueError):
        sim.command({'p0_m0':MotorCommand('velocity',2),'p0_m1':MotorCommand('torque',np.nan)})
    assert sim._commands=={}
    with pytest.raises(KeyError): sim.set_velocity('missing',0)
    with pytest.raises(ValueError): sim.set_velocity('p0_m0',41)
    with pytest.raises(ValueError): sim.step(.0001)

def test_dynamic_fold_remains_closed(sim):
    initial=sim.read()['fold_rad']
    errors=[]
    for i in range(200):
        sim.command({n:MotorCommand('velocity',6 if j%2 else -6) for j,n in enumerate(sim.names)})
        errors.append(sim.step(.01)['closure_error_m'])
    assert max(errors)<.001
    assert np.isfinite(sim.data.qpos).all()
    assert np.all(sim.data.warning.number==0)
    assert np.linalg.norm(np.array(sim.read()['fold_rad'])-initial)>.001

def test_hardware_bridge_and_watchdog(sim,monkeypatch):
    class Fake:
        stopped=0
        def send(self,commands): self.last=commands
        def read(self): return {'encoder':0}
        def stop(self): self.stopped+=1
    now=[0.]; monkeypatch.setattr('miura_robot.control.time.monotonic',lambda:now[0])
    wire=Fake(); robot=HardwareBridge(wire,sim.names,sim.config)
    with pytest.raises(RuntimeError): robot.set_velocity('p0_m0',1)
    robot.arm(); robot.set_velocity('p0_m0',2)
    assert wire.last['p0_m0']==MotorCommand('velocity',2)
    assert len(wire.last)==16
    now[0]=.2; robot.set_velocity('p0_m1',3)
    now[0]=.3; robot.tick()
    assert wire.last['p0_m0']==MotorCommand('torque',0)
    assert wire.last['p0_m1']==MotorCommand('velocity',3)
    robot.stop(); assert wire.stopped==2

def test_three_gear_transmission_and_edge_wheels(sim):
    sim.model.opt.gravity[:]=0
    for _ in range(100):
        sim.set_velocity('p0_m2',8)
        sim.step(.001)
    speeds=[float(sim.data.joint(n).qvel[0]) for n in ['p0_m2','p0_m2_idler','p0_m2_output']]
    assert speeds[0]==pytest.approx(8,abs=.1)
    assert speeds[1]==pytest.approx(-speeds[0],abs=.01)
    assert speeds[2]==pytest.approx(speeds[0],abs=.01)
    for name in sim.names:
        input_pos=sim.model.body(name+'_drive').pos
        output_pos=sim.model.body(name+'_output_drive').pos
        axis=sim.model.joint(name).axis
        assert np.linalg.norm(input_pos-output_pos)==pytest.approx(.022,abs=1e-8)
        assert abs(np.dot(input_pos-output_pos,axis))<1e-8
        motor_pos=sim.model.geom(name+'_motor_housing').pos
        assert np.linalg.norm(np.cross(motor_pos-input_pos,axis))<1e-8
        assert sim.model.geom(name+'_wheel').bodyid==sim.model.body(name+'_output_drive').id

def test_original_interlocks_align_without_added_parts(sim):
    import json
    from miura_robot.build import ROOT,layout,load_config
    manifest=json.loads((ROOT/'models/manifest.json').read_text())
    interlocks=manifest['interlock_sites']
    assert len(interlocks)==16
    for crease in range(4):
        sites=[v for v in interlocks if v['crease']==crease]
        panels=sorted({v['panel'] for v in sites})
        left=[v for v in sites if v['panel']==panels[0]]
        right=[v for v in sites if v['panel']==panels[1]]
        for a in left:
            assert min(np.linalg.norm(sim.data.site(a['site']).xpos-sim.data.site(b['site']).xpos) for b in right)<1e-7
    _,rotations,_,length=layout(load_config())
    assert length==150
    assert all(r[2,2]==pytest.approx(1) for r in rotations)
    assert not any(sim.model.geom(i).name.startswith('connector_') for i in range(sim.model.ngeom))
    assert sim.data.ncon==0
    sim.reset(flat=True)
    assert sim.read()['fold_rad']==[0.,0.,0.]
    assert sim.read()['closure_error_m']<1e-8
    for p in range(4):
        g=sim.model.geom(f'panel_collision_{p}')
        assert sim.model.geom_contype[g.id]==2<<p
        assert sim.model.geom_conaffinity[g.id] & (2<<((p+1)%4))
