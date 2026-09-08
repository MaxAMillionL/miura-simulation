"""High-level requests converted to motor commands; never edits robot pose."""
import math
import numpy as np
from .control import MotorCommand

class WheelDrive:
    """Project a requested body twist onto each wheel's rolling direction.

    This is a kinematic speed mixer, not a guarantee of chassis tracking. Actual
    motion depends on contact, friction, folding pose and available torque.
    """
    def __init__(self, sim, drive_motors=None):
        self.sim=sim
        self.names=tuple(drive_motors if drive_motors is not None else sim.names)
        self.radius=.008

    def targets(self, forward_m_s=0., yaw_rad_s=0., lateral_m_s=0.):
        values=[forward_m_s,yaw_rad_s,lateral_m_s]
        if not all(math.isfinite(x) for x in values): raise ValueError('Motion commands must be finite')
        s=self.sim
        orientation=s.data.xmat[s.model.body('panel_0').id].reshape(3,3)
        forward=orientation[:,0].copy(); forward[2]=0
        forward/=max(np.linalg.norm(forward),1e-8)
        left=np.cross([0.,0.,1.],forward)
        velocity=forward_m_s*forward+lateral_m_s*left
        center=s.data.subtree_com[s.model.body('panel_0').id]
        speeds={}
        for name in self.names:
            joint=s.model.joint(name+'_output').id
            axis=s.data.xaxis[joint]
            rolling=np.cross(axis,[0.,0.,1.]); norm=np.linalg.norm(rolling)
            if norm<1e-6: speeds[name]=0.; continue
            rolling/=norm
            lever=s.data.geom(name+'_tire').xpos-center
            local_velocity=velocity+yaw_rad_s*np.cross([0.,0.,1.],lever)
            speeds[name]=float(np.dot(local_velocity,rolling)/self.radius*s.config['gear_ratio'])
        largest=max([abs(v) for v in speeds.values()]+[0.])
        scale=min(1.,s.config['motor_speed_limit_rad_s']/max(largest,1e-9))
        return {n:MotorCommand('velocity',v*scale) for n,v in speeds.items()}

class RobotMotion:
    """Drive perimeter wheels and coordinate the shafts attached to fold creases.

    Eight shared-edge shafts turn wheels AND mating connectors. Their speeds
    are constrained by the one-DOF fold, so locomotion uses the other eight.
    All outputs pass through Simulation's ordinary motor torque controller.
    """
    def __init__(self, sim):
        self.sim=sim
        coupled={c['motor'] for c in sim.fold_couplings}
        self.wheels=WheelDrive(sim,[n for n in sim.names if n not in coupled])
        self.forward_m_s=0.; self.yaw_rad_s=0.
        self._integral={n:0. for n in coupled}
        self.set_fold(float(sim.data.joint('fold_1').qpos[0]))

    def set_fold(self, angle):
        from .build import folded_angles
        if not math.isfinite(angle) or not 0 <= angle <= 1.2:
            raise ValueError('Fold target must be in [0, 1.2] radians')
        self.fold_target_rad=float(angle)
        self._fold_angles=folded_angles(angle)

    def drive(self, forward_m_s=0., yaw_rad_s=0.):
        """Unfold using motors to put the perimeter wheels on the floor."""
        if not all(math.isfinite(v) for v in [forward_m_s,yaw_rad_s]):
            raise ValueError('Drive speeds must be finite')
        self.forward_m_s=forward_m_s; self.yaw_rad_s=yaw_rad_s
        if forward_m_s or yaw_rad_s: self.set_fold(0.)

    def stop(self):
        """Brake perimeter wheels and hold the current fold with motor torque."""
        self.forward_m_s=self.yaw_rad_s=0.
        self.set_fold(float(np.clip(self.sim.data.joint('fold_1').qpos[0],0,1.2)))

    def targets(self, dt=.01):
        if not math.isfinite(dt) or dt<0: raise ValueError("dt must be finite and nonnegative")
        s=self.sim
        result=self.wheels.targets(self.forward_m_s,self.yaw_rad_s)
        for coupling in s.fold_couplings:
            name=coupling['motor']; index=int(coupling['fold_joint'][-1])-1
            desired=coupling['sign']*self._fold_angles[index]*s.config['gear_ratio']
            error=desired-float(s.data.joint(name).qpos[0])
            integral_limit=s.config['fold_integral_limit_rad_s']
            self._integral[name]=float(np.clip(self._integral[name]+error*dt,-integral_limit,integral_limit))
            speed=float(np.clip(s.config['fold_position_gain']*error+s.config['fold_integral_gain']*self._integral[name],
                -s.config['fold_speed_limit_rad_s'],s.config['fold_speed_limit_rad_s']))
            result[name]=MotorCommand('velocity',speed)
        return result
