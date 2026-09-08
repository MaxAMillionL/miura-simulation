"""Display-only MuJoCo model with genuine motor-speed sliders in rad/s."""
import copy
import mujoco
import numpy as np

class MotorSpeedView:
    def __init__(self, sim):
        self.sim=sim
        self.model=copy.copy(sim.model)
        self.data=mujoco.MjData(self.model)
        limit=sim.config['motor_speed_limit_rad_s']
        self.model.actuator_ctrllimited[:]=1
        self.model.actuator_ctrlrange[:]=[-limit,limit]
        # The display model is never stepped. Its sliders are velocity requests,
        # while the real model retains torque actuators and physical limits.
        self.model.actuator_gainprm[:]=0
        self.model.actuator_biasprm[:]=0
        self._shown=np.zeros(sim.model.nu)
        self.publish({})

    def edits(self):
        changed=np.isfinite(self.data.ctrl)&(~np.isclose(self.data.ctrl,self._shown,atol=1e-7,rtol=0))
        limit=self.sim.config['motor_speed_limit_rad_s']
        return {n:float(np.clip(self.data.ctrl[aid],-limit,limit))
                for n,aid in self.sim._ids.items() if changed[aid]}

    def publish(self, targets):
        source=self.sim.data
        self.data.time=source.time
        self.data.qpos[:]=source.qpos; self.data.qvel[:]=source.qvel
        self.data.act[:]=source.act
        self.data.mocap_pos[:]=source.mocap_pos; self.data.mocap_quat[:]=source.mocap_quat
        self.data.qfrc_applied[:]=0; self.data.xfrc_applied[:]=0
        self.data.ctrl[:]=0
        for n,v in targets.items(): self.data.ctrl[self.sim._ids[n]]=v
        self._shown[:]=self.data.ctrl
        mujoco.mj_forward(self.model,self.data)
