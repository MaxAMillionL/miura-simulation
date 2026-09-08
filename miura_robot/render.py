"""Render the corrected assembly and one panel for placement review."""
import mujoco
from PIL import Image
from .build import ROOT
from .control import Simulation


def render():
    sim=Simulation(); m,d=sim.model,sim.data
    with mujoco.Renderer(m,height=900,width=1200) as renderer:
        camera=mujoco.MjvCamera(); camera.lookat[:]=[0,0,.06]
        camera.distance=.8; camera.azimuth=125; camera.elevation=-50
        option=mujoco.MjvOption(); option.geomgroup[3]=False; option.geomgroup[4]=False
        renderer.update_scene(d,camera=camera,scene_option=option)
        Image.fromarray(renderer.render()).save(ROOT/'output/robot-preview.png')
        # Flat detail makes the alternating original profiles easy to inspect.
        sim.reset(flat=True)
        camera.lookat[:]=d.site('interlock_0_1_19.65').xpos
        camera.distance=.09; camera.azimuth=110; camera.elevation=-45
        renderer.update_scene(d,camera=camera,scene_option=option)
        Image.fromarray(renderer.render()).save(ROOT/'output/interlock-closeup.png')
        camera.lookat[:]=[0,0,d.xpos[m.body('panel_0').id,2]]
        camera.distance=.8; camera.azimuth=90; camera.elevation=-90
        renderer.update_scene(d,camera=camera,scene_option=option)
        Image.fromarray(renderer.render()).save(ROOT/'output/interlocked-flat.png')
        # Hide the other panels to inspect the motor -> idler -> outer wheel train.
        for gid in range(m.ngeom):
            bid=int(m.geom_bodyid[gid])
            while bid and not m.body(bid).name.startswith('panel_'):
                bid=int(m.body_parentid[bid])
            if bid and m.body(bid).name!='panel_0': m.geom_rgba[gid,3]=0
        camera.lookat[:]=d.xipos[m.body('panel_0').id]
        camera.distance=.43; camera.azimuth=90; camera.elevation=-90
        renderer.update_scene(d,camera=camera,scene_option=option)
        Image.fromarray(renderer.render()).save(ROOT/'output/panel-drivetrain.png')
    print(ROOT/'output/robot-preview.png')

if __name__=='__main__': render()
