"""Reproducible CAD decomposition and closed-loop MJCF generation."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]

def vec(x):
    return ' '.join(f'{v:.10g}' for v in np.asarray(x).ravel())

def quat(r):
    q = Rotation.from_matrix(r).as_quat()
    return vec(q[[3,0,1,2]])

def rotation(axis, angle):
    return Rotation.from_rotvec(np.asarray(axis)*angle).as_matrix()

def load_config(path=None):
    c = json.loads(Path(path or ROOT/'config.json').read_text())
    for key in ['units_to_meters','timestep','panel_mass_kg','motor_mass_kg',
                'wheel_mass_kg','gear_mass_kg','gear_ratio','gear_efficiency',
                'motor_torque_limit_nm','motor_speed_limit_rad_s','velocity_kp',
                'command_timeout_s','wheel_friction']:
        if not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if c['gear_efficiency'] > 1 or not 0.1 <= c['initial_fold_rad'] <= 1.2:
        raise ValueError('efficiency must be <=1; initial_fold_rad must be in [0.1,1.2]')
    return c

def build(config=None):
    c = load_config(config)
    out = ROOT/'models'; out.mkdir(exist_ok=True)
    gen = out/'meshes'; gen.mkdir(exist_ok=True)
    source = trimesh.load_mesh(ROOT/'assets/mori-design-2.stl')
    components = list(source.split(only_watertight=False))
    gears = [s for s in components if len(s.faces)==3536]
    if len(gears)!=4:
        raise ValueError('Expected four detached gear components in supplied CAD')
    gears.sort(key=lambda s: (round(s.centroid[0],1), s.centroid[1]))
    frame = trimesh.util.concatenate([s for s in components if len(s.faces)!=3536])
    # Gear-axis centers define an inferred rhombus. These are calibration points,
    # not dimensions guaranteed by STL metadata. Millimetres in original CAD.
    origin = np.array(c["cad_calibration"]["hinge_vertex_stl_units"])
    length = c["cad_calibration"]["edge_length_stl_units"]
    a=np.array([0.,-length,0]); b=np.array([length*np.sqrt(3)/2,-length/2,0])
    origins=[origin,origin+b,origin+a,origin]
    src_edges=[(a,b),(-b,a),(-a,b),(a,b)]
    rays=[np.array([np.cos(t),np.sin(t),0]) for t in np.deg2rad([0,60,180,300])]
    rotations=[]
    for p,(s1,s2) in enumerate(src_edges):
        r1,r2=rays[p],rays[(p+1)%4]
        src=np.column_stack((s1/length,s2/length,np.cross(s1,s2)/np.linalg.norm(np.cross(s1,s2))))
        dst=np.column_stack((r1,r2,np.cross(r1,r2)/np.linalg.norm(np.cross(r1,r2))))
        rotations.append(dst@np.linalg.inv(src))
    root=ET.Element('mujoco',model='four_panel_miura')
    ET.SubElement(root,'compiler',angle='radian',meshdir='meshes',autolimits='true')
    ET.SubElement(root,'option',timestep=str(c['timestep']),integrator='implicitfast',iterations='100',tolerance='1e-10')
    visual=ET.SubElement(root,'visual'); ET.SubElement(visual,'global',offwidth='1200',offheight='900')
    asset=ET.SubElement(root,'asset')
    def mesh(name,m):
        m.export(gen/(name+'.stl'))
        ET.SubElement(asset,'mesh',name=name,file=name+'.stl')
    def convert(m,offset,r):
        m=m.copy(); m.vertices=(m.vertices-offset)@r.T*c['units_to_meters']; return m
    wheel=trimesh.load_mesh(ROOT/'assets/wheel.stl'); wheel.vertices-=wheel.bounds.mean(axis=0)
    wheel.vertices*=c['units_to_meters']; mesh('wheel',wheel)
    motor=trimesh.load_mesh(ROOT/'assets/motor.stl'); motor.vertices-=motor.bounds.mean(axis=0)
    motor.vertices*=c['units_to_meters']; mesh('motor',motor)
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='0 -0.2 1',dir='0 0 -1',diffuse='.9 .9 .9')
    ET.SubElement(world,'geom',name='floor',type='plane',size='2 2 .1',rgba='.16 .19 .24 1',contype='1',conaffinity='2')
    ET.SubElement(world,'camera',name='overview',pos='.55 -.65 .65',xyaxes='.763 .646 0 -.431 .510 .744')
    base=ET.SubElement(world,'body',name='panel_0',pos='0 0 .15')
    if not c['fixed_base']: ET.SubElement(base,'freejoint',name='base')
    panels=[base]
    for p in range(1,4):
        body=ET.SubElement(panels[-1],'body',name=f'panel_{p}')
        ET.SubElement(body,'joint',name=f'fold_{p}',axis=vec(rays[p]),damping=str(c['hinge_damping']),armature='0.00001',limited='true',range='-2.3 2.3')
        panels.append(body)
    actuators=ET.SubElement(root,'actuator')
    colors=['.2 .65 .78 1','.95 .55 .22 1','.45 .73 .48 1','.66 .48 .83 1']
    manifest=[]
    for p,body in enumerate(panels):
        r=rotations[p]; originp=origins[p]
        m=convert(frame,originp,r); mesh(f'frame_{p}',m)
        ET.SubElement(body,'geom',name=f'panel_visual_{p}',type='mesh',mesh=f'frame_{p}',rgba=colors[p],mass='0',contype='0',conaffinity='0',group='1')
        # Explicit mass/inertia avoid treating open STL cavities as solid material.
        center=(m.bounds.mean(axis=0))
        ET.SubElement(body,'inertial',pos=vec(center),mass=str(c['panel_mass_kg']),diaginertia=vec([c['panel_mass_kg']*.1**2/12]*2+[c['panel_mass_kg']*.1**2/6]))
        # Four thin edge capsules: stable ground contact, no artificial filled holes.
        corners=[np.zeros(3),rays[p]*length*c['units_to_meters'],(rays[p]+rays[(p+1)%4])*length*c['units_to_meters'],rays[(p+1)%4]*length*c['units_to_meters']]
        for e in range(4):
            ET.SubElement(body,'geom',name=f'panel_collision_{p}_{e}',type='capsule',fromto=vec([corners[e],corners[(e+1)%4]]),size='.002',mass='0',rgba=colors[p],group='3',contype='2',conaffinity='1')
        for e,g in enumerate(gears):
            name=f'p{p}_m{e}'
            center=g.bounds.mean(axis=0)
            # Unique principal-inertia direction is the shaft axis. Orient signs
            # consistently, then keep the same calibration convention on hardware.
            _,v=np.linalg.eigh(g.moment_inertia); axis=v[:,0]
            if axis[np.argmax(abs(axis))]<0: axis=-axis
            axis=r@axis; axis/=np.linalg.norm(axis)
            pos=r@(center-originp)*c['units_to_meters']
            gm=convert(g,center,r); mesh(name+'_gear',gm)
            drive=ET.SubElement(body,'body',name=name+'_drive',pos=vec(pos))
            ET.SubElement(drive,'joint',name=name,axis=vec(axis),damping='.00001',armature='.000002',limited='false')
            ET.SubElement(drive,'geom',type='mesh',mesh=name+'_gear',rgba='.75 .77 .8 1',mass=str(c['gear_mass_kg']),contype='0',conaffinity='0')
            # STL wheel spindle is +Y; use a proper rotation to align with shaft.
            wr=Rotation.align_vectors([axis],[np.array([0.,1.,0.])])[0].as_matrix()
            wheelpos=axis*c['wheel_axial_offset_m']
            ET.SubElement(drive,'geom',name=name+'_wheel',type='mesh',mesh='wheel',pos=vec(wheelpos),quat=quat(wr),rgba='.12 .13 .15 1',mass='0',contype='0',conaffinity='0')
            cr=Rotation.align_vectors([axis],[np.array([0.,0.,1.])])[0].as_matrix()
            ET.SubElement(drive,'geom',name=name+'_tire',type='cylinder',pos=vec(wheelpos),quat=quat(cr),size='.008 .002',mass=str(c['wheel_mass_kg']),rgba='.12 .13 .15 0',group='3',contype='2',conaffinity='1',condim='4',friction=f'{c["wheel_friction"]} .002 .0001')
            # Rear mount centers inferred from the four small detached CAD pins.
            rear=np.array(c["cad_calibration"]["motor_rear_mounts_stl_units"])[e]
            motor_axis=np.array(c["cad_calibration"]["motor_rear_axes"])[e]
            motorpos=r@(rear-motor_axis*18.6-originp)*c['units_to_meters']
            mr=Rotation.align_vectors([r@motor_axis],[np.array([1.,0.,0.])])[0].as_matrix()
            ET.SubElement(body,'geom',name=name+'_motor_housing',type='mesh',mesh='motor',pos=vec(motorpos),quat=quat(mr),mass='0',rgba='.28 .3 .33 1',contype='0',conaffinity='0')
            housing=ET.SubElement(body,'body',pos=vec(motorpos))
            ET.SubElement(housing,'inertial',pos='0 0 0',mass=str(c['motor_mass_kg']),diaginertia='0.0000002 0.0000012 0.0000012')
            ET.SubElement(actuators,'motor',name=name,joint=name,gear=str(c['gear_ratio']*c['gear_efficiency']),ctrllimited='true',ctrlrange=f'-{c["motor_torque_limit_nm"]} {c["motor_torque_limit_nm"]}')
            manifest.append(dict(name=name,panel=p,edge=e,axis_panel=axis.tolist(),position_panel_m=pos.tolist(),gear_ratio=c['gear_ratio']))
    endpoint=rays[0]*length*c['units_to_meters']
    for p in [0,3]: ET.SubElement(panels[p],'site',name=f'closure_{p}',pos=vec(endpoint),size='.002',rgba='1 .1 .1 1')
    equality=ET.SubElement(root,'equality')
    ET.SubElement(equality,'connect',name='fourth_crease',site1='closure_0',site2='closure_3',solref='.005 1')
    # Three hinges meet at the central vertex. Matching a second point closes
    # the fourth crease, leaving the physical one-dimensional folding motion.
    q1=c['initial_fold_rad']
    def residual(x):
        rr=rotation(rays[1],q1)@rotation(rays[2],x[0])@rotation(rays[3],x[1])
        return rr@rays[0]-rays[0]
    sol=least_squares(residual,[-.9,-q1],xtol=1e-13,ftol=1e-13,gtol=1e-13)
    if np.linalg.norm(sol.fun)>1e-8: raise ValueError('Could not initialize closed fold')
    path=out/'robot.xml'; ET.indent(root); ET.ElementTree(root).write(path,encoding='unicode')
    metadata=dict(config=c,motors=manifest,initial_fold=[q1,*sol.x.tolist()],crease_rays=[x.tolist() for x in rays],inferred_edge_length_m=length*c['units_to_meters'])
    (out/'manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return path

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(); parser.add_argument('--config'); args=parser.parse_args()
    print(build(args.config))
