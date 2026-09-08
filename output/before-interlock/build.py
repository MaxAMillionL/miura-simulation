"""CAD decomposition, connector-based panel layout, and three-gear drives."""
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
    c=json.loads(Path(path or ROOT/'config.json').read_text())
    for key in ['units_to_meters','timestep','panel_mass_kg','motor_mass_kg',
                'wheel_mass_kg','gear_mass_kg','gear_ratio','gear_efficiency',
                'motor_torque_limit_nm','motor_speed_limit_rad_s','velocity_kp',
                'command_timeout_s','wheel_friction','panel_connector_gap_m']:
        if not math.isfinite(c[key]) or c[key]<=0:
            raise ValueError(f'{key} must be finite and positive')
    if c['gear_efficiency']>1 or not .1<=c['initial_fold_rad']<=1.2:
        raise ValueError('efficiency must be <=1; initial_fold_rad must be in [0.1,1.2]')
    return c

def layout(c):
    """Rigidly place unchanged CAD panels inside enlarged Miura facets.

    A gap moves the crease outboard of the real connector line, and short
    brackets join the actual connector ends to that line. No STL is stretched.
    """
    cal=c['cad_calibration']; scale=c['units_to_meters']
    halfgap=c['panel_connector_gap_m']/scale/2
    length=cal['connector_edge_length_stl_units']+4*halfgap/np.sqrt(3)
    origin=np.array(cal['connector_vertex_stl_units'])+[-halfgap,np.sqrt(3)*halfgap,0]
    a=np.array([0.,-length,0]); b=np.array([length*np.sqrt(3)/2,-length/2,0])
    origins=[origin,origin+b,origin+a,origin]
    src_edges=[(a,b),(-b,a),(-a,b),(a,b)]
    rays=[np.array([np.cos(t),np.sin(t),0]) for t in np.deg2rad([0,60,180,300])]
    matrices=[]
    for p,(s1,s2) in enumerate(src_edges):
        src=np.column_stack((s1/length,s2/length,np.cross(s1,s2)/np.linalg.norm(np.cross(s1,s2))))
        dst=np.column_stack((rays[p],rays[(p+1)%4],np.cross(rays[p],rays[(p+1)%4])/np.linalg.norm(np.cross(rays[p],rays[(p+1)%4]))))
        matrices.append(dst@np.linalg.inv(src))
    return origins,matrices,rays,length

def folded_angles(angle):
    rays=[np.array([np.cos(t),np.sin(t),0]) for t in np.deg2rad([0,60,180,300])]
    def residual(x):
        return rotation(rays[1],angle)@rotation(rays[2],x[0])@rotation(rays[3],x[1])@rays[0]-rays[0]
    sol=least_squares(residual,[angle*.5,angle],xtol=1e-13,ftol=1e-13,gtol=1e-13)
    if np.linalg.norm(sol.fun)>1e-8: raise ValueError('Could not initialize closed fold')
    return [angle,*sol.x.tolist()]

def cad_parts(c):
    source=trimesh.load_mesh(ROOT/'assets/mori-design-2.stl')
    parts=list(source.split(only_watertight=False))
    gears=sorted([s for s in parts if len(s.faces)==3536],key=lambda s:(round(s.centroid[0],1),s.centroid[1]))
    if len(gears)!=4: raise ValueError('Expected four detached input gears in supplied CAD')
    fixed=trimesh.util.concatenate([s for s in parts if len(s.faces)!=3536])
    axes=np.asarray(c['cad_calibration']['motor_rear_axes'],dtype=float)
    axes/=np.linalg.norm(axes,axis=1)[:,None]
    reference=next(g for g in gears if np.ptp(g.vertices[:,1])<10.01 and g.centroid[0]<50)
    # The input gear's axial centroid differs from the center of its 10 mm extent.
    offset=reference.bounds.mean(axis=0)[1]-reference.centroid[1]
    centers=[g.centroid+axis*offset for g,axis in zip(gears,axes)]
    pitch=c['cad_calibration']['gear_pitch_stl_units']
    cutters=[]
    for center,axis in zip(centers,axes):
        inward=np.array([axis[1],-axis[0],0.])
        # The other two gear crowns are fused into the exported frame STL.
        # Cut only their 10 mm axial band, retaining the support shafts/frame.
        basis=np.column_stack((inward,axis,[0,0,1]))
        cutter=trimesh.creation.box(extents=[2*pitch+12,10.002,12.002])
        cutter.vertices=cutter.vertices@basis.T+center-inward*(pitch+pitch/2)
        cutters.append(cutter)
    fixed=trimesh.boolean.difference([fixed,*cutters],engine='manifold')
    return fixed,gears,axes,centers

def build(config=None):
    c=load_config(config); scale=c['units_to_meters']; cal=c['cad_calibration']
    out=ROOT/'models'; out.mkdir(exist_ok=True); gen=out/'meshes'; gen.mkdir(exist_ok=True)
    frame,gears,axes,centers=cad_parts(c)
    origins,rotations,rays,length=layout(c)
    root=ET.Element('mujoco',model='four_panel_miura')
    ET.SubElement(root,'compiler',angle='radian',meshdir='meshes',autolimits='true')
    option=ET.SubElement(root,'option',timestep=str(c['timestep']),integrator='implicitfast',iterations='150',tolerance='1e-10')
    ET.SubElement(option,'flag',filterparent='disable')
    visual=ET.SubElement(root,'visual'); ET.SubElement(visual,'global',offwidth='1200',offheight='900')
    asset=ET.SubElement(root,'asset')
    def mesh(name,m):
        m.export(gen/(name+'.stl')); ET.SubElement(asset,'mesh',name=name,file=name+'.stl')
    def convert(m,offset,r):
        result=m.copy(); result.vertices=(result.vertices-offset)@r.T*scale; return result
    wheel=trimesh.load_mesh(ROOT/'assets/wheel.stl'); wheel.vertices-=wheel.bounds.mean(axis=0); wheel.vertices*=scale; mesh('wheel',wheel)
    motor=trimesh.load_mesh(ROOT/'assets/motor.stl'); motor.vertices-=motor.bounds.mean(axis=0); motor.vertices*=scale; mesh('motor',motor)
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='0 -0.2 1',dir='0 0 -1',diffuse='.9 .9 .9')
    ET.SubElement(world,'geom',name='floor',type='plane',size='2 2 .1',rgba='.16 .19 .24 1',contype='1',conaffinity='30')
    base=ET.SubElement(world,'body',name='panel_0',pos='0 0 .2')
    if not c['fixed_base']: ET.SubElement(base,'freejoint',name='base')
    panels=[base]
    for p in range(1,4):
        body=ET.SubElement(panels[-1],'body',name=f'panel_{p}')
        ET.SubElement(body,'joint',name=f'fold_{p}',axis=vec(rays[p]),damping=str(c['hinge_damping']),armature='.00001',limited='true',range='-1.5 1.5')
        panels.append(body)
    equality=ET.SubElement(root,'equality'); actuators=ET.SubElement(root,'actuator')
    colors=['.2 .65 .78 1','.95 .55 .22 1','.45 .73 .48 1','.66 .48 .83 1']
    manifest=[]; mounts=[]
    for p,body in enumerate(panels):
        r=rotations[p]; origin=origins[p]; bit=2<<p
        contact=dict(contype=str(bit),conaffinity=str(31^bit),friction=f'{c["wheel_friction"]} .002 .0001')
        m=convert(frame,origin,r); mesh(f'frame_{p}',m)
        ET.SubElement(body,'geom',name=f'panel_visual_{p}',type='mesh',mesh=f'frame_{p}',rgba=colors[p],mass='0',contype='0',conaffinity='0',group='1')
        ET.SubElement(body,'inertial',pos=vec(m.bounds.mean(axis=0)),mass=str(c['panel_mass_kg']),diaginertia=vec([c['panel_mass_kg']*.15**2/12]*2+[c['panel_mass_kg']*.15**2/6]))
        # Conservative convex CAD hull keeps neighboring panels from passing
        # through one another. Same-panel masks permit bearings and gear meshes.
        ET.SubElement(body,'geom',name=f'panel_collision_{p}',type='mesh',mesh=f'frame_{p}',mass='0',group='3',rgba='1 0 0 0',**contact)
        for e,(g,axis_src,center) in enumerate(zip(gears,axes,centers)):
            name=f'p{p}_m{e}'; inward=np.array([axis_src[1],-axis_src[0],0.])
            axis=r@axis_src; gear_positions=[]
            gm=convert(g,center,r); mesh(name+'_gear',gm)
            for stage,suffix in enumerate(['','_idler','_output']):
                source_pos=center-inward*cal['gear_pitch_stl_units']*stage
                pos=r@(source_pos-origin)*scale; gear_positions.append(pos)
                drive=ET.SubElement(body,'body',name=name+suffix+'_drive',pos=vec(pos))
                jointname=name+suffix
                ET.SubElement(drive,'joint',name=jointname,axis=vec(axis),damping='.000002',armature='.000002' if stage==0 else '.0000003',limited='false')
                ET.SubElement(drive,'geom',name=name+suffix+'_gear_visual',type='mesh',mesh=name+'_gear',rgba='.73 .76 .8 1',mass=str(c['gear_mass_kg']),contype='0',conaffinity='0')
                if stage:
                    ratio=-1 if stage==1 else 1/c['gear_ratio']
                    ET.SubElement(equality,'joint',name=name+suffix+'_transmission',joint1=jointname,joint2=name,polycoef=f'0 {ratio} 0 0 0',solref='.002 1')
                if stage==2:
                    wheelpos=axis*c['wheel_axial_offset_m']
                    wr=Rotation.align_vectors([axis],[np.array([0.,1.,0.])])[0].as_matrix()
                    ET.SubElement(drive,'geom',name=name+'_wheel',type='mesh',mesh='wheel',pos=vec(wheelpos),quat=quat(wr),rgba='.12 .13 .15 1',mass='0',contype='0',conaffinity='0')
                    cr=Rotation.align_vectors([axis],[np.array([0.,0.,1.])])[0].as_matrix()
                    ET.SubElement(drive,'geom',name=name+'_tire',type='cylinder',pos=vec(wheelpos),quat=quat(cr),size='.008 .002',mass=str(c['wheel_mass_kg']),rgba='.12 .13 .15 0',group='3',condim='4',**contact)
            # Center the motor on the input gear shaft inside the actual inner
            # rectangular slot; the former outer mount was a panel connector.
            motorpos=r@(center+axis_src*cal['motor_center_offset_stl_units']-origin)*scale
            angle=np.arctan2(axis_src[1],axis_src[0]); mr=r@rotation([0,0,1],angle)
            ET.SubElement(body,'geom',name=name+'_motor_housing',type='mesh',mesh='motor',pos=vec(motorpos),quat=quat(mr),mass='0',rgba='.28 .3 .33 1',contype='0',conaffinity='0')
            ET.SubElement(body,'geom',name=name+'_motor_collision',type='box',pos=vec(motorpos),quat=quat(mr),size='.0186 .006 .005',mass='0',rgba='0 0 0 0',group='3',**contact)
            housing=ET.SubElement(body,'body',pos=vec(motorpos))
            ET.SubElement(housing,'inertial',pos='0 0 0',mass=str(c['motor_mass_kg']),diaginertia='0.0000002 0.0000012 0.0000012')
            ET.SubElement(actuators,'motor',name=name,joint=name,gear=str(c['gear_efficiency']),ctrllimited='true',ctrlrange=f'-{c["motor_torque_limit_nm"]} {c["motor_torque_limit_nm"]}')
            # Brackets start on the outer connector tabs, not on a gear center.
            # Attach only the two edges facing another panel in this Miura cell.
            for station in cal['connector_stations_stl_units']:
                connector_src=center-inward*(2*cal['gear_pitch_stl_units'])+axis_src*station
                connector=r@(connector_src-origin)*scale
                normal=r@inward
                for crease in [p,(p+1)%4]:
                    ray=rays[crease]
                    foot=ray*np.dot(connector,ray)
                    delta=foot-connector
                    if np.linalg.norm(delta+c['panel_connector_gap_m']/2*normal) < 1e-6:
                        ET.SubElement(body,'geom',name=f'connector_{p}_{e}_{station:g}',type='capsule',fromto=vec([connector,foot]),size='.002',rgba='.28 .3 .33 1',mass='.001',contype='0',conaffinity='0')
                        ET.SubElement(body,'site',name=f'mount_{p}_{e}_{station:g}',pos=vec(connector),size='.002',rgba='.3 .3 .3 1',group='4')
                        # A short sleeve visibly identifies the physical attachment axis.
                        ET.SubElement(body,'geom',type='capsule',fromto=vec([foot-ray*.004,foot+ray*.004]),size='.003',rgba='.35 .36 .38 1',mass='.001',contype='0',conaffinity='0')
                        mounts.append(dict(panel=p,edge=e,crease=crease,tab_m=connector.tolist(),hinge_m=foot.tolist()))
            manifest.append(dict(name=name,panel=p,edge=e,axis_panel=axis.tolist(),position_panel_m=gear_positions[0].tolist(),gear_positions_panel_m=[v.tolist() for v in gear_positions],motor_center_panel_m=motorpos.tolist(),output_joint=name+'_output',gear_ratio=c['gear_ratio']))
    # One continuous axle joins both panels' connector sleeves along each crease.
    for crease,ray in enumerate(rays):
        stations=[np.dot(v['hinge_m'],ray) for v in mounts if v['crease']==crease]
        owner=panels[max(0,crease-1)]
        ET.SubElement(owner,'geom',name=f'connector_axle_{crease}',type='capsule',
            fromto=vec([ray*min(stations),ray*max(stations)]),size='.0015',
            rgba='.26 .28 .31 1',mass='.003',contype='0',conaffinity='0')
    endpoint=rays[0]*length*scale
    for p in [0,3]: ET.SubElement(panels[p],'site',name=f'closure_{p}',pos=vec(endpoint),size='.002',rgba='1 .1 .1 0',group='4')
    ET.SubElement(equality,'connect',name='fourth_crease',site1='closure_0',site2='closure_3',solref='.005 1')
    if len(mounts)!=16: raise ValueError(f'Expected two brackets on each of eight joined panel edges, got {len(mounts)}')
    path=out/'robot.xml'; ET.indent(root); ET.ElementTree(root).write(path,encoding='unicode')
    metadata=dict(config=c,motors=manifest,connector_mounts=mounts,initial_fold=folded_angles(c['initial_fold_rad']),crease_rays=[v.tolist() for v in rays],inferred_edge_length_m=length*scale)
    (out/'manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return path

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(); parser.add_argument('--config'); args=parser.parse_args()
    print(build(args.config))
