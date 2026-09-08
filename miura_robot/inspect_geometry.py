"""Check actual CAD-solid overlap across panels, independent of collision proxies."""
import json
from itertools import combinations
import mujoco
import numpy as np
import trimesh
from .control import Simulation
from .build import ROOT, folded_angles
from scipy.spatial.transform import Rotation


def panel_solids(sim):
    groups={p:[] for p in range(4)}
    m,d=sim.model,sim.data
    for gid in range(m.ngeom):
        name=m.geom(gid).name
        if m.geom_type[gid]!=mujoco.mjtGeom.mjGEOM_MESH or m.geom_group[gid]==3:
            continue
        bid=int(m.geom_bodyid[gid])
        while bid and not m.body(bid).name.startswith('panel_'):
            bid=int(m.body_parentid[bid])
        if not bid: continue
        panel=int(m.body(bid).name.split('_')[1])
        mid=m.geom_dataid[gid]
        mesh=trimesh.load_mesh(ROOT/'models/meshes'/f'{m.mesh(mid).name}.obj',process=False)
        # Assets retain double precision. Undo the compiler's mesh frame before
        # applying the current geom pose, including output-shaft rotation.
        q=m.mesh_quat[mid]; rotation=Rotation.from_quat([q[1],q[2],q[3],q[0]]).as_matrix()
        vertices=(mesh.vertices-m.mesh_pos[mid])@rotation
        vertices=vertices@d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid]
        mesh.vertices=(vertices-d.xpos[bid])@d.xmat[bid].reshape(3,3)
        groups[panel].append(mesh)
    return {p:trimesh.boolean.union(parts,engine='manifold') for p,parts in groups.items()}


def check_geometry():
    sim=Simulation(); samples=[]
    metadata=json.loads((ROOT/'models/manifest.json').read_text())
    interlocks=metadata['interlock_sites']
    for angle in [0,.3,.55,.9,1.2]:
        sim.reset(fold_angle=angle)
        solids=panel_solids(sim)
        posed=[]
        for p in range(4):
            mesh=solids[p].copy(); bid=sim.model.body(f'panel_{p}').id
            mesh.vertices=mesh.vertices@sim.data.xmat[bid].reshape(3,3).T+sim.data.xpos[bid]
            posed.append(mesh)
        pairs=[]
        for p,q in combinations(range(4),2):
            intersection=trimesh.boolean.intersection([posed[p],posed[q]],engine='manifold')
            volume=0. if len(intersection.faces)==0 else max(0.,float(intersection.volume)*1e9)
            # Existing mating profiles overlap in outline and can intersect
            # under ideal revolute motion. Report this explicitly and distinguish
            # it from unintended overlap elsewhere in the assembly.
            allowances=[]
            for crease in range(4):
                sites=[v for v in interlocks if v['crease']==crease]
                if {v['panel'] for v in sites}!={p,q}: continue
                for site in [v for v in sites if v['panel']==p]:
                    bid=sim.model.body(f'panel_{p}').id
                    axis=sim.data.xmat[bid].reshape(3,3)@np.array(site['axis'])
                    rr=Rotation.align_vectors([axis],[[0,0,1]])[0].as_matrix()
                    region=trimesh.creation.cylinder(radius=.0063,height=.004,sections=64)
                    region.vertices=region.vertices@rr.T+sim.data.site(site['site']).xpos
                    allowances.append(region)
                # The CAD also has two circular bearing seats on this same
                # axis, 27 and 123 mm from the rhombus vertex. Faceted circles
                # produce tiny slivers when rotated against one another.
                bid=sim.model.body(f'panel_{p}').id
                ray=np.array(metadata['crease_rays'][crease])
                axis=sim.data.xmat[bid].reshape(3,3)@ray
                rr=Rotation.align_vectors([axis],[[0,0,1]])[0].as_matrix()
                for station in [.027,.123]:
                    region=trimesh.creation.cylinder(radius=.0051,height=.0032,sections=64)
                    region.vertices=region.vertices@rr.T+sim.data.xpos[bid]+axis*station
                    allowances.append(region)
            remaining=intersection
            if volume>.000001 and allowances:
                remaining=trimesh.boolean.difference([intersection,*allowances],engine='manifold')
            outside=0. if len(remaining.faces)==0 else max(0.,float(remaining.volume)*1e9)
            if outside>.001:
                remaining.export(ROOT/f'output/outside-interlock-{angle}-{p}-{q}.stl')
            pairs.append(dict(panels=[p,q],total_overlap_mm3=volume,
                interlock_region_overlap_mm3=max(0.,volume-outside),outside_interlock_overlap_mm3=outside))
        samples.append(dict(fold_rad=angle,pairs=pairs))
        print(f'fold={angle:.2f}: max outside-interlock overlap={max(x["outside_interlock_overlap_mm3"] for x in pairs):.6f} mm^3; total profile overlap={sum(x["interlock_region_overlap_mm3"] for x in pairs):.3f} mm^3',flush=True)
    report=dict(checked='Actual frame, motor, gear and wheel solids. Existing mating profiles and circular bearing seats are reported separately; no added connector geometry.',
        interlock_region_radius_mm=6.3,interlock_region_axial_half_length_mm=2,bearing_region_radius_mm=5.1,bearing_region_axial_half_length_mm=1.6,
        limitation='Interlocks use ideal revolute joints, not resolved tooth contact or elastic deformation.',samples=samples)
    (ROOT/'output/geometry-check.json').write_text(json.dumps(report,indent=2)+'\n')
    if any(pair['outside_interlock_overlap_mm3']>.001 for sample in samples for pair in sample['pairs']):
        raise RuntimeError('CAD solids intersect outside the original mating profiles')
    return report

if __name__=='__main__': check_geometry()
