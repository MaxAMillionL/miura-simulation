import numpy as np,trimesh
from miura_robot.build import rotation,folded_angles
from itertools import combinations
m=trimesh.load_mesh('assets/mori-design-2.stl')
rays=[np.array([np.cos(t),np.sin(t),0]) for t in np.deg2rad([0,60,180,300])]
for normal2 in [False,True]:
 for gap in [0,-1,-2,1,2]:
  d=gap/2;length=150+4*d/np.sqrt(3);origin=np.array([5.94933362-d,225+np.sqrt(3)*d,6])
  a=np.array([0.,-length,0]);b=np.array([length*np.sqrt(3)/2,-length/2,0])
  origins=[origin,origin+b,origin+a,origin];edges=[(a,b),(-b,a),(b,-a) if normal2 else (-a,b),(a,b)]
  matrices=[]
  for p,(s1,s2) in enumerate(edges):
   src=np.column_stack((s1/length,s2/length,np.cross(s1,s2)/np.linalg.norm(np.cross(s1,s2))))
   dst=np.column_stack((rays[p],rays[(p+1)%4],np.cross(rays[p],rays[(p+1)%4])/np.linalg.norm(np.cross(rays[p],rays[(p+1)%4]))))
   matrices.append(dst@np.linalg.inv(src))
  for angle in [0,.55]:
   qs=folded_angles(angle);rots=[np.eye(3)]
   for p,q in enumerate(qs,1): rots.append(rots[-1]@rotation(rays[p],q))
   meshes=[]
   for p in range(4):
    mm=m.copy();mm.vertices=(mm.vertices-origins[p])@matrices[p].T@rots[p].T;meshes.append(mm)
   hits=[]
   for p,q in combinations(range(4),2):
    x=trimesh.boolean.intersection([meshes[p],meshes[q]],engine='manifold')
    if len(x.faces) and x.volume>1e-5:
     hits.append([p,q,round(float(x.volume),3)])
     if normal2 and gap==0: x.export(f'output/interlock-overlap-{angle}-{p}-{q}.stl')
   print('upright p2',normal2,'gap',gap,'fold',angle,'intersection mm3',hits,flush=True)
