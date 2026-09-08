import numpy as np, trimesh
from miura_robot.build import rotation
from scipy.optimize import least_squares
m=trimesh.load_mesh('assets/mori-design-2.stl')
rays=[np.array([np.cos(t),np.sin(t),0]) for t in np.deg2rad([0,60,180,300])]
for length in [150,158,166,174]:
 # Symmetric outward crease shift d = (L-150)*sqrt(3)/4.
 # Keeps all panels rigid. Crease lines offset outward from nominal connectors.
 d=(length-150)*np.sqrt(3)/4
 origin=np.array([5.94933362-d,225+np.sqrt(3)*d,6])
 a=np.array([0.,-length,0]); b=np.array([length*np.sqrt(3)/2,-length/2,0])
 origins=[origin,origin+b,origin+a,origin]; edges=[(a,b),(-b,a),(-a,b),(a,b)]
 rs=[]
 for p,(s1,s2) in enumerate(edges):
  src=np.column_stack((s1/length,s2/length,np.cross(s1,s2)/np.linalg.norm(np.cross(s1,s2))))
  dst=np.column_stack((rays[p],rays[(p+1)%4],np.cross(rays[p],rays[(p+1)%4])/np.linalg.norm(np.cross(rays[p],rays[(p+1)%4]))))
  rs.append(dst@np.linalg.inv(src))
 for q1 in [0,.55,1.2]:
  sol=least_squares(lambda x:rotation(rays[1],q1)@rotation(rays[2],x[0])@rotation(rays[3],x[1])@rays[0]-rays[0],[q1*.5,q1],gtol=1e-12)
  fold=[np.eye(3),rotation(rays[1],q1)];fold.append(fold[-1]@rotation(rays[2],sol.x[0]));fold.append(fold[-1]@rotation(rays[3],sol.x[1]))
  meshes=[]
  for p in range(4):
   x=m.copy(); x.vertices=(x.vertices-origins[p])@rs[p].T@fold[p].T;meshes.append(x)
  vals=[]
  for p in range(4):
   for q in range(p+1,4):
    it=trimesh.boolean.intersection([meshes[p],meshes[q]],engine='manifold')
    if it.volume>1e-4: vals.append((p,q,round(it.volume,3)))
  print('length',length,'gap from nominal',round(d*2,3),'fold',q1,'intersection mm3',vals,flush=True)
