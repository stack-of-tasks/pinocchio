import pinocchio as se3
from pinocchio.utils import *

model = se3.Model.BuildEmptyModel()
assert(model.nbody==1 and model.nq==0 and model.nv==0)

model = se3.Model.BuildHumanoidSimple()
nb=28 # We should have 28 bodies, thus 27 joints, one of them a free-flyer.
assert(model.nbody==nb and model.nq==nb-1+6 and model.nv==nb-1+5)
model.inertias[1] = model.inertias[2]
assert( isapprox(model.inertias[1].np,model.inertias[2].np) )
model.jointPlacements[1] = model.jointPlacements[2]
assert( isapprox(model.jointPlacements[1].np,model.jointPlacements[2].np) )
assert(model.parents[0]==0 and model.parents[1] == 0)
model.parents[2] = model.parents[1]
assert( model.parents[2] == model.parents[1] )
assert(model.names[0] == "universe" )
assert( isapprox(model.gravity.np,np.matrix('0; 0; -9.81; 0; 0; 0')) )

data = model.createData()


q = zero(model.nq)
qdot = zero(model.nv)
qddot = zero(model.nv)
for i in range(model.nbody): data.a[i] = se3.Motion.Zero()

se3.rnea(model,data,q,qdot,qddot)
for i in range(model.nbody):
    assert( isapprox(data.v[i].np,zero(6)) )
assert( isapprox(data.a[0].np,-model.gravity.np) )
assert( isapprox(data.f[-1],model.inertias[-1]*data.a[-1]) )
