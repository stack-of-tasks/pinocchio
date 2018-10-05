import pinocchio

model = pinocchio.Model.BuildHumanoidSimple()
data = model.createData()

q = pinocchio.neutral(model)
v = pinocchio.utils.zero(model.nv)
a = pinocchio.utils.zero(model.nv)

rnea = pinocchio.rnea(model,data,q,v,a)
print(rnea.T)
