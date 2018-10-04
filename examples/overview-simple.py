import pinocchio

model = pinocchio.Model.BuildHumanoidSimple()
data = model.createData()

q = pinocchio.randomConfiguration(model)
v = pinocchio.utils.rand(model.nv)
a = pinocchio.utils.rand(model.nv)

pinocchio.rnea(model,data,q,v,a)

