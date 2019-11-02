from __future__ import print_function

import pinocchio

model = pinocchio.buildSampleModelManipulator()
data = model.createData()

q = pinocchio.neutral(model)
v = pinocchio.utils.zero(model.nv)
a = pinocchio.utils.zero(model.nv)

tau = pinocchio.rnea(model,data,q,v,a)
print('tau = ', tau.T)
