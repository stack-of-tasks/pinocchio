import numpy as np
import pinocchio as pin
import pinocchio.cppad as ADpin
from pinocchio.utils import isapprox
from pycppad import AD, ADFun, Independent

pinmodel = pin.buildSampleModelHumanoidRandom()
model = ADpin.Model(pinmodel)
data = model.createData()

nq = model.nq
nv = model.nv

q = ADpin.neutral(model)
v = np.array([AD(1.0)] * nv)
a = np.zeros(nv, dtype=AD)

# declare independent variables and starting recording
Independent(v)

y = ADpin.rnea(model, data, q, v, a)

# create f: v -> y and stop tape recording
f = ADFun(v, y)

# first-order derivates wrt v
dv = np.ones(nv)
ADdtau_dv = f.Jacobian(dv).reshape(nv, nv)
(dtau_dq, dtau_dv, dtau_da) = pin.computeRNEADerivatives(
    pinmodel, pinmodel.createData(), pin.neutral(pinmodel), np.ones(nv), np.zeros(nv)
)
isapprox(ADdtau_dv, dtau_dv, 1e-12)
