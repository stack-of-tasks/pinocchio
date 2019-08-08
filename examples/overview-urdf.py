import pinocchio
from sys import argv

filename = "ur5.urdf" if len(argv)<2 else argv[1]
model    = pinocchio.buildModelFromUrdf(filename)
data     = model.createData()
q        = pinocchio.randomConfiguration(model)
print('q = ', q.T)

pinocchio.forwardKinematics(model,data,q)

for k in range(model.njoints):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}"
          .format( model.names[k], *data.oMi[k].translation.T.flat )))
