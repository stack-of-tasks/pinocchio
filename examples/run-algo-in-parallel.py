import pinocchio as pin
import numpy as np

model = pin.buildSampleModelHumanoid()
model.lowerPositionLimit[:7] = -np.ones(7)
model.upperPositionLimit[:7] = +np.ones(7)

pool = pin.ModelPool(model)

num_threads = pin.omp_get_max_threads()
batch_size = 128
q = np.empty((model.nq, batch_size))
for k in range(batch_size):
    q[:, k] = pin.randomConfiguration(model)

v = np.zeros((model.nv, batch_size))
a = np.zeros((model.nv, batch_size))
tau = np.zeros((model.nv, batch_size))

print("num_threads: {}".format(num_threads))
print("batch_size: {}".format(batch_size))

# Call RNEA
res_rnea = np.empty((model.nv, batch_size))
pin.rneaInParallel(num_threads, pool, q, v, a, res_rnea)  # Without allocation
res_rnea2 = pin.rneaInParallel(num_threads, pool, q, v, a)  # With allocation

# Call ABA
res_aba = np.empty((model.nv, batch_size))
pin.abaInParallel(num_threads, pool, q, v, tau, res_aba)  # Without allocation
res_aba2 = pin.abaInParallel(num_threads, pool, q, v, tau)  # With allocation
