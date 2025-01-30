from pathlib import Path

import numpy as np
import pinocchio

# Get URDF model
pinocchio_model_dir = Path(__file__).parent.parent / "models"
model_path = pinocchio_model_dir / "baxter_simple.urdf"
model_full = pinocchio.buildModelFromUrdf(model_path)

# To use the mimic tag from URDF set mimic flag to true
# (otherwise the tag will simply be ignored)
model_mimic_from_urdf = pinocchio.buildModelFromUrdf(model_path, mimic=True)

print(f"{model_full.nq=}")
print(f"{model_mimic_from_urdf.nq=}")

# Alternatively a "mimic" model can also be made by-hand
model_mimic = pinocchio.transformJointIntoMimic(model_full, 9, 10, -1.0, 0.0)
model_mimic = pinocchio.transformJointIntoMimic(model_mimic, 18, 19, -1.0, 0.0)

print(f"{(model_mimic_from_urdf == model_mimic)=}")  # True

# Creating manually the G matrix (cf. Featherstone's Rigid Body Dynamics Algorithms,
# chapter 10 about Gears if you want to know
# more about how joints mimic are handled here)
G = np.zeros([model_mimic.nv, model_full.nv])
for i in range(model_full.njoints):
    mimic_scaling = getattr(model_mimic.joints[i].extract(), "scaling", None)
    if mimic_scaling:
        G[model_mimic.joints[i].idx_v, model_full.joints[i].idx_v] = mimic_scaling
    else:
        # Not a mimic joint
        G[model_mimic.joints[i].idx_v, model_full.joints[i].idx_v] = 1
print("G = ")
print(np.array_str(G, precision=0, suppress_small=True, max_line_width=80))

# Random data
q_mimic = pinocchio.neutral(model_mimic)
v_mimic = np.random.random(model_mimic.nv)
a_mimic = np.random.random(model_mimic.nv)

# Expand configuration using the G matrix for the full model
# Possible because offset is 0, otherwise
# you need to add it to q of joints mimic
q_full = G.transpose() @ q_mimic

v_full = G.transpose() @ v_mimic
a_full = G.transpose() @ a_mimic

# Supported algorithms works as-is
data_mimic = model_mimic.createData()
data_full = model_full.createData()

tau_mimic = pinocchio.rnea(model_mimic, data_mimic, q_mimic, v_mimic, a_mimic)
tau_full = pinocchio.rnea(model_full, data_full, q_full, v_full, a_full)
print(f"{np.allclose(tau_mimic, G @ tau_full)=}")

M_mimic = pinocchio.crba(model_mimic, data_mimic, q_mimic)
M_full = pinocchio.crba(model_full, data_full, q_full)
print(f"{np.allclose(M_mimic, G @ M_full @ G.transpose())=}")

C_mimic = pinocchio.nle(model_mimic, data_mimic, q_mimic, v_mimic)
# For forward dynamics, aba does not support joints mimic.
# However it's still possible to compute acceleration of the
# mimic model, using the equation of motion tau = M*a + C
a_computed = np.linalg.solve(M_mimic, tau_mimic - C_mimic)
print(f"{np.allclose(a_mimic,a_computed)=}")
