import os
from pathlib import Path

import pinocchio as pin

model_path = Path(os.environ.get("EXAMPLE_ROBOT_DATA_MODEL_DIR"))
urdf_filename = "anymal.urdf"
urdf_model_path = model_path / "anymal_b_simple_description/robots" / urdf_filename
model = pin.buildModelFromUrdf(urdf_model_path)
data = model.createData()

q0 = pin.neutral(model)

# Add feet frames
feet_names = ["LH_FOOT", "RH_FOOT", "LF_FOOT", "RF_FOOT"]
feet_frame_ids = []
for foot_name in feet_names:
    frame_id = model.getFrameId(foot_name)
    feet_frame_ids.append(frame_id)

contact_models = []
for fid in feet_frame_ids:
    frame = model.frames[fid]
    cmodel = pin.RigidConstraintModel(
        pin.ContactType.CONTACT_3D,
        frame.parent,
        frame.placement,
        pin.LOCAL_WORLD_ALIGNED,
    )
    contact_models.append(cmodel)

contact_data = [cmodel.createData() for cmodel in contact_models]

pin.initConstraintDynamics(model, data, contact_models)
pin.crba(model, data, q0)

data.contact_chol.compute(model, data, contact_models, contact_data)

delassus_matrix = data.contact_chol.getInverseOperationalSpaceInertiaMatrix()
delassus_matrix_inv = data.contact_chol.getOperationalSpaceInertiaMatrix()
