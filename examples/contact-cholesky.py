import pinocchio as pin

from os.path import join, dirname, abspath

pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")
urdf_filename = (
    pinocchio_model_dir
    + "/example-robot-data/robots/anymal_b_simple_description/robots/anymal.urdf"
)
model = pin.buildModelFromUrdf(urdf_filename)
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
