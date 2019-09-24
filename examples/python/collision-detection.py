##
## Copyright (c) 2018-2019 CNRS INRIA
##

import pinocchio as pin
pin.switchToNumpyMatrix()
from pinocchio.romeo_wrapper import RomeoWrapper

import os
current_path = os.getcwd()

# The model of Romeo is contained in the path PINOCCHIO_GIT_REPOSITORY/models/romeo
model_path = current_path + "/" + "../../models/romeo"
mesh_dir = model_path
urdf_filename = "romeo_small.urdf"
urdf_model_path = model_path + "/romeo_description/urdf/" + urdf_filename

# Load model
model = pin.buildModelFromUrdf(urdf_model_path,pin.JointModelFreeFlyer())

# Load collision geometries
geom_model = pin.buildGeomFromUrdf(model,urdf_model_path,model_path,pin.GeometryType.COLLISION)

# Add collisition pairs
geom_model.addAllCollisionPairs()

# Remove collision pairs listed in the SRDF file
srdf_filename = "romeo_small.srdf"
srdf_model_path = model_path + "/romeo_description/srdf/" + srdf_filename

pin.removeCollisionPairs(model,geom_model,srdf_model_path)

# Load reference configuration
pin.loadReferenceConfigurations(model,srdf_model_path)

# Retrieve the half sitting position from the SRDF file
q = model.referenceConfigurations["half_sitting"]

# Create data structures
data = model.createData()
geom_data = pin.GeometryData(geom_model)

# Compute all the collisions
pin.computeCollisions(model,data,geom_model,geom_data,q,False)

# Compute for a single pair of collision
pin.updateGeometryPlacements(model,data,geom_model,geom_data,q)
pin.computeCollision(geom_model,geom_data,0)

