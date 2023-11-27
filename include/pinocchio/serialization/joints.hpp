//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_serialization_joints_hpp__
#define __pinocchio_serialization_joints_hpp__

#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/multibody/joint/joint-composite.hpp"
#include "pinocchio/multibody/joint/joint-collection.hpp"

#include "pinocchio/serialization/fwd.hpp"
#include "pinocchio/serialization/vector.hpp"
#include "pinocchio/serialization/aligned-vector.hpp"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/variant.hpp>

#include "pinocchio/serialization/joints-transform.hpp"
#include "pinocchio/serialization/joints-motion.hpp"
#include "pinocchio/serialization/joints-constraint.hpp"
#include "pinocchio/serialization/joints-model.hpp"
#include "pinocchio/serialization/joints-data.hpp"

#endif // ifndef __pinocchio_serialization_joints_hpp__
