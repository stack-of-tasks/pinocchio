//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/joint/joint-derived.hpp"
#include "pinocchio/bindings/python/multibody/joint/joints-variant.hpp"
#include "pinocchio/bindings/python/multibody/joint/joint.hpp"

#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeJoints()
    {
      boost::mpl::for_each<JointModelVariant::types>(JointModelExposer());
      boost::mpl::for_each<JointDataVariant::types>(JointDataExposer());
      
      JointModelPythonVisitor::expose();
      StdAlignedVectorPythonVisitor<JointModel>::expose("StdVec_JointModelVector");
    }
    
  } // namespace python
} // namespace pinocchio
