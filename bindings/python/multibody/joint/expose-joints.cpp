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
    
    static void exposeVariants()
    {
      boost::mpl::for_each<JointModelVariant::types>(JointModelExposer());
      bp::to_python_converter<pinocchio::JointModelVariant,
                              JointVariantVisitor<pinocchio::JointModelVariant > >();

      boost::mpl::for_each<JointDataVariant::types>(JointDataExposer());
      bp::to_python_converter<pinocchio::JointDataVariant,
                              JointVariantVisitor<pinocchio::JointDataVariant > >();
    }
    
    void exposeJoints()
    {
      exposeVariants();
      JointModelPythonVisitor::expose();
      StdAlignedVectorPythonVisitor<JointModelVector,true>::expose("StdVec_JointModelVector");
    }
    
  } // namespace python
} // namespace pinocchio
