//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/joint/joint-derived.hpp"
#include "pinocchio/bindings/python/multibody/joint/joints-variant.hpp"
#include "pinocchio/bindings/python/multibody/joint/joint-model.hpp"
#include "pinocchio/bindings/python/multibody/joint/joint-data.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    static void exposeJointVariants()
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
      exposeJointVariants();
      
      JointModelPythonVisitor::expose();
      StdAlignedVectorPythonVisitor<JointModel>::expose("StdVec_JointModelVector");
      
      JointDataPythonVisitor::expose();
      StdAlignedVectorPythonVisitor<JointData>::expose("StdVec_JointDataVector");
    }
    
  } // namespace python
} // namespace pinocchio
