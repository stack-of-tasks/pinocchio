//
// Copyright (c) 2015-2021 CNRS INRIA
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

    void exposeJoints()
    {
      typedef context::JointCollectionDefault::JointModelVariant JointModelVariant;
      boost::mpl::for_each<JointModelVariant::types>(JointModelExposer());
      bp::to_python_converter<JointModelVariant, JointVariantVisitor<JointModelVariant>>();
      JointModelPythonVisitor<context::JointModel>::expose();
      StdAlignedVectorPythonVisitor<context::JointModel>::expose("StdVec_JointModelVector");

      typedef context::JointCollectionDefault::JointDataVariant JointDataVariant;
      boost::mpl::for_each<JointDataVariant::types>(JointDataExposer());
      bp::to_python_converter<JointDataVariant, JointVariantVisitor<JointDataVariant>>();
      JointDataPythonVisitor<context::JointData>::expose();
      StdAlignedVectorPythonVisitor<context::JointData>::expose("StdVec_JointDataVector");
    }

  } // namespace python
} // namespace pinocchio
