//
// Copyright (c) 2015-2016 CNRS
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/joint/joint-derived.hpp"
#include "pinocchio/bindings/python/multibody/joint/joints-variant.hpp"
#include "pinocchio/bindings/python/multibody/joint/joint.hpp"

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

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
      bp::class_<JointModelVector>("StdVec_JointModelVector")
      .def(bp::vector_indexing_suite<JointModelVector,true>());
    }
    
  } // namespace python
} // namespace pinocchio
