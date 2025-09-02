//
// Copyright (c) 2025 INRIA
//

#include <boost/python.hpp>

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/parsers/graph/model-configuration-converter.hpp"

namespace pinocchio
{
  namespace python
  {
    void exposeModelConfigurationConverter()
    {
      graph::python::ModelConfigurationConverterVisitor<
        context::Scalar, context::Options, JointCollectionDefaultTpl>::expose();
    }
  } // namespace python
} // namespace pinocchio
