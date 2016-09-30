//
// Copyright (c) 2015-2016 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/multibody/joint/joint-derived.hpp"
#include "pinocchio/bindings/python/multibody/joint/joints-variant.hpp"
#include "pinocchio/bindings/python/multibody/joint/joint.hpp"

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace se3
{
  namespace python
  {
    
    static void exposeVariants()
    {
      boost::mpl::for_each<JointModelVariant::types>(exposer());
      bp::to_python_converter<se3::JointModelVariant, jointModelVariantVisitor>();
    }
    
    void exposeJoints()
    {
      exposeVariants();
      JointModelPythonVisitor::expose();
      bp::class_<JointModelVector>("StdVec_JointModelVector")
      .def(bp::vector_indexing_suite<JointModelVector,true>());
    }
    
  } // namespace python
} // namespace se3
