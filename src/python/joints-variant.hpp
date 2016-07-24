//
// Copyright (c) 2015 CNRS
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

#ifndef __se3_python_joints_variant_hpp__
#define __se3_python_joints_variant_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/multibody/joint/joint-variant.hpp"
#include "pinocchio/python/joints-models.hpp"



namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    struct jointModelVariantVisitor : boost::static_visitor<PyObject *>
    {
      static result_type convert(JointModelVariant const &jv)
      {
        return apply_visitor(jointModelVariantVisitor(), jv);
      }

      template<typename T>
      result_type operator()(T const &t) const
      {
        return boost::python::incref(boost::python::object(t).ptr());
      }
    };

    
    struct exposer {
      template<class T> inline void operator()(T)
      {
        expose_constructors<T>(bp::class_<T>(T::classname().c_str(),bp::init<>()).def(JointPythonVisitor<T>()));
        bp::implicitly_convertible<T,se3::JointModelVariant>();
      }

      
    };

    // For the moment, only expose models of joint. Not data ( to do it, split exposer into exposerModels & exposer_Datas and do another for_each)  
    static void exposeVariants()
    {

      boost::mpl::for_each<JointModelVariant::types>(exposer());

      bp::to_python_converter<se3::JointModelVariant, jointModelVariantVisitor>();

      // bp::def("make_variant", se3::make_variant);

    }

    


  }} // namespace se3::python

#endif // ifndef __se3_python_joints_variant_hpp__

