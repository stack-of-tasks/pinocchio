//
// Copyright (c) 2020-2021 INRIA
//

#include <boost/python.hpp>

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/utils/registration.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/memory.hpp>

#include <Eigen/Geometry>
#include <eigenpy/geometry.hpp>
#include <eigenpy/quaternion.hpp>
#include <eigenpy/angle-axis.hpp>

#include <eigenpy/user-type.hpp>
#include <eigenpy/ufunc.hpp>

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::python::context::Quaternion);
EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::python::context::AngleAxis);

namespace pinocchio
{
  namespace python
  {
  
    namespace bp = boost::python;
  
    namespace internal
    {
      template<typename Scalar, bool is_floating_point = boost::is_floating_point<Scalar>::value>
      struct exposeTypeAlgo
      {
        static void run() {};
      };
    
      template<typename Scalar>
      struct exposeTypeAlgo<Scalar,false>
      {
        static void run()
        {
          eigenpy::exposeType<context::Scalar>();
          eigenpy::exposeType<context::Scalar,Eigen::RowMajor>();
        };
      };
    
      template<typename Scalar>
      void exposeType()
      {
        exposeTypeAlgo<Scalar>::run();
      }
    }

    void exposeEigenTypes()
    {
#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
      if(! register_symbolic_link_to_registered_type<context::Quaternion>())
        eigenpy::expose<context::Quaternion>();
      if(! register_symbolic_link_to_registered_type<context::AngleAxis>())
        eigenpy::expose<context::AngleAxis>();
#endif
      
      StdContainerFromPythonList< std::vector<std::string> >::register_converter();

      typedef Eigen::Matrix<context::Scalar,6,6,context::Options> Matrix6s;
      typedef Eigen::Matrix<context::Scalar,6,1,context::Options> Vector6s;
      typedef Eigen::Matrix<context::Scalar,6,Eigen::Dynamic,context::Options> Matrix6xs;
      typedef Eigen::Matrix<context::Scalar,3,Eigen::Dynamic,context::Options> Matrix3xs;

      internal::exposeType<context::Scalar>();
      eigenpy::enableEigenPySpecific<context::Matrix1s>();
      eigenpy::enableEigenPySpecific<Matrix6s>();
      eigenpy::enableEigenPySpecific<Vector6s>();
      eigenpy::enableEigenPySpecific<context::Vector7s>();
      eigenpy::enableEigenPySpecific<Matrix6xs>();
      eigenpy::enableEigenPySpecific<Matrix3xs>();
    }
    
  } // namespace python
} // namespace pinocchio

