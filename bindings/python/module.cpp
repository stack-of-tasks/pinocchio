//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/geometry.hpp>

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/bindings/python/utils/version.hpp"

namespace bp = boost::python;
using namespace se3::python;

BOOST_PYTHON_MODULE(libpinocchio_pywrap)
{
  eigenpy::enableEigenPy();
  eigenpy::exposeAngleAxis();
  eigenpy::exposeQuaternion();

  typedef Eigen::Matrix<double,6,6> Matrix6d;
  typedef Eigen::Matrix<double,6,1> Vector6d;
  typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6x;
  typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;
  
  eigenpy::enableEigenPySpecific<Matrix6d>();
  eigenpy::enableEigenPySpecific<Vector6d>();
  eigenpy::enableEigenPySpecific<Matrix6x>();
  eigenpy::enableEigenPySpecific<Matrix3x>();

  exposeSE3();
  exposeForce();
  exposeMotion();
  exposeInertia();
  exposeJoints();
  exposeExplog();
  
  bp::enum_< ::se3::ReferenceFrame >("ReferenceFrame")
  .value("WORLD",::se3::WORLD)
  .value("LOCAL",::se3::LOCAL)
  ;

  exposeModel();
  exposeFrame();
  exposeData();
  exposeGeometry();
  
  exposeAlgorithms();
  exposeParsers();
  
#ifdef WITH_HPP_FCL
  exposeFCL();
#endif // WITH_HPP_FCL
  
  exposeVersion();
  
}
 
