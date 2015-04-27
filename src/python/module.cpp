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

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/geometry.hpp>
#include "pinocchio/python/python.hpp"

#include <iostream>

namespace bp = boost::python;

BOOST_PYTHON_MODULE(libpinocchio_pywrap)
{
  eigenpy::enableEigenPy();
  eigenpy::exposeAngleAxis();
  eigenpy::exposeQuaternion();

  typedef Eigen::Matrix<double,6,6> Matrix6d;
  typedef Eigen::Matrix<double,6,1> Vector6d;
  
  eigenpy::enableEigenPySpecific<Matrix6d,Matrix6d>();
  eigenpy::enableEigenPySpecific<Vector6d,Vector6d>();

  se3::python::exposeSE3();
  se3::python::exposeForce();
  se3::python::exposeMotion();
  se3::python::exposeInertia();

  se3::python::exposeModel();
  se3::python::exposeAlgorithms();
  se3::python::exposeParsers();
}
 
