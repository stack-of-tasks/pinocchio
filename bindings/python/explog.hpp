//
// Copyright (c) 2015 CNRS
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

#ifndef __se3_python_explog_hpp__
# define __se3_python_explog_hpp__

# include <eigenpy/eigenpy.hpp>

# include "pinocchio/spatial/explog.hpp"
# include "pinocchio/bindings/python/se3.hpp"
# include "pinocchio/bindings/python/motion.hpp"

namespace se3
{
  namespace python
  {
    struct ExplogPythonVisitor
    {
      typedef Eigen::Matrix3d Matrix3d;
      typedef Eigen::Vector3d Vector3d;
      typedef Eigen::Matrix4d Matrix4d;
      typedef Eigen::Matrix<double,6,1> Vector6d;
      typedef eigenpy::Matrix3d_fx Matrix3d_fx;
      typedef eigenpy::Vector3d_fx Vector3d_fx;
      typedef eigenpy::Matrix4d_fx Matrix4d_fx;
      typedef eigenpy::UnalignedEquivalent<Vector6d>::type Vector6d_fx;
      typedef SE3PythonVisitor<SE3>::SE3_fx SE3_fx;
      typedef MotionPythonVisitor<Motion>::Motion_fx Motion_fx;

      static Matrix3d exp3_proxy(const Vector3d_fx & v)
      {
        return exp3(v);
      }

      static Vector3d log3_proxy(const Matrix3d_fx & R)
      {
        return log3(R);
      }

      static SE3_fx exp6FromMotion_proxy(const Motion_fx & nu)
      {
        return exp6(nu);
      }

      static SE3_fx exp6FromVector_proxy(const Vector6d_fx & v)
      {
        return exp6(v);
      }

      static Motion_fx log6FromSE3_proxy(const SE3_fx & m)
      {
        return log6(m);
      }

      static Motion_fx log6FromMatrix_proxy(const Matrix4d_fx & M)
      {
        return log6(M);
      }

      static void expose()
      {
        bp::def("exp3",exp3_proxy,
                bp::args("Angular velocity (double size 3)"),
                "Exp: so3 -> SO3. Return the integral of the input"
                " angular velocity during time 1.");
        bp::def("log3",log3_proxy,
                bp::args("Rotation matrix (double size 3x3)"),
                "Log: SO3 -> so3. Pseudo-inverse of log from SO3"
                " -> { v in so3, ||v|| < 2pi }.Exp: so3 -> SO3.");
        bp::def("exp6FromMotion",exp6FromMotion_proxy,
                bp::args("Spatial velocity (se3.Motion)"),
                "Exp: se3 -> SE3. Return the integral of the input"
                " spatial velocity during time 1.");
        bp::def("exp6FromVector",exp6FromVector_proxy,
                bp::args("Spatial velocity (double size 6)"),
                "Exp: se3 -> SE3. Return the integral of the input"
                " spatial velocity during time 1.");
        bp::def("log6FromSE3",log6FromSE3_proxy,
                bp::args("Spatial transform (se3.SE3)"),
                "Log: SE3 -> se3. Pseudo-inverse of exp from SE3"
                " -> { v,w in se3, ||w|| < 2pi }.");
        bp::def("log6FromMatrix",log6FromMatrix_proxy,
                bp::args("Spatial transform (double size 4x4)"),
                "Log: SE3 -> se3. Pseudo-inverse of exp from SE3"
                " -> { v,w in se3, ||w|| < 2pi }.");
      }
    };
  } // namespace python
} //namespace se3

#endif // ifndef __se3_python_explog_hpp__
