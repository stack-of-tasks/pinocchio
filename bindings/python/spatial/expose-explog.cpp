//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/explog.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    void exposeExplog()
    {
      
      bp::def("exp3",&exp3_proxy<Eigen::Vector3d>,
              bp::arg("v"),
              "Exp: so3 -> SO3. Return the integral of the input"
              " angular velocity during time 1.");
      
      bp::def("Jexp3",&Jexp3_proxy<Eigen::Vector3d>,
              bp::arg("v"),
              "Jacobian of exp(R) which maps from the tangent of SO(3) at exp(v) to"
              " the tangent of SO(3) at Identity.");
      
      bp::def("log3",&log3_proxy<Eigen::Matrix3d>,
              bp::arg("R"),
              "Log: SO3 -> so3. Pseudo-inverse of log from SO3"
              " -> { v in so3, ||v|| < 2pi }.Exp: so3 -> SO3.");
      
      bp::def("Jlog3",&Jlog3_proxy<Eigen::Matrix3d>,
              bp::arg("R"),
              "Jacobian of log(R) which maps from the tangent of SO(3) at R to"
              " the tangent of SO(3) at Identity.");
      
      bp::def("Hlog3",&Hlog3_proxy<Eigen::Matrix3d, Eigen::Vector3d>,
              bp::args("R","v"),
              "Vector v to be multiplied to the hessian",
              "v^T * H where H is the Hessian of log(R)");
      
      bp::def("exp6",&exp6_proxy<double,0>,
              bp::arg("v"),
              "Exp: se3 -> SE3. Return the integral of the input"
              " spatial velocity during time 1.");
              
      bp::def("exp6",&exp6_proxy<Motion::Vector6>,
              bp::arg("v"),
              "Exp: se3 -> SE3. Return the integral of the input"
              " spatial velocity during time 1.");
      
      bp::def("Jexp6",&Jexp6_proxy<double,0>,
              bp::arg("v"),
              "Jacobian of exp(v) which maps from the tangent of SE(3) at exp(v) to"
              " the tangent of SE(3) at Identity.");
              
      bp::def("Jexp6",&Jexp6_proxy<Motion::Vector6>,
              bp::arg("v"),
              "Jacobian of exp(v) which maps from the tangent of SE(3) at exp(v) to"
              " the tangent of SE(3) at Identity.");
      
      bp::def("log6",(Motion (*)(const SE3 &))&log6<double,0>,
              bp::arg("M"),
              "Log: SE3 -> se3. Pseudo-inverse of exp from SE3"
              " -> { v,w in se3, ||w|| < 2pi }.");
      
      bp::def("log6",&log6_proxy<Eigen::Matrix4d>,
              bp::arg("H"),
              "Log: SE3 -> se3. Pseudo-inverse of exp from SE3"
              " -> { v,w in se3, ||w|| < 2pi }.");
      
      bp::def("Jlog6",&Jlog6_proxy<double,0>,
              bp::arg("M"),
              "Jacobian of log(M) which maps from the tangent of SE(3) at M to"
              " the tangent of SE(3) at Identity.");
      
    }
    
  } // namespace python
} // namespace pinocchio
