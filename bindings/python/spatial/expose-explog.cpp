//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include "pinocchio/bindings/python/spatial/explog.hpp"
#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    void exposeExplog()
    {
      
      bp::def("exp3",&exp3_proxy<Eigen::Vector3d>,
              bp::arg("w"),
              "Exp: so3 -> SO3. Return the integral of the input"
              " vector w during time 1. This is also known as the Rodrigues formula.");
      
      bp::def("Jexp3",&Jexp3_proxy<Eigen::Vector3d>,
              bp::arg("w"),
              "Jacobian of exp(v) which maps from the tangent of SO(3) at R = exp(v) to"
              " the tangent of SO(3) at Identity.");
      
      bp::def("log3",&log3_proxy<Eigen::Matrix3d>,
              bp::arg("R"),
              "Log: SO3 -> so3 is the pseudo-inverse of Exp: so3 -> SO3. Log maps from SO3"
              " -> { v in so3, ||v|| < 2pi }.");
      
      bp::def("Jlog3",&Jlog3_proxy<Eigen::Matrix3d>,
              bp::arg("R"),
              "Jacobian of log(R) which maps from the tangent of SO(3) at R to"
              " the tangent of SO(3) at Identity.");
      
      bp::def("exp6",&exp6_proxy<double,0>,
              bp::arg("motion"),
              "Exp: se3 -> SE3. Return the integral of the input"
              " spatial velocity during time 1.");
              
      bp::def("exp6",&exp6_proxy<Motion::Vector6>,
              bp::arg("v"),
              "Exp: se3 -> SE3. Return the integral of the input"
              " spatial velocity during time 1.");
      
      bp::def("Jexp6",&Jexp6_proxy<double,0>,
              bp::arg("motion"),
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
              bp::arg("homegeneous_matrix"),
              "Log: SE3 -> se3. Pseudo-inverse of Exp: so3 -> SO3. Log maps from SE3"
              " -> { v,w in se3, ||w|| < 2pi }.");
      
      bp::def("Jlog6",&Jlog6_proxy<double,0>,
              bp::arg("M"),
              "Jacobian of log(M) which maps from the tangent of SE(3) at M to"
              " the tangent of SE(3) at Identity.");
      
    }
    
  } // namespace python
} // namespace pinocchio
