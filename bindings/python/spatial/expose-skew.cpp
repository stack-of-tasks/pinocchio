//
// Copyright (c) 2015-2020 CNRS INRIA
// Copyright (c) 2020 Wandercraft
//

#include <boost/python.hpp>

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/skew.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
  
    // We need to resort to another call, because it seems that Boost.Python is not aligning the Eigen::MatrixBase. TODO: fix it!
    template<typename Vector3>
    Eigen::Matrix<typename Vector3::Scalar,3,3,Vector3::Options> skew(const Vector3 & v)
    {
      return pinocchio::skew(v);
    }
  
    // We need to resort to another call, because it seems that Boost.Python is not aligning the Eigen::MatrixBase. TODO: fix it!
    template<typename Vector3>
    Eigen::Matrix<typename Vector3::Scalar,3,3,Vector3::Options> skewSquare(const Vector3 & u, const Vector3 & v)
    {
      return pinocchio::skewSquare(u,v);
    }
  
    // We need to resort to another call, because it seems that Boost.Python is not aligning the Eigen::MatrixBase. TODO: fix it!
    template<typename Matrix3>
    Eigen::Matrix<typename Matrix3::Scalar,3,1,Matrix3::Options> unSkew(const Matrix3 & mat)
    {
      return pinocchio::unSkew(mat);
    }
  
    void exposeSkew()
    {
      typedef SE3::Matrix3 Matrix3;
      typedef SE3::Vector3 Vector3;
      
      bp::def("skew",&skew<Vector3>,
              bp::arg("u"),
              "Computes the skew representation of a given 3d vector, "
              "i.e. the antisymmetric matrix representation of the cross product operator, aka U = [u]x.\n"
              "Parameters:\n"
              "\tu: the input vector of dimension 3");
      
      bp::def("skewSquare",&skewSquare<Vector3>,
              bp::args("u","v"),
              "Computes the skew square representation of two given 3d vectors, "
              "i.e. the antisymmetric matrix representation of the chained cross product operator, u x (v x w), where w is another 3d vector.\n"
              "Parameters:\n"
              "\tu: the first input vector of dimension 3\n"
              "\tv: the second input vector of dimension 3");

      bp::def("unSkew",&unSkew<Matrix3>,
              bp::arg("U"),
              "Inverse of skew operator. From a given skew symmetric matrix U (i.e U = -U.T)"
              "of dimension 3x3, it extracts the supporting vector, i.e. the entries of U.\n"
              "Mathematically speacking, it computes v such that U.dot(x) = cross(u, x).\n"
              "Parameters:\n"
              "\tU: the input skew symmetric matrix of dimension 3x3.");
    }

  } // namespace python
} // namespace pinocchio
