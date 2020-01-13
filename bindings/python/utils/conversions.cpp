//
// Copyright (c) 2019-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/se3.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    typedef SE3::Scalar Scalar;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic,1> VectorXd;
    typedef Eigen::Matrix<Scalar, 7,1> Vector7d;
    typedef Eigen::Map<      SE3::Quaternion> QuatMap;
    typedef Eigen::Map<const SE3::Quaternion> QuatConstMap;

    VectorXd SE3ToXYZQUAT(const SE3& M)
    {
      Vector7d res;
      res.head<3>() = M.translation();
      QuatMap (res.tail<4>().data()) = M.rotation();
      return res;
    }

    bp::tuple SE3ToXYZQUATtuple(const SE3& M)
    {
      SE3::Quaternion q (M.rotation());
      return bp::make_tuple (
          M.translation()(0), M.translation()(1), M.translation()(2),
          q.x(), q.y(), q.z(), q.w());
    }

    template <typename TupleOrList>
    SE3 XYZQUATToSE3_bp(const TupleOrList& v)
    {
      //bp::extract<SE3::Scalar> to_double;
      SE3::Quaternion q (
          (Scalar)bp::extract<Scalar>(v[6]),
          (Scalar)bp::extract<Scalar>(v[3]),
          (Scalar)bp::extract<Scalar>(v[4]),
          (Scalar)bp::extract<Scalar>(v[5]));
      SE3::Vector3 t (
          (Scalar)bp::extract<Scalar>(v[0]),
          (Scalar)bp::extract<Scalar>(v[1]),
          (Scalar)bp::extract<Scalar>(v[2]));
      return SE3 (q.matrix(), t);
    }

    template <typename Vector7Like>
    SE3 XYZQUATToSE3_ei(const Vector7Like& v)
    {
      PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Vector7Like, v, 7, 1);
      QuatConstMap q (v.template tail<4>().data());
      return SE3 (q.matrix(), v.template head<3>());
    }
    
    void exposeConversions()
    {
      const char* doc1 = "Convert the input SE3 object to a numpy array.";
      bp::def("SE3ToXYZQUAT"     , SE3ToXYZQUAT     , "M", doc1);
      const char* doc1_tuple = "Convert the input SE3 object to a 7D tuple of floats [X,Y,Z,x,y,z,w].";
      bp::def("SE3ToXYZQUATtuple", SE3ToXYZQUATtuple, "M", doc1_tuple);

      const char* doc2 = "Reverse function of SE3ToXYZQUAT: convert [X,Y,Z,x,y,z,w] to an SE3 element.";
      bp::def("XYZQUATToSE3",
              static_cast<SE3 (*) (const bp::tuple&)> (XYZQUATToSE3_bp<bp::tuple>),
              bp::arg("tuple"),doc2);
      bp::def("XYZQUATToSE3",
              static_cast<SE3 (*) (const bp::list &)> (XYZQUATToSE3_bp<bp::list >),
              bp::arg("list"),doc2);
      bp::def("XYZQUATToSE3", static_cast<SE3 (*) (const VectorXd &)> (XYZQUATToSE3_ei<VectorXd >),
              bp::arg("array"),doc2);
    }
    
  } // namespace python
} // namespace pinocchio
