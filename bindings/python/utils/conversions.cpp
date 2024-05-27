//
// Copyright (c) 2019-2021 CNRS INRIA
//

#include <sstream>

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/bindings/python/spatial/se3.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    struct XYZQUATConverter
    {
      typedef context::Scalar Scalar;
      typedef context::SE3 SE3;
      typedef context::VectorXs VectorXs;
      enum
      {
        Options = context::Options
      };
      typedef Eigen::Matrix<Scalar, 7, 1, Options> Vector7s;
      typedef Eigen::Map<SE3::Quaternion> QuatMap;
      typedef Eigen::Map<const SE3::Quaternion> QuatConstMap;

      static VectorXs fromSE3(const SE3 & M)
      {
        Vector7s res;
        res.head<3>() = M.translation();
        QuatMap(res.tail<4>().data()) = M.rotation();
        return res;
      }

      static bp::tuple fromSE3tuple(const SE3 & M)
      {
        const SE3::Quaternion q(M.rotation());
        return bp::make_tuple(
          M.translation()(0), M.translation()(1), M.translation()(2), q.x(), q.y(), q.z(), q.w());
      }

      template<typename TupleOrList>
      static SE3 toSE3fromTupleOrList(const TupleOrList & v)
      {

        bp::ssize_t size = bp::len(v);
        if (size != 7)
        {
          throw std::invalid_argument(
            "Wrong size: v(" + std::to_string(size) + ") should have 7 elements");
        }

        // bp::extract<SE3::Scalar> to_double;
        const Scalar & v0 = bp::extract<Scalar>(v[0]);
        const Scalar & v1 = bp::extract<Scalar>(v[1]);
        const Scalar & v2 = bp::extract<Scalar>(v[2]);
        const Scalar & v3 = bp::extract<Scalar>(v[3]);
        const Scalar & v4 = bp::extract<Scalar>(v[4]);
        const Scalar & v5 = bp::extract<Scalar>(v[5]);
        const Scalar & v6 = bp::extract<Scalar>(v[6]);

        SE3::Quaternion q(v6, v3, v4, v5);
        SE3::Vector3 t(v0, v1, v2);
        return SE3(q.matrix(), t);
      }

      template<typename Vector7Like>
      SE3 XYZQUATToSE3_ei(const Vector7Like & v)
      {
        if (v.rows() != 7 || v.cols() != 1)
        {
          std::ostringstream shape;
          shape << "(" << v.rows() << ", " << v.cols() << ")";
          throw std::invalid_argument(
            "Wrong size: v" + shape.str() + " but should have the following shape (7, 1)");
        }
        QuatConstMap q(v.template tail<4>().data());
        return SE3(q.matrix(), v.template head<3>());
      }

      template<typename Vector7Like>
      static SE3 toSE3(const Vector7Like & v)
      {
        if (v.rows() != 7 || v.cols() != 1)
        {
          std::ostringstream shape;
          shape << "(" << v.rows() << ", " << v.cols() << ")";
          throw std::invalid_argument(
            "Wrong size: v" + shape.str() + " but should have the following shape (7, 1)");
        }

        PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Vector7Like, v, 7, 1);
        QuatConstMap q(v.template tail<4>().data());
        return SE3(q.matrix(), v.template head<3>());
      }

      static void expose()
      {

        const char * doc1 = "Convert the input SE3 object to a numpy array.";
        bp::def("SE3ToXYZQUAT", fromSE3, "M", doc1);
        const char * doc1_tuple =
          "Convert the input SE3 object to a 7D tuple of floats [X,Y,Z,x,y,z,w].";
        bp::def("SE3ToXYZQUATtuple", fromSE3tuple, "M", doc1_tuple);

        const char * doc2 =
          "Reverse function of SE3ToXYZQUAT: convert [X,Y,Z,x,y,z,w] to an SE3 element.";
        bp::def(
          "XYZQUATToSE3", static_cast<SE3 (*)(const bp::tuple &)>(toSE3fromTupleOrList<bp::tuple>),
          bp::arg("tuple"), doc2);
        bp::def(
          "XYZQUATToSE3", static_cast<SE3 (*)(const bp::list &)>(toSE3fromTupleOrList<bp::list>),
          bp::arg("list"), doc2);
        bp::def(
          "XYZQUATToSE3", static_cast<SE3 (*)(const VectorXs &)>(toSE3<VectorXs>), bp::arg("array"),
          doc2);
      }
    };

    void exposeConversions()
    {
      XYZQUATConverter::expose();
    }

  } // namespace python
} // namespace pinocchio
