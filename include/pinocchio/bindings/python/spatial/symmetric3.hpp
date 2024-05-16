//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_python_spatial_symmetric3_hpp__
#define __pinocchio_python_spatial_symmetric3_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/memory.hpp>
#include <eigenpy/eigen-to-python.hpp>
#include <boost/python/tuple.hpp>

#include "pinocchio/spatial/symmetric3.hpp"

#include "pinocchio/bindings/python/utils/cast.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Symmetric3>
    struct Symmetric3PythonVisitor
    : public boost::python::def_visitor<Symmetric3PythonVisitor<Symmetric3>>
    {
      enum
      {
        Options = Symmetric3::Options
      };
      typedef typename Symmetric3::Scalar Scalar;
      typedef typename Symmetric3::Vector3 Vector3;
      typedef typename Symmetric3::Vector6 Vector6;
      typedef typename Symmetric3::Matrix3 Matrix3;
      typedef typename Symmetric3::Matrix2 Matrix2;
      typedef typename Symmetric3::Matrix32 Matrix32;

      typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3Like;
      typedef Eigen::Matrix<Scalar, 3, 3, Options> Matrix3Like;
      typedef typename Symmetric3::SkewSquare SkewSquare;
      typedef typename Symmetric3::AlphaSkewSquare AlphaSkewSquare;

    public:
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        static const Scalar dummy_precision = Eigen::NumTraits<Scalar>::dummy_precision();
        PINOCCHIO_COMPILER_DIAGNOSTIC_PUSH
        PINOCCHIO_COMPILER_DIAGNOSTIC_IGNORED_SELF_ASSIGN_OVERLOADED
        cl.def(bp::init<>((bp::arg("self")), "Default constructor."))
          .def(bp::init<const Matrix3 &>(
            (bp::arg("self"), bp::arg("I")), "Initialize from symmetrical matrix I of size 3x3."))
          .def(bp::init<const Vector6 &>(
            (bp::arg("self"), bp::arg("I")), "Initialize from vector I of size 6."))
          .def(bp::init<
               const Scalar &, const Scalar &, const Scalar &, const Scalar &, const Scalar &,
               const Scalar &>(
            (bp::arg("self"), bp::arg("a0"), bp::arg("a1"), bp::arg("a2"), bp::arg("a3"),
             bp::arg("a4"), bp::arg("a5")),
            "Initialize from 6 scalar values."))
          .def(
            bp::init<const Symmetric3 &>((bp::arg("self"), bp::arg("other")), "Copy constructor."))
          .def("Zero", &Symmetric3::Zero, "Returns a zero 3x3 matrix.")
          .staticmethod("Zero")
          .def(
            "setZero", &Symmetric3::setZero, bp::arg("self"),
            "Set all the components of *this to zero.")
          .def("Random", &Symmetric3::Random, "Returns a random symmetric 3x3 matrix.")
          .staticmethod("Random")
          .def(
            "setRandom", &Symmetric3::setRandom, bp::arg("self"),
            "Set all the components of *this randomly.")
          .def("Identity", &Symmetric3::Identity, "Returns identity matrix.")
          .staticmethod("Identity")
          .def(
            "setIdentity", &Symmetric3::setIdentity, bp::arg("self"),
            "Set the components of *this to identity.")
#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
          .def(bp::self == bp::self)
          .def(bp::self != bp::self)
#endif
#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
          .def(
            "isApprox", &Symmetric3::isApprox,
            (bp::arg("self"), bp::arg("other"), bp::arg("prec") = dummy_precision),
            "Returns true if *this is approximately equal to other, within the precision given "
            "by prec.")
          .def(
            "isZero", &Symmetric3::isZero, (bp::arg("self"), bp::arg("prec") = dummy_precision),
            "Returns true if *this is approximately equal to the zero matrix, within the "
            "precision given by prec.")
#endif
          .def(
            "setDiagonal", &Symmetric3::template setDiagonal<Vector3Like>, bp::args("self", "diag"),
            "Set the diagonal elements of 3x3 matrix.")
          .def(
            "inverse", &Symmetric3::template inverse<Matrix3Like>, bp::args("self", "res"),
            "Invert the symmetrical 3x3 matrix.")
          .def("fill", &Symmetric3::fill, bp::args("self", "value"))
          .def(bp::self - bp::other<SkewSquare>())
          .def(bp::self -= bp::other<SkewSquare>())
          .def(bp::self - bp::other<AlphaSkewSquare>())
          .def(bp::self -= bp::other<AlphaSkewSquare>())
          .add_property(
            "data", &Symmetric3PythonVisitor::getData, &Symmetric3PythonVisitor::setData,
            "6D vector containing the data of the symmetric 3x3 matrix.")
          .def(
            "matrix", &Symmetric3::matrix, bp::arg("self"),
            "Returns a matrix representation of the data.")
          .def("vtiv", &Symmetric3::vtiv, bp::args("self", "v"))
          .def(
            "vxs", &Symmetric3::template vxs<Vector3>, bp::args("v", "S3"),
            "Performs the operation \f$ M = [v]_{\times} S_{3} \f$., Apply the cross product of "
            "v on each column of S and return result matrix M.")
          .staticmethod("vxs")
          .def(
            "svx", &Symmetric3::template vxs<Vector3>, bp::args("v", "S3"),
            "Performs the operation \f$ M = S_{3} [v]_{\times} \f$.")
          .staticmethod("svx")
          .def(
            "rhsMult", &Symmetric3::template rhsMult<Vector3, Vector3>,
            bp::args("SE3", "vin", "vout"))
          .staticmethod("rhsMult")

          .def(bp::self + bp::self)
          .def(bp::self += bp::self)
          .def(bp::self - bp::self)
          .def(bp::self -= bp::self)
          .def(bp::self *= bp::other<Scalar>())
          .def(bp::self * bp::other<Vector3Like>())
          .def(bp::self - bp::other<Matrix3Like>())
          .def(bp::self + bp::other<Matrix3Like>())

          .def(
            "decomposeltI", &Symmetric3::decomposeltI, bp::arg("self"),
            "Computes L for a symmetric matrix S.")
          .def(
            "rotate", &Symmetric3::template rotate<Matrix3>, bp::args("self", "R"),
            "Computes R*S*R'")

#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
          .def_pickle(Pickle())
#endif
          ;
        PINOCCHIO_COMPILER_DIAGNOSTIC_POP
      }

      static Vector6 getData(const Symmetric3 & self)
      {
        return self.data();
      }
      static void setData(Symmetric3 & self, Vector6 data)
      {
        self.data() = data;
      }

      static void expose()
      {
        bp::class_<Symmetric3>(
          "Symmetric3",
          "This class represents symmetric 3x3 matrices.\n\n"
          "Supported operations ...",
          bp::no_init)
          .def(Symmetric3PythonVisitor<Symmetric3>())
          .def(CastVisitor<Symmetric3>())
          .def(ExposeConstructorByCastVisitor<Symmetric3, ::pinocchio::Symmetric3>())
          .def(CopyableVisitor<Symmetric3>())
          .def(PrintableVisitor<Symmetric3>());
      }

    private:
      struct Pickle : bp::pickle_suite
      {
        static boost::python::tuple getinitargs(const Symmetric3 & I)
        {
          return bp::make_tuple(I);
        }
      };

    }; // struct Symmetric3PythonVisitor

  } // namespace python
} // namespace pinocchio

#endif // __pinocchio_python_spatial_symmetric3_hpp__
