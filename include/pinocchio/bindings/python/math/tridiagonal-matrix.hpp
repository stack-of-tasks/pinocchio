//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_python_math_tridiagonal_matrix_hpp__
#define __pinocchio_python_math_tridiagonal_matrix_hpp__

#include "pinocchio/bindings/python/fwd.hpp"
#include "pinocchio/math/tridiagonal-matrix.hpp"

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/memory.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename TridiagonalSymmetricMatrix>
    struct TridiagonalSymmetricMatrixPythonVisitor
    : public boost::python::def_visitor<
        TridiagonalSymmetricMatrixPythonVisitor<TridiagonalSymmetricMatrix>>
    {
      typedef typename TridiagonalSymmetricMatrix::Scalar Scalar;
      typedef typename TridiagonalSymmetricMatrix::CoeffVectorType CoeffVectorType;
      typedef typename TridiagonalSymmetricMatrix::PlainMatrixType PlainMatrixType;

    public:
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        static const Scalar dummy_precision = Eigen::NumTraits<Scalar>::dummy_precision();

        cl.def(bp::init<Eigen::DenseIndex>(
                 (bp::arg("self"), bp::arg("size")), "Default constructor from a given size."))
#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
          .def(bp::self == bp::self)
          .def(bp::self != bp::self)
#endif
          .def(
            "diagonal",
            (CoeffVectorType & (TridiagonalSymmetricMatrix::*)())
              & TridiagonalSymmetricMatrix::diagonal,
            bp::arg("self"),
            "Reference of the diagonal elements of the symmetric tridiagonal matrix.",
            bp::return_internal_reference<>())
          .def(
            "subDiagonal",
            (CoeffVectorType & (TridiagonalSymmetricMatrix::*)())
              & TridiagonalSymmetricMatrix::subDiagonal,
            bp::arg("self"),
            "Reference of the sub diagonal elements of the symmetric tridiagonal matrix.",
            bp::return_internal_reference<>())

          .def(
            "setIdentity", &TridiagonalSymmetricMatrix::setIdentity, bp::arg("self"),
            "Set the current tridiagonal matrix to identity.")
          .def(
            "setZero", &TridiagonalSymmetricMatrix::setZero, bp::arg("self"),
            "Set the current tridiagonal matrix to zero.")
          .def(
            "setDiagonal", &TridiagonalSymmetricMatrix::template setDiagonal<CoeffVectorType>,
            bp::args("self", "diagonal"),
            "Set the current tridiagonal matrix to a diagonal matrix given by the entry vector "
            "diagonal.")
          .def(
            "setRandom", &TridiagonalSymmetricMatrix::setRandom, bp::arg("self"),
            "Set the current tridiagonal matrix to random.")
#ifndef PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
          .def(
            "isIdentity", &TridiagonalSymmetricMatrix::isIdentity,
            (bp::arg("self"), bp::arg("prec") = dummy_precision),
            "Returns true if *this is approximately equal to the identity matrix, within the "
            "precision given by prec.")
          .def(
            "isZero", &TridiagonalSymmetricMatrix::isZero,
            (bp::arg("self"), bp::arg("prec") = dummy_precision),
            "Returns true if *this is approximately equal to the zero matrix, within the "
            "precision given by prec.")
          .def(
            "isDiagonal", &TridiagonalSymmetricMatrix::isDiagonal,
            (bp::arg("self"), bp::arg("prec") = dummy_precision),
            "Returns true if *this is approximately equal to the a diagonal matrix, within the "
            "precision given by prec.")
#endif // PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
          .def("rows", &TridiagonalSymmetricMatrix::rows, bp::arg("self"))
          .def("cols", &TridiagonalSymmetricMatrix::cols, bp::arg("self"))
          .def("matrix", &TridiagonalSymmetricMatrix::matrix, bp::arg("self"))

          .def(bp::self * PlainMatrixType())
          .def(PlainMatrixType() * bp::self);
      }

      static void expose()
      {
#if PY_MAJOR_VERSION == 3 && PY_MINOR_VERSION == 6 && EIGENPY_VERSION_AT_LEAST(2, 9, 0)
        typedef PINOCCHIO_SHARED_PTR_HOLDER_TYPE(TridiagonalSymmetricMatrix) HolderType;
#else
        typedef ::boost::python::detail::not_specified HolderType;
#endif
        bp::class_<TridiagonalSymmetricMatrix, HolderType>(
          "TridiagonalSymmetricMatrix", "Tridiagonal symmetric matrix.", bp::no_init)
          .def(TridiagonalSymmetricMatrixPythonVisitor());
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_math_tridiagonal_matrix_hpp__
