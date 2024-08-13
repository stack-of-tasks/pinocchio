//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_python_context_casadi_hpp__
#define __pinocchio_python_context_casadi_hpp__

#include "pinocchio/autodiff/casadi.hpp"

#define PINOCCHIO_PYTHON_SCALAR_TYPE ::casadi::SX
#include "pinocchio/bindings/python/context/generic.hpp"
#undef PINOCCHIO_PYTHON_SCALAR_TYPE

#define PINOCCHIO_PYTHON_SKIP_COMPARISON_OPERATIONS
#define PINOCCHIO_PYTHON_NO_SERIALIZATION
#define PINOCCHIO_PYTHON_SKIP_REACHABLE_WORKSPACE
#define PINOCCHIO_PYTHON_SKIP_ALGORITHM_CONSTRAINED_DYNAMICS

#define PINOCCHIO_PYTHON_SKIP_CASADI_UNSUPPORTED

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/user-type.hpp>
#include <eigenpy/ufunc.hpp>
#include <eigenpy/swig.hpp>

namespace eigenpy
{

  namespace bp = boost::python;

  namespace casadi
  {

    struct CasadiType
    {
      static PyTypeObject * getSXType()
      {
        return reinterpret_cast<PyTypeObject *>(getInstance().casadi_SX_type.ptr());
      }

    private:
      static const CasadiType & getInstance()
      {
        static CasadiType elt;
        return elt;
      }

      CasadiType()
      {
        casadi_module = bp::import("casadi");
        casadi_SX_type = casadi_module.attr("SX");
        Py_INCREF(casadi_module.ptr());
      }

      ~CasadiType()
      {
        casadi_SX_type.~object();
        //    casadi_module.~object();
      }

      bp::object casadi_module;
      bp::object casadi_SX_type;
    };

  } // namespace casadi

  template<typename CasadiScalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
  struct expected_pytype_for_arg<
    Eigen::Matrix<::casadi::Matrix<CasadiScalar>, Rows, Cols, Options, MaxRows, MaxCols>,
    Eigen::MatrixBase<
      Eigen::Matrix<::casadi::Matrix<CasadiScalar>, Rows, Cols, Options, MaxRows, MaxCols>>>
  {
    static PyTypeObject const * get_pytype()
    {
      return ::eigenpy::casadi::CasadiType::getSXType();
    }
  };

  template<typename CasadiScalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
  struct EigenFromPy<
    Eigen::Matrix<::casadi::Matrix<CasadiScalar>, Rows, Cols, Options, MaxRows, MaxCols>>
  {
    typedef ::casadi::Matrix<CasadiScalar> CasadiMatrix;
    typedef Eigen::Matrix<::casadi::Matrix<CasadiScalar>, Rows, Cols, Options, MaxRows, MaxCols>
      MatType;

    /// \brief Determine if pyObj can be converted into a MatType object
    static void * convertible(PyObject * pyObj);

    /// \brief Allocate memory and copy pyObj in the new storage
    static void construct(PyObject * pyObj, bp::converter::rvalue_from_python_stage1_data * memory);

    static void registration();
  };

  template<typename CasadiScalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
  void * EigenFromPy<
    Eigen::Matrix<::casadi::Matrix<CasadiScalar>, Rows, Cols, Options, MaxRows, MaxCols>>::
    convertible(PyObject * pyObj)
  {
    if (std::strcmp(pyObj->ob_type->tp_name, CasadiMatrix::type_name().c_str()) != 0)
      return 0;

#define RETURN_VALUE(value)                                                                        \
  {                                                                                                \
    Py_DECREF(reinterpret_cast<PyObject *>(casadi_matrix_swig_obj));                               \
    return value;                                                                                  \
  }

    eigenpy::PySwigObject * casadi_matrix_swig_obj = eigenpy::get_PySwigObject(pyObj);
    if (casadi_matrix_swig_obj == NULL)
      RETURN_VALUE(0);

    CasadiMatrix * casadi_matrix_ptr =
      reinterpret_cast<CasadiMatrix *>(casadi_matrix_swig_obj->ptr);
    const CasadiMatrix & casadi_matrix = *casadi_matrix_ptr;

    const casadi_int R = casadi_matrix.rows(), C = casadi_matrix.columns(),
                     size = casadi_matrix.numel();

    const int ndim = (R == 0 || C == 0) ? 0 : (R == 1 || C == 1) ? 1 : 2;

    if (MatType::IsVectorAtCompileTime)
    {
      const Eigen::DenseIndex size_at_compile_time =
        MatType::IsRowMajor ? MatType::ColsAtCompileTime : MatType::RowsAtCompileTime;

      switch (ndim)
      {
      case 0:
        RETURN_VALUE(0);
      case 1: {
        if (size_at_compile_time != Eigen::Dynamic)
        {
          // check that the sizes at compile time matche
          if (size == size_at_compile_time)
          {
            if (MatType::ColsAtCompileTime != C || MatType::RowsAtCompileTime != R)
            {
              RETURN_VALUE(0);
            }
            else
            {
              RETURN_VALUE(pyObj);
            }
          }
          else
            RETURN_VALUE(0);
        }
        else // This is a dynamic MatType
          RETURN_VALUE(pyObj);
      }
      case 2: {
        assert(R > 1 && C > 1);
        RETURN_VALUE(0);
      }
      default:
        RETURN_VALUE(0);
      }
    }
    else // this is a matrix
    {
      if (ndim == 1) // We can always convert a vector into a matrix
        RETURN_VALUE(pyObj);

      if (ndim == 2)
      {
        if ((MatType::RowsAtCompileTime != R) && (MatType::RowsAtCompileTime != Eigen::Dynamic))
          RETURN_VALUE(0);
        if ((MatType::ColsAtCompileTime != C) && (MatType::ColsAtCompileTime != Eigen::Dynamic))
          RETURN_VALUE(0);
      }
    }

    RETURN_VALUE(pyObj);
#undef RETURN_VALUE
  }

  template<typename CasadiScalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
  void EigenFromPy<
    Eigen::Matrix<::casadi::Matrix<CasadiScalar>, Rows, Cols, Options, MaxRows, MaxCols>>::
    construct(PyObject * pyObj, bp::converter::rvalue_from_python_stage1_data * memory)
  {
    eigenpy::PySwigObject * casadi_matrix_swig_obj = eigenpy::get_PySwigObject(pyObj);
    assert(casadi_matrix_swig_obj != NULL);

    CasadiMatrix * casadi_matrix_ptr =
      reinterpret_cast<CasadiMatrix *>(casadi_matrix_swig_obj->ptr);
    const CasadiMatrix & casadi_matrix = *casadi_matrix_ptr;

    const casadi_int R = casadi_matrix.rows(), C = casadi_matrix.columns();

    bp::converter::rvalue_from_python_storage<MatType> * storage =
      reinterpret_cast<bp::converter::rvalue_from_python_storage<MatType> *>(
        reinterpret_cast<void *>(memory));

    // Allocate memory
    void * storage_ptr = storage->storage.bytes;
    MatType * eigen_matrix_ptr =
      ::eigenpy::details::init_matrix_or_array<MatType>::run(R, C, storage_ptr);

    // Copy element to matrix
    pinocchio::casadi::copy(casadi_matrix, *eigen_matrix_ptr);

    memory->convertible = storage->storage.bytes;
    Py_DECREF(reinterpret_cast<PyObject *>(casadi_matrix_swig_obj));
  }

  template<typename CasadiScalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
  void EigenFromPy<
    Eigen::Matrix<::casadi::Matrix<CasadiScalar>, Rows, Cols, Options, MaxRows, MaxCols>>::
    registration()
  {
    bp::converter::registry::push_back(
      reinterpret_cast<void * (*)(_object *)>(&EigenFromPy::convertible), &EigenFromPy::construct,
      bp::type_id<MatType>()
#ifndef BOOST_PYTHON_NO_PY_SIGNATURES
        ,
      &eigenpy::expected_pytype_for_arg<MatType>::get_pytype
#endif
    );
  }

  template<typename MatType>
  struct EigenToPy<MatType, ::casadi::Matrix<::casadi::SXElem>>
  {
    typedef ::casadi::Matrix<::casadi::SXElem> CasadiMatrix;

    static PyObject *
    convert(typename boost::add_reference<typename boost::add_const<MatType>::type>::type mat)
    {
      assert(
        (mat.rows() < INT_MAX) && (mat.cols() < INT_MAX)
        && "Matrix range larger than int ... should never happen.");

      PyObject * casadi_matrix_py_ptr =
        PyObject_CallObject(reinterpret_cast<PyObject *>(casadi::CasadiType::getSXType()), NULL);

      eigenpy::PySwigObject * casadi_matrix_swig_obj =
        eigenpy::get_PySwigObject(casadi_matrix_py_ptr);
      assert(casadi_matrix_swig_obj != NULL);

      CasadiMatrix * casadi_matrix_obj_ptr =
        reinterpret_cast<CasadiMatrix *>(casadi_matrix_swig_obj->ptr);
      pinocchio::casadi::copy(mat, *casadi_matrix_obj_ptr);

      Py_DECREF(reinterpret_cast<PyObject *>(casadi_matrix_swig_obj));
      return casadi_matrix_py_ptr;
    }

    static PyTypeObject const * get_pytype()
    {
      return ::eigenpy::casadi::CasadiType::getSXType();
    }
  };

  template<typename TensorType, typename _Scalar>
  struct expose_eigen_type_impl<
    TensorType,
    Eigen::TensorBase<TensorType>,
    ::casadi::Matrix<_Scalar>>
  {
    static void run()
    {
    }
  };

  template<typename SparseType, typename _Scalar>
  struct expose_eigen_type_impl<
    SparseType,
    Eigen::SparseMatrixBase<SparseType>,
    ::casadi::Matrix<_Scalar>>
  {
    static void run()
    {
    }
  };

  template<typename MatType, int Options, typename Stride>
  struct EigenToPy<Eigen::Ref<MatType, Options, Stride>, ::casadi::Matrix<::casadi::SXElem>>
  {
    typedef ::casadi::Matrix<::casadi::SXElem> CasadiMatrix;

    static PyObject * convert(const Eigen::Ref<MatType, Options, Stride> & mat)
    {
      assert(
        (mat.rows() < INT_MAX) && (mat.cols() < INT_MAX)
        && "Matrix range larger than int ... should never happen.");
      PyObject * casadi_matrix_py_ptr =
        PyObject_CallObject(reinterpret_cast<PyObject *>(casadi::CasadiType::getSXType()), NULL);

      eigenpy::PySwigObject * casadi_matrix_swig_obj =
        eigenpy::get_PySwigObject(casadi_matrix_py_ptr);
      assert(casadi_matrix_swig_obj != NULL);

      CasadiMatrix * casadi_matrix_obj_ptr =
        reinterpret_cast<CasadiMatrix *>(casadi_matrix_swig_obj->ptr);
      pinocchio::casadi::copy(mat.derived(), *casadi_matrix_obj_ptr);

      Py_DECREF(reinterpret_cast<PyObject *>(casadi_matrix_swig_obj));
      return casadi_matrix_py_ptr;
    }

    static PyTypeObject const * get_pytype()
    {
      return ::eigenpy::casadi::CasadiType::getSXType();
    }
  };

  namespace internal
  {

    template<>
    inline npy_bool SpecialMethods<pinocchio::python::context::Scalar, NPY_USERDEF>::nonzero(
      void * ip, void * array)
    {

      typedef pinocchio::python::context::Scalar Scalar;
      PyArrayObject * py_array = static_cast<PyArrayObject *>(array);
      if (py_array == NULL || PyArray_ISBEHAVED_RO(py_array))
      {
        const Scalar & value = *static_cast<Scalar *>(ip);
        return (npy_bool)(value.is_zero());
      }
      else
      {
        Scalar tmp_value;
        PyArray_Descr * descr = PyArray_DESCR(py_array);
        PyArray_ArrFuncs * f = PyDataType_GetArrFuncs(descr);
        f->copyswap(&tmp_value, ip, PyArray_ISBYTESWAPPED(py_array), array);
        return (npy_bool)(tmp_value.is_zero());
      }
    }

  } // namespace internal

} // namespace eigenpy

namespace pinocchio
{
  namespace python
  {

    template<typename CasadiMatrix>
    struct CasadiMatrixToPython
    {

      static PyObject * convert(CasadiMatrix const & x)
      {
        PyObject * casadi_matrix_py_ptr = PyObject_CallObject(
          reinterpret_cast<PyObject *>(eigenpy::casadi::CasadiType::getSXType()), NULL);
        eigenpy::PySwigObject * casadi_matrix_swig_obj =
          eigenpy::get_PySwigObject(casadi_matrix_py_ptr);
        assert(casadi_matrix_swig_obj != NULL);

        CasadiMatrix * casadi_matrix_obj_ptr =
          reinterpret_cast<CasadiMatrix *>(casadi_matrix_swig_obj->ptr);
        *casadi_matrix_obj_ptr = x;

        Py_DECREF(reinterpret_cast<PyObject *>(casadi_matrix_swig_obj));
        return casadi_matrix_py_ptr;
      }

      static PyTypeObject const * get_pytype()
      {
        return ::eigenpy::casadi::CasadiType::getSXType();
      }

      static void registration()
      {
        boost::python::to_python_converter<CasadiMatrix, CasadiMatrixToPython, true>();
      }
    };

    template<typename CasadiMatrix>
    struct CasadiMatrixFromPython
    {
      struct Extractor
      {
        static CasadiMatrix & execute(PyObject * /*pyObj*/)
        {
          throw std::runtime_error("Should never be called");
        }
      };

      static void registration()
      {
        boost::python::converter::registry::insert(
          &extract, boost::python::detail::extractor_type_id(&Extractor::execute)
#ifndef BOOST_PYTHON_NO_PY_SIGNATURES
                      ,
          &get_pytype
#endif
        );
      }

    private:
      static void * extract(PyObject * pyObj)
      {
        if (!PyObject_TypeCheck(pyObj, ::eigenpy::casadi::CasadiType::getSXType()))
          return 0;

        eigenpy::PySwigObject * casadi_matrix_swig_obj = eigenpy::get_PySwigObject(pyObj);
        return casadi_matrix_swig_obj->ptr;
      }
#ifndef BOOST_PYTHON_NO_PY_SIGNATURES
      static PyTypeObject const * get_pytype()
      {
        return ::eigenpy::casadi::CasadiType::getSXType();
      }
#endif
    };

    //  template<typename CasadiMatrix>
    //  struct CasadiMatrixFromPython
    //  {
    //    static void* convertible(PyObject * pyObj)
    //    {
    //      if(PyFloat_Check(pyObj))
    //        return pyObj;
    //      if(std::strcmp(pyObj->ob_type->tp_name,CasadiMatrix::type_name().c_str()) != 0)
    //        return 0;
    //
    //      return pyObj;
    //    }
    //    static void construct(PyObject * pyObj,
    //                          boost::python::converter::rvalue_from_python_stage1_data * memory)
    //    {
    //      eigenpy::PySwigObject * casadi_matrix_swig_obj = eigenpy::get_PySwigObject(pyObj);
    //      assert(casadi_matrix_swig_obj != NULL);
    //
    //      CasadiMatrix * casadi_matrix_ptr =
    //      reinterpret_cast<CasadiMatrix*>(casadi_matrix_swig_obj->ptr); const CasadiMatrix &
    //      casadi_matrix = *casadi_matrix_ptr;
    //
    //      bp::converter::rvalue_from_python_storage<CasadiMatrix>* storage =
    //      reinterpret_cast<bp::converter::rvalue_from_python_storage<CasadiMatrix>*>
    //      (reinterpret_cast<void*>(memory));
    //
    //      // Allocate memory
    //      void * storage_ptr = storage->storage.bytes;
    //      CasadiMatrix * casadi_matrix_cpp = new (storage_ptr) CasadiMatrix(casadi_matrix);
    //
    //      memory->convertible = storage->storage.bytes;
    //      Py_DECREF(reinterpret_cast<PyObject *>(casadi_matrix_swig_obj));
    //    }
    //  };

    inline boost::python::object getScalarType()
    {
      namespace bp = boost::python;

      PyObject * pyObj = reinterpret_cast<PyObject *>(::eigenpy::casadi::CasadiType::getSXType());
      bp::object scalar_type(bp::handle<>(bp::borrowed(pyObj)));

      return scalar_type;
    }

    inline void exposeSpecificTypeFeatures()
    {
      typedef pinocchio::python::context::Scalar Scalar;
      CasadiMatrixToPython<Scalar>::registration();
      CasadiMatrixFromPython<Scalar>::registration();
      boost::python::implicitly_convertible<double, Scalar>();
      boost::python::implicitly_convertible<float, Scalar>();
      boost::python::implicitly_convertible<int, Scalar>();
      boost::python::implicitly_convertible<long, Scalar>();
      boost::python::implicitly_convertible<bool, Scalar>();
    };

  } // namespace python
} // namespace pinocchio

namespace eigenpy
{

  template<typename Scalar>
  struct has_operator_equal<::casadi::Matrix<Scalar>> : boost::false_type
  {
  };

} // namespace eigenpy

#endif // #ifndef __pinocchio_python_context_casadi_hpp__
