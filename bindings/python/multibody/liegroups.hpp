//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_python_lie_group_hpp__
#define __pinocchio_python_lie_group_hpp__

#include <eigenpy/memory.hpp>

#include "pinocchio/multibody/liegroup/liegroup.hpp"
#include "pinocchio/multibody/liegroup/cartesian-product-variant.hpp"
#include "pinocchio/multibody/liegroup/liegroup-generic.hpp"
#include "pinocchio/multibody/liegroup/liegroup-collection.hpp"

namespace pinocchio
{
namespace python
{
namespace bp = boost::python;

template<class LieGroupType>
struct LieGroupWrapperTpl
{
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> ConfigVector_t;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> TangentVector_t;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> JacobianMatrix_t;

  static ConfigVector_t integrate(const LieGroupType& lg,
      const ConfigVector_t& q, const TangentVector_t& v)
  {
    return lg.integrate(q, v);
  }

  static ConfigVector_t interpolate(const LieGroupType& lg,
      const ConfigVector_t& q0,
      const ConfigVector_t& q1,
      const double& u)
  {
    return lg.interpolate(q0,q1,u);
  }

  static TangentVector_t difference(const LieGroupType& lg,
      const ConfigVector_t & q0,
      const ConfigVector_t & q1)
  {
    return lg.difference(q0,q1);
  }

  static JacobianMatrix_t dDifference(const LieGroupType& lg,
      const ConfigVector_t& q0, const ConfigVector_t& q1,
      const ArgumentPosition arg)
  {
    JacobianMatrix_t J (lg.nv(), lg.nv());
    lg.dDifference(q0, q1, J, arg);
    return J;
  }

  static JacobianMatrix_t dIntegrate(const LieGroupType& lg,
      const ConfigVector_t& q, const TangentVector_t& v,
      const ArgumentPosition arg)
  {
    JacobianMatrix_t J (lg.nv(), lg.nv());
    lg.dIntegrate(q, v, J, arg);
    return J;
  }

  static void dIntegrateTransport1(const LieGroupType& lg,
      const ConfigVector_t& q, const TangentVector_t& v,
      JacobianMatrix_t& J,
      const ArgumentPosition arg)
  {
    lg.dIntegrateTransport(q, v, J, arg);
  }

  static JacobianMatrix_t dIntegrateTransport2(const LieGroupType& lg,
      const ConfigVector_t& q, const TangentVector_t& v,
      const JacobianMatrix_t& J,
      const ArgumentPosition arg)
  {
    JacobianMatrix_t Jout (lg.nv(), J.cols());
    lg.dIntegrateTransport(q, v, J, Jout, arg);
    return Jout;
  }
};

template<class LieGroupType>
struct LieGroupPythonVisitor
: public boost::python::def_visitor< LieGroupPythonVisitor<LieGroupType> >
{
public:
  
  /* --- Exposing C++ API to python through the handler ----------------- */
  template<class PyClass>
  void visit(PyClass& cl) const
  {
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> ConfigVector_t;

    typedef LieGroupWrapperTpl<LieGroupType> LieGroupWrapper;
    cl
    .def(bp::init<>("Default constructor"))
    .def("integrate", LieGroupWrapper::integrate)
    .def("dIntegrate", LieGroupWrapper::dIntegrate)
    .def("dIntegrateTransport", LieGroupWrapper::dIntegrateTransport1)
    .def("dIntegrateTransport", LieGroupWrapper::dIntegrateTransport2)

    .def("difference", LieGroupWrapper::difference)
    .def("dDifference", LieGroupWrapper::dDifference)

    .def("interpolate", LieGroupWrapper::interpolate)

    .def("random", static_cast<typename LieGroupType::ConfigVector_t (LieGroupType::*)() const>(&LieGroupType::random))
    .def("randomConfiguration", &LieGroupType::template randomConfiguration<ConfigVector_t, ConfigVector_t>)
    .def("distance", &LieGroupType::template distance<ConfigVector_t, ConfigVector_t>)
    .def("squaredDistance", &LieGroupType::template squaredDistance<ConfigVector_t, ConfigVector_t>)
    .def("normalize", &LieGroupType::template normalize<ConfigVector_t>)

    .add_property("name", &LieGroupType::name)
    .add_property("neutral", &LieGroupType::neutral)
    .add_property("nq", &LieGroupType::nq)
    .add_property("nv", &LieGroupType::nv)

    .def(bp::self * bp::self)
    .def(bp::self *= bp::self)
    .def(bp::self == bp::self)
    ;
  }

  static void expose(const char* name)
  {
    bp::class_<LieGroupType>(name, bp::no_init)
        .def(LieGroupPythonVisitor<LieGroupType>());
  }
};
} // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_geometry_model_hpp__
