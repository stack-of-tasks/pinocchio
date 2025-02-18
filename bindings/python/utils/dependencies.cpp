//
// Copyright (c) 2020-2021 INRIA
//

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    void exposeDependencies()
    {
      bp::scope().attr("WITH_HPP_FCL") =
#ifdef PINOCCHIO_WITH_HPP_FCL
        true;
#else
        false;
#endif

      bp::scope().attr("WITH_URDFDOM") =
#ifdef PINOCCHIO_WITH_URDFDOM
        true;
#else
        false;
#endif

      bp::scope().attr("WITH_CPPAD") =
#ifdef PINOCCHIO_WITH_CPPAD
        true;
#else
        false;
#endif

      bp::scope().attr("WITH_OPENMP") =
#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP
        true;
#else
        false;
#endif

      bp::scope().attr("WITH_SDFORMAT") =
#ifdef PINOCCHIO_WITH_SDFORMAT
        true;
#else
        false;
#endif
    }

  } // namespace python
} // namespace pinocchio
