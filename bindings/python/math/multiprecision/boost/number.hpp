//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_python_math_multiprecision_boost_number_hpp__
#define __pinocchio_python_math_multiprecision_boost_number_hpp__

#include "pinocchio/math/multiprecision.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

#include <boost/python.hpp>
#include <eigenpy/user-type.hpp>
#include <eigenpy/ufunc.hpp>
#include <sstream>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename BoostNumber>
    struct BoostNumberPythonVisitor
    : public boost::python::def_visitor< BoostNumberPythonVisitor<BoostNumber> >
    {

    public:

      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Default constructor.",bp::arg("self")))
        .def(bp::init<BoostNumber>("Copy constructor.",bp::args("self","value")))
        .def(bp::init<float>("Copy constructor.",bp::args("self","value")))
        .def(bp::init<double>("Copy constructor.",bp::args("self","value")))
        .def(bp::init<int>("Copy constructor.",bp::args("self","value")))
        .def(bp::init<long int>("Copy constructor.",bp::args("self","value")))
        .def(bp::init<unsigned int>("Copy constructor.",bp::args("self","value")))
        .def(bp::init<unsigned long int>("Copy constructor.",bp::args("self","value")))
        .def(bp::init<bool>("Copy constructor.",bp::args("self","value")))
        .def(bp::init<std::string>("Constructor from a string.",bp::args("self","str_value")))
        
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wself-assign-overloaded"
        .def(bp::self +  bp::self)
        .def(bp::self += bp::self)
        .def(bp::self -  bp::self)
        .def(bp::self -= bp::self)
        .def(bp::self *  bp::self)
        .def(bp::self *= bp::self)
        .def(bp::self /  bp::self)
        .def(bp::self /= bp::self)
        
        .def(bp::self <  bp::self)
        .def(bp::self <= bp::self)
        .def(bp::self >  bp::self)
        .def(bp::self >= bp::self)
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
#pragma GCC diagnostic pop
      
        .def("str",&BoostNumber::str,bp::args("self","precision","scientific"))

        .def("default_precision",
             static_cast<unsigned (*)()>(BoostNumber::default_precision),
             "Get the default precision of the class.")
        .def("default_precision",
             static_cast<void (*)(unsigned)>(BoostNumber::default_precision),bp::arg("digits10"),
             "Set the default precision of the class.")
        .staticmethod("default_precision")

        .def("precision",
             static_cast<unsigned (BoostNumber::*)() const>(&BoostNumber::precision),
             bp::arg("self"),
             "Get the precision of this.")
        .def("precision",
             static_cast<void (BoostNumber::*)(unsigned)>(&BoostNumber::precision),
             bp::args("self","digits10"),
             "Set the precision of this.")

//#ifndef PINOCCHIO_PYTHON_NO_SERIALIZATION
//        .def_pickle(Pickle())
//#endif
        ;
      }
      
      static void expose(const std::string & type_name)
      {
        bp::class_<BoostNumber>(type_name.c_str(),
                                "",bp::no_init)
        .def(BoostNumberPythonVisitor<BoostNumber>())
        .def(PrintableVisitor<BoostNumber>())
        ;
        
        eigenpy::registerNewType<BoostNumber>();
        eigenpy::registerCommonUfunc<BoostNumber>();
        
        bp::implicitly_convertible<float,BoostNumber>();
        bp::implicitly_convertible<double,BoostNumber>();
        bp::implicitly_convertible<int,BoostNumber>();
        bp::implicitly_convertible<long int,BoostNumber>();
        bp::implicitly_convertible<unsigned int,BoostNumber>();
        bp::implicitly_convertible<unsigned long int,BoostNumber>();
        bp::implicitly_convertible<bool,BoostNumber>();
        
        eigenpy::registerCast<BoostNumber,double>(false);
        eigenpy::registerCast<double,BoostNumber>(true);
        eigenpy::registerCast<BoostNumber,float>(false);
        eigenpy::registerCast<float,BoostNumber>(true);
        eigenpy::registerCast<BoostNumber,int64_t>(false);
        eigenpy::registerCast<int64_t,BoostNumber>(true);
      }
      
    private:
      
//      struct Pickle : bp::pickle_suite
//      {
//        static
//        boost::python::tuple
//        getinitargs(const SE3 & M)
//        { return bp::make_tuple((Matrix3)M.rotation(),(Vector3)M.translation()); }
//      };
    };
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_math_multiprecision_boost_number_hpp__
