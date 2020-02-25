//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_python_algorithm_proximal_hpp__
#define __pinocchio_python_algorithm_proximal_hpp__

#include <boost/python.hpp>
#include "pinocchio/algorithm/proximal.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    template<typename ProximalSettings>
    struct ProximalSettingsPythonVisitor
    : public boost::python::def_visitor< ProximalSettingsPythonVisitor<ProximalSettings> >
    {
      typedef typename ProximalSettings::Scalar Scalar;
      
    public:
      
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Default constructor."))
        .def(bp::init<Scalar,Scalar,int>
             ((bp::arg("accuracy"),
               bp::arg("mu"),
               bp::arg("max_it")),
              "Structure containing all the settings paramters for the proximal algorithms."))
        
        .add_property("accuracy",&ProximalSettings::accuracy,"Minimum residual accuracy.")
        .add_property("mu",&ProximalSettings::mu,"Regularization parameter of the Proximal algorithms.")
        .add_property("max_it",&ProximalSettings::max_it,"Maximal number of iterations.")
        ;
      }
      
      static void expose()
      {
        bp::class_<ProximalSettings>("ProximalSettings",
                                     "Structure containing all the settings paramters for the Proximal algorithms.")
        .def(ProximalSettingsPythonVisitor<ProximalSettings>())
        ;
        
      }
    };
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_algorithm_proximal_hpp__
