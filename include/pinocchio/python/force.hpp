#ifndef __se3_python_force_hpp__
#define __se3_python_force_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/spatial/force.hpp"


namespace eigenpy
{
  template<>
  struct UnalignedEquivalent<se3::Force>
  {
    typedef se3::ForceTpl<double,Eigen::DontAlign> type;
  };
} // namespace eigenpy

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Force>
    struct ForcePythonVisitor
      : public boost::python::def_visitor< ForcePythonVisitor<Force> >
    {
      typedef typename eigenpy::UnalignedEquivalent<Force>::type Force_fx;
      typedef typename Force::Matrix3 Matrix3;
      typedef typename Force::Matrix6 Matrix6;
      typedef typename Force::Vector6 Vector6;
      typedef typename Force::Vector3 Vector3;

      typedef typename Force_fx::Matrix3 Matrix3_fx;
      typedef typename Force_fx::Matrix6 Matrix6_fx;
      typedef typename Force_fx::Vector6 Vector6_fx;
      typedef typename Force_fx::Vector3 Vector3_fx;

    public:

      static PyObject* convert(Force const& m)
      {
	Force_fx m_fx (m);
	return boost::python::incref(boost::python::object(m_fx).ptr());
      }

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
	cl
	  .def(bp::init<Vector3_fx,Vector3_fx>
	       ((bp::arg("linear"),bp::arg("angular")),
		"Initialize from linear and angular components (dont mix the order)."))
	  .def(bp::init<Vector6_fx>((bp::arg("Vector 6d")),"Init from vector 6 [f,n]"))

	  .add_property("linear",&ForcePythonVisitor::getLinear,&ForcePythonVisitor::setLinear)
	  .add_property("angular",&ForcePythonVisitor::getAngular,&ForcePythonVisitor::setAngular)
	  .def("vector",&Force_fx::toVector)
	  .def("se3Action",&Force_fx::se3Action)
	  .def("se3ActionInverse",&Force_fx::se3ActionInverse)

	  .def("__str__",&ForcePythonVisitor::toString)
	  .add_property("np",&Force_fx::toVector)
	  
	  .def("Random",&Force_fx::Random)
	  .staticmethod("Random")
	  .def("Zero",&Force_fx::Zero)
	  .staticmethod("Zero")
	  ;
	  }

      static Vector3_fx getLinear( const Force_fx & self ) { return self.linear(); }
      static void setLinear( Force_fx & self, const Vector3_fx & R ) { self.linear(R); }
      static Vector3_fx getAngular( const Force_fx & self ) { return self.angular(); }
      static void setAngular( Force_fx & self, const Vector3_fx & R ) { self.angular(R); }
      static std::string toString(const Force_fx& m) 
      {	  std::ostringstream s; s << m; return s.str();       }


      static void expose()
      {
	bp::class_<Force_fx>("Force",
			     "Force vectors, in se3* == F^6.\n\n"
			     "Supported operations ...",
			     bp::init<>())
	  .def(ForcePythonVisitor<Force>())
	;
    
	bp::to_python_converter< Force,ForcePythonVisitor<Force> >();
    }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_se3_hpp__

