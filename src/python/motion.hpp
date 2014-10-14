#ifndef __se3_python_motion_hpp__
#define __se3_python_motion_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/force.hpp"


namespace eigenpy
{
  template<>
  struct UnalignedEquivalent<se3::Motion>
  {
    typedef se3::MotionTpl<double,Eigen::DontAlign> type;
  };
} // namespace eigenpy

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Motion>
    struct MotionPythonVisitor
      : public boost::python::def_visitor< MotionPythonVisitor<Motion> >
    {
      typedef typename Motion::Force Force;
      typedef typename Motion::Matrix3 Matrix3;
      typedef typename Motion::Matrix6 Matrix6;
      typedef typename Motion::Vector6 Vector6;
      typedef typename Motion::Vector3 Vector3;

      typedef typename eigenpy::UnalignedEquivalent<Motion>::type Motion_fx;
      typedef typename Motion_fx::Force Force_fx;
      typedef typename Motion_fx::Matrix3 Matrix3_fx;
      typedef typename Motion_fx::Matrix6 Matrix6_fx;
      typedef typename Motion_fx::Vector6 Vector6_fx;
      typedef typename Motion_fx::Vector3 Vector3_fx;

    public:

      static PyObject* convert(Motion const& m)
      {
	Motion_fx m_fx (m);
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

	  .add_property("linear",&MotionPythonVisitor::getLinear,&MotionPythonVisitor::setLinear)
	  .add_property("angular",&MotionPythonVisitor::getAngular,&MotionPythonVisitor::setAngular)
	  .def("vector",&Motion_fx::toVector)
	  .def("se3Action",&Motion_fx::se3Action)
	  .def("se3ActionInverse",&Motion_fx::se3ActionInverse)

	  .def("cross_motion",&MotionPythonVisitor::cross_motion)
	  .def("cross_force",&MotionPythonVisitor::cross_force)
	  
	  .def("__str__",&MotionPythonVisitor::toString)
	  .add_property("np",&Motion_fx::toVector)

	  .def("Random",&Motion_fx::Random)
	  .staticmethod("Random")
	  .def("Zero",&Motion_fx::Zero)
	  .staticmethod("Zero")
	  ;
	  }

      static Vector3_fx getLinear( const Motion_fx & self ) { return self.linear(); }
      static void setLinear( Motion_fx & self, const Vector3_fx & R ) { self.linear(R); }
      static Vector3_fx getAngular( const Motion_fx & self ) { return self.angular(); }
      static void setAngular( Motion_fx & self, const Vector3_fx & R ) { self.angular(R); }
      
      static Motion_fx cross_motion( const Motion_fx& m1,const Motion_fx& m2 ) { return m1.cross(m2); }
      static Force_fx cross_force( const Motion_fx& m,const Force_fx& f ) { return m.cross(f); }

      static std::string toString(const Motion_fx& m) 
      {	  std::ostringstream s; s << m; return s.str();       }

      static void expose()
      {
	bp::class_<Motion_fx>("Motion",
			     "Motion vectors, in se3* == F^6.\n\n"
			     "Supported operations ...",
			     bp::init<>())
	  .def(MotionPythonVisitor<Motion>())
	;
    
	bp::to_python_converter< Motion,MotionPythonVisitor<Motion> >();
    }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_se3_hpp__

