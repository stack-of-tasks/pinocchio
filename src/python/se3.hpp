#ifndef __se3_python_se3_hpp__
#define __se3_python_se3_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/spatial/se3.hpp"


namespace eigenpy
{
  template<>
  struct UnalignedEquivalent<se3::SE3>
  {
    typedef se3::SE3Tpl<double,Eigen::DontAlign> type;
  };
} // namespace eigenpy

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename SE3>
    struct SE3PythonVisitor
      : public boost::python::def_visitor< SE3PythonVisitor<SE3> >
    {
      typedef typename eigenpy::UnalignedEquivalent<SE3>::type SE3_fx;
      typedef typename SE3::Matrix3 Matrix3;
      typedef typename SE3::Matrix6 Matrix6;
      typedef typename SE3::Matrix4 Matrix4;
      typedef typename SE3::Vector6 Vector6;
      typedef typename SE3::Vector3 Vector3;

      typedef typename SE3_fx::Matrix3 Matrix3_fx;
      typedef typename SE3_fx::Matrix6 Matrix6_fx;
      typedef typename SE3_fx::Matrix4 Matrix4_fx;
      typedef typename SE3_fx::Vector6 Vector6_fx;
      typedef typename SE3_fx::Vector3 Vector3_fx;

    public:

      static PyObject* convert(SE3 const& m)
      {
	SE3_fx m_fx = m;
	return boost::python::incref(boost::python::object(m_fx).ptr());
      }

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
	cl
	  .def(bp::init<Matrix3_fx,Vector3_fx>
	       ((bp::arg("Rotation"),bp::arg("translation")),
		"Initialize from rotation and translation."))
	  .def(bp::init<int>
	       ((bp::arg("trivial arg (should be 1)")),
	   	"Init to identity."))

	  .add_property("rotation",&SE3PythonVisitor::getRotation,&SE3PythonVisitor::setRotation)
	  .add_property("translation",&SE3PythonVisitor::getTranslation,&SE3PythonVisitor::setTranslation)
	  .add_property("homogeneous",&SE3_fx::toHomogeneousMatrix)
	  .add_property("action",&SE3_fx::toActionMatrix)

	  .def("inverse", &SE3_fx::inverse)
	  .def("act_point", &SE3PythonVisitor::act_point)
	  .def("actInv_point", &SE3PythonVisitor::actInv_point)
	  .def("act_se3", &SE3PythonVisitor::act_se3)
	  .def("actInv_se3", &SE3PythonVisitor::actInv_se3)
	  
	  .def("__str__",&SE3PythonVisitor::toString)
	  .def("__invert__",&SE3_fx::inverse)
	  .def(bp::self * bp::self)
	  .add_property("np",&SE3_fx::toActionMatrix)

	  .def("Identity",&SE3_fx::Identity)
	  .staticmethod("Identity")
	  .def("Random",&SE3_fx::Random)
	  .staticmethod("Random")
	  ;
	  }

      static Matrix3_fx getRotation( const SE3_fx & self ) { return self.rotation(); }
      static void setRotation( SE3_fx & self, const Matrix3_fx & R ) { self.rotation(R); }
      static Vector3_fx getTranslation( const SE3_fx & self ) { return self.translation(); }
      static void setTranslation( SE3_fx & self, const Vector3_fx & p ) { self.translation(p); }

      static Vector3_fx act_point( const SE3_fx & self, const Vector3_fx & pt ) { return self.act(pt); }
      static Vector3_fx actInv_point( const SE3_fx & self, const Vector3_fx & pt ) { return self.actInv(pt); }
      static SE3_fx act_se3( const SE3_fx & self, const SE3_fx & x ) { return self.act(x); }
      static SE3_fx actInv_se3( const SE3_fx & self, const SE3_fx & x ) { return self.actInv(x); }
      static std::string toString(const SE3_fx& m) 
      {	  std::ostringstream s; s << m; return s.str();       }

      static void expose()
      {
	bp::class_<SE3_fx>("SE3",
			   "SE3 transformations, composing rotation and translation.\n\n"
			   "Supported operations ...",
			   bp::init<>())
	  .def(SE3PythonVisitor<SE3>())
	;
    
	bp::to_python_converter< SE3,SE3PythonVisitor<SE3> >();
    }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_se3_hpp__

