#ifndef __se3_python_inertia_hpp__
#define __se3_python_inertia_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/spatial/inertia.hpp"


namespace eigenpy
{
  template<>
  struct UnalignedEquivalent<se3::Inertia>
  {
    typedef se3::InertiaTpl<double,Eigen::DontAlign> type;
  };

} // namespace eigenpy

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename Inertia>
    struct InertiaPythonVisitor
      : public boost::python::def_visitor< InertiaPythonVisitor<Inertia> >
    {
      typedef typename eigenpy::UnalignedEquivalent<Inertia>::type Inertia_fx;
      typedef typename Inertia::Matrix3 Matrix3;
      typedef typename Inertia::Matrix6 Matrix6;
      typedef typename Inertia::Vector6 Vector6;
      typedef typename Inertia::Vector3 Vector3;

      typedef typename Inertia_fx::Matrix3 Matrix3_fx;
      typedef typename Inertia_fx::Matrix6 Matrix6_fx;
      typedef typename Inertia_fx::Vector6 Vector6_fx;
      typedef typename Inertia_fx::Vector3 Vector3_fx;
      typedef typename Inertia_fx::Motion  Motion_fx ;
    public:

      static PyObject* convert(Inertia const& m)
      {
	Inertia_fx m_fx = m;
	return bp::incref(bp::object(m_fx).ptr());
      }

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
	cl
	  .def("__init__",
	       bp::make_constructor(&InertiaPythonVisitor::makeFromMCI,
	   			    bp::default_call_policies(),
	   			    (bp::arg("mass"),bp::arg("lever"),bp::arg("inertia"))),
	       "Initialize from mass, lever and 3d inertia.")

	  .add_property("mass",&Inertia_fx::mass)
	  .add_property("lever",&InertiaPythonVisitor::lever)
	  .add_property("inertia",&InertiaPythonVisitor::inertia)

	  .def("matrix",&Inertia_fx::matrix)
	  .def("se3Action",&Inertia_fx::se3Action)
	  .def("se3ActionInverse",&Inertia_fx::se3ActionInverse)

	  .def("__str__",&InertiaPythonVisitor::toString)
	  .def( bp::self + bp::self)
	  .def( bp::self * bp::other<Motion_fx>() )
	  .add_property("np",&Inertia_fx::matrix)

	  .def("Identity",&Inertia_fx::Identity)
	  .staticmethod("Identity")
	  .def("Zero",&Inertia_fx::Zero)
	  .staticmethod("Zero")
	  .def("Random",&Inertia_fx::Random)
	  .staticmethod("Random")
	  ;
	  }

      static Inertia_fx* makeFromMCI(const double & mass,
				     const Vector3_fx & lever,
				     const Matrix3_fx & inertia) 
      {
	if(! inertia.isApprox(inertia.transpose()) ) 
	   throw eigenpy::Exception("The 3d inertia should be symmetric.");
	if( (Eigen::Vector3d::UnitX().transpose()*inertia*Eigen::Vector3d::UnitX()<0)
	    || (Eigen::Vector3d::UnitY().transpose()*inertia*Eigen::Vector3d::UnitY()<0)
	    || (Eigen::Vector3d::UnitZ().transpose()*inertia*Eigen::Vector3d::UnitZ()<0) )
	  throw eigenpy::Exception("The 3d inertia should be positive.");
	return new Inertia_fx(mass,lever,inertia); 
      }
      static Matrix3_fx inertia(const Inertia_fx& Y) { return Y.inertia().matrix(); }
      static Vector3_fx lever(const Inertia_fx& Y) { return Y.lever(); }
      static std::string toString(const Inertia_fx& m) 
      {	  std::ostringstream s; s << m; return s.str();       }

      static void expose()
      {
	bp::class_<Inertia_fx>("Inertia",
			     "Inertia matrix, in L(se3,se3*) == R^6x6.\n\n"
			     "Supported operations ...",
			     bp::init<>())
	  .def(InertiaPythonVisitor<Inertia>())
	;
    
	bp::to_python_converter< Inertia,InertiaPythonVisitor<Inertia> >();
    }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_se3_hpp__

