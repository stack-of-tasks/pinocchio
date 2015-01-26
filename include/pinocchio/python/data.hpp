#ifndef __se3_python_data_hpp__
#define __se3_python_data_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/multibody/model.hpp"

#include <boost/shared_ptr.hpp>

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    typedef Handler<Data> DataHandler;

    struct DataPythonVisitor
      : public boost::python::def_visitor< DataPythonVisitor >
    {
      typedef Data::Matrix6x Matrix6x;
      typedef Data::Matrix3x Matrix3x;

    public:

      /* --- Convert From C++ to Python ------------------------------------- */
      static PyObject* convert(DataHandler::SmartPtr_t const& ptr)
      {
	return boost::python::incref(boost::python::object(DataHandler(ptr)).ptr());
      }

#define ADD_DATA_PROPERTY(TYPE,NAME,DOC)				 \
      add_property(#NAME,						 \
		   bp::make_function(&DataPythonVisitor::NAME,		 \
				     bp::return_internal_reference<>()), \
		   DOC)
#define ADD_DATA_PROPERTY_CONST(TYPE,NAME,DOC)				 \
      add_property(#NAME,						 \
		   bp::make_function(&DataPythonVisitor::NAME),		 \
		   DOC)

#define IMPL_DATA_PROPERTY(TYPE,NAME,DOC)			\
      static TYPE & NAME( DataHandler & d ) { return d->NAME; }                
#define IMPL_DATA_PROPERTY_CONST(TYPE,NAME,DOC)			\
      static TYPE NAME( DataHandler & d ) { return d->NAME; }                
       
	 
      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
	cl
	  .ADD_DATA_PROPERTY(std::vector<Motion>,a,"Body acceleration")
	  .ADD_DATA_PROPERTY(std::vector<Motion>,v,"Body velocity")
	  .ADD_DATA_PROPERTY(std::vector<Force>,f,"Body force")
	  .ADD_DATA_PROPERTY(std::vector<SE3>,oMi,"Body absolute placement (wrt world)")
	  .ADD_DATA_PROPERTY(std::vector<SE3>,liMi,"Body relative placement (wrt parent)")
	  .ADD_DATA_PROPERTY_CONST(Eigen::VectorXd,tau,"Joint forces")
	  .ADD_DATA_PROPERTY(std::vector<Inertia>,Ycrb,"Inertia of the sub-tree composit rigid body")
	  .ADD_DATA_PROPERTY_CONST(Eigen::MatrixXd,M,"Joint Inertia")
	  .ADD_DATA_PROPERTY_CONST(std::vector<Matrix6x>,Fcrb,"Spatial forces set, used in CRBA")
	  .ADD_DATA_PROPERTY(std::vector<Model::Index>,lastChild,"Index of the last child (for CRBA)")
	  .ADD_DATA_PROPERTY(std::vector<int>,nvSubtree,"Dimension of the subtree motion space (for CRBA)")
	  .ADD_DATA_PROPERTY_CONST(Eigen::MatrixXd,U,"Joint Inertia square root (upper triangle)")
	  .ADD_DATA_PROPERTY_CONST(Eigen::VectorXd,D,"Diagonal of UDUT inertia decomposition")
	  .ADD_DATA_PROPERTY(std::vector<int>,parents_fromRow,"First previous non-zero row in M (used in Cholesky)")
	  .ADD_DATA_PROPERTY(std::vector<int>,nvSubtree_fromRow,"")
	  .ADD_DATA_PROPERTY_CONST(Eigen::MatrixXd,J,"Jacobian of joint placement")
	  .ADD_DATA_PROPERTY(std::vector<SE3>,iMf,"Body placement wrt to algorithm end effector.")
	  .ADD_DATA_PROPERTY_CONST(std::vector<Eigen::Vector3d>,com,"Subtree com position.")
	  .ADD_DATA_PROPERTY(std::vector<double>,mass,"Subtree total mass.")
	  .ADD_DATA_PROPERTY_CONST(Matrix3x,Jcom,"Jacobian of center of mass.")
	  ;
      }

      IMPL_DATA_PROPERTY(std::vector<Motion>,a,"Body acceleration")
      IMPL_DATA_PROPERTY(std::vector<Motion>,v,"Body velocity")
      IMPL_DATA_PROPERTY(std::vector<Force>,f,"Body force")
      IMPL_DATA_PROPERTY(std::vector<SE3>,oMi,"Body absolute placement (wrt world)")
      IMPL_DATA_PROPERTY(std::vector<SE3>,liMi,"Body relative placement (wrt parent)")
      IMPL_DATA_PROPERTY_CONST(Eigen::VectorXd,tau,"Joint forces")
      IMPL_DATA_PROPERTY(std::vector<Inertia>,Ycrb,"Inertia of the sub-tree composit rigid body")
      IMPL_DATA_PROPERTY_CONST(Eigen::MatrixXd,M,"Joint Inertia")
      IMPL_DATA_PROPERTY_CONST(std::vector<Matrix6x>,Fcrb,"Spatial forces set, used in CRBA")
      IMPL_DATA_PROPERTY(std::vector<Model::Index>,lastChild,"Index of the last child (for CRBA)")
      IMPL_DATA_PROPERTY(std::vector<int>,nvSubtree,"Dimension of the subtree motion space (for CRBA)")
      IMPL_DATA_PROPERTY_CONST(Eigen::MatrixXd,U,"Joint Inertia square root (upper triangle)")
      IMPL_DATA_PROPERTY_CONST(Eigen::VectorXd,D,"Diagonal of UDUT inertia decomposition")
      IMPL_DATA_PROPERTY(std::vector<int>,parents_fromRow,"First previous non-zero row in M (used in Cholesky)")
      IMPL_DATA_PROPERTY(std::vector<int>,nvSubtree_fromRow,"")
      IMPL_DATA_PROPERTY_CONST(Eigen::MatrixXd,J,"Jacobian of joint placement")
      IMPL_DATA_PROPERTY(std::vector<SE3>,iMf,"Body placement wrt to algorithm end effector.")
      IMPL_DATA_PROPERTY_CONST(std::vector<Eigen::Vector3d>,com,"Subtree com position.")
      IMPL_DATA_PROPERTY(std::vector<double>,mass,"Subtree total mass.")
      IMPL_DATA_PROPERTY_CONST(Matrix3x,Jcom,"Jacobian of center of mass.")

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
	bp::class_<DataHandler>("Data",
				"Articulated rigid body data (const)",
				bp::no_init)
	  .def(DataPythonVisitor());
    
	bp::to_python_converter< DataHandler::SmartPtr_t,DataPythonVisitor >();
      }

    };
    
  }} // namespace se3::python

#endif // ifndef __se3_python_data_hpp__

