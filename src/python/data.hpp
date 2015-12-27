//
// Copyright (c) 2015-2016 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

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
      typedef Data::Vector3 Vector3;

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
    .ADD_DATA_PROPERTY_CONST(Eigen::VectorXd,nle,"Non Linear Effects")
	  .ADD_DATA_PROPERTY(std::vector<Inertia>,Ycrb,"Inertia of the sub-tree composit rigid body")
	  .ADD_DATA_PROPERTY_CONST(Eigen::MatrixXd,M,"Joint Inertia")
	  .ADD_DATA_PROPERTY_CONST(std::vector<Matrix6x>,Fcrb,"Spatial forces set, used in CRBA")
	  .ADD_DATA_PROPERTY(std::vector<int>,lastChild,"Index of the last child (for CRBA)")
	  .ADD_DATA_PROPERTY(std::vector<int>,nvSubtree,"Dimension of the subtree motion space (for CRBA)")
	  .ADD_DATA_PROPERTY_CONST(Eigen::MatrixXd,U,"Joint Inertia square root (upper triangle)")
	  .ADD_DATA_PROPERTY_CONST(Eigen::VectorXd,D,"Diagonal of UDUT inertia decomposition")
	  .ADD_DATA_PROPERTY(std::vector<int>,parents_fromRow,"First previous non-zero row in M (used in Cholesky)")
	  .ADD_DATA_PROPERTY(std::vector<int>,nvSubtree_fromRow,"")
	  .ADD_DATA_PROPERTY_CONST(Matrix6x,J,"Jacobian of joint placement")
	  .ADD_DATA_PROPERTY(std::vector<SE3>,iMf,"Body placement wrt to algorithm end effector.")
        
	  .ADD_DATA_PROPERTY(std::vector<Vector3>,com,"Subtree com position.")
    .ADD_DATA_PROPERTY(std::vector<Vector3>,vcom,"Subtree com velocity.")
    .ADD_DATA_PROPERTY(std::vector<Vector3>,acom,"Subtree com acceleration.")
	  .ADD_DATA_PROPERTY(std::vector<double>,mass,"Subtree total mass.")
	  .ADD_DATA_PROPERTY(Matrix3x,Jcom,"Jacobian of center of mass.")

    .ADD_DATA_PROPERTY_CONST(Eigen::VectorXd,effortLimit,"Joint max effort")
    .ADD_DATA_PROPERTY_CONST(Eigen::VectorXd,velocityLimit,"Joint max velocity")
        
    .ADD_DATA_PROPERTY_CONST(Eigen::VectorXd,lowerPositionLimit,"Limit for joint lower position")
    .ADD_DATA_PROPERTY_CONST(Eigen::VectorXd,upperPositionLimit,"Limit for joint upper position")
        
        .ADD_DATA_PROPERTY_CONST(double,kinetic_energy,"Kinetic energy in [J] computed by kineticEnergy(model,data,q,v,True/False)")
        .ADD_DATA_PROPERTY_CONST(double,potential_energy,"Potential energy in [J] computed by potentialEnergy(model,data,q,True/False)")
	  ;
      }

      IMPL_DATA_PROPERTY(std::vector<Motion>,a,"Body acceleration")
      IMPL_DATA_PROPERTY(std::vector<Motion>,v,"Body velocity")
      IMPL_DATA_PROPERTY(std::vector<Force>,f,"Body force")
      IMPL_DATA_PROPERTY(std::vector<SE3>,oMi,"Body absolute placement (wrt world)")
      IMPL_DATA_PROPERTY(std::vector<SE3>,liMi,"Body relative placement (wrt parent)")
      IMPL_DATA_PROPERTY_CONST(Eigen::VectorXd,tau,"Joint forces")
      IMPL_DATA_PROPERTY_CONST(Eigen::VectorXd,nle,"Non Linear Effects")
      IMPL_DATA_PROPERTY(std::vector<Inertia>,Ycrb,"Inertia of the sub-tree composit rigid body")
      IMPL_DATA_PROPERTY_CONST(Eigen::MatrixXd,M,"Joint Inertia")
      IMPL_DATA_PROPERTY_CONST(std::vector<Matrix6x>,Fcrb,"Spatial forces set, used in CRBA")
      IMPL_DATA_PROPERTY(std::vector<int>,lastChild,"Index of the last child (for CRBA)")
      IMPL_DATA_PROPERTY(std::vector<int>,nvSubtree,"Dimension of the subtree motion space (for CRBA)")
      IMPL_DATA_PROPERTY_CONST(Eigen::MatrixXd,U,"Joint Inertia square root (upper triangle)")
      IMPL_DATA_PROPERTY_CONST(Eigen::VectorXd,D,"Diagonal of UDUT inertia decomposition")
      IMPL_DATA_PROPERTY(std::vector<int>,parents_fromRow,"First previous non-zero row in M (used in Cholesky)")
      IMPL_DATA_PROPERTY(std::vector<int>,nvSubtree_fromRow,"")
      IMPL_DATA_PROPERTY_CONST(Matrix6x,J,"Jacobian of joint placement")
      IMPL_DATA_PROPERTY(std::vector<SE3>,iMf,"Body placement wrt to algorithm end effector.")
      
      IMPL_DATA_PROPERTY(std::vector<Vector3>,com,"Subtree com position.")
      IMPL_DATA_PROPERTY(std::vector<Vector3>,vcom,"Subtree com velocity.")
      IMPL_DATA_PROPERTY(std::vector<Vector3>,acom,"Subtree com acceleration.")
      IMPL_DATA_PROPERTY(std::vector<double>,mass,"Subtree total mass.")
      IMPL_DATA_PROPERTY(Matrix3x,Jcom,"Jacobian of center of mass.")

      IMPL_DATA_PROPERTY_CONST(Eigen::VectorXd,effortLimit,"Joint max effort")
      IMPL_DATA_PROPERTY_CONST(Eigen::VectorXd,velocityLimit,"Joint max velocity")

      IMPL_DATA_PROPERTY_CONST(Eigen::VectorXd,lowerPositionLimit,"Limit for joint lower position")
      IMPL_DATA_PROPERTY_CONST(Eigen::VectorXd,upperPositionLimit,"Limit for joint upper position")
      
      IMPL_DATA_PROPERTY_CONST(double,kinetic_energy,"Kinetic energy in [J] computed by kineticEnergy(model,data,q,v,True/False)")
      IMPL_DATA_PROPERTY_CONST(double,potential_energy,"Potential energy in [J] computed by potentialEnergy(model,data,q,True/False)")
      
      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        bp::class_<DataHandler>("Data",
                                "Articulated rigid body data (const)",
                                bp::no_init)
        .def(DataPythonVisitor());
        
        bp::to_python_converter< DataHandler::SmartPtr_t,DataPythonVisitor >();
        bp::class_< std::vector<Vector3> >("StdVec_vec3d")
        .def(bp::vector_indexing_suite< std::vector<Vector3>, true >());
      }

    };
    
  }} // namespace se3::python

#endif // ifndef __se3_python_data_hpp__

