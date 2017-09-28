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

#include "pinocchio/multibody/model.hpp"

#include <eigenpy/memory.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(se3::Data)

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    struct DataPythonVisitor
      : public boost::python::def_visitor< DataPythonVisitor >
    {
      typedef Data::Matrix6x Matrix6x;
      typedef Data::Matrix3x Matrix3x;
      typedef Data::Vector3 Vector3;

    public:

#define ADD_DATA_PROPERTY(TYPE,NAME,DOC)				 \
      def_readwrite(#NAME,						 \
      &Data::NAME,		 \
      DOC)
      
#define ADD_DATA_PROPERTY_READONLY(TYPE,NAME,DOC)				 \
      def_readonly(#NAME,						 \
      &Data::NAME,		 \
      DOC)
      
#define ADD_DATA_PROPERTY_READONLY_BYVALUE(TYPE,NAME,DOC)				 \
      add_property(#NAME,						 \
      make_getter(&Data::NAME,bp::return_value_policy<bp::return_by_value>()), \
      DOC)
      
	 
      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
        .def(bp::init<Model>(bp::arg("Molde"),"Constructs a data structure from a given model."))
        
        .ADD_DATA_PROPERTY(container::aligned_vector<Motion>,a,"Body acceleration")
        .ADD_DATA_PROPERTY(container::aligned_vector<Motion>,a_gf,"Body acceleration containing also the gravity acceleration")
        .ADD_DATA_PROPERTY(container::aligned_vector<Motion>,v,"Body velocity")
        .ADD_DATA_PROPERTY(container::aligned_vector<Force>,f,"Body force")
        .ADD_DATA_PROPERTY(container::aligned_vector<SE3>,oMi,"Body absolute placement (wrt world)")
        .ADD_DATA_PROPERTY(container::aligned_vector<SE3>,oMf,"frames absolute placement (wrt world)")
        .ADD_DATA_PROPERTY(container::aligned_vector<SE3>,liMi,"Body relative placement (wrt parent)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,tau,"Joint forces")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,nle,"Non Linear Effects")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,ddq,"Joint accelerations")
        .ADD_DATA_PROPERTY(container::aligned_vector<Inertia>,Ycrb,"Inertia of the sub-tree composit rigid body")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,M,"Joint Inertia matrix")
        .ADD_DATA_PROPERTY(container::aligned_vector<Matrix6x>,Fcrb,"Spatial forces set, used in CRBA")
        .ADD_DATA_PROPERTY(std::vector<int>,lastChild,"Index of the last child (for CRBA)")
        .ADD_DATA_PROPERTY(std::vector<int>,nvSubtree,"Dimension of the subtree motion space (for CRBA)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,U,"Joint Inertia square root (upper triangle)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,D,"Diagonal of UDUT inertia decomposition")
        .ADD_DATA_PROPERTY(std::vector<int>,parents_fromRow,"First previous non-zero row in M (used in Cholesky)")
        .ADD_DATA_PROPERTY(std::vector<int>,nvSubtree_fromRow,"")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Matrix6x,J,"Jacobian of joint placement")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Matrix6x,dJ,"Time variation of the Jacobian of joint placement (data.J).")
        .ADD_DATA_PROPERTY(container::aligned_vector<SE3>,iMf,"Body placement wrt to algorithm end effector.")
        
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Matrix6x,Ag,
                                 "Centroidal matrix which maps from joint velocity to the centroidal momentum.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Matrix6x,dAg,
                                 "Time derivative of the centroidal momentum matrix Ag.")
        .ADD_DATA_PROPERTY_READONLY(Force,hg,
                                    "Centroidal momentum (expressed in the frame centered at the CoM and aligned with the inertial frame).")
        .ADD_DATA_PROPERTY_READONLY(Inertia,Ig,
                                    "Centroidal Composite Rigid Body Inertia.")
        
        .ADD_DATA_PROPERTY(container::aligned_vector<Vector3>,com,"Subtree com position.")
        .ADD_DATA_PROPERTY(container::aligned_vector<Vector3>,vcom,"Subtree com velocity.")
        .ADD_DATA_PROPERTY(container::aligned_vector<Vector3>,acom,"Subtree com acceleration.")
        .ADD_DATA_PROPERTY(std::vector<double>,mass,"Subtree total mass.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Matrix3x,Jcom,"Jacobian of center of mass.")

        
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(double,kinetic_energy,"Kinetic energy in [J] computed by kineticEnergy(model,data,q,v,True/False)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(double,potential_energy,"Potential energy in [J] computed by potentialEnergy(model,data,q,True/False)")
        
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,lambda_c,"Lagrange Multipliers linked to contact forces")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,impulse_c,"Lagrange Multipliers linked to contact impulses")
        
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,dq_after,"Generalized velocity after the impact.")
        ;
      }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        bp::class_<Data>("Data",
                         "Articulated rigid body data (const)",
                         bp::no_init)
        .def(DataPythonVisitor());
        
        bp::class_< container::aligned_vector<Vector3> >("StdVec_vec3d")
        .def(bp::vector_indexing_suite< container::aligned_vector<Vector3>, true >());
      }

    };
    
  }} // namespace se3::python

#endif // ifndef __se3_python_data_hpp__

