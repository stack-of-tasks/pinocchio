//
// Copyright (c) 2015-2018 CNRS
//

#ifndef __pinocchio_python_data_hpp__
#define __pinocchio_python_data_hpp__

#include "pinocchio/multibody/data.hpp"

#include <eigenpy/memory.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/bindings/python/utils/std-aligned-vector.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::Data)

namespace pinocchio
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
        .def(bp::init<Model>(bp::arg("model"),"Constructs a data structure from a given model."))
        
        .ADD_DATA_PROPERTY(container::aligned_vector<Motion>,a,"Joint spatial acceleration")
        .ADD_DATA_PROPERTY(container::aligned_vector<Motion>,a_gf,"Joint spatial acceleration containing also the contribution of the gravity acceleration")
        .ADD_DATA_PROPERTY(container::aligned_vector<Motion>,v,"Joint spatial velocity expressed in the joint frame.")
        .ADD_DATA_PROPERTY(container::aligned_vector<Force>,f,"Joint spatial force expresssed in the joint frame.")
        .ADD_DATA_PROPERTY(container::aligned_vector<SE3>,oMi,"Body absolute placement (wrt world)")
        .ADD_DATA_PROPERTY(container::aligned_vector<SE3>,oMf,"frames absolute placement (wrt world)")
        .ADD_DATA_PROPERTY(container::aligned_vector<SE3>,liMi,"Body relative placement (wrt parent)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,tau,"Joint torques (output of RNEA)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,nle,"Non Linear Effects (output of nle algorithm)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,ddq,"Joint accelerations (output of ABA)")
        .ADD_DATA_PROPERTY(container::aligned_vector<Inertia>,Ycrb,"Inertia of the sub-tree composit rigid body")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,M,"The joint space inertia matrix")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Data::RowMatrixXs,Minv,"The inverse of the joint space inertia matrix")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,C,"The Coriolis C(q,v) matrix such that the Coriolis effects are given by c(q,v) = C(q,v)v")
        .ADD_DATA_PROPERTY(container::aligned_vector<Matrix6x>,Fcrb,"Spatial forces set, used in CRBA")
        .ADD_DATA_PROPERTY(std::vector<int>,lastChild,"Index of the last child (for CRBA)")
        .ADD_DATA_PROPERTY(std::vector<int>,nvSubtree,"Dimension of the subtree motion space (for CRBA)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,U,"Joint Inertia square root (upper triangle)")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::VectorXd,D,"Diagonal of UDUT inertia decomposition")
        .ADD_DATA_PROPERTY(std::vector<int>,parents_fromRow,"First previous non-zero row in M (used in Cholesky)")
        .ADD_DATA_PROPERTY(std::vector<int>,nvSubtree_fromRow,"Subtree of the current row index (used in Cholesky)")
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
        
        .ADD_DATA_PROPERTY(container::aligned_vector<Vector3>,com,"CoM position of the subtree starting at joint index i.")
        .ADD_DATA_PROPERTY(container::aligned_vector<Vector3>,vcom,"CoM velocity of the subtree starting at joint index i.")
        .ADD_DATA_PROPERTY(container::aligned_vector<Vector3>,acom,"CoM acceleration of the subtree starting at joint index i..")
        .ADD_DATA_PROPERTY(std::vector<double>,mass,"Mass of the subtree starting at joint index i.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Matrix3x,Jcom,"Jacobian of center of mass.")

        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,C,"Joint space Coriolis matrix.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,dtau_dq,"Partial derivative of the joint torque vector with respect to the joint configuration.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,dtau_dv,"Partial derivative of the joint torque vector with respect to the joint velocity.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,ddq_dq,"Partial derivative of the joint acceleration vector with respect to the joint configuration.")
        .ADD_DATA_PROPERTY_READONLY_BYVALUE(Eigen::MatrixXd,ddq_dv,"Partial derivative of the joint acceleration vector with respect to the joint velocity.")
        
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
                         "Articulated rigid body data.\n"
                         "It contains all the data that can be modified by the algorithms.",
                         bp::no_init)
        .def(DataPythonVisitor());
        StdAlignedVectorPythonVisitor<Vector3, true>::expose("StdVec_vec3d");
        StdAlignedVectorPythonVisitor<Matrix6x, true>::expose("StdMat_Matrix6x");
        StdVectorPythonVisitor<int>::expose("StdVec_int");
        
        eigenpy::enableEigenPySpecific<Data::RowMatrixXs>();
      }

    };
    
  }} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_data_hpp__

