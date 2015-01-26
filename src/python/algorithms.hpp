#ifndef __se3_python_algorithm_hpp__
#define __se3_python_algorithm_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/python/model.hpp"
#include "pinocchio/python/data.hpp"

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"

namespace se3
{
  namespace python
  {
    struct AlgorithmsPythonVisitor
    {
      typedef typename eigenpy::UnalignedEquivalent<Eigen::VectorXd>::type VectorXd_fx;

      static Eigen::VectorXd rnea_proxy( const ModelHandler& model, 
			      DataHandler & data,
			      const VectorXd_fx & q,
			      const VectorXd_fx & qd,
			      const VectorXd_fx & qdd )
      { return rnea(*model,*data,q,qd,qdd); }

      static Eigen::MatrixXd crba_proxy( const ModelHandler& model, 
					 DataHandler & data,
					 const VectorXd_fx & q )
      {
	data->M.fill(0);
	return crba(*model,*data,q); 
      }

      static Eigen::VectorXd com_proxy( const ModelHandler& model, 
					DataHandler & data,
					const VectorXd_fx & q )
      { return centerOfMass(*model,*data,q); }

      static Eigen::MatrixXd Jcom_proxy( const ModelHandler& model, 
					 DataHandler & data,
					 const VectorXd_fx & q )
      { return jacobianCenterOfMass(*model,*data,q); }

      static Eigen::MatrixXd jacobian_proxy( const ModelHandler& model, 
					     DataHandler & data,
					     Model::Index jointId,
					     const VectorXd_fx & q,
					     bool local )
      {
	Eigen::MatrixXd J( 6,model->nv ); J.setZero();
	computeJacobians( *model,*data,q );
	if(local) getJacobian<true> (*model, *data, jointId, J);
	else getJacobian<false> (*model, *data, jointId, J);
	return J;
      }

      static void kinematics_proxy( const ModelHandler& model, 
				    DataHandler & data,
				    const VectorXd_fx & q,
				    const VectorXd_fx & qdot )
      {
	kinematics( *model,*data,q,qdot );
      }



      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
	bp::def("rnea",rnea_proxy,
		bp::args("Model","Data",
			 "Configuration q (size Model::nq)",
			 "Velocity qdot (size Model::nv)",
			 "Acceleration qddot (size Model::nv)"),
		"Compute the RNEA, put the result in Data and return it.");

	bp::def("crba",crba_proxy,
		bp::args("Model","Data",
			 "Configuration q (size Model::nq)"),
		"Compute CRBA, put the result in Data and return it.");

 	bp::def("centerOfMass",com_proxy,
		bp::args("Model","Data",
			 "Configuration q (size Model::nq)"),
		"Compute the center of mass, putting the result in Data and return it.");

	bp::def("jacobianCenterOfMass",Jcom_proxy,
		bp::args("Model","Data",
			 "Configuration q (size Model::nq)"),
		"Compute the jacobian of the center of mass, put the result in Data and return it.");

	bp::def("kinematics",kinematics_proxy,
		bp::args("Model","Data",
			 "Configuration q (size Model::nq)",
			 "Velocity qdot (size Model::nv)"),
		"Compute the placements and spatial velocities of all the frames of the kinematic "
		"tree and put the results in data.");

	bp::def("jacobian",jacobian_proxy,
		bp::args("Model","Data",
			 "Configuration q (size Model::nq)",
			 "Joint ID (int)",
			 "frame (true = local, false = world)"),
		"Calling computeJacobians then getJacobian, return the result. Attention: the "
		"function computes indeed all the jacobians of the model, even if just outputing "
		"the demanded one. It is therefore outrageously costly wrt a dedicated "
		"call. Function to be used only for prototyping.");

      }

    };
    
  }} // namespace se3::python

#endif // ifndef __se3_python_data_hpp__

