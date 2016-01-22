//
// Copyright (c) 2015 CNRS
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

#ifndef __se3_python_algorithm_hpp__
#define __se3_python_algorithm_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/python/model.hpp"
#include "pinocchio/python/data.hpp"

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/non-linear-effects.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/joint-limits.hpp"

#include "pinocchio/simulation/compute-all-terms.hpp"

#ifdef WITH_HPP_FCL
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/python/geometry-model.hpp"
#include "pinocchio/python/geometry-data.hpp"
#include "pinocchio/algorithm/collisions.hpp"
#endif

namespace se3
{
  namespace python
  {
    struct AlgorithmsPythonVisitor
    {
      typedef eigenpy::UnalignedEquivalent<Eigen::VectorXd>::type VectorXd_fx;

      static Eigen::VectorXd rnea_proxy( const ModelHandler& model, 
            DataHandler & data,
            const VectorXd_fx & q,
            const VectorXd_fx & qd,
            const VectorXd_fx & qdd )
      { return rnea(*model,*data,q,qd,qdd); }

      static Eigen::VectorXd nle_proxy( const ModelHandler& model,
                                        DataHandler & data,
                                        const VectorXd_fx & q,
                                        const VectorXd_fx & qd )
      { return nonLinearEffects(*model,*data,q,qd); }

      static Eigen::MatrixXd crba_proxy( const ModelHandler& model, 
           DataHandler & data,
           const VectorXd_fx & q )
      {
  data->M.fill(0);
  crba(*model,*data,q); 
  data->M.triangularView<Eigen::StrictlyLower>()
    = data->M.transpose().triangularView<Eigen::StrictlyLower>();
  return data->M;
      }

      static Eigen::VectorXd com_proxy( const ModelHandler& model, 
          DataHandler & data,
          const VectorXd_fx & q )
      { return centerOfMass(*model,*data,q); }
      
      static void com_acceleration_proxy(const ModelHandler& model,
                                         DataHandler & data,
                                         const VectorXd_fx & q,
                                         const VectorXd_fx & v,
                                         const VectorXd_fx & a)
      { return centerOfMassAcceleration(*model,*data,q,v,a); }

      static Eigen::MatrixXd Jcom_proxy( const ModelHandler& model, 
           DataHandler & data,
           const VectorXd_fx & q )
      { return jacobianCenterOfMass(*model,*data,q); }

      static Eigen::MatrixXd jacobian_proxy( const ModelHandler& model, 
               DataHandler & data,
               const VectorXd_fx & q,
               Model::Index jointId,
               bool local,
							 bool update_geometry )
      {
  Eigen::MatrixXd J( 6,model->nv ); J.setZero();
	if (update_geometry)
  	computeJacobians( *model,*data,q );
  if(local) getJacobian<true> (*model, *data, jointId, J);
  else getJacobian<false> (*model, *data, jointId, J);
  return J;
      }
      
      static void compute_jacobians_proxy(const ModelHandler& model, 
               DataHandler & data,
               const VectorXd_fx & q)
      {
        computeJacobians( *model,*data,q );
      }
      
      static void geometry_proxy(const ModelHandler & model,
                                 DataHandler & data,
                                 const VectorXd_fx & q)
      {
        geometry(*model,*data,q);
      }

      static void kinematics_proxy( const ModelHandler& model, 
            DataHandler & data,
            const VectorXd_fx & q,
            const VectorXd_fx & qdot )
      {
        kinematics( *model,*data,q,qdot );
      }

      static void dynamics_proxy(const ModelHandler& model,
                                 DataHandler & data,
                                 const VectorXd_fx & q,
                                 const VectorXd_fx & v,
                                 const VectorXd_fx & a)
      {
        dynamics(*model,*data,q,v,a);
      }

      static void computeAllTerms_proxy(const ModelHandler & model,
                                        DataHandler & data,
                                        const VectorXd_fx & q,
                                        const VectorXd_fx & v)
      {
	data->M.fill(0);
        computeAllTerms(*model,*data,q,v);
	data->M.triangularView<Eigen::StrictlyLower>()
	  = data->M.transpose().triangularView<Eigen::StrictlyLower>();
      }

      static void jointLimits_proxy(const ModelHandler & model,
                                    DataHandler & data,
                                    const VectorXd_fx & q)
      {
        jointLimits(*model,*data,q);
      }

#ifdef WITH_HPP_FCL
      static bool computeCollisions_proxy(GeometryDataHandler & data_geom,
                                          const bool stopAtFirstCollision)
      {
        return computeCollisions(*data_geom, stopAtFirstCollision);
      }

      static bool computeGeometryAndCollisions_proxy(const ModelHandler & model,
                                    DataHandler & data,
                                    const GeometryModelHandler & model_geom,
                                    GeometryDataHandler & data_geom,
                                    const VectorXd_fx & q,
                                    const bool & stopAtFirstCollision)
      {
        return computeCollisions(*model,*data,*model_geom, *data_geom, q, stopAtFirstCollision);
      }

      static void computeDistances_proxy(GeometryDataHandler & data_geom)
      {
        computeDistances(*data_geom);
      }

      static void computeGeometryAndDistances_proxy( const ModelHandler & model,
                                    DataHandler & data,
                                    const GeometryModelHandler & model_geom,
                                    GeometryDataHandler & data_geom,
                                    const Eigen::VectorXd & q
                                    )
      {
        computeDistances(*model, *data, *model_geom, *data_geom, q);
      }

#endif


      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
  bp::def("rnea",rnea_proxy,
    bp::args("Model","Data",
       "Configuration q (size Model::nq)",
       "Velocity qdot (size Model::nv)",
       "Acceleration qddot (size Model::nv)"),
    "Compute the RNEA, put the result in Data and return it.");

  bp::def("nle",nle_proxy,
    bp::args("Model","Data",
        "Configuration q (size Model::nq)",
        "Velocity qdot (size Model::nv)"),
        "Compute the Non Linear Effects (coriolis, centrifugal and gravitational effects), put the result in Data and return it.");

  bp::def("crba",crba_proxy,
    bp::args("Model","Data",
       "Configuration q (size Model::nq)"),
    "Compute CRBA, put the result in Data and return it.");

  bp::def("centerOfMass",com_proxy,
    bp::args("Model","Data",
       "Configuration q (size Model::nq)"),
    "Compute the center of mass, putting the result in Data and return it.");
        
  bp::def("centerOfMassAcceleration",com_acceleration_proxy,
    bp::args("Model","Data",
    "Configuration q (size Model::nq)",
    "Velocity v (size Model::nv)",
    "Acceleration a (size Model::nv)"),
    "Compute the center of mass position, velocity and acceleration andputting the result in Data.");

  bp::def("jacobianCenterOfMass",Jcom_proxy,
    bp::args("Model","Data",
       "Configuration q (size Model::nq)"),
    "Compute the jacobian of the center of mass, put the result in Data and return it.");

  bp::def("kinematics",kinematics_proxy,
    bp::args("Model","Data",
       "Configuration q (size Model::nq)",
       "Velocity v (size Model::nv)"),
    "Compute the placements and spatial velocities of all the frames of the kinematic "
    "tree and put the results in data.");

  bp::def("geometry",geometry_proxy,
    bp::args("Model","Data",
        "Configuration q (size Model::nq)"),
        "Compute the placements of all the frames of the kinematic "
        "tree and put the results in data.");
        
        bp::def("dynamics",dynamics_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)",
                         "Velocity v (size Model::nv)",
                         "Acceleration a (size Model::nv)"),
                "Compute the placements, spatial velocities and spatial accelerations of all the frames of the kinematic "
                "tree and put the results in data.");

  bp::def("computeAllTerms",computeAllTerms_proxy,
    bp::args("Model","Data",
             "Configuration q (size Model::nq)",
             "Velocity v (size Model::nv)"),
             "Compute all the terms M, non linear effects and Jacobians in"
             "in the same loop and put the results in data.");

  bp::def("jacobian",jacobian_proxy,
    bp::args("Model","Data",
       "Configuration q (size Model::nq)",
       "Joint ID (int)",
       "frame (true = local, false = world)",
       "update_geometry (true = update the value of the total jacobian)"),
    "Calling computeJacobians then getJacobian, return the result. Attention: the "
    "function computes indeed all the jacobians of the model, even if just outputing "
    "the demanded one if update_geometry is set to false. It is therefore outrageously costly wrt a dedicated "
    "call. Function to be used only for prototyping.");

  bp::def("computeJacobians",compute_jacobians_proxy,
    bp::args("Model","Data",
       "Configuration q (size Model::nq)"),
    "Calling computeJacobians");

  bp::def("jointLimits",jointLimits_proxy,
    bp::args("Model","Data",
        "Configuration q (size Model::nq)"),
        "Compute the maximum limits of all the joints of the model "
        "and put the results in data.");

#ifdef WITH_HPP_FCL
  bp::def("computeCollisions",computeCollisions_proxy,
    bp::args("GeometryData","bool"),
        "Determine if collisions pairs are effectively in collision."
        );

  bp::def("computeGeometryAndCollisions",computeGeometryAndCollisions_proxy,
    bp::args("Model","Data","GeometryModel","GeometryData","Configuration q (size Model::nq)", "bool"),
        "Update the geometry for a given configuration and"
        "determine if collision pars are effectively in collision"
        );

  bp::def("computeDistances",computeDistances_proxy,
    bp::args("GeometryData"),
        "Compute the distance between each collision pairs."
        );

  bp::def("computeGeometryAndDistances",computeGeometryAndDistances_proxy,
    bp::args("Model","Data","GeometryModel","GeometryData","Configuration q (size Model::nq)"),
        "Update the geometry for a given configuration and"
        "compute the distance between each collision pairs"
        );

#endif
      }
    };
    
  }} // namespace se3::python

#endif // ifndef __se3_python_data_hpp__

