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

#ifndef __se3_python_algorithm_hpp__
#define __se3_python_algorithm_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/python/model.hpp"
#include "pinocchio/python/data.hpp"

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/non-linear-effects.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/dynamics.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/operational-frames.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/joint-limits.hpp"
#include "pinocchio/algorithm/energy.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
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
                                        const VectorXd_fx & v,
                                        const VectorXd_fx & a )
      { return rnea(*model,*data,q,v,a); }

      static Eigen::VectorXd nle_proxy( const ModelHandler& model,
                                        DataHandler & data,
                                        const VectorXd_fx & q,
                                        const VectorXd_fx & v)
      { return nonLinearEffects(*model,*data,q,v); }

      static Eigen::MatrixXd crba_proxy(const ModelHandler& model,
                                        DataHandler & data,
                                        const VectorXd_fx & q)
      {
        data->M.fill(0);
        crba(*model,*data,q);
        data->M.triangularView<Eigen::StrictlyLower>()
        = data->M.transpose().triangularView<Eigen::StrictlyLower>();
        return data->M;
      }
      
      static Data::Matrix6x ccrba_proxy(const ModelHandler& model,
                                        DataHandler & data,
                                        const VectorXd_fx & q,
                                        const VectorXd_fx & v)
      {
        ccrba(*model,*data,q,v);
        return data->Ag;
      }
      
      static Eigen::MatrixXd aba_proxy(const ModelHandler & model,
                                       DataHandler & data,
                                       const VectorXd_fx & q,
                                       const VectorXd_fx & v,
                                       const VectorXd_fx & tau)
      {
        aba(*model,*data,q,v,tau);
        return data->ddq;
      }
      
      static Eigen::MatrixXd fd_llt_proxy(const ModelHandler & model,
                                          DataHandler & data,
                                          const VectorXd_fx & q,
                                          const VectorXd_fx & v,
                                          const VectorXd_fx & tau,
                                          const eigenpy::MatrixXd_fx & J,
                                          const VectorXd_fx & gamma,
                                          const bool update_kinematics = true)
      {
        forwardDynamics(*model,*data,q,v,tau,J,gamma,update_kinematics);
        return data->ddq;
      }

      static SE3::Vector3
      com_0_proxy(const ModelHandler& model,
                DataHandler & data,
                const VectorXd_fx & q,
                const bool updateKinematics = true)
      {
        return centerOfMass(*model,*data,q,
                            true,
                            updateKinematics);
      }
      
      static SE3::Vector3
      com_1_proxy(const ModelHandler& model,
                  DataHandler & data,
                  const VectorXd_fx & q,
                  const VectorXd_fx & v,
                  const bool updateKinematics = true)
      {
        return centerOfMass(*model,*data,q,v,
                            true,
                            updateKinematics);
      }
      
      static SE3::Vector3
      com_2_proxy(const ModelHandler & model,
                             DataHandler & data,
                             const VectorXd_fx & q,
                             const VectorXd_fx & v,
                             const VectorXd_fx & a,
                             const bool updateKinematics = true)
      {
        return centerOfMass(*model,*data,q,v,a,
                            true,
                            updateKinematics);
      }

      static Data::Matrix3x
      Jcom_proxy(const ModelHandler& model,
                 DataHandler & data,
                 const VectorXd_fx & q)
      { return jacobianCenterOfMass(*model,*data,q); }

      static Data::Matrix6x
      jacobian_proxy(const ModelHandler & model,
                     DataHandler & data,
                     const VectorXd_fx & q,
                     Model::JointIndex jointId,
                     bool local,
                     bool update_geometry)
      {
        Data::Matrix6x J( 6,model->nv ); J.setZero();
        if (update_geometry)
          computeJacobians( *model,*data,q );
        if(local) getJacobian<true> (*model, *data, jointId, J);
        else getJacobian<false> (*model, *data, jointId, J);
        return J;
      }
      
      static Data::Matrix6x frame_jacobian_proxy(const ModelHandler & model, 
                                                 DataHandler & data,
                                                 const VectorXd_fx & q,
                                                 Model::FrameIndex frame_id,
                                                 bool local,
                                                 bool update_geometry
                                                 )
      {
        Data::Matrix6x J( 6,model->nv ); J.setZero();

        if (update_geometry)
          computeJacobians( *model,*data,q );

        if(local) getFrameJacobian<true> (*model, *data, frame_id, J);
        else getFrameJacobian<false> (*model, *data, frame_id, J);
        
        return J;
      }

      static void compute_jacobians_proxy(const ModelHandler& model,
                                          DataHandler & data,
                                          const VectorXd_fx & q)
      {
        computeJacobians( *model,*data,q );
      }
      
      static void fk_0_proxy(const ModelHandler & model,
                             DataHandler & data,
                             const VectorXd_fx & q)
      {
        forwardKinematics(*model,*data,q);
      }

      static void fk_1_proxy(const ModelHandler& model,
                             DataHandler & data,
                             const VectorXd_fx & q,
                             const VectorXd_fx & qdot )
      {
        forwardKinematics(*model,*data,q,qdot);
      }


      static void frames_fk_0_proxy(const ModelHandler& model, 
                                    DataHandler & data,
                                    const VectorXd_fx & q
                                    )
      {
        framesForwardKinematics( *model,*data,q );
      }

      static void fk_2_proxy(const ModelHandler& model,
                             DataHandler & data,
                             const VectorXd_fx & q,
                             const VectorXd_fx & v,
                             const VectorXd_fx & a)
      {
        forwardKinematics(*model,*data,q,v,a);
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
      
      static double kineticEnergy_proxy(const ModelHandler & model,
                                        DataHandler & data,
                                        const VectorXd_fx & q,
                                        const VectorXd_fx & v,
                                        const bool update_kinematics = true)
      {
        return kineticEnergy(*model,*data,q,v,update_kinematics);
      }
      
      static double potentialEnergy_proxy(const ModelHandler & model,
                                          DataHandler & data,
                                          const VectorXd_fx & q,
                                          const bool update_kinematics = true)
      {
        return potentialEnergy(*model,*data,q,update_kinematics);
      }

      static void integrateModel_proxy(const ModelHandler & model,
                                      DataHandler & data,
                                      const VectorXd_fx & q,
                                      const VectorXd_fx & v,
                                      Eigen::VectorXd & result)
      {
        integrateModel(*model,*data,q,v,result);
      }
#ifdef WITH_HPP_FCL
      
      static void updateGeometryPlacements_proxy(const ModelHandler & model,
                                                 DataHandler & data,
                                                 const GeometryModelHandler & geom_model,
                                                 GeometryDataHandler & geom_data,
                                                 const VectorXd_fx & q
                                                 )
      {
        return updateGeometryPlacements(*model, *data, *geom_model, *geom_data, q);
      }
      
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
                                    const bool stopAtFirstCollision)
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
                         "Velocity v (size Model::nv)",
                         "Acceleration a (size Model::nv)"),
                "Compute the RNEA, put the result in Data and return it.");
        
        bp::def("nle",nle_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)",
                         "Velocity v (size Model::nv)"),
                "Compute the Non Linear Effects (coriolis, centrifugal and gravitational effects), put the result in Data and return it.");
        
        bp::def("crba",crba_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)"),
                "Compute CRBA, put the result in Data and return it.");
        
        bp::def("ccrba",ccrba_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)",
                         "Velocity v (size Model::nv)"),
                "Compute the centroidal mapping, the centroidal momentum and the Centroidal Composite Rigid Body Inertia, put the result in Data and return the centroidal mapping.");
        
        bp::def("aba",aba_proxy,
                bp::args("Model","Data",
                         "Joint configuration q (size Model::nq)",
                         "Joint velocity v (size Model::nv)",
                         "Joint torque tau (size Model::nv)"),
                "Compute ABA, put the result in Data::ddq and return it.");
        
        bp::def("forwardDynamics",fd_llt_proxy,
                bp::args("Model","Data",
                         "Joint configuration q (size Model::nq)",
                         "Joint velocity v (size Model::nv)",
                         "Joint torque tau (size Model::nv)",
                         "Contact Jacobian J (size nb_constraint * Model::nv)",
                         "Contact drift gamma (size nb_constraint)",
                         "Update kinematics (if true, it updates the dynamic variable according to the current state)"),
                "Solve the forward dynamics problem with contacts, put the result in Data::ddq and return it.");
        
        bp::def("centerOfMass",com_0_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)",
                         "Update kinematics"),
                "Compute the center of mass, putting the result in Data and return it.");
        
        bp::def("centerOfMass",com_1_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)",
                         "Velocity v (size Model::nv)",
                         "Update kinematics"),
                "Compute the center of mass position and velocuty by storing the result in Data"
                "and return the center of mass position of the full model expressed in the world frame.");
        
        bp::def("centerOfMass",com_2_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)",
                         "Velocity v (size Model::nv)",
                         "Acceleration a (size Model::nv)",
                         "Update kinematics"),
                "Compute the center of mass position, velocity and acceleration by storing the result in Data"
                "and return the center of mass position of the full model expressed in the world frame.");

        bp::def("jacobianCenterOfMass",Jcom_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)"),
                "Compute the jacobian of the center of mass, put the result in Data and return it.");
        
        
        bp::def("framesKinematics",frames_fk_0_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)"),
                "Compute the placements and spatial velocities of all the operational frames "
                "and put the results in data.");
        
        bp::def("forwardKinematics",fk_0_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)"),
                "Compute the placements of all the frames of the kinematic "
                "tree and put the results in data.");
        
        bp::def("forwardKinematics",fk_1_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)",
                         "Velocity v (size Model::nv)"),
                "Compute the placements and spatial velocities of all the frames of the kinematic "
                "tree and put the results in data.");
        
        bp::def("forwardKinematics",fk_2_proxy,
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
        
        bp::def("Framejacobian",frame_jacobian_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)",
                         "Operational frame ID (int)",
                         "frame (true = local, false = world)",
                         "update_geometry (true = recompute the kinematics)"),
                "Call computeJacobians if update_geometry is true. If not, user should call computeJacobians first."
                "Then call getJacobian and return the resulted jacobian matrix. Attention: if update_geometry is true, the "
                "function computes all the jacobians of the model, even if just outputing "
                "the demanded one. It is therefore outrageously costly wrt a dedicated "
                "call. Use only with update_geometry for prototyping.");
        
        bp::def("computeJacobians",compute_jacobians_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)"),
                "Calling computeJacobians");
        
        bp::def("jointLimits",jointLimits_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)"),
                "Compute the maximum limits of all the joints of the model "
          "and put the results in data.");
        
        bp::def("kineticEnergy",kineticEnergy_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)",
                         "Velocity v (size Model::nv)",
                         "Update kinematics (bool)"),
                "Compute the kinematic energy of the model for the "
                "given joint configuration and velocity and store it "
                " in data.kinetic_energy. By default, the kinematics of model is updated.");
        
        bp::def("potentialEnergy",potentialEnergy_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)",
                         "Update kinematics (bool)"),
                "Compute the potential energy of the model for the "
                "given the joint configuration and store it "
                " in data.potential_energy. By default, the kinematics of model is updated.");

        bp::def("integrateModel",integrateModel_proxy,
                bp::args("Model","Data",
                         "Configuration q (size Model::nq)",
                         "Velocity v (size Model::nv)",
                         "resulting Configuration result (size Model::nq)"),
                "Integrate the model for a constant derivative during unit time .");
#ifdef WITH_HPP_FCL
        
        bp::def("updateGeometryPlacements",updateGeometryPlacements_proxy,
                bp::args("Model", "Data", "GeometryModel", "GeometryData", "Configuration q (size Model::nq)"),
                "Update the placement of the collision objects according to the current configuration."
                "The algorithm also updates the current placement of the joint in Data."
                );
        
        bp::def("computeCollisions",computeCollisions_proxy,
                bp::args("GeometryData","bool"),
                "Determine if collision pairs are effectively in collision."
                );
        
        bp::def("computeGeometryAndCollisions",computeGeometryAndCollisions_proxy,
                bp::args("Model","Data","GeometryModel","GeometryData","Configuration q (size Model::nq)", "bool"),
                "Update the geometry for a given configuration and"
                "determine if all collision pairs are effectively in collision or not."
                );
        
        bp::def("computeDistances",computeDistances_proxy,
                bp::args("GeometryData"),
                "Compute the distance between each collision pair."
                );
        
        bp::def("computeGeometryAndDistances",computeGeometryAndDistances_proxy,
                bp::args("Model","Data","GeometryModel","GeometryData","Configuration q (size Model::nq)"),
                "Update the geometry for a given configuration and"
                "compute the distance between each collision pair"
                );

#endif
      }
    };
    
  }} // namespace se3::python

#endif // ifndef __se3_python_data_hpp__

