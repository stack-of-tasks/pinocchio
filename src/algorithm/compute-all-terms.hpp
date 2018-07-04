//
// Copyright (c) 2015-2017 CNRS
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

#ifndef __se3_compute_all_terms_hpp__
#define __se3_compute_all_terms_hpp__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/energy.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace se3
{
  ///
  /// \brief Computes efficiently all the terms needed for dynamic simulation. It is equivalent to the call at the same time to:
  ///         - se3::forwardKinematics
  ///         - se3::crba
  ///         - se3::nonLinearEffects
  ///         - se3::computeJointJacobians
  ///         - se3::centerOfMass
  ///         - se3::jacobianCenterOfMass
  ///         - se3::kineticEnergy
  ///         - se3::potentialEnergy
  ///
  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] data The data structure of the rigid body system.
  /// \param[in] q The joint configuration vector (dim model.nq).
  /// \param[in] v The joint velocity vector (dim model.nv).
  ///
  /// \return All the results are stored in data. Please refer to the specific algorithm for further details.
  inline void
  computeAllTerms(const Model & model,
                  Data & data,
                  const Eigen::VectorXd & q,
                  const Eigen::VectorXd & v);

} // namespace se3


/* --- Details -------------------------------------------------------------------- */
namespace se3
{
  
  struct CATForwardStep : public fusion::JointVisitor<CATForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;

    JOINT_VISITOR_INIT(CATForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v)
    {
      using namespace Eigen;
      using namespace se3;

      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q,v);
      
      // CRBA
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      data.Ycrb[i] = model.inertias[i];

      // Jacobian + NLE
      data.v[i] = jdata.v();

      if(parent>0)
      {
        data.oMi[i] = data.oMi[parent]*data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else
        data.oMi[i] = data.liMi[i];

      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());

      data.a_gf[i] = data.a[i] = jdata.c() + (data.v[i] ^ jdata.v());
      if (parent > 0)
        data.a[i] += data.liMi[i].actInv(data.a[parent]);
      
      data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);

      data.f[i] = model.inertias[i]*data.a_gf[i] + model.inertias[i].vxiv(data.v[i]); // -f_ext
      
      // CoM
      const double mass = model.inertias[i].mass();
      const SE3::Vector3 & lever = model.inertias[i].lever();
      
      data.com[i]  = mass * lever;
      data.mass[i] = mass;

      data.vcom[i] = mass * (data.v[i].angular().cross(lever) + data.v[i].linear());
    }

  };

  struct CATBackwardStep : public fusion::JointVisitor<CATBackwardStep>
  {
    typedef boost::fusion::vector<const Model &,
    Data &>  ArgsType;

    JOINT_VISITOR_INIT(CATBackwardStep);

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      /*
       * F[1:6,i] = Y*S
       * M[i,SUBTREE] = S'*F[1:6,SUBTREE]
       * if li>0
       *   Yli += liXi Yi
       *   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE]
       */
      const Model::JointIndex & i = (Model::JointIndex) jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      const SE3 & oMi = data.oMi[i];

      /* F[1:6,i] = Y*S */
      jmodel.jointCols(data.Fcrb[i]) = data.Ycrb[i] * jdata.S();

      /* M[i,SUBTREE] = S'*F[1:6,SUBTREE] */
      data.M.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i])
      = jdata.S().transpose()*data.Fcrb[i].middleCols(jmodel.idx_v(),data.nvSubtree[i]);


      jmodel.jointVelocitySelector(data.nle)  = jdata.S().transpose()*data.f[i];
      if(parent>0)
      {
        /*   Yli += liXi Yi */
        data.Ycrb[parent] += data.liMi[i].act(data.Ycrb[i]);

        /*   F[1:6,SUBTREE] = liXi F[1:6,SUBTREE] */
        Eigen::Block<typename Data::Matrix6x> jF
        = data.Fcrb[parent].block(0,jmodel.idx_v(),6,data.nvSubtree[i]);
        Eigen::Block<typename Data::Matrix6x> iF
        = data.Fcrb[i].block(0,jmodel.idx_v(),6,data.nvSubtree[i]);
        forceSet::se3Action(data.liMi[i], iF, jF);

        data.f[parent] += data.liMi[i].act(data.f[i]);
      }
      
      // CoM
      const SE3 & liMi = data.liMi[i];
      
      data.com[parent] += (liMi.rotation()*data.com[i]
                           + data.mass[i] * liMi.translation());
      
      SE3::Vector3 com_in_world (oMi.rotation() * data.com[i] + data.mass[i] * oMi.translation());
      
      data.vcom[parent] += liMi.rotation()*data.vcom[i];
      data.mass[parent] += data.mass[i];
      
      typedef Data::Matrix6x Matrix6x;
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;
      
      ColBlock Jcols = jmodel.jointCols(data.J);
      
      if( JointModel::NV==1 )
        data.Jcom.col(jmodel.idx_v())
        = data.mass[i] * Jcols.template topLeftCorner<3,1>()
        - com_in_world.cross(Jcols.template bottomLeftCorner<3,1>()) ;
      else
        jmodel.jointCols(data.Jcom)
        = data.mass[i] * Jcols.template topRows<3>()
        - skew(com_in_world) * Jcols.template bottomRows<3>();
      
      data.com[i] /= data.mass[i];
      data.vcom[i] /= data.mass[i];
    }
  };
  
  inline void
  computeAllTerms(const Model & model,
                  Data & data,
                  const Eigen::VectorXd & q,
                  const Eigen::VectorXd & v)
  {
    assert(model.check(data) && "data is not consistent with model.");
    data.v[0].setZero();
    data.a[0].setZero();
    data.a_gf[0] = -model.gravity;
    
    data.mass[0] = 0;
    data.com[0].setZero ();
    data.vcom[0].setZero ();

    for(Model::JointIndex i=1;i<(Model::JointIndex) model.njoints;++i)
    {
      CATForwardStep::run(model.joints[i],data.joints[i],
                          CATForwardStep::ArgsType(model,data,q,v));
    }

    for(Model::JointIndex i=(Model::JointIndex)(model.njoints-1);i>0;--i)
    {
      CATBackwardStep::run(model.joints[i],data.joints[i],
                           CATBackwardStep::ArgsType(model,data));
    }
    
    // CoM
    data.com[0] /= data.mass[0];
    data.vcom[0] /= data.mass[0];
    
    // JCoM
    data.Jcom /= data.mass[0];
    
    // Energy
    kineticEnergy(model, data, q, v, false);
    potentialEnergy(model, data, q, false);

  }
} // namespace se3


#endif // ifndef __se3_compute_all_terms_hpp__

