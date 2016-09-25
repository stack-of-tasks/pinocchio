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

#ifndef __se3_center_of_mass_hxx__
#define __se3_center_of_mass_hxx__

/// @cond DEV
/** \addtogroup algo_com
 *  @{
 */

namespace se3
{

  inline const SE3::Vector3 &
  centerOfMass(const Model & model, Data & data,
               const Eigen::VectorXd & q,
               const bool computeSubtreeComs,
               const bool updateKinematics)
  {
    data.mass[0] = 0;
    data.com[0].setZero ();
    
    // Forward Step
    if (updateKinematics)
      forwardKinematics(model, data, q);

    for(Model::JointIndex i=1;i<(Model::JointIndex)(model.njoints);++i)
    {
      const double mass = model.inertias[i].mass();
      const SE3::Vector3 & lever = model.inertias[i].lever();
      
      data.com[i]  = mass * lever;
      data.mass[i] = mass;
    }
    
    // Backward Step
    for(Model::JointIndex i=(Model::JointIndex)(model.njoints-1); i>0; --i)
    {
      const Model::JointIndex & parent = model.parents[i];
      
      const SE3 & liMi = data.liMi[i];
      
      data.com[parent] += (liMi.rotation()*data.com[i]
                           + data.mass[i] * liMi.translation());
      data.mass[parent] += data.mass[i];
      
      if(computeSubtreeComs)
      {
        data.com[i] /= data.mass[i];
      }
    }
    
    data.com[0] /= data.mass[0];

    return data.com[0];
  }
  
  inline const SE3::Vector3 &
  centerOfMass(const Model & model, Data & data,
               const Eigen::VectorXd & q,
               const Eigen::VectorXd & v,
               const bool computeSubtreeComs,
               const bool updateKinematics)
  {
    using namespace se3;
    
    data.mass[0] = 0;
    data.com[0].setZero ();
    data.vcom[0].setZero ();
    
    // Forward Step
    if (updateKinematics)
      forwardKinematics(model, data, q, v);
    
    for(Model::JointIndex i=1;i<(Model::JointIndex)(model.njoints);++i)
    {
      const double mass = model.inertias[i].mass();
      const SE3::Vector3 & lever = model.inertias[i].lever();
      
      const Motion & v = data.v[i];
      
      data.com[i]  = mass * lever;
      data.mass[i] = mass;
      
      data.vcom[i] = mass * (v.angular().cross(lever) + v.linear());
    }
    
    // Backward Step
    for(Model::JointIndex i=(Model::JointIndex)(model.njoints-1); i>0; --i)
    {
      const Model::JointIndex & parent = model.parents[i];
      
      const SE3 & liMi = data.liMi[i];
      
      data.com[parent] += (liMi.rotation()*data.com[i]
                           + data.mass[i] * liMi.translation());
      
      data.vcom[parent] += liMi.rotation()*data.vcom[i];
      data.mass[parent] += data.mass[i];
      
      if( computeSubtreeComs )
      {
        data.com[i] /= data.mass[i];
        data.vcom[i] /= data.mass[i];
      }
    }
    
    data.com[0] /= data.mass[0];
    data.vcom[0] /= data.mass[0];
    
    return data.com[0];
  }
  
  inline const SE3::Vector3 &
  centerOfMass(const Model & model, Data & data,
               const Eigen::VectorXd & q,
               const Eigen::VectorXd & v,
               const Eigen::VectorXd & a,
               const bool computeSubtreeComs,
               const bool updateKinematics)
  {
    using namespace se3;

    data.mass[0] = 0;
    data.com[0].setZero ();
    data.vcom[0].setZero ();
    data.acom[0].setZero ();

    // Forward Step
    if (updateKinematics)
      forwardKinematics(model, data, q, v, a);
    
    for(Model::JointIndex i=1;i<(Model::JointIndex)(model.njoints);++i)
    {
      const double mass = model.inertias[i].mass();
      const SE3::Vector3 & lever = model.inertias[i].lever();

      const Motion & v = data.v[i];
      const Motion & a = data.a[i];

      data.com[i]  = mass * lever;
      data.mass[i] = mass;

      data.vcom[i] = mass * (v.angular().cross(lever) + v.linear());
      data.acom[i] = mass * (a.angular().cross(lever) + a.linear()) + v.angular().cross(data.vcom[i]); // take into accound the coriolis part of the acceleration
    }
    
    // Backward Step
    for(Model::JointIndex i=(Model::JointIndex)(model.njoints-1); i>0; --i)
    {
      const Model::JointIndex & parent = model.parents[i];
      
      const SE3 & liMi = data.liMi[i];
      
      data.com[parent] += (liMi.rotation()*data.com[i]
       + data.mass[i] * liMi.translation());
      
      data.vcom[parent] += liMi.rotation()*data.vcom[i];
      data.acom[parent] += liMi.rotation()*data.acom[i];
      data.mass[parent] += data.mass[i];
      
      if( computeSubtreeComs )
      {
        data.com[i] /= data.mass[i];
        data.vcom[i] /= data.mass[i];
        data.acom[i] /= data.mass[i];
      }
    }
    
    data.com[0] /= data.mass[0];
    data.vcom[0] /= data.mass[0];
    data.acom[0] /= data.mass[0];
    
    return data.com[0];
  }

  inline const SE3::Vector3 &
  getComFromCrba(const Model &, Data & data)
  {
    return data.com[0] = data.liMi[1].act(data.Ycrb[1].lever());
  }

  /* --- JACOBIAN ---------------------------------------------------------- */
  /* --- JACOBIAN ---------------------------------------------------------- */
  /* --- JACOBIAN ---------------------------------------------------------- */

  struct JacobianCenterOfMassBackwardStep
  : public fusion::JointVisitor<JacobianCenterOfMassBackwardStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
                                  se3::Data &,
                                  const bool
                                  > ArgsType;
  
    JOINT_VISITOR_INIT(JacobianCenterOfMassBackwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model& model,
                     se3::Data& data,
                     const bool computeSubtreeComs )
    {
      const Model::JointIndex & i      = (Model::JointIndex) jmodel.id();
      const Model::JointIndex & parent = model.parents[i];

      data.com[parent]  += data.com[i];
      data.mass[parent] += data.mass[i];

      typedef Data::Matrix6x Matrix6x;
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;
      
      ColBlock Jcols = jmodel.jointCols(data.J);
      Jcols = data.oMi[i].act(jdata.S());
      
      if (JointModel::NV == -1)
      {
        if( jmodel.nv() ==1 )
        {
        data.Jcom.col(jmodel.idx_v())
        = data.mass[i] * Jcols.template topLeftCorner<3,1>()
        - data.com[i].cross(Jcols.template bottomLeftCorner<3,1>()) ;
        }
        else
        {
          jmodel.jointCols(data.Jcom)
          = data.mass[i] * Jcols.template topRows<3>()
          - skew(data.com[i]) * Jcols.template bottomRows<3>();
        }
      }
      else
      {
        if( JointModel::NV ==1 )
        {
        data.Jcom.col(jmodel.idx_v())
        = data.mass[i] * Jcols.template topLeftCorner<3,1>()
        - data.com[i].cross(Jcols.template bottomLeftCorner<3,1>()) ;
        }
        else
        {
          jmodel.jointCols(data.Jcom)
          = data.mass[i] * Jcols.template topRows<3>()
          - skew(data.com[i]) * Jcols.template bottomRows<3>();
        }
      }
      
    
      if(computeSubtreeComs)
        data.com[i] /= data.mass[i];
    }

  };

  inline const Data::Matrix3x &
  jacobianCenterOfMass(const Model & model, Data & data,
                       const Eigen::VectorXd & q,
                       const bool computeSubtreeComs,
                       const bool updateKinematics)
  {
    data.com[0].setZero ();
    data.mass[0] = 0;
    
    // Forward step
    if (updateKinematics)
      forwardKinematics(model, data, q);
      
    for(Model::JointIndex i=1;i<(Model::JointIndex)(model.njoints);++i)
    {
      const double mass = model.inertias[i].mass();
      const SE3::Vector3 & lever = model.inertias[i].lever();
      
      data.mass[i] = mass;
      data.com[i] = mass*data.oMi[i].act(lever);
    }
   
    // Backward step
    for( Model::JointIndex i= (Model::JointIndex) (model.njoints-1);i>0;--i )
    {
      JacobianCenterOfMassBackwardStep
      ::run(model.joints[i],data.joints[i],
            JacobianCenterOfMassBackwardStep::ArgsType(model,data,computeSubtreeComs));
    }
    
    data.com[0] /= data.mass[0];
    data.Jcom /=  data.mass[0];
    
    return data.Jcom;
  }

  inline const Data::Matrix3x &
  getJacobianComFromCrba(const Model & model, Data & data)
  {
    const SE3 & oM1 = data.liMi[1];
    
    // As the 6 first rows of M*a are a wrench, we just need to multiply by the
    // relative rotation between the first joint and the world
    const SE3::Matrix3 oR1_over_m (oM1.rotation() / data.M(0,0));
    
    // I don't know why, but the colwise multiplication is much more faster
    // than the direct Eigen multiplication
    for (long k=0; k<model.nv;++k)
      data.Jcom.col(k) = oR1_over_m * data.M.topRows<3> ().col(k);
    return data.Jcom;
  }

} // namespace se3

/** @}*/
/// @endcond

#endif // ifndef __se3_center_of_mass_hxx__
