//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_center_of_mass_hxx__
#define __pinocchio_algorithm_center_of_mass_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

/// @cond DEV

namespace pinocchio
{
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Scalar computeTotalMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
  {
    Scalar m = Scalar(0);
    for(JointIndex i=1; i<(JointIndex)(model.njoints); ++i)
    {
      m += model.inertias[i].mass();
    }
    return m;
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline Scalar computeTotalMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                 DataTpl<Scalar,Options,JointCollectionTpl> & data)
  {
    data.mass[0] = computeTotalMass(model);
    return data.mass[0];
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline void computeSubtreeMasses(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                   DataTpl<Scalar,Options,JointCollectionTpl> & data)
  {
    data.mass[0] = Scalar(0);

    // Forward Step
    for(JointIndex i=1; i<(JointIndex)(model.njoints); ++i)
    {
      data.mass[i] = model.inertias[i].mass();
    }

    // Backward Step
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      const JointIndex & parent = model.parents[i];
      data.mass[parent] += data.mass[i];
    }
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Vector3 &
  centerOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               DataTpl<Scalar,Options,JointCollectionTpl> & data,
               const Eigen::MatrixBase<ConfigVectorType> & q,
               const bool computeSubtreeComs)
  {
    forwardKinematics(model,data,q.derived());

    centerOfMass(model,data,POSITION,computeSubtreeComs);
    return data.com[0];
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Vector3 &
  centerOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               DataTpl<Scalar,Options,JointCollectionTpl> & data,
               const Eigen::MatrixBase<ConfigVectorType> & q,
               const Eigen::MatrixBase<TangentVectorType> & v,
               const bool computeSubtreeComs)
  {
    forwardKinematics(model,data,q.derived(),v.derived());

    centerOfMass(model,data,VELOCITY,computeSubtreeComs);
    return data.com[0];
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Vector3 &
  centerOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               DataTpl<Scalar,Options,JointCollectionTpl> & data,
               const Eigen::MatrixBase<ConfigVectorType> & q,
               const Eigen::MatrixBase<TangentVectorType1> & v,
               const Eigen::MatrixBase<TangentVectorType2> & a,
               const bool computeSubtreeComs)
  {
    forwardKinematics(model,data,q,v,a);

    centerOfMass(model,data,ACCELERATION,computeSubtreeComs);
    return data.com[0];
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  const typename DataTpl<Scalar,Options,JointCollectionTpl>::Vector3 &
  centerOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               DataTpl<Scalar,Options,JointCollectionTpl> & data,
               KinematicLevel kinematic_level,
               const bool computeSubtreeComs)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(kinematic_level >= 0 && kinematic_level <= 2);

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;

    typedef typename Model::JointIndex JointIndex;

    typedef typename Data::SE3 SE3;
    typedef typename Data::Motion Motion;
    typedef typename Data::Inertia Inertia;

    const bool do_position = (kinematic_level >= POSITION);
    const bool do_velocity = (kinematic_level >= VELOCITY);
    const bool do_acceleration = (kinematic_level >= ACCELERATION);

    data.mass[0] = 0;
    if(do_position)
      data.com[0].setZero();
    if(do_velocity)
      data.vcom[0].setZero();
    if(do_acceleration)
      data.acom[0].setZero();

    // Forward Step
    for(JointIndex i=1; i<(JointIndex)(model.njoints); ++i)
    {
      const typename Inertia::Scalar  & mass = model.inertias[i].mass();
      const typename SE3::Vector3 & lever = model.inertias[i].lever();

      const Motion & v = data.v[i];
      const Motion & a = data.a[i];

      data.mass[i] = mass;

      if(do_position)
        data.com[i].noalias()  = mass * lever;

      if(do_velocity)
        data.vcom[i].noalias() = mass * (v.angular().cross(lever) + v.linear());

      if(do_acceleration)
        data.acom[i].noalias() = mass * (a.angular().cross(lever) + a.linear())
                     + v.angular().cross(data.vcom[i]); // take into accound the coriolis part of the acceleration
    }

    // Backward Step
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      const JointIndex & parent = model.parents[i];
      const SE3 & liMi = data.liMi[i];

      data.mass[parent] += data.mass[i];

      if(do_position)
        data.com[parent] += (liMi.rotation()*data.com[i]
                         + data.mass[i] * liMi.translation());

      if(do_velocity)
        data.vcom[parent] += liMi.rotation()*data.vcom[i];

      if(do_acceleration)
        data.acom[parent] += liMi.rotation()*data.acom[i];

      if(computeSubtreeComs)
      {
        if(do_position)
          data.com[i] /= data.mass[i];
        if(do_velocity)
          data.vcom[i] /= data.mass[i];
        if(do_acceleration)
          data.acom[i] /= data.mass[i];
      }
    }

    if(do_position)
      data.com[0] /= data.mass[0];
    if(do_velocity)
      data.vcom[0] /= data.mass[0];
    if(do_acceleration)
      data.acom[0] /= data.mass[0];
    
    return data.com[0];
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Vector3 &
  getComFromCrba(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                 DataTpl<Scalar,Options,JointCollectionTpl> & data)
  {
    PINOCCHIO_UNUSED_VARIABLE(model);

    assert(model.check(data) && "data is not consistent with model.");
    return data.com[0] = data.liMi[1].act(data.Ycrb[1].lever());
  }

  /* --- JACOBIAN ---------------------------------------------------------- */
  /* --- JACOBIAN ---------------------------------------------------------- */
  /* --- JACOBIAN ---------------------------------------------------------- */

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix3x>
  struct JacobianCenterOfMassBackwardStep
  : public fusion::JointUnaryVisitorBase< JacobianCenterOfMassBackwardStep<Scalar,Options,JointCollectionTpl,Matrix3x> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const Eigen::MatrixBase<Matrix3x> &,
                                  const bool &
                                  > ArgsType;

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<Matrix3x> & Jcom,
                     const bool & computeSubtreeComs)
    {
      const JointIndex & i      = (JointIndex) jmodel.id();
      const JointIndex & parent = model.parents[i];

      data.com[parent]  += data.com[i];
      data.mass[parent] += data.mass[i];

      typedef typename Data::Matrix6x Matrix6x;
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;
      
      Matrix3x & Jcom_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3x,Jcom);

      ColBlock Jcols = jmodel.jointCols(data.J);
      Jcols = data.oMi[i].act(jdata.S());
      
      for(Eigen::DenseIndex col_id = 0; col_id < jmodel.nv(); ++col_id)
      {
        jmodel.jointCols(Jcom_).col(col_id)
        = data.mass[i] * Jcols.col(col_id).template segment<3>(Motion::LINEAR)
        - data.com[i].cross(Jcols.col(col_id).template segment<3>(Motion::ANGULAR));
      }

      if(computeSubtreeComs)
        data.com[i] /= data.mass[i];
    }

  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix3x &
  jacobianCenterOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                       DataTpl<Scalar,Options,JointCollectionTpl> & data,
                       const Eigen::MatrixBase<ConfigVectorType> & q,
                       const bool computeSubtreeComs)
  {
    forwardKinematics(model, data, q);
    return jacobianCenterOfMass(model, data, computeSubtreeComs);
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix3x &
  jacobianCenterOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                       DataTpl<Scalar,Options,JointCollectionTpl> & data,
                       const bool computeSubtreeComs)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;

    typedef typename Model::JointIndex JointIndex;
    
    typedef typename Data::Matrix3x Matrix3x;
    typedef typename Data::SE3 SE3;
    typedef typename Data::Inertia Inertia;

    data.com[0].setZero();
    data.mass[0] = Scalar(0);

    for(JointIndex i=1; i<(JointIndex)(model.njoints); ++i)
    {
      const typename Inertia::Scalar & mass = model.inertias[i].mass();
      const typename SE3::Vector3 & lever = model.inertias[i].lever();

      data.mass[i] = mass;
      data.com[i].noalias() = mass*data.oMi[i].act(lever);
    }

    // Backward step
    typedef JacobianCenterOfMassBackwardStep<Scalar,Options,JointCollectionTpl,Matrix3x> Pass2;
    for(JointIndex i= (JointIndex) (model.njoints-1); i>0; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data,data.Jcom,computeSubtreeComs));
    }

    data.com[0] /= data.mass[0];
    data.Jcom /=  data.mass[0];

    return data.Jcom;
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix3x>
  struct JacobianSubtreeCenterOfMassBackwardStep
  : public fusion::JointUnaryVisitorBase< JacobianSubtreeCenterOfMassBackwardStep<Scalar,Options,JointCollectionTpl,Matrix3x> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
    Data &,
    const JointIndex &,
    const Eigen::MatrixBase<Matrix3x> &
    > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const JointIndex & subtree_root_id,
                     const Eigen::MatrixBase<Matrix3x> & Jcom)
    {
      PINOCCHIO_UNUSED_VARIABLE(model);
      
      const JointIndex & i      = (JointIndex) jmodel.id();
      
      typedef typename Data::Matrix6x Matrix6x;
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;
      
      Matrix3x & Jcom_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3x,Jcom);
      
      ColBlock Jcols = jmodel.jointCols(data.J);
      Jcols = data.oMi[i].act(jdata.S());
      
      for(Eigen::DenseIndex col_id = 0; col_id < jmodel.nv(); ++col_id)
      {
        jmodel.jointCols(Jcom_).col(col_id)
        = Jcols.col(col_id).template segment<3>(Motion::LINEAR)
        - data.com[subtree_root_id].cross(Jcols.col(col_id).template segment<3>(Motion::ANGULAR));
      }
    }
    
  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename Matrix3xLike>
  inline void
  jacobianSubtreeCenterOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              DataTpl<Scalar,Options,JointCollectionTpl> & data,
                              const Eigen::MatrixBase<ConfigVectorType> & q,
                              const JointIndex & rootSubtreeId,
                              const Eigen::MatrixBase<Matrix3xLike> & res)
  {
    forwardKinematics(model, data, q);
    jacobianSubtreeCenterOfMass(model, data, rootSubtreeId, res);
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix3xLike>
  inline void
  jacobianSubtreeCenterOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              DataTpl<Scalar,Options,JointCollectionTpl> & data,
                              const JointIndex & rootSubtreeId,
                              const Eigen::MatrixBase<Matrix3xLike> & res)
  {
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT((int)rootSubtreeId < model.njoints, "Invalid joint id.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(res.rows(), 3, "the resulting matrix does not have the right size.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(res.cols(), model.nv, "the resulting matrix does not have the right size.");

    Matrix3xLike & Jcom_subtree = PINOCCHIO_EIGEN_CONST_CAST(Matrix3xLike,res);
    
    typedef typename Data::SE3 SE3;
    typedef typename Data::Inertia Inertia;
    
    typedef typename Model::IndexVector IndexVector;
    const IndexVector & subtree = model.subtrees[rootSubtreeId];
    
    const bool computeSubtreeComs = true;
    
    if(rootSubtreeId == 0)
    {
      data.mass[0] = 0;
      data.com[0].setZero();
    }
    
    for(size_t k = 0; k < subtree.size(); ++k)
    {
      const JointIndex joint_id = subtree[k];
      const typename Inertia::Scalar & mass = model.inertias[joint_id].mass();
      const typename SE3::Vector3 & lever = model.inertias[joint_id].lever();
      
      data.mass[joint_id] = mass;
      data.com[joint_id] = mass*data.oMi[joint_id].act(lever);
    }
    
    // Backward step
    typedef JacobianCenterOfMassBackwardStep<Scalar,Options,JointCollectionTpl,Matrix3xLike> Pass2;
    for(Eigen::DenseIndex k = (Eigen::DenseIndex)subtree.size() - 1; k >= 0; --k)
    {
      const JointIndex joint_id = subtree[(size_t)k];
      Pass2::run(model.joints[joint_id],data.joints[joint_id],
                 typename Pass2::ArgsType(model,data,Jcom_subtree,computeSubtreeComs));
    }
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(data.mass[rootSubtreeId] > 0., "The mass of the subtree is not positive.");
    const Scalar mass_inv_subtree = Scalar(1)/data.mass[rootSubtreeId];
    typename Data::Vector3 & com_subtree = data.com[rootSubtreeId];
    if(!computeSubtreeComs)
      com_subtree *= mass_inv_subtree;

    if(rootSubtreeId == 0)
    {
      Jcom_subtree *= mass_inv_subtree;
      return; // skip the rest
    }

    const int idx_v = model.joints[rootSubtreeId].idx_v();
    const int nv_subtree = data.nvSubtree[rootSubtreeId];

    Jcom_subtree.middleCols(idx_v,nv_subtree) *= mass_inv_subtree;
    
    // Second backward step
    typedef JacobianSubtreeCenterOfMassBackwardStep<Scalar,Options,JointCollectionTpl,Matrix3xLike> Pass3;
    for(JointIndex parent = model.parents[rootSubtreeId];
        parent > 0;
        parent = model.parents[parent])
    {
      Pass3::run(model.joints[parent],data.joints[parent],
                 typename Pass3::ArgsType(model,data,rootSubtreeId,Jcom_subtree));
    }
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix3xLike>
  inline void
  getJacobianSubtreeCenterOfMass(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                 const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                 const JointIndex & rootSubtreeId,
                                 const Eigen::MatrixBase<Matrix3xLike> & res)
  {
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;

    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(((int)rootSubtreeId < model.njoints), "Invalid joint id.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(res.rows(), 3, "the resulting matrix does not have the right size.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(res.cols(), model.nv, "the resulting matrix does not have the right size.");
    
    Matrix3xLike & Jcom_subtree = PINOCCHIO_EIGEN_CONST_CAST(Matrix3xLike,res);
    
    if(rootSubtreeId == 0)
    {
      Jcom_subtree = data.Jcom;
      return;
    }
    
    const int idx_v = model.joints[rootSubtreeId].idx_v();
    const int nv_subtree = data.nvSubtree[rootSubtreeId];
    
    const Scalar mass_ratio = data.mass[0] / data.mass[rootSubtreeId];
    Jcom_subtree.middleCols(idx_v,nv_subtree)
    = mass_ratio * data.Jcom.middleCols(idx_v,nv_subtree);

    const typename Data::Vector3 & com_subtree = data.com[rootSubtreeId];

    for(int parent = data.parents_fromRow[(size_t)idx_v];
        parent >= 0;
        parent = data.parents_fromRow[(size_t)parent])
    {
      typename Data::Matrix6x::ConstColXpr Jcol = data.J.col(parent);
      Jcom_subtree.col(parent).noalias() = Jcol.template segment<3>(Motion::LINEAR) - com_subtree.cross(Jcol.template segment<3>(Motion::ANGULAR));
    }
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::Matrix3x &
  getJacobianComFromCrba(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                         DataTpl<Scalar,Options,JointCollectionTpl> & data)
  {
    PINOCCHIO_UNUSED_VARIABLE(model);
    assert(model.check(data) && "data is not consistent with model.");

    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    typedef typename Data::SE3 SE3;

    const SE3 & oM1 = data.liMi[1];

    // Extract the total mass of the system.
    data.mass[0] = data.M(0,0);

    // As the 6 first rows of M*a are a wrench, we just need to multiply by the
    // relative rotation between the first joint and the world
    const typename SE3::Matrix3 oR1_over_m (oM1.rotation() / data.M(0,0));

    // I don't know why, but the colwise multiplication is much more faster
    // than the direct Eigen multiplication
    for(long k=0; k<model.nv;++k)
      data.Jcom.col(k).noalias() = oR1_over_m * data.M.template topRows<3>().col(k);

    return data.Jcom;
  }

} // namespace pinocchio

/// @endcond

#endif // ifndef __pinocchio_algorithm_center_of_mass_hxx__
