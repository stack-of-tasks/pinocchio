//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_joint_composite_hxx__
#define __pinocchio_joint_composite_hxx__

#include "pinocchio/multibody/visitor.hpp"

namespace pinocchio 
{

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl, typename ConfigVectorType>
  struct JointCompositeCalcZeroOrderStep
  : fusion::JointUnaryVisitorBase< JointCompositeCalcZeroOrderStep<Scalar,Options,JointCollectionTpl,ConfigVectorType> >
  {
    typedef JointModelCompositeTpl<Scalar,Options,JointCollectionTpl> JointModelComposite;
    typedef JointDataCompositeTpl<Scalar,Options,JointCollectionTpl> JointDataComposite;
    
    typedef boost::fusion::vector<const JointModelComposite &,
                                  JointDataComposite &,
                                  const ConfigVectorType &
                                  > ArgsType;

    template<typename JointModel>
    static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                     pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const JointModelComposite & model,
                     JointDataComposite & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      const JointIndex & i  = jmodel.id();
      const JointIndex succ = i+1; // successor

      jmodel.calc(jdata.derived(), q.derived());

      data.pjMi[i] = model.jointPlacements[i] * jdata.M ();

      if ( succ == model.joints.size() )
      {
        data.iMlast[i] = data.pjMi[i];
        data.S.matrix().rightCols(model.m_nvs[i]) = jdata.S().matrix();
      }
      else
      {
        const int idx_v = model.m_idx_v[i] - model.m_idx_v[0];

        data.iMlast[i] = data.pjMi[i] * data.iMlast[succ];
        data.S.matrix().middleCols(idx_v,model.m_nvs[i]) = data.iMlast[succ].actInv(jdata.S());
      }

    }
    
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  template<typename ConfigVectorType>
  inline void JointModelCompositeTpl<Scalar,Options,JointCollectionTpl>::
  calc(JointDataDerived & data, const Eigen::MatrixBase<ConfigVectorType> & qs) const
  {
    assert(joints.size() > 0);
    assert(data.joints.size() == joints.size());
    
    typedef JointCompositeCalcZeroOrderStep<Scalar,Options,JointCollectionTpl,ConfigVectorType> Algo;

    for (int i=(int)(joints.size()-1); i >= 0; --i)
    {
      Algo::run(joints[(size_t)i],
                data.joints[(size_t)i],
                typename Algo::ArgsType(*this,data,qs.derived()));
    }
    data.M = data.iMlast.front();
  }

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  struct JointCompositeCalcFirstOrderStep
  : public fusion::JointUnaryVisitorBase< JointCompositeCalcFirstOrderStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType> >
  {
    typedef JointModelCompositeTpl<Scalar,Options,JointCollectionTpl> JointModelComposite;
    typedef JointDataCompositeTpl<Scalar,Options,JointCollectionTpl> JointDataComposite;
    
    typedef boost::fusion::vector<const JointModelComposite &,
                                  JointDataComposite &,
                                  const ConfigVectorType &,
                                  const TangentVectorType &
                                  > ArgsType;

    template<typename JointModel>
    static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                     pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const JointModelComposite & model,
                     JointDataComposite & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q,
                     const Eigen::MatrixBase<TangentVectorType> & v)
    {
      const JointIndex & i  = jmodel.id();
      const JointIndex succ = i+1; // successor

      jmodel.calc(jdata.derived(), q.derived(), v.derived());

      data.pjMi[i] = model.jointPlacements[i] * jdata.M();

      if (succ == model.joints.size())
      {
        data.iMlast[i] = data.pjMi[i];
        data.S.matrix().rightCols(model.m_nvs[i]) = jdata.S().matrix();
        data.v = jdata.v();
        data.c = jdata.c();
      }
      else
      {
        const int idx_v = model.m_idx_v[i] - model.m_idx_v[0];

        data.iMlast[i] = data.pjMi[i] * data.iMlast[succ];
        data.S.matrix().middleCols(idx_v,model.m_nvs[i]) = data.iMlast[succ].actInv(jdata.S());

        typename JointModelComposite::Motion v_tmp = data.iMlast[succ].actInv(jdata.v());

        data.v += v_tmp;

        data.c -= data.v.cross(v_tmp);
        data.c += data.iMlast[succ].actInv(jdata.c());
      }
 
    }
    
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  template<typename ConfigVectorType, typename TangentVectorType>
  inline void JointModelCompositeTpl<Scalar,Options,JointCollectionTpl>
  ::calc(JointDataDerived & jdata,
         const Eigen::MatrixBase<ConfigVectorType> & qs,
         const Eigen::MatrixBase<TangentVectorType> & vs) const
  {
    assert(joints.size() > 0);
    assert(jdata.joints.size() == joints.size());
    
    typedef JointCompositeCalcFirstOrderStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType> Algo;

    for (int i=(int)(joints.size()-1); i >= 0; --i)
    {
      Algo::run(joints[(size_t)i],
                jdata.joints[(size_t)i],
                typename Algo::ArgsType(*this,jdata,qs.derived(),vs.derived()));
    }
    
    jdata.M = jdata.iMlast.front();
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_joint_composite_hxx__
