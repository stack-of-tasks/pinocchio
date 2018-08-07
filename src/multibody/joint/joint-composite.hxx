//
// Copyright (c) 2016,2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terljMj of the GNU Lesser General Public
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

#ifndef __se3_joint_composite_hxx__
#define __se3_joint_composite_hxx__

#include "pinocchio/multibody/visitor.hpp"

namespace se3 
{

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl, typename ConfigVectorType>
  struct JointCompositeCalcZeroOrderStep
  : fusion::JointVisitorBase< JointCompositeCalcZeroOrderStep<Scalar,Options,JointCollectionTpl,ConfigVectorType> >
  {
    typedef JointModelCompositeTpl<Scalar,Options,JointCollectionTpl> JointModelComposite;
    typedef JointDataCompositeTpl<Scalar,Options,JointCollectionTpl> JointDataComposite;
    
    typedef boost::fusion::vector<const JointModelComposite &,
                                  JointDataComposite &,
                                  const ConfigVectorType &
                                  > ArgsType;

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
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
        data.S.matrix().middleCols(idx_v,model.m_nvs[i]) = data.iMlast[succ].inverse().act(jdata.S()); // TODO: avoid computing inverse
      }

    }
    
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  template<typename ConfigVectorType>
  inline void JointModelCompositeTpl<Scalar,Options,JointCollectionTpl>::
  calc(JointData & data, const Eigen::MatrixBase<ConfigVectorType> & qs) const
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
  : public fusion::JointVisitorBase< JointCompositeCalcFirstOrderStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType> >
  {
    typedef JointModelCompositeTpl<Scalar,Options,JointCollectionTpl> JointModelComposite;
    typedef JointDataCompositeTpl<Scalar,Options,JointCollectionTpl> JointDataComposite;
    
    typedef boost::fusion::vector<const JointModelComposite &,
                                  JointDataComposite &,
                                  const ConfigVectorType &,
                                  const TangentVectorType &
                                  > ArgsType;

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const JointModelComposite & model,
                     JointDataComposite & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q,
                     const Eigen::MatrixBase<TangentVectorType> & v)
    {
      const JointIndex & i  = jmodel.id();
      const JointIndex succ = i+1; // successor

      jmodel.calc(jdata.derived(), q.derived(), v.derived());

      data.pjMi[i] = model.jointPlacements[i] * jdata.M ();

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
        data.S.matrix().middleCols(idx_v,model.m_nvs[i]) = data.iMlast[succ].inverse().act(jdata.S()); // TODO: avoid computing inverse

        Motion v_tmp = data.iMlast[succ].actInv(jdata.v());

        data.v += v_tmp;

        data.c -= data.v.cross(v_tmp);
        data.c += data.iMlast[succ].actInv(jdata.c());
      }
 
    }
    
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  template<typename ConfigVectorType, typename TangentVectorType>
  inline void JointModelCompositeTpl<Scalar,Options,JointCollectionTpl>
  ::calc(JointData & jdata,
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

} // namespace se3

#endif // ifndef __se3_joint_composite_hxx__
