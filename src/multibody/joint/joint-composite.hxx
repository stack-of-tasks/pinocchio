//
// Copyright (c) 2016 CNRS
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

  struct JointCompositeCalcZeroOrderStep : public fusion::JointVisitor<JointCompositeCalcZeroOrderStep>
  {
    typedef boost::fusion::vector<const se3::JointModelComposite &,
                                  se3::JointDataComposite &,
                                  const Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT (JointCompositeCalcZeroOrderStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::JointModelComposite & model,
                     se3::JointDataComposite & data,
                     const Eigen::VectorXd & q)
    {
      const JointIndex & i  = jmodel.id();
      const JointIndex succ = i+1; // successor

      jmodel.calc(jdata.derived(), q);

      data.pjMi[i] = model.jointPlacements[i] * jdata.M ();

      if ( succ == model.joints.size() )
      {
        data.iMlast[i] = data.pjMi[i];
        data.S.matrix().rightCols(model.m_nvs[i]) = constraint_xd(jdata.derived()).matrix();
      }
      else
      {
        const int idx_v = model.m_idx_v[i] - model.m_idx_v[0];

        data.iMlast[i] = data.pjMi[i] * data.iMlast[succ];
        data.S.matrix().middleCols(idx_v,model.m_nvs[i]) = data.iMlast[succ].inverse().act(jdata.S()); // TODO: avoid computing inverse
      }

    }
    
  };

  inline void jointCompositeCalcZeroOrder(const JointModelComposite & model, JointDataComposite & data, const Eigen::VectorXd & qs)
  {
    for (int i=(int)(model.joints.size()-1); i >= 0; --i)
    {
      JointCompositeCalcZeroOrderStep::run(model.joints[(size_t)i], data.joints[(size_t)i],
                                    JointCompositeCalcZeroOrderStep::ArgsType (model,data,qs)
                                    );
    }
    data.M = data.iMlast.front();
  }

  struct JointCompositeCalcFirstOrderStep : public fusion::JointVisitor<JointCompositeCalcFirstOrderStep>
  {
    typedef boost::fusion::vector<const se3::JointModelComposite &,
                                  se3::JointDataComposite &,
                                  const Eigen::VectorXd &,
                                  const Eigen::VectorXd &
                                  > ArgsType;

    JOINT_VISITOR_INIT (JointCompositeCalcFirstOrderStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::JointModelComposite & model,
                     se3::JointDataComposite & data,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v)
    {
      const JointIndex & i  = jmodel.id();
      const JointIndex succ = i+1; // successor

      jmodel.calc(jdata.derived(), q, v);

      data.pjMi[i] = model.jointPlacements[i] * jdata.M ();

      if ( succ == model.joints.size() )
      {
        data.iMlast[i] = data.pjMi[i];
        data.S.matrix().rightCols(model.m_nvs[i]) = constraint_xd(jdata.derived()).matrix();
        data.v = jdata.v();
        data.c = jdata.c();
      }
      else
      {
        const int idx_v = model.m_idx_v[i] - model.m_idx_v[0];

        data.iMlast[i] = data.pjMi[i] * data.iMlast[succ];
        data.S.matrix().middleCols(idx_v,model.m_nvs[i]) = data.iMlast[succ].inverse().act(jdata.S()); // TODO: avoid computing inverse

        Motion v_tmp = data.iMlast[succ].actInv(motion(jdata.derived())); // TODO: avoid calling motion

        data.v += v_tmp;

        data.c -= data.v.cross(v_tmp);
        data.c += data.iMlast[succ].actInv(bias(jdata.derived())); // TODO: avoid calling bias
      }
 
    }
    
  };

  inline void jointCompositeCalcFirstOrder(const JointModelComposite & model, JointDataComposite & data, const Eigen::VectorXd & qs, const Eigen::VectorXd & vs)
  {
    for (int i=(int)(model.joints.size()-1); i >= 0; --i)
    {
      JointCompositeCalcFirstOrderStep::run(model.joints[(size_t)i], data.joints[(size_t)i],
                                    JointCompositeCalcFirstOrderStep::ArgsType (model,data,qs,vs)
                                    );
    }
    data.M = data.iMlast.front();
  }

} // namespace se3

#endif // ifndef __se3_joint_composite_hxx__
