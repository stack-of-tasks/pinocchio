//
// Copyright (c) 2018 CNRS
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

#ifndef __se3_lie_group_algo_hxx__
#define __se3_lie_group_algo_hxx__

#include "pinocchio/multibody/visitor.hpp"

namespace se3
{
  
  namespace details
  {
    template<typename Algo>
    struct Dispatch
    {
      static void run (const JointModelComposite& jmodel,
                       typename Algo::ArgsType args)
      {
        for (size_t i = 0; i < jmodel.joints.size(); ++i)
          Algo::run(jmodel.joints[i], args);
      }
    };
    
#define SE3_DETAILS_WRITE_ARGS_0(JM)                               const JointModelBase<JM> & jmodel
#define SE3_DETAILS_WRITE_ARGS_1(JM) SE3_DETAILS_WRITE_ARGS_0(JM), typename boost::fusion::result_of::at_c<ArgsType, 0>::type a0
#define SE3_DETAILS_WRITE_ARGS_2(JM) SE3_DETAILS_WRITE_ARGS_1(JM), typename boost::fusion::result_of::at_c<ArgsType, 1>::type a1
#define SE3_DETAILS_WRITE_ARGS_3(JM) SE3_DETAILS_WRITE_ARGS_2(JM), typename boost::fusion::result_of::at_c<ArgsType, 2>::type a2
#define SE3_DETAILS_WRITE_ARGS_4(JM) SE3_DETAILS_WRITE_ARGS_3(JM), typename boost::fusion::result_of::at_c<ArgsType, 3>::type a3
    
#define SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_1(Visitor, Algo)                 \
  template <typename LieGroup_t> struct Algo <LieGroup_t, JointModelComposite> {                                         \
    typedef typename Visitor<LieGroup_t>::ArgsType ArgsType;                 \
    static void run (SE3_DETAILS_WRITE_ARGS_1(JointModelComposite))         \
    { ::se3::details::Dispatch< Visitor<LieGroup_t> >::run(jmodel.derived(), ArgsType(a0)); } \
  }
    
#define SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_2(Visitor, Algo)                 \
  template <typename LieGroup_t> struct Algo <LieGroup_t, JointModelComposite> {                                         \
    typedef typename Visitor<LieGroup_t>::ArgsType ArgsType;                 \
    static void run (SE3_DETAILS_WRITE_ARGS_2(JointModelComposite))         \
    { ::se3::details::Dispatch< Visitor<LieGroup_t> >::run(jmodel.derived(), ArgsType(a0, a1)); } \
  }
    
#define SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_3(Visitor, Algo)                 \
  template <typename LieGroup_t> struct Algo <LieGroup_t, JointModelComposite> {                                         \
    typedef typename Visitor<LieGroup_t>::ArgsType ArgsType;                 \
    static void run (SE3_DETAILS_WRITE_ARGS_3(JointModelComposite))         \
    { ::se3::details::Dispatch< Visitor<LieGroup_t> >::run(jmodel.derived(), ArgsType(a0, a1, a2)); } \
  }
    
#define SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_4(Visitor, Algo)                 \
  template <typename LieGroup_t> struct Algo <LieGroup_t, JointModelComposite> {                                         \
    typedef typename Visitor<LieGroup_t>::ArgsType ArgsType;                 \
    static void run (SE3_DETAILS_WRITE_ARGS_4(JointModelComposite))         \
    { ::se3::details::Dispatch< Visitor<LieGroup_t> >::run(jmodel.derived(), ArgsType(a0, a1, a2, a3)); } \
  }
    
#define SE3_DETAILS_VISITOR_METHOD_ALGO_1(Algo, TplParam)                      \
    template<typename JointModel>                                              \
    static void algo(SE3_DETAILS_WRITE_ARGS_1(JointModel))                     \
    { Algo<TplParam, JointModel>::run(jmodel, a0); }
    
#define SE3_DETAILS_VISITOR_METHOD_ALGO_2(Algo, TplParam)                      \
    template<typename JointModel>                                              \
    static void algo(SE3_DETAILS_WRITE_ARGS_2(JointModel))                     \
    { Algo<TplParam, JointModel>::run(jmodel, a0, a1); }
    
#define SE3_DETAILS_VISITOR_METHOD_ALGO_3(Algo, TplParam)                      \
    template<typename JointModel>                                              \
    static void algo(SE3_DETAILS_WRITE_ARGS_3(JointModel))                     \
    { Algo<TplParam, JointModel>::run(jmodel, a0, a1, a2); }
    
#define SE3_DETAILS_VISITOR_METHOD_ALGO_4(Algo, TplParam)                      \
    template<typename JointModel>                                              \
    static void algo(SE3_DETAILS_WRITE_ARGS_4(JointModel))                     \
    { Algo<TplParam, JointModel>::run(jmodel, a0, a1, a2, a3); }
    
  } // namespace details
  
  template<typename LieGroup_t, typename JointModel> struct IntegrateStepAlgo;
  
  template<typename LieGroup_t>
  struct IntegrateStep : public fusion::JointModelVisitor<IntegrateStep<LieGroup_t> >
  {
    typedef boost::fusion::vector<const Eigen::VectorXd &,
    const Eigen::VectorXd &,
    Eigen::VectorXd &
    > ArgsType;
    
    JOINT_MODEL_VISITOR_INIT(IntegrateStep);
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_3(IntegrateStepAlgo, LieGroup_t)
  };
  
  template<typename LieGroup_t, typename JointModel>
  struct IntegrateStepAlgo {
    static void run(const se3::JointModelBase<JointModel> & jmodel,
                    const Eigen::VectorXd & q,
                    const Eigen::VectorXd & v,
                    Eigen::VectorXd & result)
    {
      typename LieGroup_t::template operation<JointModel>::type lgo;
      lgo.integrate(jmodel.jointConfigSelector  (q),
                    jmodel.jointVelocitySelector(v),
                    jmodel.jointConfigSelector  (result));
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_3(IntegrateStep, IntegrateStepAlgo);
  
  template<typename LieGroup_t, typename JointModel> struct InterpolateStepAlgo;
  
  template<typename LieGroup_t>
  struct InterpolateStep : public fusion::JointModelVisitor<InterpolateStep<LieGroup_t> >
  {
    typedef boost::fusion::vector<const Eigen::VectorXd &,
    const Eigen::VectorXd &,
    const double,
    Eigen::VectorXd &
    > ArgsType;
    
    JOINT_MODEL_VISITOR_INIT(InterpolateStep);
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_4(InterpolateStepAlgo, LieGroup_t)
  };
  
  template<typename LieGroup_t, typename JointModel>
  struct InterpolateStepAlgo {
    static void run(const se3::JointModelBase<JointModel> & jmodel,
                    const Eigen::VectorXd & q0,
                    const Eigen::VectorXd & q1,
                    const double u,
                    Eigen::VectorXd & result)
    {
      typename LieGroup_t::template operation<JointModel>::type lgo;
      lgo.interpolate(jmodel.jointConfigSelector(q0),
                      jmodel.jointConfigSelector(q1),
                      u,
                      jmodel.jointConfigSelector(result));
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_4(InterpolateStep, InterpolateStepAlgo);
  
  template<typename LieGroup_t, typename JointModel> struct DifferenceStepAlgo;
  
  template<typename LieGroup_t>
  struct DifferenceStep : public fusion::JointModelVisitor<DifferenceStep<LieGroup_t> >
  {
    typedef boost::fusion::vector<const Eigen::VectorXd &,
    const Eigen::VectorXd &,
    Eigen::VectorXd &
    > ArgsType;
    
    JOINT_MODEL_VISITOR_INIT(DifferenceStep);
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_3(DifferenceStepAlgo, LieGroup_t)
  };
  
  template<typename LieGroup_t, typename JointModel>
  struct DifferenceStepAlgo {
    static void run(const se3::JointModelBase<JointModel> & jmodel,
                    const Eigen::VectorXd & q0,
                    const Eigen::VectorXd & q1,
                    Eigen::VectorXd & result)
    {
      typename LieGroup_t::template operation<JointModel>::type lgo;
      lgo.difference(jmodel.jointConfigSelector(q0),
                     jmodel.jointConfigSelector(q1),
                     jmodel.jointVelocitySelector(result));
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_3(DifferenceStep, DifferenceStepAlgo);
  
  template<typename LieGroup_t, typename JointModel> struct SquaredDistanceStepAlgo;
  
  template<typename LieGroup_t>
  struct SquaredDistanceStep : public fusion::JointModelVisitor<SquaredDistanceStep<LieGroup_t> >
  {
    typedef boost::fusion::vector<const JointIndex,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &,
    Eigen::VectorXd &
    > ArgsType;
    
    JOINT_MODEL_VISITOR_INIT(SquaredDistanceStep);
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_4(SquaredDistanceStepAlgo, LieGroup_t)
  };
  
  template<typename LieGroup_t, typename JointModel>
  struct SquaredDistanceStepAlgo {
    static void run(const se3::JointModelBase<JointModel> & jmodel,
                    const JointIndex i,
                    const Eigen::VectorXd & q0,
                    const Eigen::VectorXd & q1,
                    Eigen::VectorXd & distances)
    {
      typename LieGroup_t::template operation<JointModel>::type lgo;
      distances[(long)i] += lgo.squaredDistance(jmodel.jointConfigSelector(q0),
                                                jmodel.jointConfigSelector(q1));
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_4(SquaredDistanceStep, SquaredDistanceStepAlgo);
  
  template<typename LieGroup_t, typename JointModel> struct RandomConfigurationStepAlgo;
  
  template<typename LieGroup_t>
  struct RandomConfigurationStep : public fusion::JointModelVisitor<RandomConfigurationStep<LieGroup_t> >
  {
    typedef boost::fusion::vector<Eigen::VectorXd &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_MODEL_VISITOR_INIT(RandomConfigurationStep);
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_3(RandomConfigurationStepAlgo, LieGroup_t)
  };
  
  template<typename LieGroup_t, typename JointModel>
  struct RandomConfigurationStepAlgo {
    static void run(const se3::JointModelBase<JointModel> & jmodel,
                    Eigen::VectorXd & q,
                    const Eigen::VectorXd & lowerLimits,
                    const Eigen::VectorXd & upperLimits)
    {
      typename LieGroup_t::template operation<JointModel>::type lgo;
      lgo.randomConfiguration(jmodel.jointConfigSelector(lowerLimits),
                              jmodel.jointConfigSelector(upperLimits),
                              jmodel.jointConfigSelector(q));
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_3(RandomConfigurationStep, RandomConfigurationStepAlgo);
  
  
  
  template<typename LieGroup_t, typename JointModel> struct NormalizeStepAlgo;
  
  template<typename LieGroup_t>
  struct NormalizeStep : public fusion::JointModelVisitor< NormalizeStep<LieGroup_t> >
  {
    typedef boost::fusion::vector<Eigen::VectorXd &> ArgsType;
    
    JOINT_MODEL_VISITOR_INIT(NormalizeStep);
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_1(NormalizeStepAlgo, LieGroup_t)
  };
  
  template<typename LieGroup_t, typename JointModel>
  struct NormalizeStepAlgo
  {
    static void run(const se3::JointModelBase<JointModel> & jmodel,
                    Eigen::VectorXd & qout)
    {
      typename LieGroup_t::template operation<JointModel>::type lgo;
      lgo.normalize(jmodel.jointConfigSelector(qout));
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_1(NormalizeStep, NormalizeStepAlgo);
  
  template<typename LieGroup_t, typename JointModel> struct IsSameConfigurationStepAlgo;
  
  template<typename LieGroup_t>
  struct IsSameConfigurationStep : public fusion::JointModelVisitor<IsSameConfigurationStep<LieGroup_t> >
  {
    typedef boost::fusion::vector<bool &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &,
    const double&> ArgsType;
    
    JOINT_MODEL_VISITOR_INIT(IsSameConfigurationStep);
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_4(IsSameConfigurationStepAlgo, LieGroup_t)
  };
  
  template<typename LieGroup_t, typename JointModel>
  struct IsSameConfigurationStepAlgo {
    static void run(const se3::JointModelBase<JointModel> & jmodel,
                    bool & isSame,
                    const Eigen::VectorXd & q1,
                    const Eigen::VectorXd & q2,
                    const double prec)
    {
      typename LieGroup_t::template operation<JointModel>::type lgo;
      isSame &= lgo.isSameConfiguration(jmodel.jointConfigSelector(q1),
                                        jmodel.jointConfigSelector(q2),
                                        prec);
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_4(IsSameConfigurationStep, IsSameConfigurationStepAlgo);
  
  template<typename LieGroup_t, typename JointModel> struct NeutralStepAlgo;
  
  template<typename LieGroup_t>
  struct NeutralStep : public fusion::JointModelVisitor< NeutralStep<LieGroup_t> >
  {
    typedef boost::fusion::vector<Eigen::VectorXd &> ArgsType;
    
    JOINT_MODEL_VISITOR_INIT(NeutralStep);
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_1(NeutralStepAlgo, LieGroup_t)
  };
  
  template<typename LieGroup_t, typename JointModel>
  struct NeutralStepAlgo
  {
    static void run(const se3::JointModelBase<JointModel> & jmodel,
                    Eigen::VectorXd & neutral_elt)
    {
      typename LieGroup_t::template operation<JointModel>::type lgo;
      jmodel.jointConfigSelector(neutral_elt) = lgo.neutral();
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_1(NeutralStep, NeutralStepAlgo);
  
}

#endif // ifndef __se3_lie_group_algo_hxx__
