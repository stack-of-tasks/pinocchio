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
#include "pinocchio/multibody/joint/joint-composite.hpp"

namespace se3
{
  
  namespace details
  {
    template<typename Algo>
    struct Dispatch
    {
      template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ArgsType>
      static void run(const JointModelCompositeTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                      ArgsType args)
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
    
#define SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_1(Algo)                          \
  template <typename Visitor, typename JointCollection>                       \
  struct Algo <Visitor, JointModelCompositeTpl<JointCollection> > {           \
    typedef typename Visitor::ArgsType ArgsType;                              \
    static void run (SE3_DETAILS_WRITE_ARGS_1(JointModelCompositeTpl<JointCollection>))         \
    { ::se3::details::Dispatch< Visitor >::run(jmodel.derived(), ArgsType(a0)); } \
  }
   
#define SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_2(Algo)                 \
  template <typename Visitor, typename JointCollection>                    \
  struct Algo <Visitor, JointModelCompositeTpl<JointCollection> > {        \
    typedef typename Visitor::ArgsType ArgsType;                  \
    static void run (SE3_DETAILS_WRITE_ARGS_2(JointModelCompositeTpl<JointCollection>))         \
    { ::se3::details::Dispatch< Visitor >::run(jmodel.derived(), ArgsType(a0,a1)); } \
  }
    
#define SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_3(Algo)                 \
  template <typename Visitor, typename JointCollection>                    \
  struct Algo <Visitor, JointModelCompositeTpl<JointCollection> > {        \
    typedef typename Visitor::ArgsType ArgsType;                  \
    static void run (SE3_DETAILS_WRITE_ARGS_3(JointModelCompositeTpl<JointCollection>))         \
    { ::se3::details::Dispatch< Visitor >::run(jmodel.derived(), ArgsType(a0,a1,a2)); } \
  }
    
#define SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_4(Algo)                 \
  template <typename Visitor, typename JointCollection>                    \
  struct Algo <Visitor, JointModelCompositeTpl<JointCollection> > {        \
    typedef typename Visitor::ArgsType ArgsType;                  \
    static void run (SE3_DETAILS_WRITE_ARGS_4(JointModelCompositeTpl<JointCollection>))         \
    { ::se3::details::Dispatch< Visitor >::run(jmodel.derived(), ArgsType(a0,a1,a2,a3)); } \
  }
    
#define SE3_DETAILS_VISITOR_METHOD_ALGO_1(Algo, Visitor)                      \
    typedef LieGroup_t LieGroupMap;                                              \
    template<typename JointModel>                                              \
    static void algo(SE3_DETAILS_WRITE_ARGS_1(JointModel))                     \
    { AlgoDispatch<JointModel>::run(jmodel, a0); }                   \
    template<typename JointModel>                            \
    struct AlgoDispatch : Algo<Visitor, JointModel>                            \
    { using Algo<Visitor, JointModel>::run; };
    
#define SE3_DETAILS_VISITOR_METHOD_ALGO_2(Algo, Visitor)                      \
    typedef LieGroup_t LieGroupMap;                                              \
    template<typename JointModel>                                              \
    static void algo(SE3_DETAILS_WRITE_ARGS_2(JointModel))                     \
    { AlgoDispatch<JointModel>::run(jmodel, a0, a1); }               \
    template<typename JointModel>                            \
    struct AlgoDispatch : Algo<Visitor, JointModel>                            \
    { using Algo<Visitor, JointModel>::run; };
    
#define SE3_DETAILS_VISITOR_METHOD_ALGO_3(Algo, Visitor)                      \
    typedef LieGroup_t LieGroupMap;                                              \
    template<typename JointModel>                                              \
    static void algo(SE3_DETAILS_WRITE_ARGS_3(JointModel))                     \
    { AlgoDispatch<JointModel>::run(jmodel, a0, a1, a2); }           \
    template<typename JointModel>                            \
    struct AlgoDispatch : Algo<Visitor, JointModel>                            \
    { using Algo<Visitor, JointModel>::run; };
    
#define SE3_DETAILS_VISITOR_METHOD_ALGO_4(Algo, Visitor)                      \
    typedef LieGroup_t LieGroupMap;                                              \
    template<typename JointModel>                                              \
    static void algo(SE3_DETAILS_WRITE_ARGS_4(JointModel))                     \
    { AlgoDispatch<JointModel>::run(jmodel, a0, a1, a2, a3); }       \
    template<typename JointModel>                            \
    struct AlgoDispatch : Algo<Visitor, JointModel>                            \
    { using Algo<Visitor, JointModel>::run; };
  } // namespace details
  
  template<typename Visitor, typename JointModel> struct IntegrateStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorIn, typename TangentVectorIn, typename ConfigVectorOut>
  struct IntegrateStep
  : public fusion::JointVisitorBase< IntegrateStep<LieGroup_t,ConfigVectorIn,TangentVectorIn,ConfigVectorOut> >
  {
    typedef boost::fusion::vector<const ConfigVectorIn &,
                                  const TangentVectorIn &,
                                  ConfigVectorOut &
                                  > ArgsType;
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_3(IntegrateStepAlgo, IntegrateStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct IntegrateStepAlgo
  {
    template<typename ConfigVectorIn, typename TangentVector, typename ConfigVectorOut>
    static void run(const JointModelBase<JointModel> & jmodel,
                    const Eigen::MatrixBase<ConfigVectorIn> & q,
                    const Eigen::MatrixBase<TangentVector> & v,
                    const Eigen::MatrixBase<ConfigVectorOut> & result)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      
      typename LieGroupMap::template operation<JointModel>::type lgo;
      lgo.integrate(jmodel.jointConfigSelector  (q.derived()),
                    jmodel.jointVelocitySelector(v.derived()),
                    jmodel.jointConfigSelector  (EIGEN_CONST_CAST(ConfigVectorOut,result)));
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_3(IntegrateStepAlgo);
  
  template<typename Visitor, typename JointModel> struct InterpolateStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorIn1, typename ConfigVectorIn2, typename Scalar, typename ConfigVectorOut>
  struct InterpolateStep
  : public fusion::JointVisitorBase< InterpolateStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,Scalar,ConfigVectorOut  > >
  {
    typedef boost::fusion::vector<const ConfigVectorIn1 &,
                                  const ConfigVectorIn2 &,
                                  const Scalar &,
                                  ConfigVectorOut &
                                  > ArgsType;
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_4(InterpolateStepAlgo, InterpolateStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct InterpolateStepAlgo
  {
    template<typename ConfigVectorIn1, typename ConfigVectorIn2, typename Scalar, typename ConfigVectorOut>
    static void run(const JointModelBase<JointModel> & jmodel,
                    const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                    const Eigen::MatrixBase<ConfigVectorIn2> & q1,
                    const Scalar & u,
                    const Eigen::MatrixBase<ConfigVectorOut> & result)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      
      typename LieGroupMap::template operation<JointModel>::type lgo;
      lgo.interpolate(jmodel.jointConfigSelector(q0.derived()),
                      jmodel.jointConfigSelector(q1.derived()),
                      u,
                      jmodel.jointConfigSelector(EIGEN_CONST_CAST(ConfigVectorOut,result)));
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_4(InterpolateStepAlgo);
  
  template<typename Visitor, typename JointModel> struct DifferenceStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorIn1, typename ConfigVectorIn2, typename TangentVectorOut>
  struct DifferenceStep
  : public fusion::JointVisitorBase< DifferenceStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,TangentVectorOut> >
  {
    typedef boost::fusion::vector<const ConfigVectorIn1 &,
                                  const ConfigVectorIn2 &,
                                  TangentVectorOut &
                                  > ArgsType;
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_3(DifferenceStepAlgo, DifferenceStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct DifferenceStepAlgo
  {
    template<typename ConfigVectorIn1, typename ConfigVectorIn2, typename TangentVectorOut>
    static void run(const JointModelBase<JointModel> & jmodel,
                    const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                    const Eigen::MatrixBase<ConfigVectorIn2> & q1,
                    const Eigen::MatrixBase<TangentVectorOut> & result)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      
      typename LieGroupMap::template operation<JointModel>::type lgo;
      lgo.difference(jmodel.jointConfigSelector(q0.derived()),
                     jmodel.jointConfigSelector(q1.derived()),
                     jmodel.jointVelocitySelector(EIGEN_CONST_CAST(TangentVectorOut,result)));
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_3(DifferenceStepAlgo);
  
  template<typename Visitor, typename JointModel> struct SquaredDistanceStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorIn1, typename ConfigVectorIn2, typename DistanceVectorOut>
  struct SquaredDistanceStep
  : public fusion::JointVisitorBase<SquaredDistanceStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,DistanceVectorOut> >
  {
    typedef boost::fusion::vector<const JointIndex,
                                  const ConfigVectorIn1 &,
                                  const ConfigVectorIn2 &,
                                  DistanceVectorOut &
                                  > ArgsType;
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_4(SquaredDistanceStepAlgo, SquaredDistanceStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct SquaredDistanceStepAlgo
  {
    template<typename ConfigVectorIn1, typename ConfigVectorIn2, typename DistanceVectorOut>
    static void run(const JointModelBase<JointModel> & jmodel,
                    const JointIndex i,
                    const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                    const Eigen::MatrixBase<ConfigVectorIn2> & q1,
                    const Eigen::MatrixBase<DistanceVectorOut> & distances)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      
      typename LieGroupMap::template operation<JointModel>::type lgo;
      DistanceVectorOut & distances_ = EIGEN_CONST_CAST(DistanceVectorOut,distances);
      distances_[(long)i] += lgo.squaredDistance(jmodel.jointConfigSelector(q0.derived()),
                                                 jmodel.jointConfigSelector(q1.derived()));
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_4(SquaredDistanceStepAlgo);
  
  template<typename Visitor, typename JointModel> struct RandomConfigurationStepAlgo;
  
  template<typename LieGroup_t,typename ConfigVectorOut, typename ConfigVectorIn1, typename ConfigVectorIn2>
  struct RandomConfigurationStep
  : public fusion::JointVisitorBase< RandomConfigurationStep<LieGroup_t,ConfigVectorOut,ConfigVectorIn1,ConfigVectorIn2> >
  {
    typedef boost::fusion::vector<ConfigVectorOut &,
                                  const ConfigVectorIn1 &,
                                  const ConfigVectorIn2 &
                                  > ArgsType;
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_3(RandomConfigurationStepAlgo, RandomConfigurationStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct RandomConfigurationStepAlgo
  {
    template<typename ConfigVectorOut, typename ConfigVectorIn1, typename ConfigVectorIn2>
    static void run(const JointModelBase<JointModel> & jmodel,
                    const Eigen::MatrixBase<ConfigVectorOut> & q,
                    const Eigen::MatrixBase<ConfigVectorIn1> & lowerLimits,
                    const Eigen::MatrixBase<ConfigVectorIn2> & upperLimits)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      
      typename LieGroupMap::template operation<JointModel>::type lgo;
      lgo.randomConfiguration(jmodel.jointConfigSelector(lowerLimits.derived()),
                              jmodel.jointConfigSelector(upperLimits.derived()),
                              jmodel.jointConfigSelector(EIGEN_CONST_CAST(ConfigVectorOut,q)));
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_3(RandomConfigurationStepAlgo);
  
  template<typename Visitor, typename JointModel> struct NormalizeStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorType>
  struct NormalizeStep
  : public fusion::JointVisitorBase< NormalizeStep<LieGroup_t,ConfigVectorType> >
  {
    typedef boost::fusion::vector<ConfigVectorType &> ArgsType;
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_1(NormalizeStepAlgo, NormalizeStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct NormalizeStepAlgo
  {
    template<typename ConfigVectorType>
    static void run(const JointModelBase<JointModel> & jmodel,
                    const Eigen::MatrixBase<ConfigVectorType> & qout)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      
      typename LieGroupMap::template operation<JointModel>::type lgo;
      lgo.normalize(jmodel.jointConfigSelector(EIGEN_CONST_CAST(ConfigVectorType,qout)));
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_1(NormalizeStepAlgo);
  
  template<typename Visitor, typename JointModel> struct IsSameConfigurationStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorIn1, typename ConfigVectorIn2, typename Scalar>
  struct IsSameConfigurationStep
  : public fusion::JointVisitorBase< IsSameConfigurationStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,Scalar> >
  {
    typedef boost::fusion::vector<bool &,
                                  const ConfigVectorIn1 &,
                                  const ConfigVectorIn2 &,
                                  const Scalar &> ArgsType;
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_4(IsSameConfigurationStepAlgo, IsSameConfigurationStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct IsSameConfigurationStepAlgo
  {
    template<typename ConfigVectorIn1, typename ConfigVectorIn2>
    static void run(const JointModelBase<JointModel> & jmodel,
                    bool & isSame,
                    const Eigen::MatrixBase<ConfigVectorIn1> & q1,
                    const Eigen::MatrixBase<ConfigVectorIn2> & q2,
                    const typename ConfigVectorIn1::Scalar & prec)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      
      typename LieGroupMap::template operation<JointModel>::type lgo;
      isSame &= lgo.isSameConfiguration(jmodel.jointConfigSelector(q1.derived()),
                                        jmodel.jointConfigSelector(q2.derived()),
                                        prec);
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_4(IsSameConfigurationStepAlgo);
  
  template<typename Visitor, typename JointModel> struct NeutralStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorType>
  struct NeutralStep
  : public fusion::JointVisitorBase< NeutralStep<LieGroup_t,ConfigVectorType> >
  {
    typedef boost::fusion::vector<ConfigVectorType &> ArgsType;
    
    SE3_DETAILS_VISITOR_METHOD_ALGO_1(NeutralStepAlgo, NeutralStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct NeutralStepAlgo
  {
    template<typename ConfigVectorType>
    static void run(const JointModelBase<JointModel> & jmodel,
                    const Eigen::MatrixBase<ConfigVectorType> & neutral_elt)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      
      typename LieGroupMap::template operation<JointModel>::type lgo;
      jmodel.jointConfigSelector(EIGEN_CONST_CAST(ConfigVectorType,neutral_elt)) = lgo.neutral();
    }
  };
  
  SE3_DETAILS_DISPATCH_JOINT_COMPOSITE_1(NeutralStepAlgo);
  
}

#endif // ifndef __se3_lie_group_algo_hxx__
