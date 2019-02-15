//
// Copyright (c) 2018)2019 CNRS, INRIA
//

#ifndef __pinocchio_lie_group_algo_hxx__
#define __pinocchio_lie_group_algo_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/joint/joint-composite.hpp"

namespace pinocchio
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
    
#define PINOCCHIO_DETAILS_WRITE_ARGS_0(JM)                               const JointModelBase<JM> & jmodel
#define PINOCCHIO_DETAILS_WRITE_ARGS_1(JM) PINOCCHIO_DETAILS_WRITE_ARGS_0(JM), typename boost::fusion::result_of::at_c<ArgsType, 0>::type a0
#define PINOCCHIO_DETAILS_WRITE_ARGS_2(JM) PINOCCHIO_DETAILS_WRITE_ARGS_1(JM), typename boost::fusion::result_of::at_c<ArgsType, 1>::type a1
#define PINOCCHIO_DETAILS_WRITE_ARGS_3(JM) PINOCCHIO_DETAILS_WRITE_ARGS_2(JM), typename boost::fusion::result_of::at_c<ArgsType, 2>::type a2
#define PINOCCHIO_DETAILS_WRITE_ARGS_4(JM) PINOCCHIO_DETAILS_WRITE_ARGS_3(JM), typename boost::fusion::result_of::at_c<ArgsType, 3>::type a3
    
#define PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_1(Algo)                          \
  template <typename Visitor, typename JointCollection>                       \
  struct Algo <Visitor, JointModelCompositeTpl<JointCollection> > {           \
    typedef typename Visitor::ArgsType ArgsType;                              \
    static void run (PINOCCHIO_DETAILS_WRITE_ARGS_1(JointModelCompositeTpl<JointCollection>))         \
    { ::pinocchio::details::Dispatch< Visitor >::run(jmodel.derived(), ArgsType(a0)); } \
  }
   
#define PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_2(Algo)                 \
  template <typename Visitor, typename JointCollection>                    \
  struct Algo <Visitor, JointModelCompositeTpl<JointCollection> > {        \
    typedef typename Visitor::ArgsType ArgsType;                  \
    static void run (PINOCCHIO_DETAILS_WRITE_ARGS_2(JointModelCompositeTpl<JointCollection>))         \
    { ::pinocchio::details::Dispatch< Visitor >::run(jmodel.derived(), ArgsType(a0,a1)); } \
  }
    
#define PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_3(Algo)                 \
  template <typename Visitor, typename JointCollection>                    \
  struct Algo <Visitor, JointModelCompositeTpl<JointCollection> > {        \
    typedef typename Visitor::ArgsType ArgsType;                  \
    static void run (PINOCCHIO_DETAILS_WRITE_ARGS_3(JointModelCompositeTpl<JointCollection>))         \
    { ::pinocchio::details::Dispatch< Visitor >::run(jmodel.derived(), ArgsType(a0,a1,a2)); } \
  }
    
#define PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_4(Algo)                 \
  template <typename Visitor, typename JointCollection>                    \
  struct Algo <Visitor, JointModelCompositeTpl<JointCollection> > {        \
    typedef typename Visitor::ArgsType ArgsType;                  \
    static void run (PINOCCHIO_DETAILS_WRITE_ARGS_4(JointModelCompositeTpl<JointCollection>))         \
    { ::pinocchio::details::Dispatch< Visitor >::run(jmodel.derived(), ArgsType(a0,a1,a2,a3)); } \
  }
    
#define PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_1(Algo, Visitor)                      \
    typedef LieGroup_t LieGroupMap;                                              \
    template<typename JointModel>                                              \
    static void algo(PINOCCHIO_DETAILS_WRITE_ARGS_1(JointModel))                     \
    { AlgoDispatch<JointModel>::run(jmodel, a0); }                   \
    template<typename JointModel>                            \
    struct AlgoDispatch : Algo<Visitor, JointModel>                            \
    { using Algo<Visitor, JointModel>::run; };
    
#define PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_2(Algo, Visitor)                      \
    typedef LieGroup_t LieGroupMap;                                              \
    template<typename JointModel>                                              \
    static void algo(PINOCCHIO_DETAILS_WRITE_ARGS_2(JointModel))                     \
    { AlgoDispatch<JointModel>::run(jmodel, a0, a1); }               \
    template<typename JointModel>                            \
    struct AlgoDispatch : Algo<Visitor, JointModel>                            \
    { using Algo<Visitor, JointModel>::run; };
    
#define PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_3(Algo, Visitor)                      \
    typedef LieGroup_t LieGroupMap;                                              \
    template<typename JointModel>                                              \
    static void algo(PINOCCHIO_DETAILS_WRITE_ARGS_3(JointModel))                     \
    { AlgoDispatch<JointModel>::run(jmodel, a0, a1, a2); }           \
    template<typename JointModel>                            \
    struct AlgoDispatch : Algo<Visitor, JointModel>                            \
    { using Algo<Visitor, JointModel>::run; };
    
#define PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_4(Algo, Visitor)                      \
    typedef LieGroup_t LieGroupMap;                                              \
    template<typename JointModel>                                              \
    static void algo(PINOCCHIO_DETAILS_WRITE_ARGS_4(JointModel))                     \
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
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_3(IntegrateStepAlgo, IntegrateStep)
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
                    jmodel.jointConfigSelector  (PINOCCHIO_EIGEN_CONST_CAST(ConfigVectorOut,result)));
    }
  };
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_3(IntegrateStepAlgo);
  
  template<typename Visitor, typename JointModel> struct dIntegrateStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorIn, typename TangentVectorIn, typename JacobianMatrixType>
  struct dIntegrateStep
  : public fusion::JointVisitorBase< dIntegrateStep<LieGroup_t,ConfigVectorIn,TangentVectorIn,JacobianMatrixType> >
  {
    typedef boost::fusion::vector<const ConfigVectorIn &,
                                  const TangentVectorIn &,
                                  JacobianMatrixType &,
                                  const ArgumentPosition &
                                  > ArgsType;
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_4(dIntegrateStepAlgo, dIntegrateStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct dIntegrateStepAlgo
  {
    template<typename ConfigVectorIn, typename TangentVector, typename JacobianMatrixType>
    static void run(const JointModelBase<JointModel> & jmodel,
                    const Eigen::MatrixBase<ConfigVectorIn> & q,
                    const Eigen::MatrixBase<TangentVector> & v,
                    const Eigen::MatrixBase<JacobianMatrixType> & mat,
                    const ArgumentPosition & arg)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      
      typename LieGroupMap::template operation<JointModel>::type lgo;
      lgo.dIntegrate(jmodel.jointConfigSelector  (q.derived()),
                     jmodel.jointVelocitySelector(v.derived()),
                     jmodel.jointBlock(PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrixType,mat)),
                     arg);
    }
  };
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_4(dIntegrateStepAlgo);
  
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
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_4(InterpolateStepAlgo, InterpolateStep)
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
                      jmodel.jointConfigSelector(PINOCCHIO_EIGEN_CONST_CAST(ConfigVectorOut,result)));
    }
  };
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_4(InterpolateStepAlgo);
  
  template<typename Visitor, typename JointModel> struct DifferenceStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorIn1, typename ConfigVectorIn2, typename TangentVectorOut>
  struct DifferenceStep
  : public fusion::JointVisitorBase< DifferenceStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,TangentVectorOut> >
  {
    typedef boost::fusion::vector<const ConfigVectorIn1 &,
                                  const ConfigVectorIn2 &,
                                  TangentVectorOut &
                                  > ArgsType;
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_3(DifferenceStepAlgo, DifferenceStep)
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
                     jmodel.jointVelocitySelector(PINOCCHIO_EIGEN_CONST_CAST(TangentVectorOut,result)));
    }
  };
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_3(DifferenceStepAlgo);
  
  template<typename Visitor, typename JointModel> struct SquaredDistanceStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorIn1, typename ConfigVectorIn2, typename DistanceVectorOut>
  struct SquaredDistanceStep
  : public fusion::JointVisitorBase<SquaredDistanceStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,DistanceVectorOut> >
  {
    typedef boost::fusion::vector<const JointIndex &,
                                  const ConfigVectorIn1 &,
                                  const ConfigVectorIn2 &,
                                  DistanceVectorOut &
                                  > ArgsType;
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_4(SquaredDistanceStepAlgo, SquaredDistanceStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct SquaredDistanceStepAlgo
  {
    template<typename ConfigVectorIn1, typename ConfigVectorIn2, typename DistanceVectorOut>
    static void run(const JointModelBase<JointModel> & jmodel,
                    const JointIndex & i,
                    const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                    const Eigen::MatrixBase<ConfigVectorIn2> & q1,
                    const Eigen::MatrixBase<DistanceVectorOut> & distances)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      typename LieGroupMap::template operation<JointModel>::type lgo;
      DistanceVectorOut & distances_ = PINOCCHIO_EIGEN_CONST_CAST(DistanceVectorOut,distances);
      distances_[(Eigen::DenseIndex)i] += lgo.squaredDistance(jmodel.jointConfigSelector(q0.derived()),
                                                              jmodel.jointConfigSelector(q1.derived()));
    }
  };
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_4(SquaredDistanceStepAlgo);

  template<typename Visitor, typename JointModel> struct SquaredDistanceSumStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorIn1, typename ConfigVectorIn2, typename Scalar>
  struct SquaredDistanceSumStep
  : public fusion::JointVisitorBase<SquaredDistanceSumStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,Scalar> >
  {
    typedef boost::fusion::vector<const ConfigVectorIn1 &,
                                  const ConfigVectorIn2 &,
                                  Scalar &
                                  > ArgsType;
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_3(SquaredDistanceSumStepAlgo, SquaredDistanceSumStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct SquaredDistanceSumStepAlgo
  {
    template<typename ConfigVectorIn1, typename ConfigVectorIn2>
    static void run(const JointModelBase<JointModel> & jmodel,
                    const Eigen::MatrixBase<ConfigVectorIn1> & q0,
                    const Eigen::MatrixBase<ConfigVectorIn2> & q1,
                    typename ConfigVectorIn1::Scalar & squaredDistance)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      typename LieGroupMap::template operation<JointModel>::type lgo;
      squaredDistance += lgo.squaredDistance(jmodel.jointConfigSelector(q0.derived()),
                                             jmodel.jointConfigSelector(q1.derived()));
    }
  };
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_3(SquaredDistanceSumStepAlgo);
  
  template<typename Visitor, typename JointModel> struct RandomConfigurationStepAlgo;
  
  template<typename LieGroup_t,typename ConfigVectorOut, typename ConfigVectorIn1, typename ConfigVectorIn2>
  struct RandomConfigurationStep
  : public fusion::JointVisitorBase< RandomConfigurationStep<LieGroup_t,ConfigVectorOut,ConfigVectorIn1,ConfigVectorIn2> >
  {
    typedef boost::fusion::vector<ConfigVectorOut &,
                                  const ConfigVectorIn1 &,
                                  const ConfigVectorIn2 &
                                  > ArgsType;
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_3(RandomConfigurationStepAlgo, RandomConfigurationStep)
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
                              jmodel.jointConfigSelector(PINOCCHIO_EIGEN_CONST_CAST(ConfigVectorOut,q)));
    }
  };
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_3(RandomConfigurationStepAlgo);
  
  template<typename Visitor, typename JointModel> struct NormalizeStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorType>
  struct NormalizeStep
  : public fusion::JointVisitorBase< NormalizeStep<LieGroup_t,ConfigVectorType> >
  {
    typedef boost::fusion::vector<ConfigVectorType &> ArgsType;
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_1(NormalizeStepAlgo, NormalizeStep)
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
      lgo.normalize(jmodel.jointConfigSelector(PINOCCHIO_EIGEN_CONST_CAST(ConfigVectorType,qout)));
    }
  };
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_1(NormalizeStepAlgo);
  
  template<typename Visitor, typename JointModel> struct IsSameConfigurationStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorIn1, typename ConfigVectorIn2, typename Scalar>
  struct IsSameConfigurationStep
  : public fusion::JointVisitorBase< IsSameConfigurationStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,Scalar> >
  {
    typedef boost::fusion::vector<bool &,
                                  const ConfigVectorIn1 &,
                                  const ConfigVectorIn2 &,
                                  const Scalar &> ArgsType;
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_4(IsSameConfigurationStepAlgo, IsSameConfigurationStep)
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
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_4(IsSameConfigurationStepAlgo);
  
  template<typename Visitor, typename JointModel> struct NeutralStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorType>
  struct NeutralStep
  : public fusion::JointVisitorBase< NeutralStep<LieGroup_t,ConfigVectorType> >
  {
    typedef boost::fusion::vector<ConfigVectorType &> ArgsType;
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_1(NeutralStepAlgo, NeutralStep)
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
      jmodel.jointConfigSelector(PINOCCHIO_EIGEN_CONST_CAST(ConfigVectorType,neutral_elt)) = lgo.neutral();
    }
  };
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_1(NeutralStepAlgo);
  
  template<typename Visitor, typename JointModel> struct IntegrateCoeffWiseJacobianStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorType, typename JacobianMatrix>
  struct IntegrateCoeffWiseJacobianStep
  : public fusion::JointVisitorBase< IntegrateCoeffWiseJacobianStep<LieGroup_t,ConfigVectorType,JacobianMatrix> >
  {
    typedef boost::fusion::vector<const ConfigVectorType &, JacobianMatrix &> ArgsType;
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_2(IntegrateCoeffWiseJacobianStepAlgo,
                                      IntegrateCoeffWiseJacobianStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct IntegrateCoeffWiseJacobianStepAlgo
  {
    template<typename ConfigVectorType, typename JacobianMatrix>
    static void run(const JointModelBase<JointModel> & jmodel,
                    const Eigen::MatrixBase<ConfigVectorType> & q,
                    const Eigen::MatrixBase<JacobianMatrix> & jacobian)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      
      typedef typename LieGroupMap::template operation<JointModel>::type LieGroup;
      LieGroup lgo;
      lgo.integrateCoeffWiseJacobian(jmodel.jointConfigSelector(q.derived()),
                                     PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrix,jacobian).template block<LieGroup::NQ,LieGroup::NV>(jmodel.idx_q(),jmodel.idx_v(),jmodel.nq(),jmodel.nv()));
    }
  };
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_2(IntegrateCoeffWiseJacobianStepAlgo);
  
}

#endif // ifndef __pinocchio_lie_group_algo_hxx__
