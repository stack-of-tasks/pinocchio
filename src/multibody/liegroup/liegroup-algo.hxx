//
// Copyright (c) 2018-2020 CNRS INRIA
//

#ifndef __pinocchio_multibody_liegroup_liegroup_algo_hxx__
#define __pinocchio_multibody_liegroup_liegroup_algo_hxx__

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
    
#define PINOCCHIO_DETAILS_WRITE_ARGS_0(JM) const JointModelBase<JM> & jmodel
#define PINOCCHIO_DETAILS_WRITE_ARGS_1(JM) PINOCCHIO_DETAILS_WRITE_ARGS_0(JM), typename boost::fusion::result_of::at_c<ArgsType, 0>::type a0
#define PINOCCHIO_DETAILS_WRITE_ARGS_2(JM) PINOCCHIO_DETAILS_WRITE_ARGS_1(JM), typename boost::fusion::result_of::at_c<ArgsType, 1>::type a1
#define PINOCCHIO_DETAILS_WRITE_ARGS_3(JM) PINOCCHIO_DETAILS_WRITE_ARGS_2(JM), typename boost::fusion::result_of::at_c<ArgsType, 2>::type a2
#define PINOCCHIO_DETAILS_WRITE_ARGS_4(JM) PINOCCHIO_DETAILS_WRITE_ARGS_3(JM), typename boost::fusion::result_of::at_c<ArgsType, 3>::type a3
#define PINOCCHIO_DETAILS_WRITE_ARGS_5(JM) PINOCCHIO_DETAILS_WRITE_ARGS_4(JM), typename boost::fusion::result_of::at_c<ArgsType, 4>::type a4
    
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

#define PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_5(Algo)                 \
  template <typename Visitor, typename JointCollection>                    \
  struct Algo <Visitor, JointModelCompositeTpl<JointCollection> > {        \
    typedef typename Visitor::ArgsType ArgsType;                  \
    static void run (PINOCCHIO_DETAILS_WRITE_ARGS_5(JointModelCompositeTpl<JointCollection>))         \
    { ::pinocchio::details::Dispatch< Visitor >::run(jmodel.derived(), ArgsType(a0,a1,a2,a3,a4)); } \
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

#define PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_5(Algo, Visitor)                      \
    typedef LieGroup_t LieGroupMap;                                              \
    template<typename JointModel>                                              \
    static void algo(PINOCCHIO_DETAILS_WRITE_ARGS_5(JointModel))                     \
    { AlgoDispatch<JointModel>::run(jmodel, a0, a1, a2, a3, a4); }         \
    template<typename JointModel>                            \
    struct AlgoDispatch : Algo<Visitor, JointModel>                            \
    { using Algo<Visitor, JointModel>::run; };
    
  } // namespace details
  
  template<typename Visitor, typename JointModel> struct IntegrateStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorIn, typename TangentVectorIn, typename ConfigVectorOut>
  struct IntegrateStep
  : public fusion::JointUnaryVisitorBase< IntegrateStep<LieGroup_t,ConfigVectorIn,TangentVectorIn,ConfigVectorOut> >
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
  : public fusion::JointUnaryVisitorBase< dIntegrateStep<LieGroup_t,ConfigVectorIn,TangentVectorIn,JacobianMatrixType> >
  {
    typedef boost::fusion::vector<const ConfigVectorIn &,
                                  const TangentVectorIn &,
                                  JacobianMatrixType &,
                                  const ArgumentPosition &,
                                  const AssignmentOperatorType&
                                  > ArgsType;
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_5(dIntegrateStepAlgo, dIntegrateStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct dIntegrateStepAlgo
  {
    template<typename ConfigVectorIn, typename TangentVector, typename JacobianMatrixType>
    static void run(const JointModelBase<JointModel> & jmodel,
                    const Eigen::MatrixBase<ConfigVectorIn> & q,
                    const Eigen::MatrixBase<TangentVector> & v,
                    const Eigen::MatrixBase<JacobianMatrixType> & mat,
                    const ArgumentPosition & arg,
                    const AssignmentOperatorType& op)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      
      typename LieGroupMap::template operation<JointModel>::type lgo;
      lgo.dIntegrate(jmodel.jointConfigSelector  (q.derived()),
                     jmodel.jointVelocitySelector(v.derived()),
                     jmodel.jointBlock(PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrixType,mat)),
                     arg, op);
    }
  };
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_5(dIntegrateStepAlgo);

  template<typename Visitor, typename JointModel> struct dIntegrateTransportStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorIn, typename TangentVectorIn, typename JacobianMatrixInType, typename JacobianMatrixOutType>
  struct dIntegrateTransportStep
  : public fusion::JointUnaryVisitorBase< dIntegrateTransportStep<LieGroup_t,ConfigVectorIn,TangentVectorIn,JacobianMatrixInType,JacobianMatrixOutType> >
  {
    typedef boost::fusion::vector<const ConfigVectorIn &,
                                  const TangentVectorIn &,
                                  const JacobianMatrixInType &,
                                  JacobianMatrixOutType &,
                                  const ArgumentPosition &
                                  > ArgsType;
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_5(dIntegrateTransportStepAlgo, dIntegrateTransportStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct dIntegrateTransportStepAlgo
  {
    template<typename ConfigVectorIn, typename TangentVector, typename JacobianMatrixInType, typename JacobianMatrixOutType>
    static void run(const JointModelBase<JointModel> & jmodel,
                    const Eigen::MatrixBase<ConfigVectorIn> & q,
                    const Eigen::MatrixBase<TangentVector> & v,
                    const Eigen::MatrixBase<JacobianMatrixInType> & mat_in,
                    const Eigen::MatrixBase<JacobianMatrixOutType> & mat_out,
                    const ArgumentPosition & arg)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      
      typename LieGroupMap::template operation<JointModel>::type lgo;
      lgo.dIntegrateTransport(jmodel.jointConfigSelector  (q.derived()),
                              jmodel.jointVelocitySelector(v.derived()),
                              jmodel.jointRows(mat_in.derived()),
                              jmodel.jointRows(PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrixOutType,mat_out)),
                              arg);
    }
  };
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_5(dIntegrateTransportStepAlgo);


  template<typename Visitor, typename JointModel> struct dIntegrateTransportInPlaceStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorIn, typename TangentVectorIn, typename JacobianMatrixType>
  struct dIntegrateTransportInPlaceStep
  : public fusion::JointUnaryVisitorBase< dIntegrateTransportInPlaceStep<LieGroup_t,ConfigVectorIn,TangentVectorIn,JacobianMatrixType> >
  {
    typedef boost::fusion::vector<const ConfigVectorIn &,
                                  const TangentVectorIn &,
                                  JacobianMatrixType &,
                                  const ArgumentPosition &
                                  > ArgsType;
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_4(dIntegrateTransportInPlaceStepAlgo,
                                            dIntegrateTransportInPlaceStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct dIntegrateTransportInPlaceStepAlgo
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
      lgo.dIntegrateTransport(jmodel.jointConfigSelector  (q.derived()),
                              jmodel.jointVelocitySelector(v.derived()),
                              jmodel.jointRows(PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrixType,mat)),
                              arg);
    }
  };
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_4(dIntegrateTransportInPlaceStepAlgo);
  
  template<typename Visitor, typename JointModel> struct dDifferenceStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVector1, typename ConfigVector2, typename JacobianMatrix>
  struct dDifferenceStep
  : public fusion::JointUnaryVisitorBase< dDifferenceStep<LieGroup_t,ConfigVector1,ConfigVector2,JacobianMatrix> >
  {
    typedef boost::fusion::vector<const ConfigVector1 &,
                                  const ConfigVector2 &,
                                  JacobianMatrix &,
                                  const ArgumentPosition &
                                  > ArgsType;
    
    PINOCCHIO_DETAILS_VISITOR_METHOD_ALGO_4(dDifferenceStepAlgo, dDifferenceStep)
  };
  
  template<typename Visitor, typename JointModel>
  struct dDifferenceStepAlgo
  {
    template<typename ConfigVector1, typename ConfigVector2, typename JacobianMatrix>
    static void run(const JointModelBase<JointModel> & jmodel,
                    const Eigen::MatrixBase<ConfigVector1> & q0,
                    const Eigen::MatrixBase<ConfigVector2> & q1,
                    const Eigen::MatrixBase<JacobianMatrix> & mat,
                    const ArgumentPosition & arg)
    {
      typedef typename Visitor::LieGroupMap LieGroupMap;
      
      typename LieGroupMap::template operation<JointModel>::type lgo;
      lgo.dDifference(jmodel.jointConfigSelector(q0.derived()),
                      jmodel.jointConfigSelector(q1.derived()),
                      jmodel.jointBlock(PINOCCHIO_EIGEN_CONST_CAST(JacobianMatrix,mat)),
                      arg);
    }
  };
  
  PINOCCHIO_DETAILS_DISPATCH_JOINT_COMPOSITE_4(dDifferenceStepAlgo);
  
  template<typename Visitor, typename JointModel> struct InterpolateStepAlgo;
  
  template<typename LieGroup_t, typename ConfigVectorIn1, typename ConfigVectorIn2, typename Scalar, typename ConfigVectorOut>
  struct InterpolateStep
  : public fusion::JointUnaryVisitorBase< InterpolateStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,Scalar,ConfigVectorOut  > >
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
  : public fusion::JointUnaryVisitorBase< DifferenceStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,TangentVectorOut> >
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
  : public fusion::JointUnaryVisitorBase<SquaredDistanceStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,DistanceVectorOut> >
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
  : public fusion::JointUnaryVisitorBase<SquaredDistanceSumStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,Scalar> >
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
  : public fusion::JointUnaryVisitorBase< RandomConfigurationStep<LieGroup_t,ConfigVectorOut,ConfigVectorIn1,ConfigVectorIn2> >
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
  : public fusion::JointUnaryVisitorBase< NormalizeStep<LieGroup_t,ConfigVectorType> >
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
  : public fusion::JointUnaryVisitorBase< IsSameConfigurationStep<LieGroup_t,ConfigVectorIn1,ConfigVectorIn2,Scalar> >
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
  : public fusion::JointUnaryVisitorBase< NeutralStep<LieGroup_t,ConfigVectorType> >
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
  : public fusion::JointUnaryVisitorBase< IntegrateCoeffWiseJacobianStep<LieGroup_t,ConfigVectorType,JacobianMatrix> >
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

#endif // ifndef __pinocchio_multibody_liegroup_liegroup_algo_hxx__
