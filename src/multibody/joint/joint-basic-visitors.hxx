//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_joint_basic_visitors_hxx__
#define __pinocchio_joint_basic_visitors_hxx__

#include "pinocchio/multibody/joint/joint-basic-visitors.hpp"
#include "pinocchio/multibody/visitor.hpp"

namespace pinocchio
{
  /// @cond DEV
  
  /**
   * @brief      CreateJointData visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct CreateJointData
  : boost::static_visitor< JointDataTpl<Scalar,Options,JointCollectionTpl> >
  {
    typedef JointCollectionTpl<Scalar,Options> JointCollection;
    typedef typename JointCollection::JointModelVariant JointModelVariant;
    typedef JointDataTpl<Scalar,Options,JointCollectionTpl> JointDataVariant;
    
    template<typename JointModelDerived>
    JointDataVariant operator()(const JointModelBase<JointModelDerived> & jmodel) const
    { return JointDataVariant(jmodel.createData()); }
    
    static JointDataVariant run(const JointModelVariant & jmodel)
    { return boost::apply_visitor(CreateJointData(), jmodel); }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline JointDataTpl<Scalar,Options,JointCollectionTpl>
  createData(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
  {
    return CreateJointData<Scalar,Options,JointCollectionTpl>::run(jmodel);
  }

  /**
   * @brief      JointCalcZeroOrderVisitor fusion visitor
   */
  template<typename ConfigVectorType>
  struct JointCalcZeroOrderVisitor
  : fusion::JointVisitorBase< JointCalcZeroOrderVisitor<ConfigVectorType> >
  {
    typedef boost::fusion::vector<const ConfigVectorType &> ArgsType;

    template<typename JointModel>
    static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                     pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      jmodel.calc(jdata.derived(),q.derived());
    }

  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl, typename ConfigVectorType>
  inline void calc_zero_order(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                              JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata,
                              const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    typedef JointCalcZeroOrderVisitor<ConfigVectorType> Algo;
    
    Algo::run(jmodel, jdata,
              typename Algo::ArgsType(q.derived()));
  }

  /**
   * @brief      JointCalcFirstOrderVisitor fusion visitor
   */
  template<typename ConfigVectorType, typename TangentVectorType>
  struct JointCalcFirstOrderVisitor
  : fusion::JointVisitorBase< JointCalcFirstOrderVisitor<ConfigVectorType,TangentVectorType> >
  {
    typedef boost::fusion::vector<const ConfigVectorType &,
                                  const TangentVectorType &> ArgsType;

    template<typename JointModel>
    static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                     pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Eigen::MatrixBase<ConfigVectorType> & q,
                     const Eigen::MatrixBase<TangentVectorType> & v
                     )
    {
      jmodel.calc(jdata.derived(),q.derived(),v.derived());
    }

  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl,typename ConfigVectorType, typename TangentVectorType>
  inline void calc_first_order(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                               JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata,
                               const Eigen::MatrixBase<ConfigVectorType> & q,
                               const Eigen::MatrixBase<TangentVectorType> & v)
  {
    typedef JointCalcFirstOrderVisitor<ConfigVectorType,TangentVectorType> Algo;
    
    Algo::run(jmodel, jdata, typename Algo::ArgsType(q.derived(),v.derived()));
  }


  /**
   * @brief      JointCalcAbaVisitor fusion visitor
   */
  
  template<typename Matrix6Type>
  struct JointCalcAbaVisitor
  : fusion::JointVisitorBase< JointCalcAbaVisitor<Matrix6Type> >
  {
    
    typedef boost::fusion::vector<Matrix6Type &,
                                  const bool &> ArgsType;

    template<typename JointModel>
    static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                     pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Eigen::MatrixBase<Matrix6Type> & I,
                     const bool & update_I
                     )
    {
      Matrix6Type & I_ = const_cast<Matrix6Type &>(I.derived());
      jmodel.calc_aba(jdata.derived(),I_,update_I);
    }

  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl, typename Matrix6Type>
  inline void calc_aba(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                       JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata,
                       const Eigen::MatrixBase<Matrix6Type> & I,
                       const bool update_I)
  {
    typedef JointCalcAbaVisitor<Matrix6Type> Algo;
    
    Matrix6Type & I_ = const_cast<Matrix6Type &>(I.derived());
    Algo::run(jmodel, jdata, typename Algo::ArgsType(I_, update_I) );
  }
  
  template<typename OutScalar>
  struct JointEpsVisitor
  : boost::static_visitor<OutScalar>
  {
    
    template<typename JointModelDerived>
    OutScalar operator()(const JointModelBase<JointModelDerived> & jmodel) const
    { return jmodel.finiteDifferenceIncrement(); }
    
    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static OutScalar run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
    { return boost::apply_visitor(JointEpsVisitor(),jmodel); }
    
  }; // struct JointEpsVisitor
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline Scalar
  finiteDifferenceIncrement(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
  {
    return JointEpsVisitor<Scalar>::run(jmodel);
  }

  /**
   * @brief      JointNvVisitor visitor
   */
  struct JointNvVisitor
  : boost::static_visitor<int>
  {
    template<typename JointModelDerived>
    int operator()(const JointModelBase<JointModelDerived> & jmodel) const
    { return jmodel.nv(); }
    
    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static int run( const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
    { return boost::apply_visitor(JointNvVisitor(),jmodel); }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline int nv(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
  { return JointNvVisitor::run(jmodel); }


  /**
   * @brief      JointNqVisitor visitor
   */
  struct JointNqVisitor
  : boost::static_visitor<int>
  {
    template<typename JointModelDerived>
    int operator()(const JointModelBase<JointModelDerived> & jmodel) const
    { return jmodel.nq(); }
    
    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static int run( const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
    { return boost::apply_visitor(JointNqVisitor(),jmodel); }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline int nq(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
  { return JointNqVisitor::run(jmodel); }

  /**
   * @brief      JointIdxQVisitor visitor
   */
  struct JointIdxQVisitor
  : boost::static_visitor<int>
  {
    template<typename JointModelDerived>
    int operator()(const JointModelBase<JointModelDerived> & jmodel) const
    { return jmodel.idx_q(); }
    
    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static int run( const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
    { return boost::apply_visitor(JointIdxQVisitor(),jmodel); }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline int idx_q(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
  { return JointIdxQVisitor::run(jmodel); }

  /**
   * @brief      JointIdxVVisitor visitor
   */
  struct JointIdxVVisitor
  : boost::static_visitor<int>
  {
    template<typename JointModelDerived>
    int operator()(const JointModelBase<JointModelDerived> & jmodel) const
    { return jmodel.idx_v(); }
    
    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static int run( const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
    { return boost::apply_visitor(JointIdxVVisitor(),jmodel); }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline int idx_v(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel) { return JointIdxVVisitor::run(jmodel); }

  /**
   * @brief      JointIdVisitor visitor
   */
  struct JointIdVisitor
  : boost::static_visitor<JointIndex>
  {
    template<typename JointModelDerived>
    JointIndex operator()(const JointModelBase<JointModelDerived> & jmodel) const
    { return jmodel.id(); }
    
    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static JointIndex run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
    { return boost::apply_visitor(JointIdVisitor(),jmodel); }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline JointIndex id(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel) { return JointIdVisitor::run(jmodel); }

  /**
   * @brief      JointSetIndexesVisitor visitor
   */
  struct JointSetIndexesVisitor
  : boost::static_visitor<>
  {
    JointIndex id;
    int q;
    int v;

    JointSetIndexesVisitor(JointIndex id,int q,int v): id(id),q(q),v(v) {}

    template<typename JointModelDerived>
    void operator()(JointModelBase<JointModelDerived> & jmodel) const
    { jmodel.setIndexes(id, q, v); }
    
    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static void run(JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel, JointIndex id, int q, int v)
    { return boost::apply_visitor(JointSetIndexesVisitor(id, q, v),jmodel); }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline void setIndexes(JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel,
                         JointIndex id, int q,int v)
  { return JointSetIndexesVisitor::run(jmodel, id, q, v); }


  /**
   * @brief      JointShortnameVisitor visitor
   */
  struct JointShortnameVisitor
  : boost::static_visitor<std::string>
  {
    template<typename JointModelDerived>
    std::string operator()(const JointModelBase<JointModelDerived> & jmodel) const
    { return jmodel.shortname(); }
    
    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static std::string run(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
    { return boost::apply_visitor(JointShortnameVisitor(),jmodel); }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline std::string shortname(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
  { return JointShortnameVisitor::run(jmodel);}
  
  template<typename NewScalar, typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointCastVisitor
  : fusion::JointVisitorBase< JointCastVisitor<NewScalar,Scalar,Options,JointCollectionTpl>, typename CastType< NewScalar,JointModelTpl<Scalar,Options,JointCollectionTpl> >::type >
  {
    typedef fusion::NoArg ArgsType;
    
    typedef typename CastType< NewScalar,JointModelTpl<Scalar,Options,JointCollectionTpl> >::type ReturnType;
    
    template<typename JointModelDerived>
    static ReturnType algo(const JointModelBase<JointModelDerived> & jmodel)
    { return ReturnType(jmodel.template cast<NewScalar>()); }

  };
  
  template<typename NewScalar, typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  typename CastType< NewScalar,JointModelTpl<Scalar,Options,JointCollectionTpl> >::type
  cast_joint(const JointModelTpl<Scalar,Options,JointCollectionTpl> & jmodel)
  {
    typedef JointCastVisitor<NewScalar,Scalar,Options,JointCollectionTpl> Algo;
    return Algo::run(jmodel);
  }

  //
  // Visitors on JointDatas
  //
  
  /**
   * @brief      JointConstraintVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointConstraintVisitor
  : boost::static_visitor< ConstraintTpl<Eigen::Dynamic,Scalar,Options> >
  {
    typedef ConstraintTpl<Eigen::Dynamic,Scalar,Options> ReturnType;
    
    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.S().matrix());
    }
    
    static ReturnType run(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointConstraintVisitor(), jdata);
    }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline ConstraintTpl<Eigen::Dynamic,Scalar,Options>
  constraint_xd(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
  {
    return JointConstraintVisitor<Scalar,Options,JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointTransformVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointTransformVisitor
  : boost::static_visitor< SE3Tpl<Scalar,Options> >
  {
    typedef SE3Tpl<Scalar,Options> ReturnType;
    
    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.M());
    }
    
    static ReturnType run(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointTransformVisitor (), jdata);
    }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline SE3Tpl<Scalar,Options>
  joint_transform(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
  {
    return JointTransformVisitor<Scalar,Options,JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointMotionVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointMotionVisitor
  : boost::static_visitor< MotionTpl<Scalar,Options> >
  {
    typedef MotionTpl<Scalar,Options> ReturnType;
    
    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.v());
    }
    
    static ReturnType run(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointMotionVisitor(),jdata);
    }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline MotionTpl<Scalar,Options>
  motion(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
  {
    return JointMotionVisitor<Scalar,Options,JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointBiasVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointBiasVisitor
  : boost::static_visitor< MotionTpl<Scalar,Options> >
  {
    typedef MotionTpl<Scalar,Options> ReturnType;
    
    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.c());
    }
    
    static ReturnType run(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointBiasVisitor(), jdata);
    }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline MotionTpl<Scalar,Options>
  bias(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
  {
    return JointBiasVisitor<Scalar,Options,JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointUInertiaVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointUInertiaVisitor
  : boost::static_visitor< Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> >
  {
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> ReturnType;
    
    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.U());
    }
    
    static ReturnType run(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointUInertiaVisitor(), jdata);
    }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options>
  u_inertia(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
  {
    return JointUInertiaVisitor<Scalar,Options,JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointDInvInertiaVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointDInvInertiaVisitor
  : boost::static_visitor< Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> >
  {
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> ReturnType;
    
    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.Dinv());
    }
    
    static ReturnType run(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointDInvInertiaVisitor(), jdata);
    }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options>
  dinv_inertia(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
  {
    return JointDInvInertiaVisitor<Scalar,Options,JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointUDInvInertiaVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointUDInvInertiaVisitor
  : boost::static_visitor< Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> >
  {
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> ReturnType;
    
    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.UDinv());
    }
    
    static ReturnType run(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointUDInvInertiaVisitor(),jdata);
      
    }
  };
  
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options>
  udinv_inertia(const JointDataTpl<Scalar,Options,JointCollectionTpl> & jdata)
  {
    return JointUDInvInertiaVisitor<Scalar,Options,JointCollectionTpl>::run(jdata);
  }

  /// @endcond

} // namespace pinocchio

#endif // ifndef __pinocchio_joint_basic_visitors_hxx__
