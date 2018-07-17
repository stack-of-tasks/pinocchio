//
// Copyright (c) 2016,2018 CNRS
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

#ifndef __se3_joint_basic_visitors_hxx__
#define __se3_joint_basic_visitors_hxx__

#include "pinocchio/assert.hpp"
#include "pinocchio/multibody/joint/joint-basic-visitors.hpp"
//#include "pinocchio/multibody/joint/joint-composite.hpp"
#include "pinocchio/multibody/visitor.hpp"

namespace se3
{
  /// @cond DEV
  
  /**
   * @brief      CreateJointData visitor
   */
  template<typename JointCollection>
  struct CreateJointData
  : boost::static_visitor<typename JointCollection::JointDataVariant>
  {
    typedef typename JointCollection::JointModelVariant JointModelVariant;
    typedef typename JointCollection::JointDataVariant JointDataVariant;
    
    template<typename D>
    JointDataVariant operator()(const JointModelBase<D> & jmodel) const
    { return JointDataVariant(jmodel.createData()); }
    
    static JointDataVariant run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( CreateJointData(), jmodel ); }
  };
  
  template<typename JointCollection>
  inline typename JointCollection::JointDataVariant
  createData(const typename JointCollection::JointModelVariant & jmodel)
  {
    return CreateJointData<JointCollection>::run(jmodel);
  }

  /**
   * @brief      JointCalcZeroOrderVisitor fusion visitor
   */
  
  struct JointCalcZeroOrderVisitor
  : fusion::JointVisitorBase<JointCalcZeroOrderVisitor>
  {
    typedef boost::fusion::vector< const Eigen::VectorXd & > ArgsType;

    template<typename JointModel, typename ConfigVector>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Eigen::MatrixBase<ConfigVector> & q)
    {
      jmodel.calc(jdata.derived(),q);
    }

  };
  
  template<typename JointCollection>
  inline void calc_zero_order(const JointModelTpl<JointCollection> & jmodel,
                              JointDataTpl<JointCollection> & jdata,
                              const Eigen::VectorXd & q)
  {
    JointCalcZeroOrderVisitor::run(jmodel, jdata,
                                   JointCalcZeroOrderVisitor::ArgsType(q));
  }

  /**
   * @brief      JointCalcFirstOrderVisitor fusion visitor
   */
  
  struct JointCalcFirstOrderVisitor
  : fusion::JointVisitorBase<JointCalcFirstOrderVisitor>
  {
    typedef boost::fusion::vector< const Eigen::VectorXd &,
                                    const Eigen::VectorXd & > ArgsType;

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v
                     )
    {
      jmodel.calc(jdata.derived(),q,v);
    }

  };
  
  template<typename JointCollection>
  inline void calc_first_order(const JointModelTpl<JointCollection> & jmodel,
                               JointDataTpl<JointCollection> & jdata,
                               const Eigen::VectorXd & q,
                               const Eigen::VectorXd & v)
  {
    JointCalcFirstOrderVisitor::run( jmodel, jdata, JointCalcFirstOrderVisitor::ArgsType(q,v) );
  }


  /**
   * @brief      JointCalcAbaVisitor fusion visitor
   */
  
  struct JointCalcAbaVisitor
  : fusion::JointVisitorBase<JointCalcAbaVisitor>
  {
    typedef boost::fusion::vector< Inertia::Matrix6 &,
                                    const bool > ArgsType;

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     Inertia::Matrix6 & I,
                     const bool update_I
                     )
    {
      jmodel.calc_aba(jdata.derived(),I,update_I);
    }

  };
  
  template<typename JointCollection>
  inline void calc_aba(const JointModelTpl<JointCollection> & jmodel,
                       JointDataTpl<JointCollection> & jdata,
                       Inertia::Matrix6 & I,
                       const bool update_I)
  {
    JointCalcAbaVisitor::run( jmodel, jdata, JointCalcAbaVisitor::ArgsType(I, update_I) );
  }
  
  template<typename Scalar>
  struct JointEpsVisitor
  : boost::static_visitor<Scalar>
  {
    
    template<typename JointModelDerived>
    Scalar operator()(const JointModelBase<JointModelDerived> & jmodel) const
    { return jmodel.finiteDifferenceIncrement(); }
    
    template<typename JointCollection>
    static Scalar run(const JointModelTpl<JointCollection> & jmodel)
    { return boost::apply_visitor(JointEpsVisitor(),jmodel); }
    
  }; // struct JointEpsVisitor
  
  template<typename JointCollection>
  inline typename JointCollection::Scalar
  finiteDifferenceIncrement(const JointModelTpl<JointCollection> & jmodel)
  {
    return JointEpsVisitor<typename JointCollection::Scalar>::run(jmodel);
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
    
    template<typename JointCollection>
    static int run( const JointModelTpl<JointCollection> & jmodel)
    { return boost::apply_visitor(JointNvVisitor(),jmodel); }
  };
  
  template<typename JointCollection>
  inline int nv(const JointModelTpl<JointCollection> & jmodel)
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
    
    template<typename JointCollection>
    static int run( const JointModelTpl<JointCollection> & jmodel)
    { return boost::apply_visitor(JointNqVisitor(),jmodel); }
  };
  
  template<typename JointCollection>
  inline int nq(const JointModelTpl<JointCollection> & jmodel)
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
    
    template<typename JointCollection>
    static int run( const JointModelTpl<JointCollection> & jmodel)
    { return boost::apply_visitor(JointIdxQVisitor(),jmodel); }
  };
  
  template<typename JointCollection>
  inline int idx_q(const JointModelTpl<JointCollection> & jmodel)
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
    
    template<typename JointCollection>
    static int run( const JointModelTpl<JointCollection> & jmodel)
    { return boost::apply_visitor(JointIdxVVisitor(),jmodel); }
  };
  
  template<typename JointCollection>
  inline int idx_v(const JointModelTpl<JointCollection> & jmodel) { return JointIdxVVisitor::run(jmodel); }

  /**
   * @brief      JointIdVisitor visitor
   */
  struct JointIdVisitor
  : boost::static_visitor<JointIndex>
  {
    template<typename JointModelDerived>
    JointIndex operator()(const JointModelBase<JointModelDerived> & jmodel) const
    { return jmodel.id(); }
    
    template<typename JointCollection>
    static JointIndex run(const JointModelTpl<JointCollection> & jmodel)
    { return boost::apply_visitor(JointIdVisitor(),jmodel); }
  };
  
  template<typename JointCollection>
  inline JointIndex id(const JointModelTpl<JointCollection> & jmodel) { return JointIdVisitor::run(jmodel); }

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
    
    template<typename JointCollection>
    static void run(JointModelTpl<JointCollection> & jmodel, JointIndex id, int q, int v)
    { return boost::apply_visitor(JointSetIndexesVisitor(id, q, v),jmodel); }
  };
  
  template<typename JointCollection>
  inline void setIndexes(JointModelTpl<JointCollection> & jmodel,
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
    
    template<typename JointCollection>
    static std::string run(const JointModelTpl<JointCollection> & jmodel)
    { return boost::apply_visitor(JointShortnameVisitor(),jmodel); }
  };
  
  template<typename JointCollection>
  inline std::string shortname(const JointModelTpl<JointCollection> & jmodel)
  { return JointShortnameVisitor::run(jmodel);}

  //
  // Visitors on JointDatas
  //
  
  /**
   * @brief      JointConstraintVisitor visitor
   */
  template<typename JointCollection>
  struct JointConstraintVisitor
  : boost::static_visitor< ConstraintTpl<Eigen::Dynamic,typename JointCollection::Scalar, JointCollection::Options> >
  {
    typedef ConstraintTpl<Eigen::Dynamic,typename JointCollection::Scalar, JointCollection::Options> ReturnType;
    
    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.S().matrix());
    }
    
    static ReturnType run(const JointDataTpl<JointCollection> & jdata)
    {
      return boost::apply_visitor(JointConstraintVisitor(), jdata);
    }
  };
  
  template<typename JointCollection>
  inline ConstraintTpl<Eigen::Dynamic,typename JointCollection::Scalar, JointCollection::Options>
  constraint_xd(const JointDataTpl<JointCollection> & jdata)
  {
    return JointConstraintVisitor<JointCollection>::run(jdata);
  }

  /**
   * @brief      JointTransformVisitor visitor
   */
  template<typename JointCollection>
  struct JointTransformVisitor
  : boost::static_visitor< SE3Tpl<typename JointCollection::Scalar,JointCollection::Options> >
  {
    typedef SE3Tpl<typename JointCollection::Scalar,JointCollection::Options> ReturnType;
    
    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.M());
    }
    
    static ReturnType run(const JointDataTpl<JointCollection> & jdata)
    {
      return boost::apply_visitor(JointTransformVisitor (), jdata);
    }
  };
  
  template<typename JointCollection>
  inline SE3Tpl<typename JointCollection::Scalar,JointCollection::Options>
  joint_transform(const JointDataTpl<JointCollection> & jdata)
  {
    return JointTransformVisitor<JointCollection>::run(jdata);
  }

  /**
   * @brief      JointMotionVisitor visitor
   */
  template<typename JointCollection>
  struct JointMotionVisitor
  : boost::static_visitor< MotionTpl<typename JointCollection::Scalar,JointCollection::Options> >
  {
    typedef MotionTpl<typename JointCollection::Scalar,JointCollection::Options> ReturnType;
    
    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.v());
    }
    
    static ReturnType run(const JointDataTpl<JointCollection> & jdata)
    {
      return boost::apply_visitor(JointMotionVisitor(),jdata);
    }
  };
  
  template<typename JointCollection>
  inline MotionTpl<typename JointCollection::Scalar,JointCollection::Options>
  motion(const JointDataTpl<JointCollection> & jdata)
  {
    return JointMotionVisitor<JointCollection>::run(jdata);
  }

  /**
   * @brief      JointBiasVisitor visitor
   */
  template<typename JointCollection>
  struct JointBiasVisitor
  : boost::static_visitor< MotionTpl<typename JointCollection::Scalar,JointCollection::Options> >
  {
    typedef MotionTpl<typename JointCollection::Scalar,JointCollection::Options> ReturnType;
    
    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.c());
    }
    
    static ReturnType run(const JointDataTpl<JointCollection> & jdata)
    {
      return boost::apply_visitor(JointBiasVisitor(), jdata);
    }
  };
  
  template<typename JointCollection>
  inline MotionTpl<typename JointCollection::Scalar,JointCollection::Options>
  bias(const JointDataTpl<JointCollection> & jdata)
  {
    return JointBiasVisitor<JointCollection>::run(jdata);
  }

  /**
   * @brief      JointUInertiaVisitor visitor
   */
  template<typename JointCollection>
  struct JointUInertiaVisitor
  : boost::static_visitor< Eigen::Matrix<typename JointCollection::Scalar,6,Eigen::Dynamic, JointCollection::Options> >
  {
    typedef Eigen::Matrix<typename JointCollection::Scalar,6,Eigen::Dynamic, JointCollection::Options> ReturnType;
    
    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.U());
    }
    
    static ReturnType run(const JointDataTpl<JointCollection> & jdata)
    {
      return boost::apply_visitor(JointUInertiaVisitor(), jdata);
    }
  };
  
  template<typename JointCollection>
  inline Eigen::Matrix<typename JointCollection::Scalar,6,Eigen::Dynamic, JointCollection::Options>
  u_inertia(const JointDataTpl<JointCollection> & jdata)
  {
    return JointUInertiaVisitor<JointCollection>::run(jdata);
  }

  /**
   * @brief      JointDInvInertiaVisitor visitor
   */
  template<typename JointCollection>
  struct JointDInvInertiaVisitor
  : boost::static_visitor< Eigen::Matrix<typename JointCollection::Scalar,Eigen::Dynamic,Eigen::Dynamic,JointCollection::Options> >
  {
    typedef Eigen::Matrix<typename JointCollection::Scalar,Eigen::Dynamic,Eigen::Dynamic,JointCollection::Options> ReturnType;
    
    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.Dinv());
    }
    
    static ReturnType run(const JointDataTpl<JointCollection> & jdata)
    {
      return boost::apply_visitor(JointDInvInertiaVisitor(), jdata);
    }
  };
  
  template<typename JointCollection>
  inline Eigen::Matrix<typename JointCollection::Scalar,Eigen::Dynamic,Eigen::Dynamic,JointCollection::Options>
  dinv_inertia(const JointDataTpl<JointCollection> & jdata)
  {
    return JointDInvInertiaVisitor<JointCollection>::run(jdata);
  }

  /**
   * @brief      JointUDInvInertiaVisitor visitor
   */
   // Matrix6X typedefed in Data. Data not included here
  template<typename JointCollection>
  struct JointUDInvInertiaVisitor
  : boost::static_visitor< Eigen::Matrix<typename JointCollection::Scalar,6,Eigen::Dynamic,JointCollection::Options> >
  {
    typedef Eigen::Matrix<typename JointCollection::Scalar,6,Eigen::Dynamic,JointCollection::Options> ReturnType;
    
    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.UDinv());
    }
    
    static ReturnType run(const JointDataTpl<JointCollection> & jdata)
    {
      return boost::apply_visitor(JointUDInvInertiaVisitor(),jdata);
      
    }
  };
  
  template<typename JointCollection>
  inline Eigen::Matrix<typename JointCollection::Scalar,6,Eigen::Dynamic,JointCollection::Options>
  udinv_inertia(const JointDataTpl<JointCollection> & jdata)
  {
    return JointUDInvInertiaVisitor<JointCollection>::run(jdata);
  }

  /// @endcond

} // namespace se3

#endif // ifndef __se3_joint_basic_visitors_hxx__
