//
// Copyright (c) 2016 CNRS
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
#include "pinocchio/multibody/joint/joint-composite.hpp"
#include "pinocchio/multibody/visitor.hpp"

namespace se3
{
  /// @cond DEV
  
  /**
   * @brief      CreateJointData visitor
   */
  class CreateJointData: public boost::static_visitor<JointDataVariant>
  {
  public:
    template<typename D>
    JointDataVariant operator()(const JointModelBase<D> & jmodel) const
    { return JointDataVariant(jmodel.createData()); }
    
    static JointDataVariant run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( CreateJointData(), jmodel ); }
  };
  inline JointDataVariant createData(const JointModelVariant & jmodel)
  {
    return CreateJointData::run(jmodel);
  }

  /**
   * @brief      JointCalcZeroOrderVisitor fusion visitor
   */
  
  struct JointCalcZeroOrderVisitor : public fusion::JointVisitor<JointCalcZeroOrderVisitor>
  {
    typedef boost::fusion::vector< const Eigen::VectorXd & > ArgsType;

    JOINT_VISITOR_INIT(JointCalcZeroOrderVisitor);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Eigen::VectorXd & q
                     )
    {
      jmodel.calc(jdata.derived(),q);
    }

  };
  inline void calc_zero_order(const JointModelVariant & jmodel, JointDataVariant & jdata, const Eigen::VectorXd & q)
  {
    JointCalcZeroOrderVisitor::run( jmodel, jdata, JointCalcZeroOrderVisitor::ArgsType(q) );
  }

  /**
   * @brief      JointCalcFirstOrderVisitor fusion visitor
   */
  
  struct JointCalcFirstOrderVisitor : public fusion::JointVisitor<JointCalcFirstOrderVisitor>
  {
    typedef boost::fusion::vector< const Eigen::VectorXd &,
                                    const Eigen::VectorXd & > ArgsType;

    JOINT_VISITOR_INIT(JointCalcFirstOrderVisitor);

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
  inline void calc_first_order(const JointModelVariant & jmodel, JointDataVariant & jdata, const Eigen::VectorXd & q, const Eigen::VectorXd & v)
  {
    JointCalcFirstOrderVisitor::run( jmodel, jdata, JointCalcFirstOrderVisitor::ArgsType(q,v) );
  }


  /**
   * @brief      JointCalcAbaVisitor fusion visitor
   */
  
  struct JointCalcAbaVisitor : public fusion::JointVisitor<JointCalcAbaVisitor>
  {
    typedef boost::fusion::vector< Inertia::Matrix6 &,
                                    const bool > ArgsType;

    JOINT_VISITOR_INIT(JointCalcAbaVisitor);

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
  inline void calc_aba(const JointModelVariant & jmodel, JointDataVariant & jdata, Inertia::Matrix6 & I, const bool update_I)
  {
    JointCalcAbaVisitor::run( jmodel, jdata, JointCalcAbaVisitor::ArgsType(I, update_I) );
  }
  
  struct JointEpsVisitor: public boost::static_visitor<double>
  {
  public:
    
    template<typename D>
    double operator()(const JointModelBase<D> & jmodel) const
    { return jmodel.finiteDifferenceIncrement(); }
    
    static double run(const JointModelVariant & jmodel)
    { return boost::apply_visitor(JointEpsVisitor(),jmodel); }
  }; // struct JointEpsVisitor
  
  inline double finiteDifferenceIncrement(const JointModelVariant & jmodel)
  { return JointEpsVisitor::run(jmodel); }

  /**
   * @brief      JointNvVisitor visitor
   */
  class JointNvVisitor: public boost::static_visitor<int>
  {
  public:
    template<typename D>
    int operator()(const JointModelBase<D> & jmodel) const
    { return jmodel.nv(); }
    
    static int run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( JointNvVisitor(), jmodel ); }
  };
  inline int nv(const JointModelVariant & jmodel) { return JointNvVisitor::run(jmodel); }


  /**
   * @brief      JointNqVisitor visitor
   */
  class JointNqVisitor: public boost::static_visitor<int>
  {
  public:
    template<typename D>
    int operator()(const JointModelBase<D> & jmodel) const
    { return jmodel.nq(); }
    
    static int run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( JointNqVisitor(), jmodel ); }
  };
  inline int nq(const JointModelVariant & jmodel) { return JointNqVisitor::run(jmodel); }

  /**
   * @brief      JointIdxQVisitor visitor
   */
  class JointIdxQVisitor: public boost::static_visitor<int>
  {
  public:
    template<typename D>
    int operator()(const JointModelBase<D> & jmodel) const
    { return jmodel.idx_q(); }
    
    static int run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( JointIdxQVisitor(), jmodel ); }
  };
  inline int idx_q(const JointModelVariant & jmodel) { return JointIdxQVisitor::run(jmodel); }

  /**
   * @brief      JointIdxVVisitor visitor
   */
  class JointIdxVVisitor: public boost::static_visitor<int>
  {
  public:
    template<typename D>
    int operator()(const JointModelBase<D> & jmodel) const
    { return jmodel.idx_v(); }
    
    static int run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( JointIdxVVisitor(), jmodel ); }
  };
  inline int idx_v(const JointModelVariant & jmodel) { return JointIdxVVisitor::run(jmodel); }

  /**
   * @brief      JointIdVisitor visitor
   */
  class JointIdVisitor: public boost::static_visitor<JointIndex>
  {
  public:
    template<typename D>
    JointIndex operator()(const JointModelBase<D> & jmodel) const
    { return jmodel.id(); }
    
    static JointIndex run( const JointModelVariant & jmodel)
    { return boost::apply_visitor( JointIdVisitor(), jmodel ); }
  };
  inline JointIndex id(const JointModelVariant & jmodel) { return JointIdVisitor::run(jmodel); }

  /**
   * @brief      JointSetIndexesVisitor visitor
   */
  class JointSetIndexesVisitor: public boost::static_visitor<>
  {
  public:

    JointIndex id;
    int q;
    int v;

    JointSetIndexesVisitor(JointIndex id,int q,int v) : id(id),q(q),v(v) {}

    template<typename D>
    void operator()(JointModelBase<D> & jmodel) const
    { jmodel.setIndexes(id, q, v); }
    
    static void run(JointModelVariant & jmodel, JointIndex id, int q, int v)
    { return boost::apply_visitor( JointSetIndexesVisitor(id, q, v), jmodel ); }
  };
  inline void setIndexes(JointModelVariant & jmodel, JointIndex id, int q,int v) { return JointSetIndexesVisitor::run(jmodel, id, q, v); }


  /**
   * @brief      JointShortnameVisitor visitor
   */
  class JointShortnameVisitor: public boost::static_visitor<std::string>
  {
  public:

    template<typename D>
    std::string operator()(const JointModelBase<D> & jmodel) const
    { return jmodel.shortname(); }
    
    static std::string run(const JointModelVariant & jmodel)
    { return boost::apply_visitor( JointShortnameVisitor(), jmodel ); }
  };
  inline std::string shortname(const JointModelVariant & jmodel) { return JointShortnameVisitor::run(jmodel);}

  //
  // Visitors on JointDatas
  //
  
  /**
   * @brief      JointConstraintVisitor visitor
   */
  class JointConstraintVisitor: public boost::static_visitor< ConstraintXd >
  {
  public:
    template <typename D>
    ConstraintXd operator()(const JointDataBase<D> & jdata) const
    {
      return ConstraintXd(jdata.S().matrix()); }
    
    static ConstraintXd run( const JointDataVariant & jdata)
    { return boost::apply_visitor( JointConstraintVisitor (), jdata ); }
  };
  inline ConstraintXd constraint_xd(const JointDataVariant & jdata) { return JointConstraintVisitor::run(jdata); }

  /**
   * @brief      JointTransformVisitor visitor
   */
  class JointTransformVisitor: public boost::static_visitor< SE3 >
  {
  public:
    template <typename D>
    SE3 operator()(const JointDataBase<D> & jdata) const
    { return SE3(jdata.M()); }
    
    static SE3 run( const JointDataVariant & jdata)
    { return boost::apply_visitor( JointTransformVisitor (), jdata ); }
  };
  inline SE3 joint_transform(const JointDataVariant & jdata) { return JointTransformVisitor::run(jdata); }

  /**
   * @brief      JointMotionVisitor visitor
   */
  class JointMotionVisitor: public boost::static_visitor< Motion >
  {
  public:
    template <typename D>
    Motion operator()(const JointDataBase<D> & jdata) const
    { return Motion(jdata.v()); }
    
    static Motion run( const JointDataVariant & jdata)
    { return boost::apply_visitor( JointMotionVisitor (), jdata ); }
  };
  inline Motion motion(const JointDataVariant & jdata) { return JointMotionVisitor::run(jdata); }

  /**
   * @brief      JointBiasVisitor visitor
   */
  class JointBiasVisitor: public boost::static_visitor< Motion >
  {
  public:
    template <typename D>
    Motion operator()(const JointDataBase<D> & jdata) const
    { return Motion(jdata.c()); }
    
    static Motion run( const JointDataVariant & jdata)
    { return boost::apply_visitor( JointBiasVisitor (), jdata ); }
  };
  inline Motion bias(const JointDataVariant & jdata) { return JointBiasVisitor::run(jdata); }




  /**
   * @brief      JointUInertiaVisitor visitor
   */
   // Matrix6X typedefed in Data. Data not included here
  class JointUInertiaVisitor: public boost::static_visitor< Eigen::Matrix<double,6,Eigen::Dynamic> >
  {
  public:
    template <typename D>
    Eigen::Matrix<double,6,Eigen::Dynamic> operator()(const JointDataBase<D> & jdata) const
    { return Eigen::Matrix<double,6,Eigen::Dynamic>(jdata.U()); }
    
    static Eigen::Matrix<double,6,Eigen::Dynamic> run( const JointDataVariant & jdata)
    { return boost::apply_visitor( JointUInertiaVisitor (), jdata ); }
  };
  inline Eigen::Matrix<double,6,Eigen::Dynamic> u_inertia(const JointDataVariant & jdata) { return JointUInertiaVisitor::run(jdata); }

  /**
   * @brief      JointDInvInertiaVisitor visitor
   */
  class JointDInvInertiaVisitor: public boost::static_visitor< Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> >
  {
  public:
    template <typename D>
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> operator()(const JointDataBase<D> & jdata) const
    { return Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>(jdata.Dinv()); }
    
    static Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> run( const JointDataVariant & jdata)
    { return boost::apply_visitor( JointDInvInertiaVisitor (), jdata ); }
  };
  inline Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> dinv_inertia(const JointDataVariant & jdata) { return JointDInvInertiaVisitor::run(jdata); }

  /**
   * @brief      JointUDInvInertiaVisitor visitor
   */
   // Matrix6X typedefed in Data. Data not included here
  class JointUDInvInertiaVisitor: public boost::static_visitor< Eigen::Matrix<double,6,Eigen::Dynamic> >
  {
  public:
    template <typename D>
    Eigen::Matrix<double,6,Eigen::Dynamic> operator()(const JointDataBase<D> & jdata) const
    { return Eigen::Matrix<double,6,Eigen::Dynamic>(jdata.UDinv()); }
    
    static Eigen::Matrix<double,6,Eigen::Dynamic> run( const JointDataVariant & jdata)
    { return boost::apply_visitor( JointUDInvInertiaVisitor (), jdata ); }
  };
  inline Eigen::Matrix<double,6,Eigen::Dynamic> udinv_inertia(const JointDataVariant & jdata) { return JointUDInvInertiaVisitor::run(jdata); }

  /// @endcond

} // namespace se3

#endif // ifndef __se3_joint_basic_visitors_hxx__
