//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_multibody_joint_basic_visitors_hxx__
#define __pinocchio_multibody_joint_basic_visitors_hxx__

#include "pinocchio/multibody/joint/joint-basic-visitors.hpp"
#include "pinocchio/multibody/visitor.hpp"

namespace pinocchio
{
  /// @cond DEV

  /**
   * @brief      CreateJointData visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct CreateJointData : boost::static_visitor<JointDataTpl<Scalar, Options, JointCollectionTpl>>
  {
    typedef JointCollectionTpl<Scalar, Options> JointCollection;
    typedef typename JointCollection::JointModelVariant JointModelVariant;
    typedef JointDataTpl<Scalar, Options, JointCollectionTpl> JointDataVariant;

    template<typename JointModelDerived>
    JointDataVariant operator()(const JointModelBase<JointModelDerived> & jmodel) const
    {
      return JointDataVariant(jmodel.createData());
    }

    static JointDataVariant run(const JointModelVariant & jmodel)
    {
      return boost::apply_visitor(CreateJointData(), jmodel);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline JointDataTpl<Scalar, Options, JointCollectionTpl>
  createData(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
  {
    return CreateJointData<Scalar, Options, JointCollectionTpl>::run(jmodel);
  }

  /**
   * @brief      JointCalcZeroOrderVisitor fusion visitor
   */
  template<typename ConfigVectorType>
  struct JointCalcZeroOrderVisitor
  : fusion::JointUnaryVisitorBase<JointCalcZeroOrderVisitor<ConfigVectorType>>
  {
    typedef boost::fusion::vector<const ConfigVectorType &> ArgsType;

    template<typename JointModel>
    static void algo(
      const pinocchio::JointModelBase<JointModel> & jmodel,
      pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      jmodel.calc(jdata.derived(), q.derived());
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl,
    typename ConfigVectorType>
  inline void calc_zero_order(
    const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel,
    JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata,
    const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    typedef JointCalcZeroOrderVisitor<ConfigVectorType> Algo;

    Algo::run(jmodel, jdata, typename Algo::ArgsType(q.derived()));
  }

  /**
   * @brief      JointCalcFirstOrderVisitor fusion visitor
   */
  template<typename ConfigVectorType, typename TangentVectorType>
  struct JointCalcFirstOrderVisitor
  : fusion::JointUnaryVisitorBase<JointCalcFirstOrderVisitor<ConfigVectorType, TangentVectorType>>
  {
    typedef boost::fusion::vector<const ConfigVectorType &, const TangentVectorType &> ArgsType;

    template<typename JointModel>
    static void algo(
      const pinocchio::JointModelBase<JointModel> & jmodel,
      pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType> & v)
    {
      jmodel.calc(jdata.derived(), q.derived(), v.derived());
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType>
  inline void calc_first_order(
    const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel,
    JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType> & v)
  {
    typedef JointCalcFirstOrderVisitor<ConfigVectorType, TangentVectorType> Algo;

    Algo::run(jmodel, jdata, typename Algo::ArgsType(q.derived(), v.derived()));
  }

  /**
   * @brief      JointCalcFirstOrderVisitor fusion visitor
   */
  template<typename TangentVectorType>
  struct JointCalcFirstOrderVisitor<Blank, TangentVectorType>
  : fusion::JointUnaryVisitorBase<JointCalcFirstOrderVisitor<Blank, TangentVectorType>>
  {
    typedef boost::fusion::vector<const Blank, const TangentVectorType &> ArgsType;

    template<typename JointModel>
    static void algo(
      const pinocchio::JointModelBase<JointModel> & jmodel,
      pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Blank blank,
      const Eigen::MatrixBase<TangentVectorType> & v)
    {
      jmodel.calc(jdata.derived(), blank, v.derived());
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl,
    typename TangentVectorType>
  inline void calc_first_order(
    const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel,
    JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata,
    const Blank blank,
    const Eigen::MatrixBase<TangentVectorType> & v)
  {
    typedef JointCalcFirstOrderVisitor<Blank, TangentVectorType> Algo;

    Algo::run(jmodel, jdata, typename Algo::ArgsType(blank, v.derived()));
  }

  /**
   * @brief      JointCalcAbaVisitor fusion visitor
   */

  template<typename VectorLike, typename Matrix6Type>
  struct JointCalcAbaVisitor
  : fusion::JointUnaryVisitorBase<JointCalcAbaVisitor<VectorLike, Matrix6Type>>
  {

    typedef boost::fusion::vector<const VectorLike &, Matrix6Type &, bool> ArgsType;

    template<typename JointModel>
    static void algo(
      const pinocchio::JointModelBase<JointModel> & jmodel,
      pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Eigen::MatrixBase<VectorLike> & armature,
      const Eigen::MatrixBase<Matrix6Type> & I,
      bool update_I)
    {
      jmodel.calc_aba(
        jdata.derived(), armature.derived(), PINOCCHIO_EIGEN_CONST_CAST(Matrix6Type, I), update_I);
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl,
    typename VectorLike,
    typename Matrix6Type>
  inline void calc_aba(
    const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel,
    JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata,
    const Eigen::MatrixBase<VectorLike> & armature,
    const Eigen::MatrixBase<Matrix6Type> & I,
    const bool update_I)
  {
    typedef JointCalcAbaVisitor<VectorLike, Matrix6Type> Algo;
    Algo::run(
      jmodel, jdata,
      typename Algo::ArgsType(
        PINOCCHIO_EIGEN_CONST_CAST(VectorLike, armature),
        PINOCCHIO_EIGEN_CONST_CAST(Matrix6Type, I), update_I));
  }

  template<typename InputType, typename ReturnType>
  struct JointMappedConfigSelectorVisitor
  : fusion::
      JointUnaryVisitorBase<JointMappedConfigSelectorVisitor<InputType, ReturnType>, ReturnType>
  {
    typedef boost::fusion::vector<InputType> ArgsType;

    template<typename JointModel>
    static ReturnType algo(const JointModelBase<JointModel> & jmodel, InputType a)
    {
      // Converting a VectorBlock of anysize (static or dynamic) to another vector block of anysize
      // (static or dynamic) since there is no copy constructor.
      auto vectorBlock = jmodel.JointMappedConfigSelector(a);

      // VectorBlock does not implemet such getter, hack the Eigen::Block base class to retreive
      // such values.
      const Eigen::DenseIndex start =
        vectorBlock.startRow()
        + vectorBlock.startCol(); // The other dimension is always 0 (for vectors)
      const Eigen::DenseIndex size =
        vectorBlock.rows() * vectorBlock.cols(); // The other dimension is always 1 (for vectors)

      return ReturnType(vectorBlock.nestedExpression(), start, size);
    }
  };

  /**
   * @brief      JointNvVisitor visitor
   */
  struct JointNvVisitor : boost::static_visitor<int>
  {
    template<typename JointModelDerived>
    int operator()(const JointModelBase<JointModelDerived> & jmodel) const
    {
      return jmodel.nv();
    }

    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static int run(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
    {
      return boost::apply_visitor(JointNvVisitor(), jmodel);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline int nv(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
  {
    return JointNvVisitor::run(jmodel);
  }

  /**
   * @brief      JointNqVisitor visitor
   */
  struct JointNqVisitor : boost::static_visitor<int>
  {
    template<typename JointModelDerived>
    int operator()(const JointModelBase<JointModelDerived> & jmodel) const
    {
      return jmodel.nq();
    }

    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static int run(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
    {
      return boost::apply_visitor(JointNqVisitor(), jmodel);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline int nq(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
  {
    return JointNqVisitor::run(jmodel);
  }

  /**
   * @brief      JointNvExtendedVisitor visitor
   */
  struct JointNvExtendedVisitor : boost::static_visitor<int>
  {
    template<typename JointModelDerived>
    int operator()(const JointModelBase<JointModelDerived> & jmodel) const
    {
      return jmodel.nvExtended();
    }

    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static int run(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
    {
      return boost::apply_visitor(JointNvExtendedVisitor(), jmodel);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline int nvExtended(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
  {
    return JointNvExtendedVisitor::run(jmodel);
  }

  /**
   * @brief      JointConfigurationLimitVisitor visitor
   */
  struct JointConfigurationLimitVisitor : boost::static_visitor<std::vector<bool>>
  {
    template<typename JointModelDerived>
    const std::vector<bool> operator()(const JointModelBase<JointModelDerived> & jmodel) const
    {
      return jmodel.hasConfigurationLimit();
    }

    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static const std::vector<bool>
    run(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
    {
      return boost::apply_visitor(JointConfigurationLimitVisitor(), jmodel);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline const std::vector<bool>
  hasConfigurationLimit(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
  {
    return JointConfigurationLimitVisitor::run(jmodel);
  }

  /**
   * @brief      JointConfigurationLimitInTangentVisitor visitor
   */
  struct JointConfigurationLimitInTangentVisitor : boost::static_visitor<std::vector<bool>>
  {
    template<typename JointModelDerived>
    const std::vector<bool> operator()(const JointModelBase<JointModelDerived> & jmodel) const
    {
      return jmodel.hasConfigurationLimitInTangent();
    }

    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static const std::vector<bool>
    run(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
    {
      return boost::apply_visitor(JointConfigurationLimitInTangentVisitor(), jmodel);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline const std::vector<bool>
  hasConfigurationLimitInTangent(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
  {
    return JointConfigurationLimitInTangentVisitor::run(jmodel);
  }

  /**
   * @brief      JointIdxQVisitor visitor
   */
  struct JointIdxQVisitor : boost::static_visitor<int>
  {
    template<typename JointModelDerived>
    int operator()(const JointModelBase<JointModelDerived> & jmodel) const
    {
      return jmodel.idx_q();
    }

    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static int run(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
    {
      return boost::apply_visitor(JointIdxQVisitor(), jmodel);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline int idx_q(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
  {
    return JointIdxQVisitor::run(jmodel);
  }

  /**
   * @brief      JointIdxVVisitor visitor
   */
  struct JointIdxVVisitor : boost::static_visitor<int>
  {
    template<typename JointModelDerived>
    int operator()(const JointModelBase<JointModelDerived> & jmodel) const
    {
      return jmodel.idx_v();
    }

    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static int run(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
    {
      return boost::apply_visitor(JointIdxVVisitor(), jmodel);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline int idx_v(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
  {
    return JointIdxVVisitor::run(jmodel);
  }

  /**
   * @brief      JointIdxVExtendedVisitor visitor
   */
  struct JointIdxVExtendedVisitor : boost::static_visitor<int>
  {
    template<typename JointModelDerived>
    int operator()(const JointModelBase<JointModelDerived> & jmodel) const
    {
      return jmodel.idx_vExtended();
    }

    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static int run(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
    {
      return boost::apply_visitor(JointIdxVExtendedVisitor(), jmodel);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline int idx_vExtended(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
  {
    return JointIdxVExtendedVisitor::run(jmodel);
  }

  /**
   * @brief      JointIdVisitor visitor
   */
  struct JointIdVisitor : boost::static_visitor<JointIndex>
  {
    template<typename JointModelDerived>
    JointIndex operator()(const JointModelBase<JointModelDerived> & jmodel) const
    {
      return jmodel.id();
    }

    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static JointIndex run(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
    {
      return boost::apply_visitor(JointIdVisitor(), jmodel);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline JointIndex id(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
  {
    return JointIdVisitor::run(jmodel);
  }

  /**
   * @brief      JointSetIndexesVisitor visitor
   */
  struct JointSetIndexesVisitor : boost::static_visitor<>
  {
    JointIndex id;
    int q;
    int v;
    int vExtended;

    JointSetIndexesVisitor(JointIndex id, int q, int v, int vExtended)
    : id(id)
    , q(q)
    , v(v)
    , vExtended(vExtended)
    {
    }

    template<typename JointModelDerived>
    void operator()(JointModelBase<JointModelDerived> & jmodel) const
    {
      jmodel.setIndexes(id, q, v, vExtended);
    }

    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static void run(
      JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel,
      JointIndex id,
      int q,
      int v,
      int vExtended)
    {
      return boost::apply_visitor(JointSetIndexesVisitor(id, q, v, vExtended), jmodel);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline void setIndexes(
    JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel,
    JointIndex id,
    int q,
    int v,
    int vExtended)
  {
    return JointSetIndexesVisitor::run(jmodel, id, q, v, vExtended);
  }

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline void setIndexes(
    JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel, JointIndex id, int q, int v)
  {
    return JointSetIndexesVisitor::run(jmodel, id, q, v, v);
  }

  /**
   * @brief      JointModelShortnameVisitor visitor
   */
  struct JointModelShortnameVisitor : boost::static_visitor<std::string>
  {
    template<typename JointModelDerived>
    std::string operator()(const JointModelBase<JointModelDerived> & jmodel) const
    {
      return jmodel.shortname();
    }

    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static std::string run(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
    {
      return boost::apply_visitor(JointModelShortnameVisitor(), jmodel);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline std::string shortname(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
  {
    return JointModelShortnameVisitor::run(jmodel);
  }

  template<
    typename NewScalar,
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl>
  struct JointCastVisitor
  : fusion::JointUnaryVisitorBase<
      JointCastVisitor<NewScalar, Scalar, Options, JointCollectionTpl>,
      typename CastType<NewScalar, JointModelTpl<Scalar, Options, JointCollectionTpl>>::type>
  {
    typedef fusion::NoArg ArgsType;

    typedef typename CastType<NewScalar, JointModelTpl<Scalar, Options, JointCollectionTpl>>::type
      ReturnType;

    template<typename JointModelDerived>
    static ReturnType algo(const JointModelBase<JointModelDerived> & jmodel)
    {
      return ReturnType(jmodel.template cast<NewScalar>());
    }
  };

  template<
    typename NewScalar,
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl>
  typename CastType<NewScalar, JointModelTpl<Scalar, Options, JointCollectionTpl>>::type
  cast_joint(const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel)
  {
    typedef JointCastVisitor<NewScalar, Scalar, Options, JointCollectionTpl> Algo;
    return Algo::run(jmodel);
  }

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl,
    typename JointModelDerived>
  struct JointModelComparisonOperatorVisitor
  : fusion::JointUnaryVisitorBase<
      JointModelComparisonOperatorVisitor<Scalar, Options, JointCollectionTpl, JointModelDerived>,
      bool>
  {
    typedef boost::fusion::vector<const JointModelDerived &> ArgsType;

    template<typename JointModel>
    static bool
    algo(const JointModelBase<JointModel> & jmodel_lhs, const JointModelDerived & jmodel_rhs)
    {
      return internal::comparison_eq(jmodel_lhs.derived(), jmodel_rhs);
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl,
    typename JointModelDerived>
  bool isEqual(
    const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel_generic,
    const JointModelBase<JointModelDerived> & jmodel)
  {
    typedef JointModelComparisonOperatorVisitor<
      Scalar, Options, JointCollectionTpl, JointModelDerived>
      Algo;
    return Algo::run(jmodel_generic, typename Algo::ArgsType(boost::ref(jmodel.derived())));
  }

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl,
    typename JointModelDerived>
  struct JointModelHasSameIndexesVisitor
  : fusion::JointUnaryVisitorBase<
      JointModelHasSameIndexesVisitor<Scalar, Options, JointCollectionTpl, JointModelDerived>,
      bool>
  {
    typedef boost::fusion::vector<const JointModelDerived &> ArgsType;

    template<typename JointModel>
    static bool
    algo(const JointModelBase<JointModel> & jmodel_lhs, const JointModelDerived & jmodel_rhs)
    {
      return jmodel_lhs.derived().hasSameIndexes(jmodel_rhs);
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl,
    typename JointModelDerived>
  bool hasSameIndexes(
    const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel_generic,
    const JointModelBase<JointModelDerived> & jmodel)
  {
    typedef JointModelHasSameIndexesVisitor<Scalar, Options, JointCollectionTpl, JointModelDerived>
      Algo;
    return Algo::run(jmodel_generic, typename Algo::ArgsType(boost::ref(jmodel.derived())));
  }

  //
  // Visitors on JointDatas
  //

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointQVisitor
  : boost::static_visitor<
      typename JointDataTpl<Scalar, Options, JointCollectionTpl>::ConfigVector_t>
  {
    typedef typename JointDataTpl<Scalar, Options, JointCollectionTpl>::ConfigVector_t ReturnType;

    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return jdata.joint_q();
    }

    static ReturnType run(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointQVisitor(), jdata);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline typename JointDataTpl<Scalar, Options, JointCollectionTpl>::ConfigVector_t
  joint_q(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
  {
    return JointQVisitor<Scalar, Options, JointCollectionTpl>::run(jdata);
  }

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointVVisitor
  : boost::static_visitor<
      typename JointDataTpl<Scalar, Options, JointCollectionTpl>::ConfigVector_t>
  {
    typedef typename JointDataTpl<Scalar, Options, JointCollectionTpl>::TangentVector_t ReturnType;

    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return jdata.joint_v();
    }

    static ReturnType run(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointVVisitor(), jdata);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline typename JointDataTpl<Scalar, Options, JointCollectionTpl>::TangentVector_t
  joint_v(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
  {
    return JointVVisitor<Scalar, Options, JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointConstraintVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointConstraintVisitor
  : boost::static_visitor<JointMotionSubspaceTpl<Eigen::Dynamic, Scalar, Options>>
  {
    typedef JointMotionSubspaceTpl<Eigen::Dynamic, Scalar, Options> ReturnType;

    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.S().matrix());
    }

    static ReturnType run(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointConstraintVisitor(), jdata);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline JointMotionSubspaceTpl<Eigen::Dynamic, Scalar, Options>
  joint_motin_subspace_xd(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
  {
    return JointConstraintVisitor<Scalar, Options, JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointTransformVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointTransformVisitor : boost::static_visitor<SE3Tpl<Scalar, Options>>
  {
    typedef SE3Tpl<Scalar, Options> ReturnType;

    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.M());
    }

    static ReturnType run(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointTransformVisitor(), jdata);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline SE3Tpl<Scalar, Options>
  joint_transform(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
  {
    return JointTransformVisitor<Scalar, Options, JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointMotionVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointMotionVisitor : boost::static_visitor<MotionTpl<Scalar, Options>>
  {
    typedef MotionTpl<Scalar, Options> ReturnType;

    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.v());
    }

    static ReturnType run(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointMotionVisitor(), jdata);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options>
  motion(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
  {
    return JointMotionVisitor<Scalar, Options, JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointBiasVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointBiasVisitor : boost::static_visitor<MotionTpl<Scalar, Options>>
  {
    typedef MotionTpl<Scalar, Options> ReturnType;

    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.c());
    }

    static ReturnType run(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointBiasVisitor(), jdata);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options>
  bias(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
  {
    return JointBiasVisitor<Scalar, Options, JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointUInertiaVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointUInertiaVisitor
  : boost::static_visitor<Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options>>
  {
    typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options> ReturnType;

    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.U());
    }

    static ReturnType run(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointUInertiaVisitor(), jdata);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options>
  u_inertia(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
  {
    return JointUInertiaVisitor<Scalar, Options, JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointDInvInertiaVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointDInvInertiaVisitor
  : boost::static_visitor<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options>>
  {
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> ReturnType;

    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.Dinv());
    }

    static ReturnType run(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointDInvInertiaVisitor(), jdata);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options>
  dinv_inertia(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
  {
    return JointDInvInertiaVisitor<Scalar, Options, JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointUDInvInertiaVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointUDInvInertiaVisitor
  : boost::static_visitor<Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options>>
  {
    typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options> ReturnType;

    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.UDinv());
    }

    static ReturnType run(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointUDInvInertiaVisitor(), jdata);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options>
  udinv_inertia(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
  {
    return JointUDInvInertiaVisitor<Scalar, Options, JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointStUInertiaVisitor visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  struct JointStUInertiaVisitor
  : boost::static_visitor<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options>>
  {
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> ReturnType;

    template<typename JointDataDerived>
    ReturnType operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return ReturnType(jdata.StU());
    }

    static ReturnType run(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointStUInertiaVisitor(), jdata);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options>
  stu_inertia(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
  {
    return JointStUInertiaVisitor<Scalar, Options, JointCollectionTpl>::run(jdata);
  }

  /**
   * @brief      JointDataShortnameVisitor visitor
   */
  struct JointDataShortnameVisitor : boost::static_visitor<std::string>
  {
    template<typename JointDataDerived>
    std::string operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      return jdata.shortname();
    }

    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static std::string run(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
    {
      return boost::apply_visitor(JointDataShortnameVisitor(), jdata);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
  inline std::string shortname(const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata)
  {
    return JointDataShortnameVisitor::run(jdata);
  }

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl,
    typename JointDataDerived>
  struct JointDataComparisonOperatorVisitor
  : fusion::JointUnaryVisitorBase<
      JointDataComparisonOperatorVisitor<Scalar, Options, JointCollectionTpl, JointDataDerived>,
      bool>
  {
    typedef boost::fusion::vector<const JointDataDerived &> ArgsType;

    template<typename JointData>
    static bool algo(const JointDataBase<JointData> & jdata_lhs, const JointDataDerived & jdata_rhs)
    {
      return jdata_lhs.derived() == jdata_rhs;
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl,
    typename JointDataDerived>
  bool isEqual(
    const JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata_generic,
    const JointDataBase<JointDataDerived> & jdata)
  {
    typedef JointDataComparisonOperatorVisitor<
      Scalar, Options, JointCollectionTpl, JointDataDerived>
      Algo;
    return Algo::run(jdata_generic, typename Algo::ArgsType(boost::ref(jdata.derived())));
  }

  // Meta-function to check is_mimicable_t trait
  template<typename JointModel>
  struct is_mimicable
  {
    static constexpr bool value = traits<typename JointModel::JointDerived>::is_mimicable_t::value;
  };

  template<typename JointModel>
  struct CheckMimicVisitor : public boost::static_visitor<JointModel>
  {
    template<typename T>
    typename boost::enable_if_c<is_mimicable<T>::value, JointModel>::type
    operator()(const T & value) const
    {
      return value;
    }

    template<typename T>
    typename boost::disable_if_c<is_mimicable<T>::value, JointModel>::type
    operator()(const T & value) const
    {
      PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Type not supported in new variant");
      return value;
    }
  };

  template<typename JointModel>
  JointModel checkMimic(const JointModel & value)
  {
    return boost::apply_visitor(CheckMimicVisitor<JointModel>(), value);
  }

  template<typename ConfigVectorIn, typename Scalar, typename ConfigVectorOut>
  struct ConfigVectorAffineTransformVisitor : public boost::static_visitor<void>
  {
  public:
    const Eigen::MatrixBase<ConfigVectorIn> & qIn;
    const Scalar & scaling;
    const Scalar & offset;
    const Eigen::MatrixBase<ConfigVectorOut> & qOut;

    ConfigVectorAffineTransformVisitor(
      const Eigen::MatrixBase<ConfigVectorIn> & qIn,
      const Scalar & scaling,
      const Scalar & offset,
      const Eigen::MatrixBase<ConfigVectorOut> & qOut)
    : qIn(qIn)
    , scaling(scaling)
    , offset(offset)
    , qOut(qOut)
    {
    }

    template<typename JointModel>
    void operator()(const JointModel & /*jmodel*/) const
    {
      typedef typename ConfigVectorAffineTransform<typename JointModel::JointDerived>::Type
        AffineTransform;
      AffineTransform::run(qIn, scaling, offset, qOut);
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl,
    typename ConfigVectorIn,
    typename ConfigVectorOut>
  void configVectorAffineTransform(
    const JointModelTpl<Scalar, Options, JointCollectionTpl> & jmodel,
    const Eigen::MatrixBase<ConfigVectorIn> & qIn,
    const Scalar & scaling,
    const Scalar & offset,
    const Eigen::MatrixBase<ConfigVectorOut> & qOut)
  {
    boost::apply_visitor(
      ConfigVectorAffineTransformVisitor<ConfigVectorIn, Scalar, ConfigVectorOut>(
        qIn, scaling, offset, qOut),
      jmodel);
  }

  template<int Op, typename ForceType, typename ExpressionType>
  struct ApplyConstraintOnForceVisitor : public boost::static_visitor<void>
  {
    ForceType F;
    ExpressionType R;

    ApplyConstraintOnForceVisitor(ForceType F_, ExpressionType R_)
    : F(F_)
    , R(R_)
    {
    }

    template<typename JointDataDerived>
    void operator()(const JointDataBase<JointDataDerived> & jdata) const
    {
      // Since ExpressionType is often a temporary (Block, NoAlias) we need to const cast it
      switch (Op)
      {
      case SETTO:
        const_cast<ExpressionType &>(R) = jdata.S().transpose() * F;
        break;
      case ADDTO:
        const_cast<ExpressionType &>(R) += jdata.S().transpose() * F;
        break;
      case RMTO:
        const_cast<ExpressionType &>(R) -= jdata.S().transpose() * F;
        break;
      default:
        assert(false && "Wrong Op requesed value");
        break;
      }
    }

    template<typename Scalar, int Options, template<typename S, int O> class JointCollectionTpl>
    static void run(
      JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata,
      const ForceType F,
      ExpressionType R)
    {
      boost::apply_visitor(ApplyConstraintOnForceVisitor(F, R), jdata);
    }
  };

  template<
    int Op,
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl,
    typename ForceType,
    typename ExpressionType>
  void applyConstraintOnForceVisitor(
    JointDataTpl<Scalar, Options, JointCollectionTpl> & jdata, ForceType F, ExpressionType R)
  {
    return ApplyConstraintOnForceVisitor<Op, ForceType, ExpressionType>::run(jdata, F, R);
  }

  /// @endcond

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_joint_basic_visitors_hxx__
