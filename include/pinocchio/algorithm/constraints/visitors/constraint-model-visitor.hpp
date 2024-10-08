//
// Copyright (c) 2023 INRIA
//

#ifndef __pinocchio_algorithm_constraints_constraint_model_visitor_hpp__
#define __pinocchio_algorithm_constraints_constraint_model_visitor_hpp__

#include "pinocchio/algorithm/constraints/fwd.hpp"
// #include "pinocchio/algorithm/constraints/constraint-model-generic.hpp"
// #include "pinocchio/algorithm/constraints/constraint-data-generic.hpp"
#include "pinocchio/multibody/visitor/fusion.hpp"

namespace pinocchio
{

  namespace fusion
  {

    ///
    /// \brief Base structure for \b Unary visitation of a ConstraintModel.
    ///        This structure provides runners to call the right visitor according to the number of
    ///        arguments.
    ///
    template<typename ConstraintModelVisitorDerived, typename ReturnType = void>
    struct ConstraintUnaryVisitorBase
    {

      template<
        typename Scalar,
        int Options,
        template<typename, int> class ConstraintCollectionTpl,
        typename ArgsTmp>
      static ReturnType run(
        const ConstraintModelTpl<Scalar, Options, ConstraintCollectionTpl> & cmodel,
        ConstraintDataTpl<Scalar, Options, ConstraintCollectionTpl> & cdata,
        ArgsTmp args)
      {
        typedef ConstraintModelTpl<Scalar, Options, ConstraintCollectionTpl> ConstraintModel;
        InternalVisitorModelAndData<Scalar, Options, ConstraintCollectionTpl, ArgsTmp> visitor(
          cdata, args);
        return boost::apply_visitor(visitor, cmodel);
      }

      template<
        typename Scalar,
        int Options,
        template<typename, int> class ConstraintCollectionTpl,
        typename ArgsTmp>
      static ReturnType
      run(const ConstraintDataTpl<Scalar, Options, ConstraintCollectionTpl> & cdata, ArgsTmp args)
      {
        typedef ConstraintModelTpl<Scalar, Options, ConstraintCollectionTpl> ConstraintModel;
        InternalVisitorModel<Scalar, Options, ConstraintCollectionTpl, ArgsTmp> visitor(args);
        return boost::apply_visitor(visitor, cdata);
      }

    private:
      template<
        typename Scalar,
        int Options,
        template<typename, int> class ConstraintCollectionTpl,
        typename ArgsTmp>
      struct InternalVisitorModel : public boost::static_visitor<ReturnType>
      {
        typedef ConstraintModelTpl<Scalar, Options, ConstraintCollectionTpl> ConstraintModel;
        typedef ConstraintDataTpl<Scalar, Options, ConstraintCollectionTpl> ConstraintData;

        InternalVisitorModel(ArgsTmp args)
        : args(args)
        {
        }

        template<typename ConstraintModelDerived>
        ReturnType operator()(const ConstraintModelBase<ConstraintModelDerived> & cmodel) const
        {
          return bf::invoke(
            &ConstraintModelVisitorDerived::template algo<ConstraintModelDerived>,
            bf::append(boost::ref(cmodel.derived()), args));
        }

        template<typename ConstraintDataDerived>
        ReturnType operator()(const ConstraintDataBase<ConstraintDataDerived> & cdata) const
        {
          return bf::invoke(
            &ConstraintModelVisitorDerived::template algo<ConstraintDataDerived>,
            bf::append(boost::ref(cdata.derived()), args));
        }

        ArgsTmp args;
      };

      template<
        typename Scalar,
        int Options,
        template<typename, int> class ConstraintCollectionTpl,
        typename ArgsTmp>
      struct InternalVisitorModelAndData : public boost::static_visitor<ReturnType>
      {
        typedef ConstraintModelTpl<Scalar, Options, ConstraintCollectionTpl> ConstraintModel;
        typedef ConstraintDataTpl<Scalar, Options, ConstraintCollectionTpl> ConstraintData;

        InternalVisitorModelAndData(ConstraintData & cdata, ArgsTmp args)
        : cdata(cdata)
        , args(args)
        {
        }

        template<typename ConstraintModelDerived>
        ReturnType operator()(const ConstraintModelBase<ConstraintModelDerived> & cmodel) const
        {
          return bf::invoke(
            &ConstraintModelVisitorDerived::template algo<ConstraintModelDerived>,
            bf::append(
              boost::ref(cmodel.derived()),
              boost::ref(
                boost::get<typename ConstraintModelBase<ConstraintModelDerived>::ConstraintData>(
                  cdata)),
              args));
        }

        ConstraintData & cdata;
        ArgsTmp args;
      };
    };
  } // namespace fusion

  /**
   * @brief      ConstraintModelCalcVisitor fusion visitor
   */
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct ConstraintModelCalcVisitor
  : fusion::ConstraintUnaryVisitorBase<
      ConstraintModelCalcVisitor<Scalar, Options, JointCollectionTpl>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef boost::fusion::vector<const Model &, const Data &> ArgsType;

    template<typename ConstraintModel>
    static void algo(
      const pinocchio::ConstraintModelBase<ConstraintModel> & cmodel,
      typename ConstraintModel::ConstraintData & cdata,
      const Model & model,
      const Data & data)
    {
      cmodel.calc(model, data, cdata.derived());
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl,
    template<typename S, int O> class ConstraintCollectionTpl>
  void calc(
    const ConstraintModelTpl<Scalar, Options, ConstraintCollectionTpl> & cmodel,
    ConstraintDataTpl<Scalar, Options, ConstraintCollectionTpl> & cdata,
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    typedef ConstraintModelCalcVisitor<Scalar, Options, JointCollectionTpl> Algo;
    Algo::run(cmodel, cdata, typename Algo::ArgsType(model, data));
  }

  /**
   * @brief      ConstraintModelJacobianVisitor fusion visitor
   */
  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename JacobianMatrix>
  struct ConstraintModelJacobianVisitor
  : fusion::ConstraintUnaryVisitorBase<
      ConstraintModelJacobianVisitor<Scalar, Options, JointCollectionTpl, JacobianMatrix>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef boost::fusion::vector<const Model &, const Data &, JacobianMatrix &> ArgsType;

    template<typename ConstraintModel>
    static void algo(
      const pinocchio::ConstraintModelBase<ConstraintModel> & cmodel,
      typename ConstraintModel::ConstraintData & cdata,
      const Model & model,
      const Data & data,
      const Eigen::MatrixBase<JacobianMatrix> & jacobian_matrix)
    {
      cmodel.jacobian(model, data, cdata.derived(), jacobian_matrix.const_cast_derived());
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class JointCollectionTpl,
    template<typename S, int O> class ConstraintCollectionTpl,
    typename JacobianMatrix>
  void jacobian(
    const ConstraintModelTpl<Scalar, Options, ConstraintCollectionTpl> & cmodel,
    ConstraintDataTpl<Scalar, Options, ConstraintCollectionTpl> & cdata,
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<JacobianMatrix> & jacobian_matrix)
  {
    typedef ConstraintModelJacobianVisitor<Scalar, Options, JointCollectionTpl, JacobianMatrix>
      Algo;
    Algo::run(
      cmodel, cdata, typename Algo::ArgsType(model, data, jacobian_matrix.const_cast_derived()));
  }

  /**
   * @brief      ConstraintModelCreateDataVisitor fusion visitor
   */
  template<typename Scalar, int Options, template<typename S, int O> class ConstraintCollectionTpl>
  struct ConstraintModelCreateDataVisitor
  : boost::static_visitor<typename ConstraintCollectionTpl<Scalar, Options>::ConstraintDataVariant>
  {
    typedef fusion::NoArg ArgsType;
    typedef ConstraintCollectionTpl<Scalar, Options> ConstraintCollection;
    typedef typename ConstraintCollection::ConstraintModelVariant ConstraintModelVariant;
    typedef typename ConstraintCollection::ConstraintDataVariant ConstraintDataVariant;

    template<typename ConstraintModel>
    ConstraintDataVariant
    operator()(const pinocchio::ConstraintModelBase<ConstraintModel> & cmodel) const
    {
      return cmodel.createData();
    }

    static ConstraintDataVariant run(const ConstraintModelVariant & cmodel)
    {
      return boost::apply_visitor(ConstraintModelCreateDataVisitor(), cmodel);
    }
  };

  template<typename Scalar, int Options, template<typename S, int O> class ConstraintCollectionTpl>
  ConstraintDataTpl<Scalar, Options, ConstraintCollectionTpl>
  createData(const ConstraintModelTpl<Scalar, Options, ConstraintCollectionTpl> & cmodel)
  {
    return ConstraintModelCreateDataVisitor<Scalar, Options, ConstraintCollectionTpl>::run(cmodel);
  }

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class ConstraintCollectionTpl,
    typename ConstraintDataDerived>
  struct ConstraintDataComparisonOperatorVisitor
  : fusion::ConstraintUnaryVisitorBase<
      ConstraintDataComparisonOperatorVisitor<
        Scalar,
        Options,
        ConstraintCollectionTpl,
        ConstraintDataDerived>,
      bool>
  {
    typedef boost::fusion::vector<const ConstraintDataDerived &> ArgsType;

    template<typename ConstraintData>
    static bool algo(
      const ConstraintDataBase<ConstraintData> & cdata_lhs, const ConstraintDataDerived & cdata_rhs)
    {
      return cdata_lhs.derived() == cdata_rhs;
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename S, int O> class ConstraintCollectionTpl,
    typename ConstraintDataDerived>
  bool isEqual(
    const ConstraintDataTpl<Scalar, Options, ConstraintCollectionTpl> & cdata_generic,
    const ConstraintDataBase<ConstraintDataDerived> & cdata)
  {
    typedef ConstraintDataComparisonOperatorVisitor<
      Scalar, Options, ConstraintCollectionTpl, ConstraintDataDerived>
      Algo;
    return Algo::run(cdata_generic, typename Algo::ArgsType(boost::ref(cdata.derived())));
  }

} // namespace pinocchio

#endif // ifdef __pinocchio_algorithm_constraints_constraint_model_visitor_hpp__
