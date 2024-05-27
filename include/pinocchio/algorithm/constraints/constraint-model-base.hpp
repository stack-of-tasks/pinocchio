//
// Copyright (c) 2023 INRIA
//

#ifndef __pinocchio_algorithm_constraint_model_base_hpp__
#define __pinocchio_algorithm_constraint_model_base_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/fwd.hpp"

namespace pinocchio
{

  template<class Derived>
  struct ConstraintModelBase : NumericalBase<Derived>
  {
    typedef typename traits<Derived>::Scalar Scalar;
    enum
    {
      Options = traits<Derived>::Options
    };
    typedef typename traits<Derived>::ConstraintData ConstraintData;

    typedef Eigen::Matrix<bool, Eigen::Dynamic, 1, Options> BooleanVector;
    //    typedef Eigen::Matrix<Eigen::DenseIndex,Eigen::Dynamic,1,Options> IndexVector;
    typedef std::vector<Eigen::DenseIndex> IndexVector;

    Derived & derived()
    {
      return static_cast<Derived &>(*this);
    }
    const Derived & derived() const
    {
      return static_cast<const Derived &>(*this);
    }

    template<typename NewScalar>
    typename CastType<NewScalar, Derived>::type cast() const
    {
      return derived().template cast<NewScalar>();
    }

    /// \brief Evaluate the constraint values at the current state given by data and store the
    /// results in cdata.
    template<int Options, template<typename, int> class JointCollectionTpl>
    void calc(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const DataTpl<Scalar, Options, JointCollectionTpl> & data,
      ConstraintData & cdata) const
    {
      derived().calc(model, data, cdata);
    }

    template<typename JacobianMatrix, int Options, template<typename, int> class JointCollectionTpl>
    void jacobian(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const DataTpl<Scalar, Options, JointCollectionTpl> & data,
      ConstraintData & cdata,
      const Eigen::MatrixBase<JacobianMatrix> & jacobian_matrix) const
    {
      derived().jacobian(model, data, cdata, jacobian_matrix.const_cast_derived());
    }

    // Attributes common to all constraints

    /// \brief Name of the constraint
    std::string name;

    /// \brief Sparsity pattern associated to the constraint;
    BooleanVector colwise_sparsity;

    /// \brief Indexes of the columns spanned by the constraints.
    IndexVector colwise_span_indexes;

    template<typename OtherDerived>
    bool operator==(const ConstraintModelBase<OtherDerived> & other) const
    {
      return name == other.name && colwise_sparsity == other.colwise_sparsity
             && colwise_span_indexes == other.colwise_span_indexes;
    }

    template<typename OtherDerived>
    ConstraintModelBase & operator=(const ConstraintModelBase<OtherDerived> & other)
    {
      name = other.name;
      colwise_sparsity = other.colwise_sparsity;
      colwise_span_indexes = other.colwise_span_indexes;

      return *this;
    }

    ConstraintData createData() const
    {
      return derived().createData();
    }

  protected:
    template<int Options, template<typename, int> class JointCollectionTpl>
    ConstraintModelBase(const ModelTpl<Scalar, Options, JointCollectionTpl> & model)
    : colwise_sparsity(model.nv)
    {
      static const bool default_sparsity_value = false;
      colwise_sparsity.fill(default_sparsity_value);
    }

    /// \brief Default constructor
    ConstraintModelBase()
    {
    }

    ConstraintModelBase & base()
    {
      return *this;
    }
    const ConstraintModelBase & base() const
    {
      return *this;
    }
  };

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_constraint_model_base_hpp__
