//
// Copyright (c) 2023 INRIA
//

#ifndef __pinocchio_algorithm_constraint_model_base_hpp__
#define __pinocchio_algorithm_constraint_model_base_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/fwd.hpp"

namespace pinocchio {

  template<typename Derived>
  struct ConstraintModelBase : NumericalBase<Derived>
  {
    typedef typename traits<Derived>::Scalar Scalar;
    enum { Options = traits<Derived>::Options };
    typedef typename traits<Derived>::ConstraintData ConstraintData;

    typedef Eigen::Matrix<bool,Eigen::Dynamic,1,Options> BooleanVector;
    typedef Eigen::Matrix<Eigen::DenseIndex,Eigen::Dynamic,1,Options> IndexVector;

    Derived & derived() { return static_cast<Derived&>(*this); }
    const Derived & derived() const { return static_cast<const Derived&>(*this); }

    /// \brief Evaluate the constraint values at the current state given by data and store the results in cdata.
    template<int Options, template<typename,int> class JointCollectionTpl>
    void calc(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const DataTpl<Scalar,Options,JointCollectionTpl> & data,
              ConstraintData & cdata) const
    {
      derived().calc(model,data,cdata);
    }

    template<typename JacobianMatrix, int Options, template<typename,int> class JointCollectionTpl>
    void jacobian(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  ConstraintData & cdata,
                  const Eigen::MatrixBase<JacobianMatrix> & jacobian_matrix) const
    {
      derived().jacobian(model,data,cdata,jacobian_matrix.const_cast_derived());
    }

    // Attributes common to all constraints

    /// \brief Name of the constraint
    std::string name;

    /// \brief Sparsity pattern associated to the constraint;
    BooleanVector colwise_sparsity;

    /// \brief Indexes of the columns spanned by the constraints.
    IndexVector colwise_span_indexes;

    bool operator==(const ConstraintModelBase & other) const
    {
      return
         name == other.name
      && colwise_sparsity == other.colwise_sparsity
      && colwise_span_indexes == other.colwise_span_indexes;
    }

    ConstraintModelBase & operator=(const ConstraintModelBase & other)
    {
      name = other.name;
      colwise_sparsity = other.colwise_sparsity;
      colwise_span_indexes = other.colwise_span_indexes;

      return *this;
    }

  protected:

    /// \brief Default constructor
    template<int Options, template<typename,int> class JointCollectionTpl>
    ConstraintModelBase(const ModelTpl<Scalar,Options,JointCollectionTpl> & model)
    : colwise_sparsity(model.nv)
    {
      static const bool default_sparsity_value = false;
      colwise_sparsity.fill(default_sparsity_value);
    }

    ConstraintModelBase & base() { return *this; }
    const ConstraintModelBase & base() const { return *this; }
  };

  template<typename Derived>
  struct ConstraintDataBase : NumericalBase<Derived>
  {
    typedef typename traits<Derived>::Scalar Scalar;
    typedef typename traits<Derived>::ConstraintModel ConstraintModel;

    Derived & derived() { return static_cast<Derived&>(*this); }
    const Derived & derived() const { return static_cast<const Derived&>(*this); }
  };

}


#endif // ifndef __pinocchio_algorithm_constraint_model_base_hpp__
