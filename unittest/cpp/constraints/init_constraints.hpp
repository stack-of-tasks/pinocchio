#pragma once

#include "pinocchio/constraints.hpp"

namespace pinocchio
{

  template<class ConstraintModel>
  struct init_constraint_default
  {
    template<typename S, int O, template<typename, int> class JointCollectionTpl>
    static ConstraintModel run(const ModelTpl<S, O, JointCollectionTpl> & model)
    {
      return ConstraintModel(model);
    }
  };

  template<typename Scalar, int Options>
  struct init_constraint_default<RigidConstraintModelTpl<Scalar, Options>>
  {
    typedef RigidConstraintModelTpl<Scalar, Options> ConstraintModel;

    template<typename S, int O, template<typename, int> class JointCollectionTpl>
    static ConstraintModel run(const ModelTpl<S, O, JointCollectionTpl> & model)
    {
      return ConstraintModel(CONTACT_3D, model, 0, SE3::Random());
    }
  };

  template<typename Scalar, int Options>
  struct init_constraint_default<PointContactConstraintModelTpl<Scalar, Options>>
  {
    typedef PointContactConstraintModelTpl<Scalar, Options> ConstraintModel;

    template<typename S, int O, template<typename, int> class JointCollectionTpl>
    static ConstraintModel run(const ModelTpl<S, O, JointCollectionTpl> & model)
    {
      return ConstraintModel(model, 0, SE3::Random());
    }
  };

  template<
    class ConstraintModel,
    typename S,
    int O,
    template<typename, int> class JointCollectionTpl>
  ConstraintModel init_constraint(const ModelTpl<S, O, JointCollectionTpl> & model)
  {
    return init_constraint_default<ConstraintModel>::run(model);
  }

} // namespace pinocchio
