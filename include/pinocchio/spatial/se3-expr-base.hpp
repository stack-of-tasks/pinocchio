//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_spatial_se3_expr_base_hpp__
#define __pinocchio_spatial_se3_expr_base_hpp__

namespace pinocchio
{
  // Forward declaration
  // TODO: This should go into spatial/fwd.hpp but because of a strange include path
  // in context/cppadcg.hpp we can include se3.hpp without including spatial/fwd.hpp first.
  template<typename Derived>
  struct SE3ExprBase;
  template<typename Derived>
  struct SE3ExprNoalias;
  template<typename RotProduct, typename TransProduct>
  struct SE3ExprProduct;
  template<typename RotProduct, typename TransProduct>
  SE3ExprProduct<RotProduct, TransProduct>
  make_se3_expr_product(RotProduct rot_prod, TransProduct trans_prod);

// Forward traits typedef
#define PINOCCHIO_SE3_EXPR_TYPEDEF_TPL(Derived)                                                    \
  typedef typename traits<Derived>::Scalar Scalar;                                                 \
  typedef typename traits<Derived>::AngularType AngularType;                                       \
  typedef typename traits<Derived>::LinearType LinearType;                                         \
  typedef typename traits<Derived>::AngularRef AngularRef;                                         \
  typedef typename traits<Derived>::LinearRef LinearRef;                                           \
  typedef typename traits<Derived>::ConstAngularRef ConstAngularRef;                               \
  typedef typename traits<Derived>::ConstLinearRef ConstLinearRef;                                 \
  typedef typename traits<Derived>::PlainType PlainType;                                           \
  enum                                                                                             \
  {                                                                                                \
    Options = traits<Derived>::Options,                                                            \
  }

  // Use Eigen3 expression to optimize SE3 operations.
  template<typename Derived>
  struct SE3ExprBase
  {
    PINOCCHIO_SE3_EXPR_TYPEDEF_TPL(Derived);

    Derived & derived()
    {
      return *static_cast<Derived *>(this);
    }
    const Derived & derived() const
    {
      return *static_cast<const Derived *>(this);
    }

    ConstAngularRef rotation() const
    {
      return derived().rotation_impl();
    }
    ConstLinearRef translation() const
    {
      return derived().translation_impl();
    }
    AngularRef rotation()
    {
      return derived().rotation_impl();
    }
    LinearRef translation()
    {
      return derived().translation_impl();
    }

    template<typename OtherDerived>
    auto operator*(const SE3ExprBase<OtherDerived> & other) const
    {
      return make_se3_expr_product(
        rotation() * other.rotation(), translation() + rotation() * other.translation());
    }

    template<typename OtherDerived>
    Derived & operator=(const SE3ExprBase<OtherDerived> & other)
    {
      rotation() = other.rotation();
      translation() = other.translation();
      return derived();
    }

    SE3ExprNoalias<Derived> noalias()
    {
      return SE3ExprNoalias<Derived>(derived());
    }
  };

  // SE3Noalias use noalias in operator=
  template<typename Derived>
  struct SE3ExprNoalias
  {
    SE3ExprNoalias(Derived & expr)
    : expr(expr)
    {
    }

    template<typename OtherDerived>
    Derived & operator=(const SE3ExprBase<OtherDerived> & other)
    {
      expr.rotation().noalias() = other.rotation();
      expr.translation().noalias() = other.translation();
      return expr;
    }

    Derived & expr;
  };

  template<typename RotProduct, typename TransProduct>
  struct traits<SE3ExprProduct<RotProduct, TransProduct>>
  {
    enum
    {
      Options = 0
    };
    typedef double Scalar;
    typedef RotProduct AngularType;
    typedef RotProduct & AngularRef;
    typedef const RotProduct & ConstAngularRef;
    typedef TransProduct LinearType;
    typedef TransProduct & LinearRef;
    typedef const TransProduct & ConstLinearRef;
    typedef SE3ExprProduct<RotProduct, TransProduct> PlainType;
  };

  template<typename RotProduct, typename TransProduct>
  struct SE3ExprProduct : SE3ExprBase<SE3ExprProduct<RotProduct, TransProduct>>
  {
    PINOCCHIO_SE3_EXPR_TYPEDEF_TPL(SE3ExprProduct);

    SE3ExprProduct(RotProduct rot_prod, TransProduct trans_prod)
    : rot_prod(rot_prod)
    , trans_prod(trans_prod)
    {
    }

    ConstAngularRef rotation_impl() const
    {
      return rot_prod;
    }
    ConstLinearRef translation_impl() const
    {
      return trans_prod;
    }
    AngularRef rotation_impl()
    {
      return rot_prod;
    }
    LinearRef translation_impl()
    {
      return trans_prod;
    }

    RotProduct rot_prod;
    TransProduct trans_prod;
  };

  template<typename RotProduct, typename TransProduct>
  inline SE3ExprProduct<RotProduct, TransProduct>
  make_se3_expr_product(RotProduct rot_prod, TransProduct trans_prod)
  {
    return SE3ExprProduct<RotProduct, TransProduct>(rot_prod, trans_prod);
  }
} // namespace pinocchio

#endif // __pinocchio_spatial_se3_expr_base_hpp__
