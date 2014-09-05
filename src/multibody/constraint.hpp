#ifndef __se3_constraint_hpp__
#define __se3_constraint_hpp__


#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/motion.hpp"


// S   : v   \in M^6              -> v_J \in lie(Q) ~= R^nv
// S^T : f_J \in lie(Q)^* ~= R^nv -> f    \in F^6

namespace se3
{
  template<int _Dim, typename _Scalar, int _Options=0>
  class ConstraintTpl
  { 
  public:
    enum { NV = _Dim, Options = _Options };
    typedef _Scalar Scalar;

    typedef Eigen::Matrix<Scalar,NV,1,Options> JointMotion;
    typedef Eigen::Matrix<Scalar,NV,1,Options> JointForce;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef ForceTpl<Scalar,Options> Force;
    typedef Eigen::Matrix<Scalar,6,NV> DenseBase;
  public:
    template<typename D>
    ConstraintTpl( const Eigen::MatrixBase<D> & _S ) : S(_S) {}

    ConstraintTpl() : S() 
    {
      EIGEN_STATIC_ASSERT_FIXED_SIZE(DenseBase);
      S.fill( NAN ); 
    } 
    ConstraintTpl(const int dim) : S(dim,6)     {      S.fill( NAN );     } 

    Motion operator* (const JointMotion& vj) const
    { return Motion(S*vj); }

    struct Transpose
    {
      const ConstraintTpl & ref;
      Transpose( const ConstraintTpl & ref ) : ref(ref) {}

      JointForce operator* (const Force& f) const
      { return ref.S.transpose()*f.toVector(); }
    };
    Transpose transpose() const { return Transpose(*this); }

    DenseBase & matrix() { return S; }
    const DenseBase & matrix() const { return S; }

    int nv() const { return NV; }

  private:
    DenseBase S;
  };

  typedef ConstraintTpl<1,double,0> Constraint1d;
  typedef ConstraintTpl<6,double,0> Constraint6d;
  typedef ConstraintTpl<Eigen::Dynamic,double,0> ConstraintXd;

} // namespace se3

#endif // ifndef __se3_constraint_hpp__
