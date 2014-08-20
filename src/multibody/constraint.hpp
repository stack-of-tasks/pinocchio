#ifndef __se3_constraint_hpp__
#define __se3_constraint_hpp__


#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/motion.hpp"


// S   : v   \in M^6              -> v_J \in lie(Q) ~= R^nv
// S^T : f_J \in lie(Q)^* ~= R^nv -> f    \in F^6

namespace se3
{
  template<int _Dim, typename _Scalar, int _Options>
  class ConstraintTpl
  { 
    enum { nv = _Dim, Options = _Options };
    typedef _Scalar Scalar;
    typedef Eigen::Matrix<Scalar,nv,6> S;

    typedef Eigen::Matrix<Scalar,nv,1,Options> JointMotion;
    typedef Eigen::Matrix<Scalar,nv,1,Options> JointForce;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef ForceTpl<Scalar,Options> Force;

  public:
    template<D>
    ConstraintTpl( const Eigen::MatrixBase<D> & _S ) : S(_S)
    {  EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(S,_S); }

    ConstraintTpl() : S() { s.fill(nan); } 


    Motion operator* (const JointMotion& vj)
    { return Motion(S*vj); }

    struct Transpose
    {
      ConstraintTpl & ref;
      Transpose( ConstraintTpl & ref ) : ref(ref) {}

      Force operator* (const Force& f)
      { return S.transpose()*f.toVector(); }
    };
    Transpose transpose() { return Transpose(*this); }
  };

};



#endif // ifndef __se3_constraint_hpp__
