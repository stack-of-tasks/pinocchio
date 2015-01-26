#ifndef __se3_se3_hpp__
#define __se3_se3_hpp__

#include <Eigen/Geometry>
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/skew.hpp"

namespace se3
{

  /* Type returned by the "se3Action" and "se3ActionInverse" functions. */
  namespace internal 
  {
    template<typename D>
    struct ActionReturn    { typedef D Type; };
  }

  /** The rigid transform aMb can be seen in two ways: 
   *
   * - given a point p expressed in frame B by its coordinate vector Bp, aMb
   * computes its coordinates in frame A by Ap = aMb Bp.
   * - aMb displaces a solid S centered at frame A into the solid centered in
   * B. In particular, the origin of A is displaced at the origin of B: $^aM_b
   * ^aA = ^aB$.

   * The rigid displacement is stored as a rotation matrix and translation vector by:
   * aMb (x) =  aRb*x + aAB
   * where aAB is the vector from origin A to origin B expressed in coordinates A.
   */

  template<typename _Scalar, int _Options>
  class SE3Tpl
  {
  public:

    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,4,1,Options> Vector4;
    typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,4,4,Options> Matrix4;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef Eigen::Quaternion<Scalar,Options> Quaternion;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef ForceTpl<Scalar,Options> Force;
    //typedef ActionTpl<Scalar,Options> Action;
    enum { LINEAR = 0, ANGULAR = 3 };

  public:
    // Constructors
    SE3Tpl() : rot(), trans() {}
    template<typename M3,typename v3>
    SE3Tpl(const Eigen::MatrixBase<M3> & R, const Eigen::MatrixBase<v3> & p) 
      : rot(R), trans(p) {}
    SE3Tpl(int) : rot(Matrix3::Identity()), trans(Vector3::Zero()) {}
    template<typename S2, int O2>
    SE3Tpl( const SE3Tpl<S2,O2> clone ) 
      : rot(clone.rotation()),trans(clone.translation()) {}
    
    template<typename S2, int O2>
    SE3Tpl & operator= (const SE3Tpl<S2,O2> & other)
    {
      rot = other.rotation ();
      trans = other.translation ();
      return *this;
    }

    const Matrix3 & rotation()    const { return rot;   }
    const Vector3 & translation() const { return trans; }
    Matrix3 & rotation()                { return rot;   }
    Vector3 & translation()             { return trans; }
    void rotation(const Matrix3 & R)    { rot=R;   }
    void translation(const Vector3 & p) { trans=p; }

    static SE3Tpl Identity()
    {
      return SE3Tpl(1);
    }
    static SE3Tpl Random()
    {
      Eigen::Quaternion<Scalar,Options> q(Vector4::Random());
      q.normalize();
      return SE3Tpl(q.matrix(),Vector3::Random());
    }

    Eigen::Matrix<Scalar,4,4,Options> toHomogeneousMatrix() const
    {
      Eigen::Matrix<Scalar,4,4,Options> M;
      M.template block<3,3>(0,0) = rot;
      M.template block<3,1>(0,3) = trans;
      M.template block<1,3>(3,0).setZero();
      M(3,3) = 1;
      return M;
    }

    /// Vb.toVector() = bXa.toMatrix() * Va.toVector()
    Matrix6 toActionMatrix() const
    {
      Matrix6 M;
      M.template block<3,3>(ANGULAR,ANGULAR)
	= M.template block<3,3>(LINEAR,LINEAR) = rot;
      M.template block<3,3>(ANGULAR,LINEAR).setZero();
      M.template block<3,3>(LINEAR,ANGULAR)
	= skew(trans) * M.template block<3,3>(ANGULAR,ANGULAR);
      return M;
    }
    
    /// aXb = bXa.inverse()
    SE3Tpl inverse() const
    {
      return SE3Tpl(rot.transpose(), -rot.transpose()*trans);
    }

    void disp(std::ostream & os) const
    {
      os << "  R =\n" << rot << std::endl
	 << "  p =\n" << trans.transpose() << std::endl;
    }


    /* --- GROUP ACTIONS ON M6, F6 and I6 --- */

     /// ay = aXb.act(by)
    template<typename D> typename internal::ActionReturn<D>::Type act   (const D & d) const 
    { return d.se3Action(*this); }
    /// by = aXb.actInv(ay)
    template<typename D> typename internal::ActionReturn<D>::Type actInv(const D & d) const
    { return d.se3ActionInverse(*this); }

    Vector3 act   (const Vector3& p) const { return (rot*p+trans).eval(); }
    Vector3 actInv(const Vector3& p) const { return (rot.transpose()*(p-trans)).eval(); }

    SE3Tpl act    (const SE3Tpl& m2) const { return SE3Tpl( rot*m2.rot,trans+rot*m2.trans);}
    SE3Tpl actInv (const SE3Tpl& m2) const { return SE3Tpl( rot.transpose()*m2.rot,
							    rot.transpose()*(m2.trans-trans));}
      
    /* --- OPERATORS -------------------------------------------------------- */
    operator Matrix4() const { return toHomogeneousMatrix(); }
    operator Matrix6() const { return toActionMatrix(); }
    SE3Tpl operator*(const SE3Tpl & m2) const    { return this->act(m2); }
    friend std::ostream & operator << (std::ostream & os,const SE3Tpl & X)
    { X.disp(os); return os; }

  public:
  private:
    Matrix3 rot;
    Vector3 trans;
  };

  typedef SE3Tpl<double,0> SE3;

} // namespace se3

#endif // ifndef __se3_se3_hpp__

