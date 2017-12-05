//
// Copyright (c) 2015-2016 CNRS
// Copyright (c) 2016 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __se3_se3_hpp__
#define __se3_se3_hpp__

#include <Eigen/Geometry>
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/skew.hpp"
#include "pinocchio/macros.hpp"

namespace se3
{

  /* Type returned by the "se3Action" and "se3ActionInverse" functions. */
  namespace internal 
  {
    template<typename D>
    struct SE3GroupAction    { typedef D ReturnType; };
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
  template<class Derived>
  class SE3Base
  {
  protected:

    typedef Derived  Derived_t;
    SPATIAL_TYPEDEF_TEMPLATE(Derived_t);

  public:
      Derived_t & derived() { return *static_cast<Derived_t*>(this); }
      const Derived_t& derived() const { return *static_cast<const Derived_t*>(this); }

      ConstAngular_t & rotation() const  { return derived().rotation_impl(); }
      ConstLinear_t & translation() const  { return derived().translation_impl(); }
      Angular_t & rotation()  { return derived().rotation_impl(); }
      Linear_t & translation()   { return derived().translation_impl(); }
      void rotation(const Angular_t & R) { derived().rotation_impl(R); }
      void translation(const Linear_t & R) { derived().translation_impl(R); }


      Matrix4 toHomogeneousMatrix() const
      {
        return derived().toHomogeneousMatrix_impl();
      }
      operator Matrix4() const { return toHomogeneousMatrix(); }

      Matrix6 toActionMatrix() const
      {
        return derived().toActionMatrix_impl();
      }
      operator Matrix6() const { return toActionMatrix(); }
    
    Matrix6 toDualActionMatrix() const { return derived().toDualActionMatrix_impl(); }



      void disp(std::ostream & os) const
      {
        static_cast<const Derived_t*>(this)->disp_impl(os);
      }

      Derived_t operator*(const Derived_t & m2) const    { return derived().__mult__(m2); }

      /// ay = aXb.act(by)
      template<typename D>
      typename internal::SE3GroupAction<D>::ReturnType act   (const D & d) const 
      { 
        return derived().act_impl(d);
      }
      
      /// by = aXb.actInv(ay)
      template<typename D> typename internal::SE3GroupAction<D>::ReturnType actInv(const D & d) const
      {
        return derived().actInv_impl(d);
      }


      Derived_t act   (const Derived_t& m2) const { return derived().act_impl(m2); }
      Derived_t actInv(const Derived_t& m2) const { return derived().actInv_impl(m2); }


    bool operator==(const Derived_t & other) const { return derived().__equal__(other); }
    bool operator!=(const Derived_t & other) const { return !(*this == other); }

      bool isApprox (const Derived_t & other, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
      {
        return derived().isApprox_impl(other, prec);
      }

      friend std::ostream & operator << (std::ostream & os,const SE3Base<Derived> & X)
      { 
        X.disp(os);
        return os;
      }
    
    ///
    /// \returns true if *this is approximately equal to the identity placement, within the precision given by prec.
    ///
    bool isIdentity(const typename traits<Derived>::Scalar & prec = Eigen::NumTraits<typename traits<Derived>::Scalar>::dummy_precision()) const
    {
      return derived().isIdentity(prec);
    }

  }; // class SE3Base


  template<typename T, int U>
  struct traits< SE3Tpl<T, U> >
  {
    typedef T Scalar;
    typedef Eigen::Matrix<T,3,1,U> Vector3;
    typedef Eigen::Matrix<T,4,1,U> Vector4;
    typedef Eigen::Matrix<T,6,1,U> Vector6;
    typedef Eigen::Matrix<T,3,3,U> Matrix3;
    typedef Eigen::Matrix<T,4,4,U> Matrix4;
    typedef Eigen::Matrix<T,6,6,U> Matrix6;
    typedef Matrix3 Angular_t;
    typedef const Matrix3 ConstAngular_t;
    typedef Vector3 Linear_t;
    typedef const Vector3 ConstLinear_t;
    typedef Matrix6 ActionMatrix_t;
    typedef Eigen::Quaternion<T,U> Quaternion_t;
    typedef SE3Tpl<T,U> SE3;
    typedef ForceTpl<T,U> Force;
    typedef MotionTpl<T,U> Motion;
    typedef Symmetric3Tpl<T,U> Symmetric3;
    enum {
      LINEAR = 0,
      ANGULAR = 3
    };
  }; // traits SE3Tpl

  template<typename _Scalar, int _Options>
  class SE3Tpl : public SE3Base< SE3Tpl< _Scalar, _Options > >
  {

  public:
    friend class SE3Base< SE3Tpl< _Scalar, _Options > >;
    SPATIAL_TYPEDEF_TEMPLATE(SE3Tpl);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    SE3Tpl(): rot(), trans() {};


    template<typename M3,typename v3>
    SE3Tpl(const Eigen::MatrixBase<M3> & R, const Eigen::MatrixBase<v3> & p) 
    : rot(R), trans(p)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(v3,3)
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M3,3,3)
    }

    template<typename M4>
    explicit SE3Tpl(const Eigen::MatrixBase<M4> & m) 
    : rot(m.template block<3,3>(LINEAR,LINEAR)), trans(m.template block<3,1>(LINEAR,ANGULAR))
    {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(M4,4,4);
    }

    SE3Tpl(int) : rot(Matrix3::Identity()), trans(Vector3::Zero()) {}

    template<typename S2, int O2>
    SE3Tpl( const SE3Tpl<S2,O2> & clone )
    : rot(clone.rotation()),trans(clone.translation()) {}


    template<typename S2, int O2>
    SE3Tpl & operator= (const SE3Tpl<S2,O2> & other)
    {
      rot = other.rotation ();
      trans = other.translation ();
      return *this;
    }

    static SE3Tpl Identity()
    {
      return SE3Tpl(1);
    }

    SE3Tpl & setIdentity () { rot.setIdentity (); trans.setZero (); return *this;}

    /// aXb = bXa.inverse()
    SE3Tpl inverse() const
    {
      return SE3Tpl(rot.transpose(), -rot.transpose()*trans);
    }

    static SE3Tpl Random()
    {
      Quaternion_t q(Vector4::Random());
      q.normalize();
      return SE3Tpl(q.matrix(),Vector3::Random());
    }

    SE3Tpl & setRandom ()
    {
      Quaternion_t q(Vector4::Random());
      q.normalize ();
      rot = q.matrix ();
      trans.setRandom ();

      return *this;
    }
    
    Matrix4 toHomogeneousMatrix_impl() const
    {
      Matrix4 M;
      M.template block<3,3>(LINEAR,LINEAR) = rot;
      M.template block<3,1>(LINEAR,ANGULAR) = trans;
      M.template block<1,3>(ANGULAR,LINEAR).setZero();
      M(3,3) = 1;
      return M;
    }

    /// Vb.toVector() = bXa.toMatrix() * Va.toVector()
    Matrix6 toActionMatrix_impl() const
    {
      typedef Eigen::Block<Matrix6,3,3> Block3;
      Matrix6 M;
      M.template block<3,3>(ANGULAR,ANGULAR)
      = M.template block<3,3>(LINEAR,LINEAR) = rot;
      M.template block<3,3>(ANGULAR,LINEAR).setZero();
      Block3 B = M.template block<3,3>(LINEAR,ANGULAR);
      
      B.col(0) = trans.cross(rot.col(0));
      B.col(1) = trans.cross(rot.col(1));
      B.col(2) = trans.cross(rot.col(2));
      return M;
    }
    
    Matrix6 toDualActionMatrix_impl() const
    {
      typedef Eigen::Block<Matrix6,3,3> Block3;
      Matrix6 M;
      M.template block<3,3>(ANGULAR,ANGULAR)
      = M.template block<3,3>(LINEAR,LINEAR) = rot;
      M.template block<3,3>(LINEAR,ANGULAR).setZero();
      Block3 B = M.template block<3,3>(ANGULAR,LINEAR);
      
      B.col(0) = trans.cross(rot.col(0));
      B.col(1) = trans.cross(rot.col(1));
      B.col(2) = trans.cross(rot.col(2));
      return M;
    }

    void disp_impl(std::ostream & os) const
    {
      os << "  R =\n" << rot << std::endl
      << "  p = " << trans.transpose() << std::endl;
    }

    /// --- GROUP ACTIONS ON M6, F6 and I6 --- 

    /// ay = aXb.act(by)
    template<typename D>
    typename internal::SE3GroupAction<D>::ReturnType act_impl   (const D & d) const 
    { 
      return d.se3Action(*this);
    }
    /// by = aXb.actInv(ay)
    template<typename D> typename internal::SE3GroupAction<D>::ReturnType actInv_impl(const D & d) const
    {
      return d.se3ActionInverse(*this);
    }

    template<typename EigenDerived>
    typename EigenDerived::PlainObject actOnEigenObject(const Eigen::MatrixBase<EigenDerived> & p) const
    { return (rot*p+trans).eval(); }

    template<typename MapDerived>
    Vector3 actOnEigenObject(const Eigen::MapBase<MapDerived> & p) const
    { return Vector3(rot*p+trans); }

    template<typename EigenDerived>
    typename EigenDerived::PlainObject actInvOnEigenObject(const Eigen::MatrixBase<EigenDerived> & p) const
    { return (rot.transpose()*(p-trans)).eval(); }

    template<typename MapDerived>
    Vector3 actInvOnEigenObject(const Eigen::MapBase<MapDerived> & p) const
    { return Vector3(rot.transpose()*(p-trans)); }

    Vector3 act_impl   (const Vector3& p) const { return Vector3(rot*p+trans); }
    Vector3 actInv_impl(const Vector3& p) const { return Vector3(rot.transpose()*(p-trans)); }

    SE3Tpl act_impl    (const SE3Tpl& m2) const { return SE3Tpl( rot*m2.rot,trans+rot*m2.trans);}
    SE3Tpl actInv_impl (const SE3Tpl& m2) const { return SE3Tpl( rot.transpose()*m2.rot, rot.transpose()*(m2.trans-trans));}


    SE3Tpl __mult__(const SE3Tpl & m2) const { return this->act(m2);}

    bool __equal__( const SE3Tpl & m2 ) const
    {
      return (rotation_impl() == m2.rotation() && translation_impl() == m2.translation());
    }

    bool isApprox_impl (const SE3Tpl & m2, const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return rot.isApprox(m2.rot, prec) && trans.isApprox(m2.trans, prec);
    }
    
    bool isIdentity(const Scalar & prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
    {
      return rot.isIdentity(prec) && trans.isZero(prec);
    }

    ConstAngular_t & rotation_impl() const { return rot; }
    Angular_t & rotation_impl() { return rot; }
    void rotation_impl(const Angular_t & R) { rot = R; }
    ConstLinear_t & translation_impl() const { return trans;}
    Linear_t & translation_impl() { return trans;}
    void translation_impl(const Linear_t & p) { trans=p; }

  protected:
    Angular_t rot;
    Linear_t trans;
    
  }; // class SE3Tpl

} // namespace se3

#endif // ifndef __se3_se3_hpp__

