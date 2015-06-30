//
// Copyright (c) 2015 CNRS
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
  template< class Derived>
  class SE3Base
  {
  protected:

    typedef Derived  Derived_t;
    SPATIAL_TYPEDEF_ARG(Derived_t);

  public:
      Derived_t & derived() { return *static_cast<Derived_t*>(this); }
      const Derived_t& derived() const { return *static_cast<const Derived_t*>(this); }

      const Angular_t & rotation() const  { return derived().rotation_impl(); }
      const Linear_t & translation() const  { return derived().translation_impl(); }
      Angular_t & rotation()  { return derived().rotation_impl(); }
      Linear_t & translation()   { return derived().translation_impl(); }
      void rotation(const Angular_t & R) { derived().rotation_impl(R); }
      void translation(const Linear_t & R) { derived().translation_impl(R); }


      //  Unable to change this with Matrix4 toHomogen...
      Matrix4 toHomogeneousMatrix() const
      {
        // std::cout << "2Homo base" << std::endl;
        return derived().toHomogeneousMatrix_impl();
      }
      operator Matrix4() const { return toHomogeneousMatrix(); }

      Matrix6 toActionMatrix() const
      {
        // std::cout << "2Action base" << std::endl;
        return derived().toActionMatrix_impl();
      }
      operator Matrix6() const { return toActionMatrix(); }


      void disp(std::ostream & os) const
      {
        os << "base disp" << std::endl;
        static_cast<const Derived_t*>(this)->disp_impl(os);
      }

      Derived_t operator*(const Derived_t & m2) const    { return derived().__mult__(m2); }

      // ay = aXb.act(by)
      template<typename D>
      typename internal::ActionReturn<D>::Type act   (const D & d) const 
      { 
        return derived().act_impl(d);
      }
        /// by = aXb.actInv(ay)
      template<typename D> typename internal::ActionReturn<D>::Type actInv(const D & d) const
      {
        return derived().actInv_impl(d);
      }

      // Vector3 act   (const Vector3& p) const { return derived().act_impl(p); }
      // Vector3 actInv(const Vector3& p) const { return derived().actInv_impl(p); }

      Derived_t act   (const Derived_t& m2) const { return derived().act_impl(m2); }
      Derived_t actInv(const Derived_t& m2) const { return derived().actInv_impl(m2); }

      friend std::ostream & operator << (std::ostream & os,const SE3Base<Derived> & X)
      { 
        os << "base <<" << std::endl;
        X.disp(os);
        return os;
      }

  };


    template<typename T, int U>
    struct traits< SE3Tpl<T, U> >
    {
      typedef T Scalar_t;
      typedef Eigen::Matrix<T,3,1,U> Vector3;
      typedef Eigen::Matrix<T,4,1,U> Vector4;
      typedef Eigen::Matrix<T,6,1,U> Vector6;
      typedef Eigen::Matrix<T,3,3,U> Matrix3;
      typedef Eigen::Matrix<T,4,4,U> Matrix4;
      typedef Eigen::Matrix<T,6,6,U> Matrix6;
      typedef Matrix3 Angular_t;
      typedef Vector3 Linear_t;
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
    };

    template<typename _Scalar, int _Options>
    class SE3Tpl : public SE3Base< SE3Tpl< _Scalar, _Options > >
    {

    public:
      friend class SE3Base< SE3Tpl< _Scalar, _Options > >;
      SPATIAL_TYPEDEF_ARG(SE3Tpl);


    public:
      SE3Tpl(): rot(), trans() {};


      template<typename M3,typename v3>
      SE3Tpl(const Eigen::MatrixBase<M3> & R, const Eigen::MatrixBase<v3> & p) 
      : rot(R), trans(p)
      {
      }

      SE3Tpl(int) : rot(Matrix3::Identity()), trans(Vector3::Zero()) {}

      template<typename S2, int O2>
      SE3Tpl( const SE3Tpl<S2,O2> & clone )  //cf SE3Tpl
        : rot(clone.rotation()),trans(clone.translation()) {}

      template<typename S2, int O2>
      SE3Tpl & operator= (const SE3Tpl<S2,O2> & other) // cf SE3TplTpl
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

    public:
      //  Unable to change this with Matrix4 toHomogen...
      Matrix4 toHomogeneousMatrix_impl() const
      {
        // std::cout << "2Homo derived" << std::endl;
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
        Matrix6 M;
        M.template block<3,3>(ANGULAR,ANGULAR)
        = M.template block<3,3>(LINEAR,LINEAR) = rot;
        M.template block<3,3>(ANGULAR,LINEAR).setZero();
        M.template block<3,3>(LINEAR,ANGULAR)
        = skew(trans) * M.template block<3,3>(ANGULAR,ANGULAR);
        return M;
      }

      void disp_impl(std::ostream & os) const
      {
        os << "SE3Tpl disp" << std::endl;
        os << "  R =\n" << rot << std::endl
        << "  p = " << trans.transpose() << std::endl;
      }

      /// --- GROUP ACTIONS ON M6, F6 and I6 --- 
      // ay = aXb.act(by)
      template<typename D>
      typename internal::ActionReturn<D>::Type act_impl   (const D & d) const 
      { 
        return d.se3Action(*this);
      }
        /// by = aXb.actInv(ay)
      template<typename D> typename internal::ActionReturn<D>::Type actInv_impl(const D & d) const
      {
        return d.se3ActionInverse(*this);
      }

      Vector3 act_impl   (const Vector3& p) const { return (rot*p+trans).eval(); }
      Vector3 actInv_impl(const Vector3& p) const { return (rot.transpose()*(p-trans)).eval(); }

      SE3Tpl act_impl    (const SE3Tpl& m2) const { return SE3Tpl( rot*m2.rot,trans+rot*m2.trans);}
      SE3Tpl actInv_impl (const SE3Tpl& m2) const { return SE3Tpl( rot.transpose()*m2.rot, rot.transpose()*(m2.trans-trans));}

      /// Operators 
      // operator Matrix4() const { return toHomogeneousMatrix(); }
      // operator Matrix6() const { return toActionMatrix(); }

      // friend std::ostream & operator << (std::ostream & os,const SE3Tpl & X)
      // { 
      //   os << "derived <<" << std::endl;
      //   return os;
      //   // X.disp(os); return os;
      // }

      SE3Tpl __mult__(const SE3Tpl & m2) const { return this->act(m2);}
      // SE3Tpl operator*(const SE3Tpl & m2) const    { return this->act(m2); }


    public:
      const Angular_t & rotation_impl() const { return rot; }
      Angular_t & rotation_impl() { return rot; }
      void rotation_impl(const Angular_t & R) { rot = R; }
      const Linear_t & translation_impl() const { return trans;}
      Linear_t & translation_impl() { return trans;}
      void translation_impl(const Linear_t & p) { trans=p; }

    protected:
      Angular_t rot;
      Linear_t trans;
    };




































 //  template<typename _Scalar, int _Options>
 //  class SE3Tpl
 //  {
 //  public:

 //    typedef _Scalar Scalar;
 //    enum { Options = _Options };
 //    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
 //    typedef Eigen::Matrix<Scalar,4,1,Options> Vector4;
 //    typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
 //    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
 //    typedef Eigen::Matrix<Scalar,4,4,Options> Matrix4;
 //    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
 //    typedef Eigen::Quaternion<Scalar,Options> Quaternion;
 //    typedef MotionTpl<Scalar,Options> Motion;
 //    typedef ForceTpl<Scalar,Options> Force;
 //    //typedef ActionTpl<Scalar,Options> Action;
 //    enum { LINEAR = 0, ANGULAR = 3 };

 //  public:
 //    // Constructors
 //    SE3Tpl() : rot(), trans() {}
 //    template<typename M3,typename v3>
 //    SE3Tpl(const Eigen::MatrixBase<M3> & R, const Eigen::MatrixBase<v3> & p) 
 //      : rot(R), trans(p) {}
 //    SE3Tpl(int) : rot(Matrix3::Identity()), trans(Vector3::Zero()) {}
 //    template<typename S2, int O2>
 //    SE3Tpl( const SE3Tpl<S2,O2> clone ) 
 //      : rot(clone.rotation()),trans(clone.translation()) {}
    
 //    template<typename S2, int O2>
 //    SE3Tpl & operator= (const SE3Tpl<S2,O2> & other)
 //    {
 //      rot = other.rotation ();
 //      trans = other.translation ();
 //      return *this;
 //    }

 //    const Matrix3 & rotation()    const { return rot;   }
 //    const Vector3 & translation() const { return trans; }
 //    Matrix3 & rotation()                { return rot;   }
 //    Vector3 & translation()             { return trans; }
 //    void rotation(const Matrix3 & R)    { rot=R;   }
 //    void translation(const Vector3 & p) { trans=p; }

 //    static SE3Tpl Identity()
 //    {
 //      return SE3Tpl(1);
 //    }
 //    static SE3Tpl Random()
 //    {
 //      Eigen::Quaternion<Scalar,Options> q(Vector4::Random());
 //      q.normalize();
 //      return SE3Tpl(q.matrix(),Vector3::Random());
 //    }

 //    SE3Tpl & setIdentity () { rot.setIdentity (); trans.setZero (); return *this;}
 //    SE3Tpl & setRandom ()
 //    {
 //      Quaternion q(Vector4::Random());
 //      q.normalize ();
 //      rot = q.matrix ();
 //      trans.setRandom ();

 //      return *this;
 //    }

 //    Eigen::Matrix<Scalar,4,4,Options> toHomogeneousMatrix() const
 //    {
 //      Eigen::Matrix<Scalar,4,4,Options> M;
 //      M.template block<3,3>(0,0) = rot;
 //      M.template block<3,1>(0,3) = trans;
 //      M.template block<1,3>(3,0).setZero();
 //      M(3,3) = 1;
 //      return M;
 //    }

 //    /// Vb.toVector() = bXa.toMatrix() * Va.toVector()
 //    Matrix6 toActionMatrix() const
 //    {
 //      Matrix6 M;
 //      M.template block<3,3>(ANGULAR,ANGULAR)
	// = M.template block<3,3>(LINEAR,LINEAR) = rot;
 //      M.template block<3,3>(ANGULAR,LINEAR).setZero();
 //      M.template block<3,3>(LINEAR,ANGULAR)
	// = skew(trans) * M.template block<3,3>(ANGULAR,ANGULAR);
 //      return M;
 //    }
    
 //    /// aXb = bXa.inverse()
 //    SE3Tpl inverse() const
 //    {
 //      return SE3Tpl(rot.transpose(), -rot.transpose()*trans);
 //    }

 //    void disp(std::ostream & os) const
 //    {
 //      os << "  R =\n" << rot << std::endl
	//  << "  p = " << trans.transpose() << std::endl;
 //    }


 //    /* --- GROUP ACTIONS ON M6, F6 and I6 --- */

 //     /// ay = aXb.act(by)
 //    template<typename D> typename internal::ActionReturn<D>::Type act   (const D & d) const 
 //    { return d.se3Action(*this); }
 //    /// by = aXb.actInv(ay)
 //    template<typename D> typename internal::ActionReturn<D>::Type actInv(const D & d) const
 //    { return d.se3ActionInverse(*this); }

 //    Vector3 act   (const Vector3& p) const { return (rot*p+trans).eval(); }
 //    Vector3 actInv(const Vector3& p) const { return (rot.transpose()*(p-trans)).eval(); }

 //    SE3Tpl act    (const SE3Tpl& m2) const { return SE3Tpl( rot*m2.rot,trans+rot*m2.trans);}
 //    SE3Tpl actInv (const SE3Tpl& m2) const { return SE3Tpl( rot.transpose()*m2.rot,
	// 						    rot.transpose()*(m2.trans-trans));}
      
 //    /* --- OPERATORS -------------------------------------------------------- */
 //    operator Matrix4() const { return toHomogeneousMatrix(); }
 //    operator Matrix6() const { return toActionMatrix(); }
 //    SE3Tpl operator*(const SE3Tpl & m2) const    { return this->act(m2); }
 //    friend std::ostream & operator << (std::ostream & os,const SE3Tpl & X)
 //    { X.disp(os); return os; }

 //  public:
 //  private:
 //    Matrix3 rot;
 //    Vector3 trans;
 //  };

  typedef SE3Tpl<double,0> SE3;

} // namespace se3

#endif // ifndef __se3_se3_hpp__

