//
// Copyright (c) 2015 CNRS
//

#ifndef __pinocchio_force_set_hpp__
#define __pinocchio_force_set_hpp__

#include "pinocchio/spatial/fwd.hpp"
#include <Eigen/Geometry>

namespace pinocchio
{
  template<typename _Scalar, int _Options>
  class ForceSetTpl
  {
  public:
    typedef _Scalar Scalar;
    enum { Options = _Options };
    typedef Eigen::Matrix<Scalar,3,1,Options> Vector3;
    typedef Eigen::Matrix<Scalar,3,3,Options> Matrix3;
    typedef Eigen::Matrix<Scalar,6,1,Options> Vector6;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef SE3Tpl<Scalar,Options> SE3;

    typedef Eigen::Matrix<Scalar,3,Eigen::Dynamic,Options> Matrix3x;
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> Matrix6x;

  public:
    // Constructors
    ForceSetTpl(const int & ncols ) : size(ncols),m_f(3,ncols), m_n(3,ncols) 
    { m_f.fill(NAN); m_n.fill(NAN); }
    ForceSetTpl(const Matrix3x & linear, const Matrix3x & angular)
      : size((int)linear.cols()),m_f(linear), m_n(angular)
    {  assert( linear.cols() == angular.cols() ); }

    Matrix6x matrix() const
    {
      Matrix6x F(6,size); F << m_f, m_n;
      // F.template topRows<3>() = m_f;
      // F.template bottomRows<3>()  = m_n;
      return F;
    }
    operator Matrix6x () const { return matrix(); }

    // Getters
    const Matrix3x & linear() const { return m_f; }
    const Matrix3x & angular() const { return m_n; }

    /// af = aXb.act(bf)
    ForceSetTpl se3Action(const SE3 & m) const
    {
      Matrix3x Rf (m.rotation()*linear());
      return ForceSetTpl(Rf,skew(m.translation())*Rf+m.rotation()*angular());
      // TODO check if nothing better than explicitely calling skew
    }
    /// bf = aXb.actInv(af)
    ForceSetTpl se3ActionInverse(const SE3 & m) const
    {
      return ForceSetTpl(m.rotation().transpose()*linear(),
		      m.rotation().transpose()*(angular() - skew(m.translation())*linear()) );
      // TODO check if nothing better than explicitely calling skew
    }

    friend std::ostream & operator << (std::ostream & os, const ForceSetTpl & phi)
    {
      os
	<< "F =\n" << phi.linear() << std::endl
	<< "Tau =\n" << phi.angular() << std::endl;
      return os;
    }

    /* --- BLOCK ------------------------------------------------------------ */
    struct Block
    {
      ForceSetTpl & ref;
      int idx,len;
      Block( ForceSetTpl & ref, const int & idx, const int & len )
	: ref(ref), idx(idx), len(len) {}
      
      Eigen::Block<ForceSetTpl::Matrix3x> linear() { return ref.m_f.block(0,idx,3,len); }
      Eigen::Block<ForceSetTpl::Matrix3x> angular() { return ref.m_n.block(0,idx,3,len); }
      Eigen::Block<const ForceSetTpl::Matrix3x> linear()  const 
      { return ((const ForceSetTpl::Matrix3x &)(ref.m_f)).block(0,idx,3,len); }
      Eigen::Block<const ForceSetTpl::Matrix3x> angular() const 
      { return ((const ForceSetTpl::Matrix3x &)(ref.m_n)).block(0,idx,3,len); }

      ForceSetTpl::Matrix6x matrix() const
      {
	ForceSetTpl::Matrix6x res(6,len); res << linear(),angular(); 
	return res;
      }

      Block& operator= (const ForceSetTpl & copy)
      {
	assert(copy.size == len);
	linear() = copy.linear(); //ref.m_f.block(0,idx,3,len) = copy.m_f;
	angular() = copy.angular(); //ref.m_n.block(0,idx,3,len) = copy.m_n;
	return *this;
      }

      Block& operator= (const ForceSetTpl::Block & copy)
      {
	assert(copy.len == len);
	linear() = copy.linear(); //ref.m_f.block(0,idx,3,len) = copy.ref.m_f.block(0,copy.idx,3,copy.len);
	angular() = copy.angular(); //ref.m_n.block(0,idx,3,len) = copy.ref.m_n.block(0,copy.idx,3,copy.len);
	return *this;
      }

      template <typename D>
      Block& operator= (const Eigen::MatrixBase<D> & m)
      {
        eigen_assert(D::RowsAtCompileTime == 6);
        assert(m.cols() == len);
        linear() = m.template topRows<3>();
        angular() = m.template bottomRows<3>();
        return *this;
      }

      /// af = aXb.act(bf)
      ForceSetTpl se3Action(const SE3 & m) const
      {
	// const Eigen::Block<const Matrix3x> linear = ref.linear().block(0,idx,3,len);
	// const Eigen::Block<const Matrix3x> angular = ref.angular().block(0,idx,3,len);
	Matrix3x Rf ((m.rotation()*linear()));
	return ForceSetTpl(Rf,skew(m.translation())*Rf+m.rotation()*angular());
	// TODO check if nothing better than explicitely calling skew
      }
      /// bf = aXb.actInv(af)
      ForceSetTpl se3ActionInverse(const SE3 & m) const
      {
	// const Eigen::Block<const Matrix3x> linear = ref.linear().block(0,idx,3,len);
	// const Eigen::Block<const Matrix3x> angular = ref.angular().block(0,idx,3,len);
	return ForceSetTpl(m.rotation().transpose()*linear(),
			   m.rotation().transpose()*(angular() - skew(m.translation())*linear()) );
	// TODO check if nothing better than explicitely calling skew
      }

    };

    Block block(const int & idx, const int & len) { return Block(*this,idx,len); }

    /* CRBA joint operators
     *   - ForceSet::Block = ForceSet
     *   - ForceSet operator* (Inertia Y,Constraint S)
     *   - MatrixBase operator* (Constraint::Transpose S, ForceSet::Block)
     *   - SE3::act(ForceSet::Block)
     */


  public: //
  private:
    int size;
    Matrix3x m_f,m_n;
  };

  typedef ForceSetTpl<double,0> ForceSet;

  template<>
  struct SE3GroupAction<ForceSet::Block> { typedef ForceSet ReturnType; };


} // namespace pinocchio

#endif // ifndef __pinocchio_force_set_hpp__

