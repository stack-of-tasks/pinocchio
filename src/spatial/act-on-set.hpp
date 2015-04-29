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

#ifndef __se3_act_on_set_hpp__
#define __se3_act_on_set_hpp__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "pinocchio/spatial/fwd.hpp"

namespace se3
{
  namespace forceSet
  {
    /* SE3 action on a set of forces, represented by a 6xN matrix whose each
     * column represent a spatial force. */
    template<typename Mat,typename MatRet>
    static void se3Action( const SE3 & m, 
			   const Eigen::MatrixBase<Mat> & iF,
			   Eigen::MatrixBase<MatRet> & jF );
  }  // namespace forceSet

  namespace motionSet
  {
    /* SE3 action on a set of motions, represented by a 6xN matrix whose each
     * column represent a spatial motion. */
    template<typename Mat,typename MatRet>
    static void se3Action( const SE3 & m, 
			   const Eigen::MatrixBase<Mat> & iV,
			   Eigen::MatrixBase<MatRet> & jV );
  }  // namespace MotionSet

  /* --- DETAILS --------------------------------------------------------- */

  namespace internal 
  {

    template<typename Mat,typename MatRet, int NCOLS>
    struct ForceSetSe3Action
    {
      /* Compute jF = jXi * iF, where jXi is the dual action matrix associated
       * with m, and iF, jF are matrices whose columns are forces. The resolution
       * is done by block operation. It is less efficient than the colwise
       * operation and should not be used. */ 
      static void run( const SE3 & m, 
		       const Eigen::MatrixBase<Mat> & iF,
		       Eigen::MatrixBase<MatRet> & jF );
      // {
      //   typename Mat::template ConstNRowsBlockXpr<3>::Type linear  = iF.template topRows<3>();
      //   typename MatRet::template ConstNRowsBlockXpr<3>::Type angular = iF.template bottomRows<3>();
      
      //   jF.template topRows   <3>().noalias() = m.rotation()*linear;
      //   jF.template bottomRows<3>().noalias()
      // 	= skew(m.translation())*jF.template topRows<3>() +
      //      m.rotation()*angular;
      // }
      
    };

    template<typename Mat,typename MatRet>
    struct ForceSetSe3Action<Mat,MatRet,1>
    {
      /* Compute jF = jXi * iF, where jXi is the dual action matrix associated with m,
       * and iF, jF are vectors. */
      static void run( const SE3 & m, 
		       const Eigen::MatrixBase<Mat> & iF,
		       Eigen::MatrixBase<MatRet> & jF )
      { 
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatRet);
	Eigen::VectorBlock<const Mat,3> linear = iF.template head<3>();
	Eigen::VectorBlock<const Mat,3> angular = iF.template tail<3>();
	
	jF.template head <3>() = m.rotation()*linear;
	jF.template tail <3>() = (m.translation().cross(jF.template head<3>())
				  + m.rotation()*angular);
      }
    };

    /* Specialized implementation of block action, using colwise operation.  It
     * is empirically much faster than the true block operation, although I do
     * not understand why. */
    template<typename Mat,typename MatRet,int NCOLS>
    void ForceSetSe3Action<Mat,MatRet,NCOLS>::
    run( const SE3 & m, 
	 const Eigen::MatrixBase<Mat> & iF,
	 Eigen::MatrixBase<MatRet> & jF )
    {
      for(int col=0;col<jF.cols();++col) 
	{
	  typename MatRet::ColXpr jFc = jF.col(col);
	  forceSet::se3Action(m,iF.col(col),jFc);
	}
    }

  } // namespace internal

  namespace forceSet
  {
    template<typename Mat,typename MatRet>
    static void se3Action( const SE3 & m, 
			   const Eigen::MatrixBase<Mat> & iF,
			   Eigen::MatrixBase<MatRet> & jF )
    {
      internal::ForceSetSe3Action<Mat,MatRet,Mat::ColsAtCompileTime>::run(m,iF,jF);
    }

  }  // namespace forceSet


  namespace internal 
  {

    template<typename Mat,typename MatRet, int NCOLS>
    struct MotionSetSe3Action
    {
      /* Compute jF = jXi * iF, where jXi is the action matrix associated
       * with m, and iF, jF are matrices whose columns are motions. The resolution
       * is done by block operation. It is less efficient than the colwise
       * operation and should not be used. */ 
      static void run( const SE3 & m, 
		       const Eigen::MatrixBase<Mat> & iF,
		       Eigen::MatrixBase<MatRet> & jF );
      // {
      //   typename Mat::template ConstNRowsBlockXpr<3>::Type linear  = iF.template topRows<3>();
      //   typename MatRet::template ConstNRowsBlockXpr<3>::Type angular = iF.template bottomRows<3>();
      
      //   jF.template topRows   <3>().noalias() = m.rotation()*linear;
      //   jF.template bottomRows<3>().noalias()
      // 	= skew(m.translation())*jF.template topRows<3>() +
      //      m.rotation()*angular;
      // }
      
    };

    template<typename Mat,typename MatRet>
    struct MotionSetSe3Action<Mat,MatRet,1>
    {
      /* Compute jV = jXi * iV, where jXi is the action matrix associated with m,
       * and iV, jV are 6D vectors representing spatial velocities. */
      static void run( const SE3 & m, 
		       const Eigen::MatrixBase<Mat> & iV,
		       Eigen::MatrixBase<MatRet> & jV )
      { 
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatRet);
	Eigen::VectorBlock<const Mat,3> linear = iV.template head<3>();
	Eigen::VectorBlock<const Mat,3> angular = iV.template tail<3>();
	
	/* ( R*v + px(Rw),  Rw ) */
	jV.template tail <3>() = m.rotation()*angular;
	jV.template head <3>() = (m.translation().cross(jV.template tail<3>())
				  + m.rotation()*linear);
      }
    };

    /* Specialized implementation of block action, using colwise operation.  It
     * is empirically much faster than the true block operation, although I do
     * not understand why. */
    template<typename Mat,typename MatRet,int NCOLS>
    void MotionSetSe3Action<Mat,MatRet,NCOLS>::
    run( const SE3 & m, 
	 const Eigen::MatrixBase<Mat> & iV,
	 Eigen::MatrixBase<MatRet> & jV )
    {
      for(int col=0;col<jV.cols();++col) 
	{
	  typename MatRet::ColXpr jVc = jV.col(col);
	  motionSet::se3Action(m,iV.col(col),jVc);
	}
    }

  } // namespace internal

  namespace motionSet
  {
    template<typename Mat,typename MatRet>
    static void se3Action( const SE3 & m, 
			   const Eigen::MatrixBase<Mat> & iV,
			   Eigen::MatrixBase<MatRet> & jV )
    {
      internal::MotionSetSe3Action<Mat,MatRet,Mat::ColsAtCompileTime>::run(m,iV,jV);
    }

  }  // namespace motionSet


} // namespace se3

#endif // ifndef __se3_act_on_set_hpp__

