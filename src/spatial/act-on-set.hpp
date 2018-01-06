//
// Copyright (c) 2015-2018 CNRS
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

#include "pinocchio/macros.hpp"
#include "pinocchio/spatial/fwd.hpp"

namespace se3
{
  
  enum AssignmentOperatorType
  {
    SETTO,
    ADDTO,
    RMTO
  };
  
  namespace forceSet
  {
    ///
    /// \brief SE3 action on a set of forces, represented by a 6xN matrix whose each
    ///        column represent a spatial force.
    ///
    template<int Op,typename Mat,typename MatRet>
    static void se3Action(const SE3 & m,
                          const Eigen::MatrixBase<Mat> & iF,
                          Eigen::MatrixBase<MatRet> const & jF);
    
    /// \brief Default implementation with assignment operator=
    template<typename Mat,typename MatRet>
    static void se3Action(const SE3 & m,
                          const Eigen::MatrixBase<Mat> & iF,
                          Eigen::MatrixBase<MatRet> const & jF);
    
    ///
    /// \brief Inverse SE3 action on a set of forces, represented by a 6xN matrix whose each
    ///        column represent a spatial force.
    ///
    template<int Op,typename Mat,typename MatRet>
    static void se3ActionInverse(const SE3 & m,
                                 const Eigen::MatrixBase<Mat> & iF,
                                 Eigen::MatrixBase<MatRet> const & jF);
    
    /// \brief Default implementation with assignment operator=
    template<typename Mat,typename MatRet>
    static void se3ActionInverse(const SE3 & m,
                                 const Eigen::MatrixBase<Mat> & iF,
                                 Eigen::MatrixBase<MatRet> const & jF);
    
    ///
    /// \brief Action of a motion on a set of forces, represented by a 6xN matrix whose each
    ///        column represent a spatial force.
    ///
    template<int Op, typename MotionDerived, typename Mat, typename MatRet>
    static void motionAction(const MotionDense<MotionDerived> & v,
                             const Eigen::MatrixBase<Mat> & iF,
                             Eigen::MatrixBase<MatRet> const & jF);
    
    /// \brief Default implementation with assignment operator=
    template<typename MotionDerived, typename Mat, typename MatRet>
    static void motionAction(const MotionDense<MotionDerived> & v,
                             const Eigen::MatrixBase<Mat> & iF,
                             Eigen::MatrixBase<MatRet> const & jF);
    
  }  // namespace forceSet

  namespace motionSet
  {
    ///
    /// \brief SE3 action on a set of motions, represented by a 6xN matrix whose
    ///        column represent a spatial motion.
    ///
    template<int Op, typename Mat,typename MatRet>
    static void se3Action(const SE3 & m,
                          const Eigen::MatrixBase<Mat> & iV,
                          Eigen::MatrixBase<MatRet> const & jV);
    
    /// \brief Default implementation with assignment operator=
    template<typename Mat,typename MatRet>
    static void se3Action(const SE3 & m,
                          const Eigen::MatrixBase<Mat> & iV,
                          Eigen::MatrixBase<MatRet> const & jV);
    
    ///
    /// \brief Inverse SE3 action on a set of motions, represented by a 6xN matrix whose
    ///        column represent a spatial motion.
    ///
    template<int Op, typename Mat,typename MatRet>
    static void se3ActionInverse(const SE3 & m,
                                 const Eigen::MatrixBase<Mat> & iV,
                                 Eigen::MatrixBase<MatRet> const & jV);
    
    /// \brief Default implementation with assignment operator=
    template<typename Mat,typename MatRet>
    static void se3ActionInverse(const SE3 & m,
                                 const Eigen::MatrixBase<Mat> & iV,
                                 Eigen::MatrixBase<MatRet> const & jV);
    
    ///
    /// \brief Action of a motion on a set of motions, represented by a 6xN matrix whose
    ///        columns represent a spatial motion.
    ///
    template<int Op, typename MotionDerived, typename Mat, typename MatRet>
    static void motionAction(const MotionDense<MotionDerived> & v,
                             const Eigen::MatrixBase<Mat> & iF,
                             Eigen::MatrixBase<MatRet> const & jF);
    
    /// \brief Default implementation with assignment operator=
    template<typename MotionDerived, typename Mat, typename MatRet>
    static void motionAction(const MotionDense<MotionDerived> & v,
                             const Eigen::MatrixBase<Mat> & iF,
                             Eigen::MatrixBase<MatRet> const & jF);
    
    ///
    /// \brief Action of an Inertia matrix on a set of motions, represented by a 6xN matrix whose
    ///        columns represent a spatial motion.
    ///
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet>
    static void inertiaAction(const InertiaTpl<Scalar,Options> & I,
                              const Eigen::MatrixBase<Mat> & iF,
                              Eigen::MatrixBase<MatRet> const & jF);
    
    /// \brief Default implementation with assignment operator=
    template<typename Scalar, int Options, typename Mat, typename MatRet>
    static void inertiaAction(const InertiaTpl<Scalar,Options> & I,
                              const Eigen::MatrixBase<Mat> & iF,
                              Eigen::MatrixBase<MatRet> const & jF);
    
    ///
    /// \brief Action of a motion set on a force object.
    ///        The input motion set is represented by a 6xN matrix whose each
    ///        column represent a spatial motion.
    ///        The output force set is represented by a 6xN matrix whose each
    ///        column represent a spatial force.
    ///
    template<int Op, typename ForceDerived, typename Mat, typename MatRet>
    static void act(const Eigen::MatrixBase<Mat> & iV,
                    const ForceDense<ForceDerived> & f,
                    Eigen::MatrixBase<MatRet> const & jF);
    
    /// \brief Default implementation with assignment operator=
    template<typename ForceDerived, typename Mat, typename MatRet>
    static void act(const Eigen::MatrixBase<Mat> & iV,
                    const ForceDense<ForceDerived> & f,
                    Eigen::MatrixBase<MatRet> const & jF);
    
  }  // namespace MotionSet

} // namespace se3

#include "pinocchio/spatial/act-on-set.hxx"

#endif // ifndef __se3_act_on_set_hpp__
