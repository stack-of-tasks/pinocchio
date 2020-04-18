//
// Copyright (c) 2015-2018 CNRS
//

#ifndef __pinocchio_act_on_set_hpp__
#define __pinocchio_act_on_set_hpp__

#include "pinocchio/macros.hpp"
#include "pinocchio/spatial/fwd.hpp"

namespace pinocchio
{
  
  namespace forceSet
  {
    ///
    /// \brief SE3 action on a set of forces, represented by a 6xN matrix whose each
    ///        column represent a spatial force.
    ///
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet>
    static void se3Action(const SE3Tpl<Scalar,Options> & m,
                          const Eigen::MatrixBase<Mat> & iF,
                          Eigen::MatrixBase<MatRet> const & jF);
    
    /// \brief Default implementation with assignment operator=
    template<typename Scalar, int Options, typename Mat, typename MatRet>
    static void se3Action(const SE3Tpl<Scalar,Options> & m,
                          const Eigen::MatrixBase<Mat> & iF,
                          Eigen::MatrixBase<MatRet> const & jF);
    
    ///
    /// \brief Inverse SE3 action on a set of forces, represented by a 6xN matrix whose each
    ///        column represent a spatial force.
    ///
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet>
    static void se3ActionInverse(const SE3Tpl<Scalar,Options> & m,
                                 const Eigen::MatrixBase<Mat> & iF,
                                 Eigen::MatrixBase<MatRet> const & jF);
    
    /// \brief Default implementation with assignment operator=
    template<typename Scalar, int Options, typename Mat, typename MatRet>
    static void se3ActionInverse(const SE3Tpl<Scalar,Options> & m,
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
    template<int Op, typename Scalar, int Options, typename Mat,typename MatRet>
    static void se3Action(const SE3Tpl<Scalar,Options> & m,
                          const Eigen::MatrixBase<Mat> & iV,
                          Eigen::MatrixBase<MatRet> const & jV);
    
    /// \brief Default implementation with assignment operator=
    template<typename Scalar, int Options, typename Mat,typename MatRet>
    static void se3Action(const SE3Tpl<Scalar,Options> & m,
                          const Eigen::MatrixBase<Mat> & iV,
                          Eigen::MatrixBase<MatRet> const & jV);
    
    ///
    /// \brief Inverse SE3 action on a set of motions, represented by a 6xN matrix whose
    ///        column represent a spatial motion.
    ///
    template<int Op, typename Scalar, int Options, typename Mat,typename MatRet>
    static void se3ActionInverse(const SE3Tpl<Scalar,Options> & m,
                                 const Eigen::MatrixBase<Mat> & iV,
                                 Eigen::MatrixBase<MatRet> const & jV);
    
    /// \brief Default implementation with assignment operator=
    template<typename Scalar, int Options, typename Mat,typename MatRet>
    static void se3ActionInverse(const SE3Tpl<Scalar,Options> & m,
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

} // namespace pinocchio

#include "pinocchio/spatial/act-on-set.hxx"

#endif // ifndef __pinocchio_act_on_set_hpp__
