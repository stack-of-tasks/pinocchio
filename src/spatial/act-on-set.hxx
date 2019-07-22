//
// Copyright (c) 2015-2018 CNRS INRIA
//

#ifndef __pinocchio_act_on_set_hxx__
#define __pinocchio_act_on_set_hxx__

namespace pinocchio
{
  
  namespace internal 
  {

    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet, int NCOLS>
    struct ForceSetSe3Action
    {
      /* Compute jF = jXi * iF, where jXi is the dual action matrix associated
       * with m, and iF, jF are matrices whose columns are forces. The resolution
       * is done by block operation. It is less efficient than the colwise
       * operation and should not be used. */ 
      static void run(const SE3Tpl<Scalar,Options> & m,
                      const Eigen::MatrixBase<Mat> & iF,
                      Eigen::MatrixBase<MatRet> const & jF);
      
    };
    
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet>
    struct ForceSetSe3Action<Op,Scalar,Options,Mat,MatRet,1>
    {
      /* Compute jF = jXi * iF, where jXi is the dual action matrix associated with m,
       * and iF, jF are vectors. */
      static void run(const SE3Tpl<Scalar,Options> & m,
                      const Eigen::MatrixBase<Mat> & iF,
                      Eigen::MatrixBase<MatRet> const & jF)
      {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatRet);
        
        typedef ForceRef<const Mat> ForceRefOnMat;
        typedef ForceRef<MatRet> ForceRefOnMatRet;
        
        ForceRefOnMat fin(iF.derived());
        ForceRefOnMatRet fout(PINOCCHIO_EIGEN_CONST_CAST(MatRet,jF));
        
        switch(Op)
        {
          case SETTO:
            fout = m.act(fin);
            break;
          case ADDTO:
            fout += m.act(fin);
            break;
          case RMTO:
            fout -= m.act(fin);
            break;
          default:
            assert(false && "Wrong Op requesed value");
            break;
        }
      }
    };
    
    /* Specialized implementation of block action, using colwise operation.  It
     * is empirically much faster than the true block operation, although I do
     * not understand why. */
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet, int NCOLS>
    void ForceSetSe3Action<Op,Scalar,Options,Mat,MatRet,NCOLS>::
    run(const SE3Tpl<Scalar,Options> & m,
        const Eigen::MatrixBase<Mat> & iF,
        Eigen::MatrixBase<MatRet> const & jF)
    {
      for(int col=0;col<jF.cols();++col)
      {
        typename MatRet::ColXpr jFc
        = PINOCCHIO_EIGEN_CONST_CAST(MatRet,jF).col(col);
        forceSet::se3Action<Op>(m,iF.col(col),jFc);
      }
    }
    
    template<int Op, typename MotionDerived, typename Mat, typename MatRet, int NCOLS>
    struct ForceSetMotionAction
    {
      /* Compute dF = v ^ F, where  is the dual action operation associated
       * with v, and F, dF are matrices whose columns are forces. */
      static void run(const MotionDense<MotionDerived> & v,
                      const Eigen::MatrixBase<Mat> & iF,
                      Eigen::MatrixBase<MatRet> const & jF);
      
    };
    
    template<int Op, typename MotionDerived, typename Mat, typename MatRet>
    struct ForceSetMotionAction<Op,MotionDerived,Mat,MatRet,1>
    {
      template<typename Fin, typename Fout>
      static void run(const MotionDense<MotionDerived> & v,
                      const ForceDense<Fin> & fin,
                      ForceDense<Fout> & fout)
      {
        switch(Op)
        {
          case SETTO:
            fin.motionAction(v,fout);
            break;
          case ADDTO:
            fout += v.cross(fin);
            break;
          case RMTO:
            fout -= v.cross(fin);
            break;
          default:
            assert(false && "Wrong Op requesed value");
            break;
        }
      }
      
      static void run(const MotionDense<MotionDerived> & v,
                      const Eigen::MatrixBase<Mat> & iF,
                      Eigen::MatrixBase<MatRet> const & jF)
      {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatRet);
        
        typedef ForceRef<const Mat> ForceRefOnMat;
        typedef ForceRef<MatRet> ForceRefOnMatRet;
        
        ForceRefOnMat fin(iF.derived());
        ForceRefOnMatRet fout(PINOCCHIO_EIGEN_CONST_CAST(MatRet,jF));

        run(v,fin,fout);
      }
    };

    template<int Op, typename MotionDerived, typename Mat, typename MatRet, int NCOLS>
    void ForceSetMotionAction<Op,MotionDerived,Mat,MatRet,NCOLS>::
    run(const MotionDense<MotionDerived> & v,
        const Eigen::MatrixBase<Mat> & iF,
        Eigen::MatrixBase<MatRet> const & jF)
    {
      for(int col=0;col<jF.cols();++col)
      {
        typename MatRet::ColXpr jFc
        = PINOCCHIO_EIGEN_CONST_CAST(MatRet,jF).col(col);
        forceSet::motionAction<Op>(v,iF.col(col),jFc);
      }
    }
    
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet, int NCOLS>
    struct ForceSetSe3ActionInverse
    {
      /* Compute jF = jXi * iF, where jXi is the dual action matrix associated
       * with m, and iF, jF are matrices whose columns are forces. The resolution
       * is done by block operation. It is less efficient than the colwise
       * operation and should not be used. */
      static void run(const SE3Tpl<Scalar,Options> & m,
                      const Eigen::MatrixBase<Mat> & iF,
                      Eigen::MatrixBase<MatRet> const & jF);
      
    };
    
    template<int Op, typename Scalar, int Options, typename Mat , typename MatRet>
    struct ForceSetSe3ActionInverse<Op,Scalar,Options,Mat,MatRet,1>
    {
      /* Compute jF = jXi * iF, where jXi is the dual action matrix associated with m,
       * and iF, jF are vectors. */
      static void run(const SE3Tpl<Scalar,Options> & m,
                      const Eigen::MatrixBase<Mat> & iF,
                      Eigen::MatrixBase<MatRet> const & jF)
      {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatRet);
        
        typedef ForceRef<const Mat> ForceRefOnMat;
        typedef ForceRef<MatRet> ForceRefOnMatRet;
        
        ForceRefOnMat fin(iF.derived());
        ForceRefOnMatRet fout(PINOCCHIO_EIGEN_CONST_CAST(MatRet,jF));
        
        switch(Op)
        {
          case SETTO:
            fout = m.actInv(fin);
            break;
          case ADDTO:
            fout += m.actInv(fin);
            break;
          case RMTO:
            fout -= m.actInv(fin);
            break;
          default:
            assert(false && "Wrong Op requesed value");
            break;
        }
      }
      
    };
    
    /* Specialized implementation of block action, using colwise operation.  It
     * is empirically much faster than the true block operation, although I do
     * not understand why. */
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet, int NCOLS>
    void ForceSetSe3ActionInverse<Op,Scalar,Options,Mat,MatRet,NCOLS>::
    run(const SE3Tpl<Scalar,Options> & m,
        const Eigen::MatrixBase<Mat> & iF,
        Eigen::MatrixBase<MatRet> const & jF)
    {
      for(int col=0;col<jF.cols();++col)
      {
        typename MatRet::ColXpr jFc
        = PINOCCHIO_EIGEN_CONST_CAST(MatRet,jF).col(col);
        forceSet::se3ActionInverse<Op>(m,iF.col(col),jFc);
      }
    }
    
  } // namespace internal

  namespace forceSet
  {
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet>
    static void se3Action(const SE3Tpl<Scalar,Options> & m,
                          const Eigen::MatrixBase<Mat> & iF,
                          Eigen::MatrixBase<MatRet> const & jF)
    {
      internal::ForceSetSe3Action<Op,Scalar,Options,Mat,MatRet,Mat::ColsAtCompileTime>::run(m,iF,jF);
    }
    
    template<typename Scalar, int Options, typename Mat, typename MatRet>
    static void se3Action(const SE3Tpl<Scalar,Options> & m,
                          const Eigen::MatrixBase<Mat> & iF,
                          Eigen::MatrixBase<MatRet> const & jF)
    {
      internal::ForceSetSe3Action<SETTO,Scalar,Options,Mat,MatRet,Mat::ColsAtCompileTime>::run(m,iF,jF);
    }
    
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet>
    static void se3ActionInverse(const SE3Tpl<Scalar,Options> & m,
                                 const Eigen::MatrixBase<Mat> & iF,
                                 Eigen::MatrixBase<MatRet> const & jF)
    {
      internal::ForceSetSe3ActionInverse<Op,Scalar,Options,Mat,MatRet,Mat::ColsAtCompileTime>::run(m,iF,jF);
    }
    
    template<typename Scalar, int Options, typename Mat, typename MatRet>
    static void se3ActionInverse(const SE3Tpl<Scalar,Options> & m,
                                 const Eigen::MatrixBase<Mat> & iF,
                                 Eigen::MatrixBase<MatRet> const & jF)
    {
      internal::ForceSetSe3ActionInverse<SETTO,Scalar,Options,Mat,MatRet,Mat::ColsAtCompileTime>::run(m,iF,jF);
    }
    
    template<int Op, typename MotionDerived, typename Mat, typename MatRet>
    static void motionAction(const MotionDense<MotionDerived> & v,
                             const Eigen::MatrixBase<Mat> & iF,
                             Eigen::MatrixBase<MatRet> const & jF)
    {
      internal::ForceSetMotionAction<Op,MotionDerived,Mat,MatRet,Mat::ColsAtCompileTime>::run(v,iF,jF);
    }
    
    template<typename MotionDerived, typename Mat, typename MatRet>
    static void motionAction(const MotionDense<MotionDerived> & v,
                             const Eigen::MatrixBase<Mat> & iF,
                             Eigen::MatrixBase<MatRet> const & jF)
    {
      internal::ForceSetMotionAction<SETTO,MotionDerived,Mat,MatRet,Mat::ColsAtCompileTime>::run(v,iF,jF);
    }

  }  // namespace forceSet

  namespace internal 
  {

    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet, int NCOLS>
    struct MotionSetSe3Action
    {
      /* Compute jF = jXi * iF, where jXi is the action matrix associated
       * with m, and iF, jF are matrices whose columns are motions. */
      static void run(const SE3Tpl<Scalar,Options> & m,
                      const Eigen::MatrixBase<Mat> & iF,
                      Eigen::MatrixBase<MatRet> const & jF);
      
    };

    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet>
    struct MotionSetSe3Action<Op,Scalar,Options,Mat,MatRet,1>
    {
      /* Compute jV = jXi * iV, where jXi is the action matrix associated with m,
       * and iV, jV are 6D vectors representing spatial velocities. */
      static void run(const SE3Tpl<Scalar,Options> & m,
                      const Eigen::MatrixBase<Mat> & iV,
                      Eigen::MatrixBase<MatRet> const & jV)
      { 
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatRet);
        
        typedef MotionRef<const Mat> MotionRefOnMat;
        typedef MotionRef<MatRet> MotionRefOnMatRet;
        
        MotionRefOnMat min(iV.derived());
        MotionRefOnMatRet mout(PINOCCHIO_EIGEN_CONST_CAST(MatRet,jV));
        
        switch(Op)
        {
          case SETTO:
            mout = m.act(min);
            break;
          case ADDTO:
            mout += m.act(min);
            break;
          case RMTO:
            mout -= m.act(min);
            break;
          default:
            assert(false && "Wrong Op requesed value");
            break;
        }
      }
    };
    
    /* Specialized implementation of block action, using colwise operation.  It
     * is empirically much faster than the true block operation, although I do
     * not understand why. */
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet, int NCOLS>
    void MotionSetSe3Action<Op,Scalar,Options,Mat,MatRet,NCOLS>::
    run(const SE3Tpl<Scalar,Options> & m,
        const Eigen::MatrixBase<Mat> & iV,
        Eigen::MatrixBase<MatRet> const & jV)
    {
      for(int col=0;col<jV.cols();++col)
      {
        typename MatRet::ColXpr jVc
        = PINOCCHIO_EIGEN_CONST_CAST(MatRet,jV).col(col);
        motionSet::se3Action<Op>(m,iV.col(col),jVc);
      }
    }
    
    template<int Op, typename MotionDerived, typename Mat, typename MatRet, int NCOLS>
    struct MotionSetMotionAction
    {
      /* Compute dV = v ^ V, where  is the action operation associated
       * with v, and V, dV are matrices whose columns are motions. */
      static void run(const MotionDense<MotionDerived> & v,
                      const Eigen::MatrixBase<Mat> & iV,
                      Eigen::MatrixBase<MatRet> const & jV);
      
    };
    
    template<int Op, typename MotionDerived, typename Mat, typename MatRet, int NCOLS>
    void MotionSetMotionAction<Op,MotionDerived,Mat,MatRet,NCOLS>::
    run(const MotionDense<MotionDerived> & v,
        const Eigen::MatrixBase<Mat> & iV,
        Eigen::MatrixBase<MatRet> const & jV)
    {
      for(int col=0;col<jV.cols();++col)
      {
        typename MatRet::ColXpr jVc
        = PINOCCHIO_EIGEN_CONST_CAST(MatRet,jV).col(col);
        motionSet::motionAction<Op>(v,iV.col(col),jVc);
      }
    }
    
    template<int Op, typename MotionDerived, typename Mat, typename MatRet>
    struct MotionSetMotionAction<Op,MotionDerived,Mat,MatRet,1>
    {
      static void run(const MotionDense<MotionDerived> & v,
                      const Eigen::MatrixBase<Mat> & iV,
                      Eigen::MatrixBase<MatRet> const & jV)
      {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatRet);
        
        typedef MotionRef<const Mat> MotionRefOnMat;
        typedef MotionRef<MatRet> MotionRefOnMatRet;
        
        MotionRefOnMat min(iV.derived());
        MotionRefOnMatRet mout(PINOCCHIO_EIGEN_CONST_CAST(MatRet,jV));
        
        switch(Op)
        {
          case SETTO:
            min.motionAction(v,mout);
            break;
          case ADDTO:
            mout += v.cross(min);
            break;
          case RMTO:
            mout -= v.cross(min);
            break;
          default:
            assert(false && "Wrong Op requesed value");
            break;
        }
      }
    };
    
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet, int NCOLS>
    struct MotionSetSe3ActionInverse
    {
      /* Compute jF = jXi * iF, where jXi is the action matrix associated
       * with m, and iF, jF are matrices whose columns are motions. The resolution
       * is done by block operation. It is less efficient than the colwise
       * operation and should not be used. */
      static void run(const SE3Tpl<Scalar,Options> & m,
                      const Eigen::MatrixBase<Mat> & iF,
                      Eigen::MatrixBase<MatRet> const & jF);
      
    };
    
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet>
    struct MotionSetSe3ActionInverse<Op,Scalar,Options,Mat,MatRet,1>
    {
      /* Compute jV = jXi * iV, where jXi is the action matrix associated with m,
       * and iV, jV are 6D vectors representing spatial velocities. */
      static void run(const SE3Tpl<Scalar,Options> & m,
                      const Eigen::MatrixBase<Mat> & iV,
                      Eigen::MatrixBase<MatRet> const & jV)
      {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatRet);
        
        typedef MotionRef<Mat> MotionRefOnMat;
        typedef MotionRef<MatRet> MotionRefOnMatRet;
        
        MotionRefOnMat min(iV.derived());
        MotionRefOnMatRet mout(PINOCCHIO_EIGEN_CONST_CAST(MatRet,jV));
        
        switch(Op)
        {
          case SETTO:
            mout = m.actInv(min);
            break;
          case ADDTO:
            mout += m.actInv(min);
            break;
          case RMTO:
            mout -= m.actInv(min);
            break;
          default:
            assert(false && "Wrong Op requesed value");
            break;
        }
      }
    };
    
    /* Specialized implementation of block action, using colwise operation.  It
     * is empirically much faster than the true block operation, although I do
     * not understand why. */
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet, int NCOLS>
    void MotionSetSe3ActionInverse<Op,Scalar,Options,Mat,MatRet,NCOLS>::
    run(const SE3Tpl<Scalar,Options> & m,
        const Eigen::MatrixBase<Mat> & iV,
        Eigen::MatrixBase<MatRet> const & jV)
    {
      for(int col=0;col<jV.cols();++col)
      {
        typename MatRet::ColXpr jVc
        = PINOCCHIO_EIGEN_CONST_CAST(MatRet,jV).col(col);
        motionSet::se3ActionInverse<Op>(m,iV.col(col),jVc);
      }
    }
    
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet, int NCOLS>
    struct MotionSetInertiaAction
    {
      /* Compute dV = v ^ V, where  is the action operation associated
       * with v, and V, dV are matrices whose columns are motions. */
      static void run(const InertiaTpl<Scalar,Options> & I,
                      const Eigen::MatrixBase<Mat> & iV,
                      Eigen::MatrixBase<MatRet> const & jV);
      
    };
    
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet, int NCOLS>
    void MotionSetInertiaAction<Op,Scalar,Options,Mat,MatRet,NCOLS>::
    run(const InertiaTpl<Scalar,Options> & I,
        const Eigen::MatrixBase<Mat> & iV,
        Eigen::MatrixBase<MatRet> const & jV)
    {
      for(int col=0;col<jV.cols();++col)
      {
        typename MatRet::ColXpr jVc
        = PINOCCHIO_EIGEN_CONST_CAST(MatRet,jV).col(col);
        motionSet::inertiaAction<Op>(I,iV.col(col),jVc);
      }
    }
    
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet>
    struct MotionSetInertiaAction<Op,Scalar,Options,Mat,MatRet,1>
    {
      static void run(const InertiaTpl<Scalar,Options> & I,
                      const Eigen::MatrixBase<Mat> & iV,
                      Eigen::MatrixBase<MatRet> const & jV)
      {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatRet);
        
        typedef MotionRef<const Mat> MotionRefOnMat;
        typedef ForceRef<MatRet> ForceRefOnMatRet;
        MotionRefOnMat min(iV.derived());
        ForceRefOnMatRet fout(PINOCCHIO_EIGEN_CONST_CAST(MatRet,jV));
       
        switch(Op)
        {
          case SETTO:
            I.__mult__(min,fout);
            break;
          case ADDTO:
            fout += I*min;
            break;
          case RMTO:
            fout -= I*min;
            break;
          default:
            assert(false && "Wrong Op requesed value");
            break;
        }
      }
    };
    
    template<int Op, typename ForceDerived, typename Mat, typename MatRet, int NCOLS>
    struct MotionSetActOnForce
    {
      static void run(const Eigen::MatrixBase<Mat> & iV,
                      const ForceDense<ForceDerived> & f,
                      Eigen::MatrixBase<MatRet> const & jF)
      {
        for(int col=0;col<jF.cols();++col)
        {
          typename MatRet::ColXpr jFc
          = PINOCCHIO_EIGEN_CONST_CAST(MatRet,jF).col(col);
          motionSet::act<Op>(iV.col(col),f,jFc);
        }
      }
      
    };
    
    template<int Op, typename ForceDerived, typename Mat, typename MatRet>
    struct MotionSetActOnForce<Op,ForceDerived,Mat,MatRet,1>
    {
      /* Compute jF = jXi * iF, where jXi is the dual action matrix associated with m,
       * and iF, jF are vectors. */
      static void run(const Eigen::MatrixBase<Mat> & iV,
                      const ForceDense<ForceDerived> & f,
                      Eigen::MatrixBase<MatRet> const & jF)
      {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Mat);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(MatRet);
        
        typedef MotionRef<const Mat> MotionRefOnMat;
        typedef ForceRef<MatRet> ForceRefOnMatRet;
        
        MotionRefOnMat vin(iV.derived());
        ForceRefOnMatRet fout(PINOCCHIO_EIGEN_CONST_CAST(MatRet,jF));
        ForceSetMotionAction<Op,MotionRefOnMat,Mat,MatRet,1>::run(vin,f,fout);
      }
      
    };
    

  } // namespace internal

  namespace motionSet
  {
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet>
    static void se3Action(const SE3Tpl<Scalar,Options> & m,
                          const Eigen::MatrixBase<Mat> & iV,
                          Eigen::MatrixBase<MatRet> const & jV)
    {
      internal::MotionSetSe3Action<Op,Scalar,Options,Mat,MatRet,Mat::ColsAtCompileTime>::run(m,iV,jV);
    }
    
    template<typename Scalar, int Options, typename Mat, typename MatRet>
    static void se3Action(const SE3Tpl<Scalar,Options> & m,
                          const Eigen::MatrixBase<Mat> & iV,
                          Eigen::MatrixBase<MatRet> const & jV)
    {
      internal::MotionSetSe3Action<SETTO,Scalar,Options,Mat,MatRet,Mat::ColsAtCompileTime>::run(m,iV,jV);
    }
    
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet>
    static void se3ActionInverse(const SE3Tpl<Scalar,Options> & m,
                                 const Eigen::MatrixBase<Mat> & iV,
                                 Eigen::MatrixBase<MatRet> const & jV)
    {
      internal::MotionSetSe3ActionInverse<Op,Scalar,Options,Mat,MatRet,Mat::ColsAtCompileTime>::run(m,iV,jV);
    }
    
    template<typename Scalar, int Options, typename Mat, typename MatRet>
    static void se3ActionInverse(const SE3Tpl<Scalar,Options> & m,
                                 const Eigen::MatrixBase<Mat> & iV,
                                 Eigen::MatrixBase<MatRet> const & jV)
    {
      internal::MotionSetSe3ActionInverse<SETTO,Scalar,Options,Mat,MatRet,Mat::ColsAtCompileTime>::run(m,iV,jV);
    }
    
    template<int Op, typename MotionDerived, typename Mat, typename MatRet>
    static void motionAction(const MotionDense<MotionDerived> & v,
                             const Eigen::MatrixBase<Mat> & iV,
                             Eigen::MatrixBase<MatRet> const & jV)
    {
      internal::MotionSetMotionAction<Op,MotionDerived,Mat,MatRet,Mat::ColsAtCompileTime>::run(v,iV,jV);
    }
    
    template<typename MotionDerived, typename Mat, typename MatRet>
    static void motionAction(const MotionDense<MotionDerived> & v,
                             const Eigen::MatrixBase<Mat> & iV,
                             Eigen::MatrixBase<MatRet> const & jV)
    {
      internal::MotionSetMotionAction<SETTO,MotionDerived,Mat,MatRet,Mat::ColsAtCompileTime>::run(v,iV,jV);
    }
    
    template<int Op, typename Scalar, int Options, typename Mat, typename MatRet>
    static void inertiaAction(const InertiaTpl<Scalar,Options> & I,
                              const Eigen::MatrixBase<Mat> & iV,
                              Eigen::MatrixBase<MatRet> const & jV)
    {
      internal::MotionSetInertiaAction<Op,Scalar,Options,Mat,MatRet,MatRet::ColsAtCompileTime>::run(I,iV,jV);
    }
    
    template<typename Scalar, int Options, typename Mat, typename MatRet>
    static void inertiaAction(const InertiaTpl<Scalar,Options> & I,
                              const Eigen::MatrixBase<Mat> & iV,
                              Eigen::MatrixBase<MatRet> const & jV)
    {
      internal::MotionSetInertiaAction<SETTO,Scalar,Options,Mat,MatRet,MatRet::ColsAtCompileTime>::run(I,iV,jV);
    }
    
    template<int Op, typename ForceDerived, typename Mat, typename MatRet>
    static void act(const Eigen::MatrixBase<Mat> & iV,
                    const ForceDense<ForceDerived> & f,
                    Eigen::MatrixBase<MatRet> const & jF)
    {
      internal::MotionSetActOnForce<Op,ForceDerived,Mat,MatRet,MatRet::ColsAtCompileTime>::run(iV,f,jF);
    }
    
    template<typename ForceDerived, typename Mat, typename MatRet>
    static void act(const Eigen::MatrixBase<Mat> & iV,
                    const ForceDense<ForceDerived> & f,
                    Eigen::MatrixBase<MatRet> const & jF)
    {
      internal::MotionSetActOnForce<SETTO,ForceDerived,Mat,MatRet,MatRet::ColsAtCompileTime>::run(iV,f,jF);
    }

  }  // namespace motionSet

} // namespace pinocchio

#endif // ifndef __pinocchio_act_on_set_hxx__
