//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_math_matrix_block_hpp__
#define __pinocchio_math_matrix_block_hpp__

#include <Eigen/Core>

namespace pinocchio
{
  template<int NV>
  struct SizeDepType
  {
    template<class Mat>
    struct SegmentReturn
    {
      typedef typename Mat::template FixedSegmentReturnType<NV>::Type Type;
      typedef typename Mat::template ConstFixedSegmentReturnType<NV>::Type ConstType;
    };
    
    template<typename D>
    static typename SegmentReturn<D>::ConstType
    segment(const Eigen::MatrixBase<D> & mat,
            typename Eigen::DenseBase<D>::Index start,
            typename Eigen::DenseBase<D>::Index size = NV)
    {
      PINOCCHIO_UNUSED_VARIABLE(size);
      return mat.template segment<NV>(start);
    }
    
    template<typename D>
    static typename SegmentReturn<D>::Type
    segment(Eigen::MatrixBase<D> & mat,
            typename Eigen::DenseBase<D>::Index start,
            typename Eigen::DenseBase<D>::Index size = NV)
    {
      PINOCCHIO_UNUSED_VARIABLE(size);
      return mat.template segment<NV>(start);
    }
    
    template<class Mat>
    struct ColsReturn
    {
      typedef typename Mat::template NColsBlockXpr<NV>::Type Type;
      typedef typename Mat::template ConstNColsBlockXpr<NV>::Type ConstType;
    };
    
    template<typename D>
    static typename ColsReturn<D>::ConstType
    middleCols(const Eigen::MatrixBase<D> & mat,
               typename Eigen::DenseBase<D>::Index start,
               typename Eigen::DenseBase<D>::Index size = NV)
    {
      PINOCCHIO_UNUSED_VARIABLE(size);
      return mat.template middleCols<NV>(start);
    }
    
    template<typename D>
    static typename ColsReturn<D>::Type
    middleCols(Eigen::MatrixBase<D> & mat,
               typename Eigen::DenseBase<D>::Index start,
               typename Eigen::DenseBase<D>::Index size = NV)
    {
      PINOCCHIO_UNUSED_VARIABLE(size);
      return mat.template middleCols<NV>(start);
    }
    
    template<class Mat>
    struct RowsReturn
    {
      typedef typename Mat::template NRowsBlockXpr<NV>::Type Type;
      typedef typename Mat::template ConstNRowsBlockXpr<NV>::Type ConstType;
    };
    
    template<typename D>
    static typename RowsReturn<D>::ConstType
    middleRows(const Eigen::MatrixBase<D> & mat,
               typename Eigen::DenseBase<D>::Index start,
               typename Eigen::DenseBase<D>::Index size = NV)
    {
      PINOCCHIO_UNUSED_VARIABLE(size);
      return mat.template middleRows<NV>(start);
    }
    
    template<typename D>
    static typename RowsReturn<D>::Type
    middleRows(Eigen::MatrixBase<D> & mat,
               typename Eigen::DenseBase<D>::Index start,
               typename Eigen::DenseBase<D>::Index size = NV)
    {
      PINOCCHIO_UNUSED_VARIABLE(size);
      return mat.template middleRows<NV>(start);
    }
    
    template<class Mat>
    struct BlockReturn
    {
      typedef Eigen::Block<Mat, NV, NV> Type;
      typedef const Eigen::Block<const Mat, NV, NV> ConstType;
    };
    
    template<typename D>
    static typename BlockReturn<D>::ConstType
    block(const Eigen::MatrixBase<D> & mat,
          typename Eigen::DenseBase<D>::Index row_id,
          typename Eigen::DenseBase<D>::Index col_id,
          typename Eigen::DenseBase<D>::Index row_size_block = NV,
          typename Eigen::DenseBase<D>::Index col_size_block = NV)
    {
      PINOCCHIO_UNUSED_VARIABLE(row_size_block);
      PINOCCHIO_UNUSED_VARIABLE(col_size_block);
      return mat.template block<NV,NV>(row_id,col_id);
    }
    
    template<typename D>
    static typename BlockReturn<D>::Type
    block(Eigen::MatrixBase<D> & mat,
          typename Eigen::DenseBase<D>::Index row_id,
          typename Eigen::DenseBase<D>::Index col_id,
          typename Eigen::DenseBase<D>::Index row_size_block = NV,
          typename Eigen::DenseBase<D>::Index col_size_block = NV)
    {
      PINOCCHIO_UNUSED_VARIABLE(row_size_block);
      PINOCCHIO_UNUSED_VARIABLE(col_size_block);
      return mat.template block<NV,NV>(row_id,col_id);
    }
  };
  
  template<>
  struct SizeDepType<Eigen::Dynamic>
  {
    template<class Mat>
    struct SegmentReturn
    {
      typedef typename Mat::SegmentReturnType Type;
      typedef typename Mat::ConstSegmentReturnType ConstType;
    };
    
    template<typename D>
    static typename SegmentReturn<D>::ConstType
    segment(const Eigen::MatrixBase<D> & mat,
            typename Eigen::DenseBase<D>::Index start,
            typename Eigen::DenseBase<D>::Index size)
    {
      return mat.segment(start,size);
    }
    
    template<typename D>
    static typename SegmentReturn<D>::Type
    segment(Eigen::MatrixBase<D> & mat,
            typename Eigen::DenseBase<D>::Index start,
            typename Eigen::DenseBase<D>::Index size)
    {
      return mat.segment(start,size);
    }
    
    template<class Mat>
    struct ColsReturn
    {
      typedef typename Mat::ColsBlockXpr Type;
      typedef typename Mat::ConstColsBlockXpr ConstType;
    };
    
    template<typename D>
    static typename ColsReturn<D>::ConstType
    middleCols(const Eigen::MatrixBase<D> & mat,
               typename Eigen::DenseBase<D>::Index start,
               typename Eigen::DenseBase<D>::Index size)
    {
      return mat.middleCols(start,size);
    }
    
    template<typename D>
    static typename ColsReturn<D>::Type
    middleCols(Eigen::MatrixBase<D> & mat,
               typename Eigen::DenseBase<D>::Index start,
               typename Eigen::DenseBase<D>::Index size)
    {
      return mat.middleCols(start,size);
    }
    
    template<class Mat>
    struct RowsReturn
    {
      typedef typename Mat::RowsBlockXpr Type;
      typedef typename Mat::ConstRowsBlockXpr ConstType;
    };
    
    template<typename D>
    static typename RowsReturn<D>::ConstType
    middleRows(const Eigen::MatrixBase<D> & mat,
               typename Eigen::DenseBase<D>::Index start,
               typename Eigen::DenseBase<D>::Index size)
    {
      return mat.middleRows(start,size);
    }
    
    template<typename D>
    static typename RowsReturn<D>::Type
    middleRows(Eigen::MatrixBase<D> & mat,
               typename Eigen::DenseBase<D>::Index start,
               typename Eigen::DenseBase<D>::Index size)
    {
      return mat.middleRows(start,size);
    }
    
    template<class Mat>
    struct BlockReturn
    {
      typedef Eigen::Block<Mat> Type;
      typedef const Eigen::Block<const Mat> ConstType;
    };
    
    template<typename D>
    static typename BlockReturn<D>::ConstType
    block(const Eigen::MatrixBase<D> & mat,
          typename Eigen::DenseBase<D>::Index row_id,
          typename Eigen::DenseBase<D>::Index col_id,
          typename Eigen::DenseBase<D>::Index row_size_block,
          typename Eigen::DenseBase<D>::Index col_size_block)
    {
      return mat.block(row_id,col_id,row_size_block,col_size_block);
    }
    
    template<typename D>
    static typename BlockReturn<D>::Type
    block(Eigen::MatrixBase<D> & mat,
          typename Eigen::DenseBase<D>::Index row_id,
          typename Eigen::DenseBase<D>::Index col_id,
          typename Eigen::DenseBase<D>::Index row_size_block,
          typename Eigen::DenseBase<D>::Index col_size_block)
    {
      return mat.block(row_id,col_id,row_size_block,col_size_block);
    }
  };
}

#endif // ifndef __pinocchio_math_matrix_block_hpp__
