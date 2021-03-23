//
// Copyright (c) 2017-2020 CNRS INRIA
//

#ifndef __pinocchio_serialization_eigen_matrix_hpp__
#define __pinocchio_serialization_eigen_matrix_hpp__

#include <Eigen/Dense>
#include "pinocchio/math/tensor.hpp"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/array.hpp>

namespace boost
{
  namespace serialization
  {
    
    template <class Archive, typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
    void save(Archive & ar, const Eigen::Matrix<Scalar,Rows,Cols,Options,MaxRows,MaxCols> & m, const unsigned int /*version*/)
    {
      Eigen::DenseIndex rows(m.rows()), cols(m.cols());
      if (Rows == Eigen::Dynamic)
        ar & BOOST_SERIALIZATION_NVP(rows);
      if (Cols == Eigen::Dynamic)
        ar & BOOST_SERIALIZATION_NVP(cols);
      ar & make_nvp("data",make_array(m.data(), (size_t)m.size()));
    }
    
    template <class Archive, typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
    void load(Archive & ar, Eigen::Matrix<Scalar,Rows,Cols,Options,MaxRows,MaxCols> & m, const unsigned int /*version*/)
    {
      Eigen::DenseIndex rows = Rows, cols = Cols;
      if (Rows == Eigen::Dynamic)
        ar >> BOOST_SERIALIZATION_NVP(rows);
      if (Cols == Eigen::Dynamic)
        ar >> BOOST_SERIALIZATION_NVP(cols);
      m.resize(rows,cols);
      ar >> make_nvp("data",make_array(m.data(), (size_t)m.size()));
    }
    
    template <class Archive, typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
    void serialize(Archive & ar, Eigen::Matrix<Scalar,Rows,Cols,Options,MaxRows,MaxCols> & m, const unsigned int version)
    {
      split_free(ar,m,version);
    }
  
    template <class Archive, typename PlainObjectBase, int MapOptions, typename StrideType>
    void save(Archive & ar, const Eigen::Map<PlainObjectBase,MapOptions,StrideType> & m, const unsigned int /*version*/)
    {
      Eigen::DenseIndex rows(m.rows()), cols(m.cols());
      if (PlainObjectBase::RowsAtCompileTime == Eigen::Dynamic)
        ar & BOOST_SERIALIZATION_NVP(rows);
      if (PlainObjectBase::ColsAtCompileTime == Eigen::Dynamic)
        ar & BOOST_SERIALIZATION_NVP(cols);
      ar & make_nvp("data",make_array(m.data(), (size_t)m.size()));
    }
  
    template <class Archive, typename PlainObjectBase, int MapOptions, typename StrideType>
    void load(Archive & ar, Eigen::Map<PlainObjectBase,MapOptions,StrideType> & m, const unsigned int /*version*/)
    {
      Eigen::DenseIndex rows = PlainObjectBase::RowsAtCompileTime, cols = PlainObjectBase::ColsAtCompileTime;
      if (PlainObjectBase::RowsAtCompileTime == Eigen::Dynamic)
        ar >> BOOST_SERIALIZATION_NVP(rows);
      if (PlainObjectBase::ColsAtCompileTime == Eigen::Dynamic)
        ar >> BOOST_SERIALIZATION_NVP(cols);
      m.resize(rows,cols);
      ar >> make_nvp("data",make_array(m.data(), (size_t)m.size()));
    }
  
    template <class Archive, typename PlainObjectBase, int MapOptions, typename StrideType>
    void serialize(Archive & ar, Eigen::Map<PlainObjectBase,MapOptions,StrideType> & m, const unsigned int version)
    {
      split_free(ar,m,version);
    }
  
#if !defined(PINOCCHIO_WITH_EIGEN_TENSOR_MODULE) && ((__cplusplus <= 199711L && EIGEN_COMP_MSVC < 1900) || defined(__CUDACC__) || defined(EIGEN_AVOID_STL_ARRAY))
    template <class Archive, typename _IndexType, std::size_t _NumIndices>
    void save(Archive & ar, const Eigen::array<_IndexType,_NumIndices> & a, const unsigned int /*version*/)
    {
      ar & make_nvp("array",make_array(&a.front(),_NumIndices));
    }
  
    template <class Archive, typename _IndexType, std::size_t _NumIndices>
    void load(Archive & ar, Eigen::array<_IndexType,_NumIndices> & a, const unsigned int /*version*/)
    {
      ar >> make_nvp("array",make_array(&a.front(),_NumIndices));
    }
  
    template <class Archive, typename _IndexType, std::size_t _NumIndices>
    void serialize(Archive & ar, Eigen::array<_IndexType,_NumIndices> & a, const unsigned int version)
    {
      split_free(ar,a,version);
    }
#else
  template <class Archive, class T, std::size_t N>
  void save(Archive& ar, const std::array<T,N> & a, const unsigned int version)
  {
    typedef std::array<T,N> Array;
    serialize(ar,const_cast<Array&>(a),version);
  }
  
  template <class Archive, class T, std::size_t N>
  void load(Archive& ar, std::array<T,N> & a, const unsigned int version)
  {
    serialize(ar,a,version);
  }
#endif
  
#ifdef PINOCCHIO_WITH_EIGEN_TENSOR_MODULE
  
    template <class Archive, typename _IndexType, int _NumIndices>
    void save(Archive & ar, const Eigen::DSizes<_IndexType,_NumIndices> & ds, const unsigned int version)
    {
      save(ar,static_cast<const Eigen::array<_IndexType,_NumIndices> &>(ds),version);
    }
  
    template <class Archive, typename _IndexType, int _NumIndices>
    void load(Archive & ar, Eigen::DSizes<_IndexType,_NumIndices> & ds, const unsigned int version)
    {
      load(ar,static_cast<Eigen::array<_IndexType,_NumIndices> &>(ds),version);
    }
  
    template <class Archive, typename _IndexType, int _NumIndices>
    void serialize(Archive & ar, Eigen::DSizes<_IndexType,_NumIndices> & ds, const unsigned int version)
    {
      split_free(ar,static_cast<Eigen::array<_IndexType,_NumIndices> &>(ds),version);
    }
  
#endif
  
    template <class Archive, typename _Scalar, int _NumIndices, int _Options, typename _IndexType>
    void save(Archive & ar, const ::pinocchio::Tensor<_Scalar,_NumIndices,_Options,_IndexType> & t, const unsigned int /*version*/)
    {
      typedef ::pinocchio::Tensor<_Scalar,_NumIndices,_Options,_IndexType> Tensor;
      const typename Tensor::Dimensions & dimensions = t.dimensions();
      
      ar & BOOST_SERIALIZATION_NVP(dimensions);
      ar & make_nvp("data",make_array(t.data(), (size_t)t.size()));
    }
    
    template <class Archive, typename _Scalar, int _NumIndices, int _Options, typename _IndexType>
    void load(Archive & ar, ::pinocchio::Tensor<_Scalar,_NumIndices,_Options,_IndexType> & t, const unsigned int /*version*/)
    {
      typedef ::pinocchio::Tensor<_Scalar,_NumIndices,_Options,_IndexType> Tensor;
      typename Tensor::Dimensions dimensions;
      
      ar >> BOOST_SERIALIZATION_NVP(dimensions);
      t.resize(dimensions);
      
      ar >> make_nvp("data",make_array(t.data(), (size_t)t.size()));
    }
    
    template <class Archive, typename _Scalar, int _NumIndices, int _Options, typename _IndexType>
    void serialize(Archive & ar, ::pinocchio::Tensor<_Scalar,_NumIndices,_Options,_IndexType> & t, const unsigned int version)
    {
      split_free(ar,t,version);
    }
    
  }
}

#endif // ifndef __pinocchio_serialization_eigen_matrix_hpp__
