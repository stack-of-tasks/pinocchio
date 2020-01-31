//
// Copyright (c) 2017-2020 CNRS INRIA
//

/*
 Code adapted from: https://gist.githubusercontent.com/mtao/5798888/raw/5be9fa9b66336c166dba3a92c0e5b69ffdb81501/eigen_boost_serialization.hpp
 Copyright (c) 2015 Michael Tao
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 */

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
    
    template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void save(Archive & ar, const Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int /*version*/)
    {
      Eigen::DenseIndex rows(m.rows()), cols(m.cols());
      ar & BOOST_SERIALIZATION_NVP(rows);
      ar & BOOST_SERIALIZATION_NVP(cols);
      ar & make_nvp("data",make_array(m.data(), (size_t)m.size()));
    }
    
    template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void load(Archive & ar, Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int /*version*/)
    {
      Eigen::DenseIndex rows,cols;
      ar >> BOOST_SERIALIZATION_NVP(rows);
      ar >> BOOST_SERIALIZATION_NVP(cols);
      m.resize(rows,cols);
//      if(m.size() > 0)
        ar >> make_nvp("data",make_array(m.data(), (size_t)m.size()));
    }
    
    template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    void serialize(Archive & ar, Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int version)
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
