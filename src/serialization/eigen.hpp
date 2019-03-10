//
// Copyright (c) 2017-2019 CNRS INRIA
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
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

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
    
  }
}

#endif // ifndef __pinocchio_serialization_eigen_matrix_hpp__
