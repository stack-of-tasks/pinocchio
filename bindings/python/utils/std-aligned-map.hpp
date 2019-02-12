//
// Copyright (c) 2019 LAAS-CNRS
//

#ifndef __pinocchio_python_utils_std_aligned_map_hpp__
#define __pinocchio_python_utils_std_aligned_map_hpp__

#include <boost/python.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <string>
#include <map>

namespace pinocchio
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    ///
    /// \brief Expose an std::map from a type given as template argument. Uses the Eigen::aligned_allocater in the value.
    ///
    /// \tparam KeyType, ValType Type to expose as std::map<KeyType, ValType, Eigen::aligned_allocator<KeyType, ValType>>.
    ///
    /// \sa StdAlignedMapPythonVisitor
    ///
    template<class KyeType, class ValType, bool NoProxy = true>
    struct StdAlignedMapPythonVisitor
      : public bp::map_indexing_suite< typename std::map<KyeType, ValType, std::less<KyeType>, Eigen::aligned_allocator<std::pair<const KyeType, ValType> > >, NoProxy >
    {
      static void expose(const std::string & class_name,
                         const std::string & doc_string = "")
      {
        bp::class_< std::map<KyeType, ValType,std::less<KyeType>, Eigen::aligned_allocator<std::pair<const KyeType, ValType> > > >(class_name.c_str(),doc_string.c_str())
        .def(StdAlignedMapPythonVisitor());
      }
    };
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_std_map_hpp__
