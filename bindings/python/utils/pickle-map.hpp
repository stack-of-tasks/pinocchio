//
// Copyright (c) 2019-2020 CNRS INRIA
//

#ifndef __pinocchio_python_utils_pickle_map_hpp__
#define __pinocchio_python_utils_pickle_map_hpp__

#include <boost/python.hpp>
#include <boost/python/tuple.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    ///
    /// \brief Create a pickle interface for the std::map and aligned map
    ///
    /// \tparam MapType Map Type to pickle
    ///
    /// \sa PickleVector
    ///
    template<typename MapType>
    struct PickleMap : boost::python::pickle_suite
    {
      static boost::python::tuple getinitargs(const MapType&)
      {
        return boost::python::make_tuple();
      }
           
      static boost::python::tuple getstate(boost::python::object op)
      {
        boost::python::extract<const MapType&> get_map(op);
        if(get_map.check())
        {
          const MapType & map = get_map();
          boost::python::list list;
          for(typename MapType::const_iterator it = map.begin();
              it != map.end();
              ++it)
          {
            list.append(boost::python::make_tuple(it->first,it->second));
          }
          return boost::python::make_tuple(list);
        }
        return boost::python::make_tuple();
      }
      
      static void setstate(bp::object op, bp::tuple tup)
      {
        typedef typename MapType::key_type key_type;
        typedef typename MapType::mapped_type mapped_type;
        
        if(bp::len(tup) > 0)
        {
          bp::extract<MapType&> get_map(op);
          if(get_map.check())
          {
            MapType & map = get_map();
            boost::python::list list = bp::extract<boost::python::list>(tup[0])();
            for(boost::python::ssize_t k = 0; k < boost::python::len(list); ++k)
            {
              boost::python::tuple entry = bp::extract<boost::python::tuple>(list[k])();
              key_type key = bp::extract<key_type>(entry[0])();
              map[key] = bp::extract<mapped_type>(entry[1])();
            }
          }
        }
      }
    };
  }
}

#endif // ifndef __pinocchio_python_utils_pickle_map_hpp__
