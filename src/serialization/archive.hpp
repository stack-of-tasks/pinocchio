//
// Copyright (c) 2017-2019 CNRS INRIA
//

#ifndef __pinocchio_serialization_archive_hpp__
#define __pinocchio_serialization_archive_hpp__

#include "pinocchio/serialization/fwd.hpp"

#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

/* undo some macro defs in pyport.h */
#if defined(_PY_PORT_CTYPE_UTF8_ISSUE)
#undef isalnum
#undef isalpha
#undef islower
#undef isspace
#undef isupper
#undef tolower
#undef toupper
#endif

// Handle NAN inside TXT or XML archives
#include <boost/math/special_functions/nonfinite_num_facets.hpp>

namespace pinocchio
{
  namespace serialization
  {
    
    ///
    /// \brief Loads an object from a TXT file.
    ///
    /// \tparam T Type of the object to deserialize.
    ///
    /// \param[out] object Object in which the loaded data are copied.
    /// \param[in]  filename Name of the file containing the serialized data.
    ///
    template<typename T>
    inline void loadFromText(T & object,
                             const std::string & filename) throw (std::invalid_argument)
    {
      std::ifstream ifs(filename.c_str());
      if(ifs)
      {
        std::locale const new_loc(ifs.getloc(), new boost::math::nonfinite_num_get<char>);
        ifs.imbue(new_loc);
        boost::archive::text_iarchive ia(ifs,boost::archive::no_codecvt);
        ia >> object;
      }
      else
      {
        const std::string exception_message(filename + " does not seem to be a valid file.");
        throw std::invalid_argument(exception_message);
      }
    }
    
    ///
    /// \brief Saves an object inside a TXT file.
    ///
    /// \tparam T Type of the object to deserialize.
    ///
    /// \param[out] object Object in which the loaded data are copied.
    /// \param[in]  filename Name of the file containing the serialized data.
    ///
    template<typename T>
    inline void saveToText(const T & object,
                           const std::string & filename) throw (std::invalid_argument)
    {
      std::ofstream ofs(filename.c_str());
      if(ofs)
      {
        boost::archive::text_oarchive oa(ofs);
        oa & object;
      }
      else
      {
        const std::string exception_message(filename + " does not seem to be a valid file.");
        throw std::invalid_argument(exception_message);
      }
    }
    
    ///
    /// \brief Loads an object from a XML file.
    ///
    /// \tparam T Type of the object to deserialize.
    ///
    /// \param[out] object Object in which the loaded data are copied.
    /// \param[in] filename Name of the file containing the serialized data.
    /// \param[in] tag_name XML Tag for the given object.
    ///
    template<typename T>
    inline void loadFromXML(T & object,
                            const std::string & filename,
                            const std::string & tag_name) throw (std::invalid_argument)
    {
      assert(!tag_name.empty());
      
      std::ifstream ifs(filename.c_str());
      if(ifs)
      {
        std::locale const new_loc(ifs.getloc(), new boost::math::nonfinite_num_get<char>);
        ifs.imbue(new_loc);
        boost::archive::xml_iarchive ia(ifs,boost::archive::no_codecvt);
        ia >> boost::serialization::make_nvp(tag_name.c_str(),object);
      }
      else
      {
        const std::string exception_message(filename + " does not seem to be a valid file.");
        throw std::invalid_argument(exception_message);
      }
    }
    
    ///
    /// \brief Saves an object inside a XML file.
    ///
    /// \tparam T Type of the object to deserialize.
    ///
    /// \param[out] object Object in which the loaded data are copied.
    /// \param[in] filename Name of the file containing the serialized data.
    /// \param[in] tag_name XML Tag for the given object.
    ///
    template<typename T>
    inline void saveToXML(const T & object,
                          const std::string & filename,
                          const std::string & tag_name) throw (std::invalid_argument)
    {
      assert(!tag_name.empty());
      
      std::ofstream ofs(filename.c_str());
      if(ofs)
      {
        boost::archive::xml_oarchive oa(ofs);
        oa & boost::serialization::make_nvp(tag_name.c_str(),object);
      }
      else
      {
        const std::string exception_message(filename + " does not seem to be a valid file.");
        throw std::invalid_argument(exception_message);
      }
    }
    
    ///
    /// \brief Loads an object from a binary file.
    ///
    /// \tparam T Type of the object to deserialize.
    ///
    /// \param[out] object Object in which the loaded data are copied.
    /// \param[in] filename Name of the file containing the serialized data.
    ///
    template<typename T>
    inline void loadFromBinary(T & object,
                               const std::string & filename) throw (std::invalid_argument)
    {
      std::ifstream ifs(filename.c_str());
      if(ifs)
      {
        boost::archive::binary_iarchive ia(ifs);
        ia >> object;
      }
      else
      {
        const std::string exception_message(filename + " does not seem to be a valid file.");
        throw std::invalid_argument(exception_message);
      }
    }
    
    ///
    /// \brief Saves an object inside a binary file.
    ///
    /// \tparam T Type of the object to deserialize.
    ///
    /// \param[out] object Object in which the loaded data are copied.
    /// \param[in] filename Name of the file containing the serialized data.
    ///
    template<typename T>
    void saveToBinary(const T & object,
                      const std::string & filename) throw (std::invalid_argument)
    {
      std::ofstream ofs(filename.c_str());
      if(ofs)
      {
        boost::archive::binary_oarchive oa(ofs);
        oa & object;
      }
      else
      {
        const std::string exception_message(filename + " does not seem to be a valid file.");
        throw std::invalid_argument(exception_message);
      }
    }
    
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_serialization_archive_hpp__
