//
// Copyright (c) 2020 LAAS-CNRS
//

// modified from https://gist.github.com/rudolfovich/f250900f1a833e715260a66c87369d15

#ifndef __pinocchio_utils_file_io_hpp__
#define __pinocchio_utils_file_io_hpp__

#include <string>
#include <fstream>
#include <sstream>

namespace pinocchio
{
  class CsvStream
  {
    std::ofstream fs_;
    bool is_first_;
    const std::string separator_;
    const std::string escape_seq_;
    const std::string special_chars_;

  public:
    CsvStream(const std::string filename, const std::string separator = ",")
    : fs_()
    , is_first_(true)
    , separator_(separator)
    , escape_seq_("\"")
    , special_chars_("\"")
    {
      fs_.exceptions(std::ios::failbit | std::ios::badbit);
      fs_.open(filename.c_str());
    }

    ~CsvStream()
    {
      flush();
      fs_.close();
    }

    void flush()
    {
      fs_.flush();
    }

    inline static CsvStream & endl(CsvStream & file)
    {
      file.endrow();
      return file;
    }

    void endrow()
    {
      fs_ << std::endl;
      is_first_ = true;
    }

    CsvStream & operator<<(CsvStream & (*val)(CsvStream &))
    {
      return val(*this);
    }

    CsvStream & operator<<(const char * val)
    {
      return write(escape(val));
    }

    CsvStream & operator<<(const std::string & val)
    {
      return write(escape(val));
    }

    template<typename T>
    CsvStream & operator<<(const T & val)
    {
      return write(val);
    }

  private:
    template<typename T>
    CsvStream & write(const T & val)
    {
      if (!is_first_)
      {
        fs_ << separator_;
      }
      else
      {
        is_first_ = false;
      }
      fs_ << val;
      return *this;
    }

    std::string escape(const std::string & val)
    {
      std::ostringstream result;
      result << '"';
      std::string::size_type to, from = 0u, len = val.length();
      while (from < len && std::string::npos != (to = val.find_first_of(special_chars_, from)))
      {
        result << val.substr(from, to - from) << escape_seq_ << val[to];
        from = to + 1;
      }
      result << val.substr(from) << '"';
      return result.str();
    }
  };
} // namespace pinocchio

#endif
