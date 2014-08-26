#ifndef __se3_exception_hpp__
#define __se3_exception_hpp__

#include <exception>
#include <string>

namespace se3
{
  class Exception : public std::exception
  {
  public:
    Exception() : message() {}
    Exception(std::string msg) : message(msg) {}
    const char *what() const throw()
    {
      return this->getMessage().c_str();
    }
    ~Exception() throw() {}
    virtual const std::string & getMessage() const { return message; }
    std::string copyMessage() const { return getMessage(); }

  protected:
    std::string message;
   };

} // namespace 

#endif // ifndef __se3_exception_hpp__
