//
// Copyright (c) 2020 INRIA
//

#include <boost/python.hpp>
#include "pinocchio/bindings/python/parsers/urdf.hpp"
#include "pinocchio/bindings/python/utils/registration.hpp"

#ifdef PINOCCHIO_WITH_URDFDOM
  #include <console_bridge/console.h>
#endif

namespace pinocchio
{
  namespace python
  {
    void exposeConsoleBridge()
    {
      namespace bp = boost::python;

#ifdef PINOCCHIO_WITH_URDFDOM

      // fix CONSOLE_BRIDGE warning level
      ::console_bridge::setLogLevel(::console_bridge::CONSOLE_BRIDGE_LOG_ERROR);

      typedef ::console_bridge::LogLevel LogLevel;
      if (!register_symbolic_link_to_registered_type<LogLevel>())
      {
        bp::enum_<LogLevel>("LogLevel")
          .value("CONSOLE_BRIDGE_LOG_DEBUG", ::console_bridge::CONSOLE_BRIDGE_LOG_DEBUG)
          .value("CONSOLE_BRIDGE_LOG_INFO", ::console_bridge::CONSOLE_BRIDGE_LOG_INFO)
          .value("CONSOLE_BRIDGE_LOG_WARN", ::console_bridge::CONSOLE_BRIDGE_LOG_WARN)
          .value("CONSOLE_BRIDGE_LOG_ERROR", ::console_bridge::CONSOLE_BRIDGE_LOG_ERROR)
          .value("CONSOLE_BRIDGE_LOG_NONE", ::console_bridge::CONSOLE_BRIDGE_LOG_NONE);
      }

#endif
    }
  } // namespace python
} // namespace pinocchio
