// Copyright (c) 2026 INRIA

#ifdef PINOCCHIO_WITH_URDFDOM
  #include <console_bridge/console.h>
#endif

#include "pinocchio/bindings/python-nb/fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

void exposeConsoleBridge(nb::module_ m)
{
#ifdef PINOCCHIO_WITH_URDFDOM
  using LogLevel = ::console_bridge::LogLevel;

  // Suppress verbose URDF parser output by default
  ::console_bridge::setLogLevel(::console_bridge::CONSOLE_BRIDGE_LOG_ERROR);

  nb::enum_<LogLevel>(m, "LogLevel")
    .value("CONSOLE_BRIDGE_LOG_DEBUG", ::console_bridge::CONSOLE_BRIDGE_LOG_DEBUG)
    .value("CONSOLE_BRIDGE_LOG_INFO", ::console_bridge::CONSOLE_BRIDGE_LOG_INFO)
    .value("CONSOLE_BRIDGE_LOG_WARN", ::console_bridge::CONSOLE_BRIDGE_LOG_WARN)
    .value("CONSOLE_BRIDGE_LOG_ERROR", ::console_bridge::CONSOLE_BRIDGE_LOG_ERROR)
    .value("CONSOLE_BRIDGE_LOG_NONE", ::console_bridge::CONSOLE_BRIDGE_LOG_NONE);
#endif
  (void)m;
}

PINOCCHIO_PYTHON_NAMESPACE_END
