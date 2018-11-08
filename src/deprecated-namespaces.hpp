//
// Copyright (c) 2018 INRIA
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __pinocchio_deprecated_namespaces_hpp__
#define __pinocchio_deprecated_namespaces_hpp__

#if PINOCCHIO_ENABLE_COMPATIBILITY_WITH_VERSION_1 // do not warn
  #define se3 ::pinocchio
#else
  #define se3 PINOCCHIO_PRAGMA_MESSAGE_CALL("The se3 namespace has been deprecated since version 2.0.0. Please use namespace pinocchio instead.") ::pinocchio
#endif

#endif // ifndef __pinocchio_deprecated_namespaces_hpp__
