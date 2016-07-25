//
// Copyright (c) 2015 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __se3_python_python_hpp__
#define __se3_python_python_hpp__

namespace se3
{
  namespace python
  {
    void exposeSE3();
    void exposeForce();
    void exposeMotion();
    void exposeInertia();
    void exposeJoints();
    void exposeExplog();

    void exposeModel();
    void exposeAlgorithms();
    void exposeParsers();

  }} // namespace se3::python

#endif // ifndef __se3_python_python_hpp__

