#
# Copyright (c) 2016-2019 CNRS INRIA
#
# This file is part of Pinocchio
# pinocchio is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
# pinocchio is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# pinocchio If not, see
# <http://www.gnu.org/licenses/>.

ADD_SUBDIRECTORY(python)
IF(BUILD_PYTHON_INTERFACE)
  SET(PYWRAP ${PYWRAP} PARENT_SCOPE)
ENDIF(BUILD_PYTHON_INTERFACE)
