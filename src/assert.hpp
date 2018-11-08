//
// Copyright (c) 2015 CNRS
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

#ifndef __pinocchio_assert_hpp__
#define __pinocchio_assert_hpp__

namespace pinocchio
{
  namespace internal 
  {

    template<bool condition>
    struct static_assertion
    {
      enum
      {
        THIS_METHOD_SHOULD_NOT_BE_CALLED_ON_DERIVED_CLASS,
        MIXING_JOINT_MODEL_AND_DATA_OF_DIFFERENT_TYPE
      };
    };
    
    template<>
    struct static_assertion<true>
    {
      enum 
	{
	  THE_JOINT_MAX_NV_SIZE_IS_EXCEDEED__INCREASE_MAX_JOINT_NV
	};
    };
  } // namespace internat


} // namespace pinocchio 

#define SE3_STATIC_ASSERT(CONDITION,MSG)			\
  if(pinocchio::internal::static_assertion<bool(CONDITION)>::MSG) {}

#endif // ifndef __pinocchio_assert_hpp__
