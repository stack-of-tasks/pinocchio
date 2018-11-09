//
// Copyright (c) 2015 CNRS
//

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
