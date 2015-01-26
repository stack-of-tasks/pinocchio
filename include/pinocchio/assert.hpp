#ifndef __se3_assert_hpp__
#define __se3_assert_hpp__

namespace se3
{
  namespace internal 
  {

    template<bool condition>
    struct static_assertion {};
    
    template<>
    struct static_assertion<true>
    {
      enum 
	{
	  THE_JOINT_MAX_NV_SIZE_IS_EXCEDEED__INCREASE_MAX_JOINT_NV
	};
    };
  } // namespace internat


} // namespace se3 

#define SE3_STATIC_ASSERT(CONDITION,MSG)			\
  if(se3::internal::static_assertion<bool(CONDITION)>::MSG) {}

#endif // ifndef __se3_assert_hpp__
