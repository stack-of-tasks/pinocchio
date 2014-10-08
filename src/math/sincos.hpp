#ifndef __math_sincos_hpp__
#define __math_sincos_hpp__

#include <cmath>

#ifdef __linux__
#define SINCOS sincos
#elif __APPLE__
#define SINCOS __sincos
#endif

#endif //#ifndef __math_sincos_hpp__
