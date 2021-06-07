#ifndef MATH_H_
#define MATH_H_


/**************************************************************************************/
/***************                     Math Section                  ********************/
/**************************************************************************************/

#define RADX    0.017453292519f      // M_PI/180.0f    = 0.017453292519f     
#define RADX10  0.001745329252f      // M_PI/1800.0f   = 0.001745329252f
#define RADX100 0.000174532925f      // M_PI/18000.0f  = 0.0001745329252f 

#ifndef cosf(x)
  #define cosf(x) cos(x)
#endif

#ifndef sinf(x)
  #define sinf(x) sin(x)
#endif

float sin_approx(int16_t angle);
float cos_approx(int16_t angle);


float InvSqrt (float x);
int16_t _atan2(int32_t y, int32_t x);

int32_t mul(int16_t a, int16_t b);

#endif /* MATH_H_ */
 
