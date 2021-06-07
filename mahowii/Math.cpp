#include "Arduino.h"
#include "Math.h"

/**************************************************************************************************/
/***************  Math Section - Useful trigonometric approximation functions  ********************/
/**************************************************************************************************/

/********* Useful trigonometric approximation functions *********/
/* Test result here http://forum.rcdesign.ru/f123/thread305721.html#post4032762 */

// faster than default sin for 84%
float _sin (float x) {
  x = x * 0.31831f;
  float y = x - x * abs(x);
  return y * (3.1f + 3.6f * abs(y));
}

// faster than default cos for 23%
float _cos (float x) {
  x = x * 0.31831f + 0.5f;
  float z = (x + 25165824.0f);
  x = x - (z - 25165824.0f);
  float y = x - x * abs(x);
  return y * (3.1f + 3.6f * abs(y));
}


float _atan(float x) {
  uint32_t ux_s  = 0x80000000 & (uint32_t &)x;
  float bx_a = ::fabs( 0.596227f * x );
  float num = bx_a + x * x;
  float atan_1q = num / ( 1.f + bx_a + num );
  uint32_t atan_2q = ux_s | (uint32_t &)atan_1q;
  return (float &)atan_2q * 1.5708f;
}


//return angle , unit: 1/10 degree
int16_t _atan2(int32_t y, int32_t x){
  float z = y;
  int16_t a;
  uint8_t c;
  c = abs(y) < abs(x);
  if ( c ) {z = z / x;} else {z = x / z;}
  a = 2046.43 * (z / (3.5714 +  z * z));
  if ( c ){
   if (x<0) {
     if (y<0) a -= 1800;
     else a += 1800;
   }
  } else {
    a = 900 - a;
    if (y<0) a -= 1800;
  }
  return a;
}

// prev multiwii version
int16_t _atan2_v2(int32_t y, int32_t x) {
  float z = (float)y / x;
  int16_t a;
  if ( abs(y) < abs(x) ){
    a = 573 * z / (1.0f + 0.28f * z * z);
    if (x<0) {
      if (y<0) a -= 1800;
      else a += 1800;
    }
  }
  else {
    a = 900 - 573 * z / (z * z + 0.28f);
    if (y<0) a -= 1800;
  }
  return a;
}


// 1/sqrt(x)
float isqrt(float y) {
  float x2 = y * 0.5f;
  long i = * ( long * ) &y;    //evil floating point bit level hacking
  i = 0x5f3759df - ( i >> 1 ); //what the fuck?
  y = * ( float * ) &i;
  y = y * ( 1.5f - ( x2 * y * y ) );
  return y;
}


float InvSqrt (float x){
  union{
    int32_t i;
    float   f;
  } conv;
  conv.f = x;
  conv.i = 0x5f1ffff9 - (conv.i >> 1);
  return conv.f * (1.68191409f - 0.703952253f * x * conv.f * conv.f);
}

// 1/sqrt(x) multiwii version
// todo: need compare with isqrt() for performance
float InvSqrt_v2 (float x){
  union{
    int32_t i;
    float   f;
  }
  conv;
  conv.f = x;
  conv.i = 0x5f3759df - (conv.i >> 1);
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}



static const uint16_t pgm_sinLUT[91] PROGMEM = { 0, 17, 35, 52, 70, 87, 105, 122, 139, 156, 174, 191, 208, 225, 242, 259, 276, 292, 309, 326, 342, 358, 375,
        391, 407, 423, 438, 454, 469, 485, 500, 515, 530, 545, 559, 574, 588, 602, 616, 629, 643, 656, 669, 682, 695, 707, 719, 731, 743, 755, 766, 777, 788,
        799, 809, 819, 829, 839, 848, 857, 866, 875, 883, 891, 899, 906, 914, 921, 927, 934, 940, 946, 951, 956, 961, 966, 970, 974, 978, 982, 985, 988, 990,
        993, 995, 996, 998, 999, 999, 1000, 1000 };

// !!! precision > 0.0009
// performance = 208%
// faster than default sin for 108%
// angle in multiple of 0.1 degree    180 deg = 1800
float sin_approx(int16_t angle) {
    int8_t m, n;
    int16_t sin_value;

    if (angle < 0) {
        m = -1;
        angle = -angle;
    } else {
        m = 1;
    }

    // 0 - 360 only
    if (abs(angle) >= 3600) {
        angle %= 3600;
    }

    // check quadrant
    if (angle <= 900) {
        n = 1; // first quadrant
    } else if ((angle > 900) && (angle <= 1800)) {
        angle = 1800 - angle;
        n = 1;  // second quadrant
    } else if ((angle > 1800) && (angle <= 2700)) {
        angle = angle - 1800;
        n = -1; // third quadrant
    } else {
        angle = 3600 - angle;
        n = -1;
    }        // fourth quadrant

    if (angle < 105) { // for angles < 10.5 degree error will be less than 0.001 comparing to result of sin() function
        //return (angle * m * n) * RADX10;
        return ((float) (angle * m * n)) * RADX10;
    }

    // get lookup value
    sin_value = pgm_read_word(&pgm_sinLUT[angle / 10]);

    // calculate sinus value
    return (float) (sin_value * m * n) / 1000;
}

// !!! precision > 0.0009
// performance = 201%
// faster than default cos for 101%
// angle in multiple of 0.1 degree    180 deg = 1800
float cos_approx(int16_t angle) {
    return (sin_approx(900 - angle));
}



// signed16 * signed16
// 22 cycles
// http://mekonik.wordpress.com/2009/03/18/arduino-avr-gcc-multiplication/
#define MultiS16X16to32(longRes, intIn1, intIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %A2 \n\t" \
"movw %A0, r0 \n\t" \
"muls %B1, %B2 \n\t" \
"movw %C0, r0 \n\t" \
"mulsu %B2, %A1 \n\t" \
"sbc %D0, r26 \n\t" \
"add %B0, r0 \n\t" \
"adc %C0, r1 \n\t" \
"adc %D0, r26 \n\t" \
"mulsu %B1, %A2 \n\t" \
"sbc %D0, r26 \n\t" \
"add %B0, r0 \n\t" \
"adc %C0, r1 \n\t" \
"adc %D0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (longRes) \
: \
"a" (intIn1), \
"a" (intIn2) \
: \
"r26" \
)

int32_t  __attribute__ ((noinline)) mul(int16_t a, int16_t b) {
  int32_t r;
  MultiS16X16to32(r, a, b);
  //r = (int32_t)a*b; without asm requirement
  return r;
}

