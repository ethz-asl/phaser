/* SIMD (SSE1+MMX or SSE2) implementation of sin, cos, exp and log

   Inspired by Intel Approximate Math library, and based on the
   corresponding algorithms of the cephes math library

   The default is to use the SSE1 version. If you define USE_SSE2 the
   the SSE2 intrinsics will be used in place of the MMX intrinsics. Do
   not expect any significant performance improvement with SSE2.
*/

/* Copyright (C) 2007  Julien Pommier

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.

  (this is the zlib license)
*/

#ifndef PHASER_PRE_COMMON_SSE_MATHFUN_H_
#define PHASER_PRE_COMMON_SSE_MATHFUN_H_

#include <xmmintrin.h>

namespace preproc {

#define ALIGN16_BEG
#define ALIGN16_END __attribute__((aligned(16)))

/* __m128 is ugly to write */
typedef __m128 v4sf;  // vector of 4 float (sse1)

#ifdef USE_SSE2
#include <emmintrin.h>
typedef __m128i v4si;  // vector of 4 int (sse2)
#else
typedef __m64 v2si;  // vector of 2 int (mmx)
#endif

/* declare some SSE constants -- why can't I figure a better way to do that? */
#define _PS_CONST(Name, Val)                                   \
  static const ALIGN16_BEG float _ps_##Name[4] ALIGN16_END = { \
      Val, Val, Val, Val}
#define _PI32_CONST(Name, Val)                                 \
  static const ALIGN16_BEG int _pi32_##Name[4] ALIGN16_END = { \
      Val, Val, Val, Val}
#define _PS_CONST_TYPE(Name, Type, Val) \
  static const ALIGN16_BEG Type _ps_##Name[4] ALIGN16_END = {Val, Val, Val, Val}

#ifndef USE_SSE2
typedef union xmm_mm_union {
  __m128 xmm;
  __m64 mm[2];
} xmm_mm_union;

#define COPY_XMM_TO_MM(xmm_, mm0_, mm1_) \
  {                                      \
    xmm_mm_union u;                      \
    u.xmm = xmm_;                        \
    mm0_ = u.mm[0];                      \
    mm1_ = u.mm[1];                      \
  }

#define COPY_MM_TO_XMM(mm0_, mm1_, xmm_) \
  {                                      \
    xmm_mm_union u;                      \
    u.mm[0] = mm0_;                      \
    u.mm[1] = mm1_;                      \
    xmm_ = u.xmm;                        \
  }

#endif  // USE_SSE2

v4sf log_ps(v4sf x);
v4sf exp_ps(v4sf x);
v4sf sin_ps(v4sf x);
v4sf cos_ps(v4sf x);
void sincos_ps(v4sf x, v4sf* s, v4sf* c);

}  // namespace preproc

#endif  // PHASER_PRE_COMMON_SSE_MATHFUN_H_
