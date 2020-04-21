/*
sse_mathfun_extension.h - zlib license
Written by Tolga Mizrak 2016
Extension of sse_mathfun.h, which is written by Julien Pommier

Based on the corresponding algorithms of the cephes math library

This is written as an extension to sse_mathfun.h instead of modifying it, just
because I didn't want to maintain a modified version of the original library.
This way switching to a newer version of the library won't be a hassle.

Note that non SSE2 implementations of tan_ps, atan_ps, cot_ps and atan2_ps are
not implemented yet. As such, currently you need to #define USE_SSE2 to compile.

With tan_ps, cot_ps you get good precision on input ranges that are further away
from the domain borders (-PI/2, PI/2 for tan and 0, 1 for cot). See the results
on the deviations for these functions on my machine: checking tan on [-0.25*Pi,
0.25*Pi] max deviation from tanf(x): 1.19209e-07 at 0.250000006957*Pi, max
deviation from cephes_tan(x): 5.96046e-08
   ->> precision OK for the tan_ps <<-

checking tan on [-0.49*Pi, 0.49*Pi]
max deviation from tanf(x): 3.8147e-06 at -0.490000009841*Pi, max deviation from
cephes_tan(x): 9.53674e-07
   ->> precision OK for the tan_ps <<-

checking cot on [0.2*Pi, 0.7*Pi]
max deviation from cotf(x): 1.19209e-07 at 0.204303119606*Pi, max deviation from
cephes_cot(x): 1.19209e-07
   ->> precision OK for the cot_ps <<-

checking cot on [0.01*Pi, 0.99*Pi]
max deviation from cotf(x): 3.8147e-06 at 0.987876517942*Pi, max deviation from
cephes_cot(x): 9.53674e-07
   ->> precision OK for the cot_ps <<-

With atan_ps and atan2_ps you get pretty good precision, atan_ps max deviation
is < 2e-7 and atan2_ps max deviation is < 2.5e-7
*/

/* Copyright (C) 2016 Tolga Mizrak

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

#ifndef PHASER_PRE_COMMON_SSE_MATHFUN_EXTENSION_H_
#define PHASER_PRE_COMMON_SSE_MATHFUN_EXTENSION_H_

#ifndef USE_SSE2
#error sse1 & mmx version not implemented
#endif

#include "phaser_pre/common/sse-mathfun.h"

namespace preproc {

v4sf tancot_ps(v4sf x, int cotFlag);
v4sf tan_ps(v4sf x);
v4sf cot_ps(v4sf x);
v4sf atan_ps(v4sf x);
v4sf atan2_ps(v4sf y, v4sf x);

float sqrt_ps(float x);
float rsqrt_ps(float x);
float atan2_ref(float y, float x);

}  // namespace preproc

#endif  // PHASER_PRE_COMMON_SSE_MATHFUN_EXTENSION_H_
