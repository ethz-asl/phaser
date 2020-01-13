/***************************************************************************
  **************************************************************************
  
  SOFT: SO(3) Fourier Transforms
  Version 2.0

  Copyright (c) 2003, 2004, 2007 Peter Kostelec, Dan Rockmore
  
  This file is part of SOFT.

  SOFT is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  SOFT is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  See the accompanying LICENSE file for details.
  
  ************************************************************************
  ************************************************************************/

/*

  header file for basic utility functions of SO(3) routines. Similar
  functions are defined in the SpharmonicKit code, but I wanted to change
  the names in the SO(3) case, in the event that the source from both
  packages lives in the same directory (and routines from both are called
  within the same routine, e.g. correlation
*/

/*
  vec_add_so3() - add two vectors together

  vec_add_scalar_so3() - add a scalar to a vector

  vec_mul_so3() - multiply vector by a scalar

  vec_mul_inplace_so3() - multiplies the vector 'data1' by scalar
                in place!!!

  vec_pt_mul_so3() - pointwise-multiply two vectors

  vec_inner_so3() - inner product of two vectors

  transpose_so3() - transpose matrix -> separate real/imaginary arrays!

  totalCoeffs_so3() - how many SO(3) Fourier coefficients

  numCoeffs_so3() - in a full forward SO3 transform, how many coefficients
                    are computed for order m1 and bandwidth bw ?
  howMany_so3() - how many coeffs computed at orders m1, m2 and bandwidth bw?

  sampLoc_so3() -  At order m1, m2, bandwidth bw, where in the 3-D,
                   fft'd array does the data necessary for the Wigner-(m1,m2)
		   transform start?
  coefLoc_so3() -  where in the coefficient array I should start placing
                   the coeffs of an order m1, m2 transform
  
  so3CoefLoc() -  Where, in the coefficient array of the
              SO(3) signal of bandwidth bw does the
	      coefficient f_{MM'}^l reside?

  as well as declaring some macros
*/


#ifndef _UTILS_SO3_H
#define _UTILS_SO3_H 1


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif    /* M_PI */

#define MAX(A, B) ((A) > (B) ? (A) : (B))
#define MIN(A, B) ((A) > (B) ? (B) : (A))
#define ABS(A) ((A) > 0 ? (A) : (-A))

/* if m >=0,  k >= 0 */
#define HPP(M, K, BW) ((BW) - MAX((M), (K)))

/* if m >= 0, k < 0 */
#define HPM(M, K, BW) ((BW) - MAX((M), (-K)))

/* if m < 0, k >= 0 */
#define HMP(M, K, BW) ((BW) - MAX((-M), (K)))

/* if m < 0, k < 0 */
#define HMM(M, K, BW) ((BW) - MAX((-M), (-K)))


extern void vec_add_so3( double * ,
			 double * ,
			 double * ,
			 int ) ;

extern void vec_add_scalar_so3( double ,
				double * ,
				double * ,
				int ) ;

extern void vec_mul_so3( double ,
			 double * ,
			 double * ,
			 int ) ;

extern void vec_mul_inplace_so3( double ,
				 double * ,
				 int ) ;

extern void vec_pt_mul_so3( double * ,
			    double * ,
			    double * ,
			    int ) ;

extern double vec_inner_so3( double * ,
			     double * ,
			     int ) ;

extern void transpose_so3( double * ,
			   double * ,
			   int ,
			   int ) ;

extern int totalCoeffs_so3( int ) ;

extern int numCoeffs_so3( int ,
			  int ) ;

extern int howMany_so3( int ,
			int ,
			int ) ;
extern int sampLoc_so3( int ,
			int ,
			int ) ;

extern int coefLoc_so3( int ,
			int ,
			int ) ;


extern int so3CoefLoc( int ,
		       int ,
		       int ,
		       int ) ;



#endif /* _UTILS_SO3_H */
