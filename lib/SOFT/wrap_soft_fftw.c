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

#include <stdio.h>
#include <stdlib.h>

#include "fftw3.h"
#include "soft/utils_so3.h"
#include "soft/makeweights.h"
#include "soft/soft_fftw.h"

/***********

 Forward_SO3_Naive_fftw_W: wrapped version of Forward_SO3_Naive_fftw

 - given an input signal, will compute its SO(3) Fourier coefficients

 bw - bandwidth of input

 signal: fftw_complex ptr to real/imaginary (interleaved) parts
         of the input signal; for bandwidth bw, is a pointer
         to a fftw_complex array  of size (2*bw)^3

 coeffs: fftw_complex ptr to an array of size (4*bw^3 - bw)/3;
         will contain the real/imaginary (interleaved) parts
         of the computed Fourier coefficients

 flag: = 0 -> input samples are COMPLEX numbers
       = 1 -> input samples are double (in which case, basically,
              the signal array is interleaved with 0s)


 - read soft_fx.pdf (included within this distribution) for how
   the function samples and coefficients are arranged

*************/


void Forward_SO3_Naive_fftw_W( int bw,
			       fftw_complex *signal,
			       fftw_complex *coeffs,
			       int flag )
{
  int n, n3 ;
  fftw_complex *workspace_cx ;
  fftw_complex *workspace_cx2 ;
  fftw_plan p1 ;
  int na[2], inembed[2], onembed[2] ;
  int rank, howmany, istride, idist, ostride, odist ;
  double *workspace_re ;
  double *weights ;

  n = 2 * bw ;
  n3 = n * n * n ;

  /* now for LOTS OF workspace */
  workspace_cx = fftw_malloc(sizeof( fftw_complex ) * n3 ) ;
  workspace_cx2 = fftw_malloc(sizeof( fftw_complex ) * n3 ) ;
  workspace_re = ( double * ) malloc(sizeof( double ) *
				     ( 24 * bw + 2 * bw * bw) );

  /* space for weights */
  weights = (double *) malloc(sizeof(double) * ( 2 * bw ));

  /* check if any problems allocating memory */
  if ( ( weights == NULL ) ||
       ( workspace_cx == NULL ) ||
       ( workspace_cx2 == NULL ) || ( workspace_re == NULL ) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }

  /* create the plans */
  howmany = n*n ;
  idist = n ;
  odist = n ;
  rank = 2 ;
  inembed[0] = n ;
  inembed[1] = n*n ;
  onembed[0] = n ;
  onembed[1] = n*n ;
  istride = 1 ;
  ostride = 1 ;
  na[0] = 1 ;
  na[1] = n ;

  p1 = fftw_plan_many_dft( rank, na, howmany,
			   workspace_cx2, inembed,
			   istride, idist,
			   workspace_cx, onembed,
			   ostride, odist,
			   FFTW_BACKWARD, FFTW_MEASURE );

  /* make the weights */
  makeweights2( bw, weights );

  /* now do the forward transform */
  Forward_SO3_Naive_fftw( bw,
			  signal,
			  coeffs,
			  workspace_cx,
			  workspace_cx2,
			  workspace_re,
			  weights,
			  &p1,
			  flag );

  /* destroy fftw plan */
  fftw_destroy_plan( p1 );

  /* free up memory (and there's lots of it) */
  free( weights );
  free( workspace_re );
  fftw_free( workspace_cx2 );
  fftw_free( workspace_cx );
}


/*********************************************************/
/*********************************************************/

/***********

 Inverse_SO3_Naive_fftw_W: wrapped version of Inverse_SO3_Naive_fftw

 - given the Fourier coefficients of a function defined on
   SO(3), perform the inverse transform to obtain the samples

 bw - bandwidth

 coeffs: fftw_complex ptr to input array, of size (4*bw^3 - bw)/3,
         containing the real/imaginary (interleaved) parts of
	 the Fourier coefficients

 signal: fftw_complex ptr to an array, of size (2*bw)^3; will
         contain the real/imaginary (interleaved) parts of the
         computed function samples


 flag: = 0 -> output samples are COMPLEX numbers
       = 1 -> output samples are double (in which case, basically,
             signal will be interleaved with 0s)

 - read soft_fx.pdf (included within this distribution) for how
   the function samples and coefficients are arranged

*************/

void Inverse_SO3_Naive_fftw_W( int bw,
			       fftw_complex *coeffs,
			       fftw_complex *signal,
			       int flag )
{
  int n, n3 ;
  fftw_complex *workspace_cx ;
  fftw_complex *workspace_cx2 ;
  fftw_plan p1 ;
  int na[2], inembed[2], onembed[2] ;
  int rank, howmany, istride, idist, ostride, odist ;
  double *workspace_re ;

  n = 2 * bw ;
  n3 = n * n * n ;

  /* now for LOTS OF workspace */
  workspace_cx = fftw_malloc(sizeof( fftw_complex ) * n3 ) ;
  workspace_cx2 = fftw_malloc(sizeof( fftw_complex ) * n ) ;
  workspace_re = ( double * ) malloc(sizeof( double ) *
				     ( 24 * bw + 2 * bw * bw) );

  /* check if any problems allocating memory */
  if ( ( workspace_cx == NULL ) ||
       ( workspace_cx2 == NULL ) || ( workspace_re == NULL ) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }


  /* create the plans */
  howmany = n*n ;
  idist = n ;
  odist = n ;
  rank = 2 ;
  inembed[0] = n ;
  inembed[1] = n*n ;
  onembed[0] = n ;
  onembed[1] = n*n ;
  istride = 1 ;
  ostride = 1 ;
  na[0] = 1 ;
  na[1] = n ;

  p1 = fftw_plan_many_dft( rank, na, howmany,
			   workspace_cx, inembed,
			   istride, idist,
			   signal, onembed,
			   ostride, odist,
			   FFTW_FORWARD, FFTW_MEASURE );

  /* now do inverse transform */
  Inverse_SO3_Naive_fftw( bw,
			  coeffs,
			  signal,
			  workspace_cx,
			  workspace_cx2,
			  workspace_re,
			  &p1,
			  flag ) ;

  /* destroy fftw plan */
  fftw_destroy_plan( p1 );

  /* free up memory (and there's lots of it) */
  free( workspace_re );
  fftw_free( workspace_cx2 );
  fftw_free( workspace_cx );

}
