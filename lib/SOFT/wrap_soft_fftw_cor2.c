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

#include <math.h>
#include <stdlib.h>

#include "fftw3.h"
#include "soft/makeweights.h"
#include "soft/so3_correlate_fftw.h"
#include "soft/soft_fftw.h"
#include "soft/s2_cospmls.h"
#include "soft/s2_legendreTransforms.h"
#include "soft/s2_semi_memo.h"
#include "soft/wrap_fftw.h"

#define NORM( x ) ( (x[0])*(x[0]) + (x[1])*(x[1]) )

/****************************************

 softFFTWCor2: simple wrapper for correlating two functions defined on the sphere;
              if efficiency is important to you, or want more control,
              e.g. want to correlate lots and lots of times without having
	      reallocate tmp workspace, or change the bandwidth
	      you want to correlate at, or correlate complex-valued
              functions, you should look at

	      test_soft_fftw_correlate2.c

	      as an example of how to do it. softFFTWCor2() is basically
	      test_soft_fftw_correlate2.c turned into a wrapper, with
	      some simplifying assumptions.
	      
  bw: bandwidth of signal and pattern

  isReal: int defining whether or not the signal and pattern are
          strictly real, or interleaved (complex)
          = 1 -> strictly real
          = 0 -> complex/interleaved

  sig: double ptr to SIGNAL function samples;
       for bandwidth bw, then, is a pointer to a
       double array of size (2*bw)^3 + (isReal*(2*bw)^3)

  pat: double ptr to PATTERN function samples
       for bandwidth bw, then, is a pointer to a
       double array of size (2*bw)^3 + (isReal*(2*bw)^3)

  alpha, beta, gamma: ptrs to doubles; at the end of the routine,
               will "contain" the angles alpha, beta, and gamma needed
	       in order to rotate the SIGNAL to match the PATTERN; the
	       order of rotation is:

                   1) rotate by gamma about the z-axis
                   2) rotate by beta about the y-axis
                   3) rotate by alpha about the z-axis.
		   
	       where
             
	           0 <= alpha, gamma < 2*pi
	           0 <= beta <= pi


***********************************/


void softFFTWCor2( int bw,
		   double *sig,
		   double *pat,
		   double *alpha, double *beta, double *gamma,
       double *maxval, double **signal_values, int isReal )
{
  int i ;
  int n, bwIn, bwOut, degLim ;
  fftw_complex *workspace1, *workspace2  ;
  double *workspace3 ;
  double *tmpR, *tmpI ;
  double *sigCoefR, *sigCoefI ;
  double *patCoefR, *patCoefI ;
  fftw_complex *so3Sig, *so3Coef ;
  fftw_plan p1 ;
  int na[2], inembed[2], onembed[2] ;
  int rank, howmany, istride, idist, ostride, odist ;
  int tmp, maxloc, ii, jj, kk ;
  double tmpval;
  double *weights ;
  double *seminaive_naive_tablespace  ;
  double **seminaive_naive_table ;
  fftw_plan dctPlan, fftPlan ;
  int howmany_rank ;
  fftw_iodim dims[1], howmany_dims[1];

  
  bwIn = bw ;
  bwOut = bw ;
  degLim = bw - 1 ;
  n = 2 * bwIn ;

	const int bwOutp3 = bwOut*bwOut*bwOut;
  tmpR = (double *) malloc( sizeof(double) * ( n * n ) );
  tmpI = (double *) malloc( sizeof(double) * ( n * n ) );
  so3Sig = fftw_malloc( sizeof(fftw_complex) * (8*bwOutp3) );
  *signal_values = (double *) malloc( sizeof(double) * (8*bwOutp3) ) ;
  workspace1 = fftw_malloc( sizeof(fftw_complex) * (8*bwOutp3) );
  workspace2 = fftw_malloc( sizeof(fftw_complex) * ((14*bwIn*bwIn) + (48 * bwIn)));
  workspace3 = (double *) malloc( sizeof(double) * (12*n + n*bwIn));
  sigCoefR = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
  sigCoefI = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
  patCoefR = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
  patCoefI = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
  so3Coef = fftw_malloc( sizeof(fftw_complex) * ((4*bwOutp3-bwOut)/3) ) ;
  seminaive_naive_tablespace =
    (double *) malloc(sizeof(double) *
		      (Reduced_Naive_TableSize(bwIn,bwIn) +
		       Reduced_SpharmonicTableSize(bwIn,bwIn)));

  weights = (double *) malloc(sizeof(double) * (4*bwIn));

  /****
       At this point, check to see if all the memory has been
       allocated. If it has not, there's no point in going further.
  ****/

  if ( (seminaive_naive_tablespace == NULL) || (weights == NULL) ||
       (tmpR == NULL) || (tmpI == NULL) ||
       (so3Coef == NULL) ||
       (workspace1 == NULL) || (workspace2 == NULL) ||
       (workspace3 == NULL) ||
       (sigCoefR == NULL) || (sigCoefI == NULL) ||
       (patCoefR == NULL) || (patCoefI == NULL) ||
	   (so3Sig == NULL) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }

  /* create fftw plans for the S^2 transforms */
  /* first for the dct */
  dctPlan = fftw_plan_r2r_1d( 2*bwIn, weights, workspace3,
			      FFTW_REDFT10, FFTW_ESTIMATE ) ;

  /* now for the fft */
  /* 
     IMPORTANT NOTE!!! READ THIS!!!

     Now to make the fft plans.

     Please note that the planning-rigor flag *must be* FFTW_ESTIMATE!
     Why? Well, to try to keep things simple. I am using some of the
     pointers to arrays in rotateFct's arguments in the fftw-planning
     routines. If the planning-rigor is *not* FFTW_ESTIMATE, then
     the arrays will be written over during the planning stage.

     Therefore, unless you are really really sure you know what
     you're doing, keep the rigor as FFTW_ESTIMATE !!!
  */

  /*
    fftw "preamble" ;
    note  that this places in the transposed array
  */

  rank = 1 ;
  dims[0].n = 2*bwIn ;
  dims[0].is = 1 ;
  dims[0].os = 2*bwIn ;
  howmany_rank = 1 ;
  howmany_dims[0].n = 2*bwIn ;
  howmany_dims[0].is = 2*bwIn ;
  howmany_dims[0].os = 1 ;

  fftPlan = fftw_plan_guru_split_dft( rank, dims,
				      howmany_rank, howmany_dims,
				      tmpR, tmpI,
				      (double *) workspace2,
				      (double *) workspace2 + (n*n),
				      FFTW_ESTIMATE );

  /* create plan for inverse SO(3) transform */
  n = 2 * bwOut ;
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
			   workspace1, inembed,
			   istride, idist,
			   so3Sig, onembed,
			   ostride, odist,
			   FFTW_FORWARD, FFTW_ESTIMATE );


  seminaive_naive_table = SemiNaive_Naive_Pml_Table(bwIn, bwIn,
						    seminaive_naive_tablespace,
						    (double *) workspace2);


  /* make quadrature weights for the S^2 transform */
  makeweights( bwIn, weights ) ;

  n = 2 * bwIn ;
  /* load SIGNAL samples into temp array */
  if ( isReal )
    for ( i = 0 ; i < n * n ; i ++ )
      {
	tmpR[i] = sig[i];
	tmpI[i] = 0. ;
      }
  else
    for ( i = 0 ; i < n * n ; i ++ )
      {
	tmpR[i] = sig[2*i];
	tmpI[i] = sig[2*i+1] ;
      }

  /* spherical transform of SIGNAL */
  FST_semi_memo( tmpR, tmpI,
		 sigCoefR, sigCoefI,
		 bwIn, seminaive_naive_table,
		 (double *) workspace2, isReal, bwIn,
		 &dctPlan, &fftPlan,
		 weights );

  /* load PATTERN samples into temp array; note that I'm
     also providing 0s in the imaginary part */
  if ( isReal )
    for (i = 0 ; i < n * n ; i ++ )
      {
	tmpR[i] = pat[i] ;
	tmpI[i] = 0.  ;
      }
  else
    for (i = 0 ; i < n * n ; i ++ )
      {
	tmpR[i] = pat[2*i] ;
	tmpI[i] = pat[2*i+1] ;
      }

  /* spherical transform of PATTERN */
  FST_semi_memo( tmpR, tmpI,
		 patCoefR, patCoefI,
		 bwIn, seminaive_naive_table,
		 (double *) workspace2, isReal, bwIn,
		 &dctPlan, &fftPlan,
		 weights ) ;

  /* all done with the spherical transform, so free up
     some memory before continuing */
  free( seminaive_naive_table ) ;
  free( seminaive_naive_tablespace ) ;


  /* combine coefficients */
  so3CombineCoef_fftw( bwIn, bwOut, degLim,
		       sigCoefR, sigCoefI,
		       patCoefR, patCoefI,
		       so3Coef ) ;

  /* now inverse so(3) */
  Inverse_SO3_Naive_fftw( bwOut,
			  so3Coef,
			  so3Sig,
			  workspace1,
			  workspace2,
			  workspace3,
			  &p1,
			  isReal ) ;

  /* now find max value */
  *maxval = 0.0 ;
  maxloc = 0 ;
  for ( i = 0 ; i < 8*bwOut*bwOut*bwOut ; i ++ ) {
    tmpval = NORM( so3Sig[i] );
		(*signal_values)[i] = tmpval;
    if ( tmpval > *maxval ) {
      *maxval = tmpval;
      maxloc = i ;
    }
  }

  ii = floor( maxloc / (4.*bwOut*bwOut) );
  tmp = maxloc - (ii*4.*bwOut*bwOut);
  jj = floor( tmp / (2.*bwOut) );
  tmp = maxloc - (ii *4*bwOut*bwOut) - jj*(2*bwOut);
  kk = tmp ;

  *alpha = M_PI*jj/((double) bwOut) ;
  *beta =  M_PI*(2*ii+1)/(4.*bwOut) ;
  *gamma = M_PI*kk/((double) bwOut) ;

  /* clean up */

  fftw_destroy_plan( p1 );
  fftw_destroy_plan( fftPlan );
  fftw_destroy_plan( dctPlan );

  free( weights );
  fftw_free( so3Coef ) ;
  free( patCoefI );
  free( patCoefR );
  free( sigCoefI );
  free( sigCoefR );
  free( workspace3 );
  fftw_free( workspace2 );
  fftw_free( workspace1 );
  fftw_free( so3Sig ) ;
  free( tmpI );
  free( tmpR );

}
