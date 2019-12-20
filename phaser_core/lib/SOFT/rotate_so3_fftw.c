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
  functions having to do with rotating functions on the
  sphere by massaging their Fourier coefficients with
  the Wigner D functions

*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <fftw3/fftw3.h>
#include "soft/makeweights.h"
#include "soft/utils_so3.h"
#include "soft/rotate_so3_utils.h"
#include "soft/s2_semi_memo.h"
#include "soft/s2_semi_fly.h"


/*****************************************************************/
/*****************************************************************/
/*
  rotateFctFFTW: given a function defined on the sphere,
             this routine will rotate it. The rotation
	     will be described of the Euler angles
	     alpha, beta, gamma, with
	     
	     0 <= alpha, gamma < 2*pi
	     0 <= beta <= pi
	     
	     Here are order of rotation events:
		   1) rotate by gamma about the z-axis
		   2) rotate by beta about the y-axis
		   3) rotate by alpha about the z-axis.

  arguments:

           bwIn: bandlimit of the inputed function

	   bwOut: bandlimit of the outputed, rotated function

	   degOut: Now, instead of rotating the coefficients of
	           all the degrees 0 <= L < bwIn, for whatever
		   reason you may want to go only "so far" and
		   rotate only through L = degOut. The rotated
		   coefficients for degrees degOut+1 <= L < bwIn
		   will be considered to be 0.

           sigInR: double array of the *real* parts of the signal sampled
	           on the usual 2bwIn * 2bwIn grid on the sphere

	   sigInI: double array of the *imaginary* parts of the signal
	           sampled on the usual 2bwIn * 2bwIn grid on the sphere

           sigOutR: double array of the *real* parts of the rotated signal
	            sampled on the usual 2bwOut * 2bwOut grid on the sphere

	   sigOutI: double array of the *imaginary* parts of the rotated
	            signal sampled on the usual 2bwOut * 2bwOut grid on
		    the sphere

	   alpha, beta, gamma: the Euler angles defining the rotation

	   scratch: double array of size (gulp)
		    14*bw^2 + 48*bw + 2*bwIn
		    where bw = max(bwIn, bwOut)

	   spharmonic_pml_table: 
                  should be a (double **) pointer to
		  the result of a call to Spharmonic_Pml_Table. Because this
		  table is re-used in the inverse transform, and because for
		  timing purposes the computation of the table is not included,
		  it is passed in as an argument.
		  
		  spharmonic_pml_table will be an array of (double *) pointers
		  the array being of length TableSize(m,bwIn)

	   transpose_spharmonic_pml_table:
	          should be the (double **) result of a call
		  to Transpose_Spharmonic_Pml_Table()


  NOTE: Since this routine requires huge amounts of memory, it might
        be more convenient if, instead of using rotateFct, you instead
	use rotateCoefAll on its own, e.g.

	1) use your favourite S^2 program to get the coefficients
	   of the function, and write them to disk

	2) in a separate program, read in the coefficients and then
	   call rotateCoefAll, then write out the massaged coefficients
	   to disk

	3) use your favourite S^2 program to take the inverse transform
	   of the massaged coefficients and hence obtain the rotated
	   function.

	Most of the memory in this function is devoted to doing the forward
	and inverse S^2 transforms, e.g. arrays to contain the precomputed
	DCTs of the Legendre functions.



*/

void rotateFctFFTW( int bwIn, int bwOut, int degOut,
		    double *sigInR, double *sigInI,
		    double *sigOutR, double *sigOutI,
		    double alpha, double beta, double gamma,
		    double *scratch,
		    double **spharmonic_pml_table,
		    double **transpose_spharmonic_pml_table )
{
  double *rcoeffs, *icoeffs, *workspace ;
  double *coefOutR, *coefOutI ;
  double *weights;
  fftw_plan dctPlan, idctPlan, fftPlan, ifftPlan ;
  int rank, howmany_rank ;
  fftw_iodim dims[1], howmany_dims[1];

  
  rcoeffs = scratch ;
  icoeffs = rcoeffs + (bwIn*bwIn) ;
  coefOutR = icoeffs + (bwIn*bwIn) ;
  coefOutI = coefOutR + (bwOut*bwOut) ;
  weights = coefOutI + (bwOut*bwOut) ;
  workspace = weights + (4*bwIn) ;

  memset( coefOutR, 0, sizeof(double) * (bwOut*bwOut) );
  memset( coefOutI, 0, sizeof(double) * (bwOut*bwOut) );

  /* forward DCT plan */
  dctPlan = fftw_plan_r2r_1d( 2*bwIn, weights, workspace,
			      FFTW_REDFT10, FFTW_ESTIMATE ) ;

  /* inverse DCT */
  idctPlan = fftw_plan_r2r_1d( 2*bwOut, weights, workspace,
			       FFTW_REDFT01, FFTW_ESTIMATE );


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
    note that this plan places the output in a transposed array
  */
  rank = 1 ;
  dims[0].n = 2*bwIn ;
  dims[0].is = 1 ;
  dims[0].os = 2*bwIn ;
  howmany_rank = 1 ;
  howmany_dims[0].n = 2*bwIn ;
  howmany_dims[0].is = 2*bwIn ;
  howmany_dims[0].os = 1 ;
  
  /* forward fft */
  /* the plan assumes that sigInI is allocated immediately
     after sigInR, for the guru execution of the plan to work */
  fftPlan = fftw_plan_guru_split_dft( rank, dims,
				      howmany_rank, howmany_dims,
				      sigInR, sigInI,
				      workspace, workspace+(4*bwIn*bwIn),
				      FFTW_ESTIMATE );
  
  /*
    now plan for inverse fft - note that this plans assumes
    that I'm working with a transposed array, e.g. the inputs
    for a length 2*bwOut transform are placed every 2*bwOut apart,
    the output will be consecutive entries in the array
  */
  rank = 1 ;
  dims[0].n = 2*bwOut ;
  dims[0].is = 2*bwOut ;
  dims[0].os = 1 ;
  howmany_rank = 1 ;
  howmany_dims[0].n = 2*bwOut ;
  howmany_dims[0].is = 1 ;
  howmany_dims[0].os = 2*bwOut ;

  /* inverse fft */
  ifftPlan = fftw_plan_guru_split_dft( rank, dims,
				       howmany_rank, howmany_dims,
				       sigOutR, sigOutI,
				       workspace, workspace+(4*bwOut*bwOut),
				       FFTW_ESTIMATE );

  
  /* make the weights for forward transform */
  makeweights( bwIn, weights );

  /* from s2kit10 */
  FST_semi_memo( sigInR, sigInI,
		 rcoeffs, icoeffs,
		 bwIn, spharmonic_pml_table,
		 workspace, 0, bwIn,
		 &dctPlan, &fftPlan,
		 weights );

  /* now massage the coefficients */
  /* note that I'm using the workspace array again */
  rotateCoefAll( bwIn, bwOut, degOut,
		 alpha, beta, gamma,
		 rcoeffs, icoeffs,
		 coefOutR, coefOutI,
		 workspace ) ;


  /* take inverse spherical transform */
  InvFST_semi_memo( coefOutR, coefOutI,
		    sigOutR, sigOutI,
		    bwOut,
		    transpose_spharmonic_pml_table,
		    workspace,
		    0, bwOut,
		    &idctPlan, &ifftPlan );

  /* cleanup */
  fftw_destroy_plan( ifftPlan );
  fftw_destroy_plan( fftPlan );
  fftw_destroy_plan( idctPlan );
  fftw_destroy_plan( dctPlan );
  

  /* and that should be that ... */

}


/*******************************************************/
/*******************************************************/
/*

 rotateFctFFTWS: Just like the above rotateFct, except that
 the interface is simpler, i.e. the user does not have
 allocate scratch space, and the spherical transforms
 are done on the fly, so no precomputing of Legendres
 required.

 If you plan on doing lots and lots of rotations, over and
 over again, you might want to use rotateFctFFTW instead,
 i.e. go through the trouble of precomputing the Legendres.

 rotateFctFFTWS: given a function defined on the sphere,
             this routine will rotate it. The rotation
	     will be described of the Euler angles
	     alpha, beta, gamma, with
	     
	     0 <= alpha, gamma < 2*pi
	     0 <= beta <= pi
	     
	     Here are order of rotation events:
		   1) rotate by gamma about the z-axis
		   2) rotate by beta about the y-axis
		   3) rotate by alpha about the z-axis.

  arguments:

           bwIn: bandlimit of the inputed function

	   bwOut: bandlimit of the outputed, rotated function

	   degOut: Now, instead of rotating the coefficients of
	           all the degrees 0 <= L < bwIn, for whatever
		   reason you may want to go only "so far" and
		   rotate only through L = degOut. The rotated
		   coefficients for degrees degOut+1 <= L < bwIn
		   will be considered to be 0.

           sigInR: double array of the *real* parts of the signal sampled
	           on the usual 2bwIn * 2bwIn grid on the sphere

	   sigInI: double array of the *imaginary* parts of the signal
	           sampled on the usual 2bwIn * 2bwIn grid on the sphere

           sigOutR: double array of the *real* parts of the rotated signal
	            sampled on the usual 2bwOut * 2bwOut grid on the sphere

	   sigOutI: double array of the *imaginary* parts of the rotated
	            signal sampled on the usual 2bwOut * 2bwOut grid on
		    the sphere

	   alpha, beta, gamma: the Euler angles defining the rotation

*/

void rotateFctFFTWS( int bwIn, int bwOut, int degOut,
		     double *sigInR, double *sigInI,
		     double *sigOutR, double *sigOutI,
		     double alpha, double beta, double gamma )
{
  double *scratch, *rcoeffs, *icoeffs, *workspace ;
  double *coefOutR, *coefOutI ;
  double *weights ;
  fftw_plan dctPlan, idctPlan, fftPlan, ifftPlan ;
  int rank, howmany_rank ;
  fftw_iodim dims[1], howmany_dims[1];

  if ( bwOut > bwIn )
    scratch =
      (double *) malloc(sizeof(double)*((14*bwOut*bwOut) + (48*bwOut) + (2*bwIn)));
  else
    scratch =
      (double *) malloc(sizeof(double)*((14*bwIn*bwIn) + (48*bwIn) + (2*bwIn)));
  
  rcoeffs = scratch ;
  icoeffs = rcoeffs + (bwIn*bwIn) ;
  coefOutR = icoeffs + (bwIn*bwIn) ;
  coefOutI = coefOutR + (bwOut*bwOut) ;
  weights = coefOutI + (bwOut*bwOut) ;
  workspace = weights + (4*bwIn) ;


  memset( coefOutR, 0, sizeof(double) * (bwOut*bwOut) );
  memset( coefOutI, 0, sizeof(double) * (bwOut*bwOut) );

  /* forward DCT plan */
  dctPlan = fftw_plan_r2r_1d( 2*bwIn, weights, workspace,
			      FFTW_REDFT10, FFTW_ESTIMATE ) ;

  /* inverse DCT */
  idctPlan = fftw_plan_r2r_1d( 2*bwOut, weights, workspace,
			       FFTW_REDFT01, FFTW_ESTIMATE );


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
    note that this plan places the output in a transposed array
  */
  rank = 1 ;
  dims[0].n = 2*bwIn ;
  dims[0].is = 1 ;
  dims[0].os = 2*bwIn ;
  howmany_rank = 1 ;
  howmany_dims[0].n = 2*bwIn ;
  howmany_dims[0].is = 2*bwIn ;
  howmany_dims[0].os = 1 ;
  
  /* forward fft */
  fftPlan = fftw_plan_guru_split_dft( rank, dims,
				      howmany_rank, howmany_dims,
				      sigInR, sigInI,
				      rcoeffs, rcoeffs+(4*bwIn*bwIn),
				      FFTW_ESTIMATE );
  
  /*
    now plan for inverse fft - note that this plans assumes
    that I'm working with a transposed array, e.g. the inputs
    for a length 2*bwOut transform are placed every 2*bwOut apart,
    the output will be consecutive entries in the array
  */
  rank = 1 ;
  dims[0].n = 2*bwOut ;
  dims[0].is = 2*bwOut ;
  dims[0].os = 1 ;
  howmany_rank = 1 ;
  howmany_dims[0].n = 2*bwOut ;
  howmany_dims[0].is = 1 ;
  howmany_dims[0].os = 2*bwOut ;

  /* inverse fft */
  ifftPlan = fftw_plan_guru_split_dft( rank, dims,
				       howmany_rank, howmany_dims,
				       sigOutR, sigOutI,
				       rcoeffs, rcoeffs+(4*bwOut*bwOut),
				       FFTW_ESTIMATE );

  
  /* make the weights for forward transform */
  makeweights( bwIn, weights );

  /* from s2kit10 */
  FST_semi_fly( sigInR, sigInI,
		rcoeffs, icoeffs,
		bwIn,
		workspace, 0, bwIn,
		&dctPlan, &fftPlan,
		weights );


  /* now massage the coefficients */
  /* note that I'm using the workspace array again */
  rotateCoefAll( bwIn, bwOut, degOut,
		 alpha, beta, gamma,
		 rcoeffs, icoeffs,
		 coefOutR, coefOutI,
		 workspace ) ;


  /* take inverse spherical transform */
  InvFST_semi_fly( coefOutR, coefOutI,
		    sigOutR, sigOutI,
		    bwOut,
		    workspace,
		    0, bwOut,
		    &idctPlan, &ifftPlan );

  /* cleanup */
  fftw_destroy_plan( ifftPlan );
  fftw_destroy_plan( fftPlan );
  fftw_destroy_plan( idctPlan );
  fftw_destroy_plan( dctPlan );

  free( scratch ) ;


  /* and that should be that ... */

}


/*****************************************************************/
/*****************************************************************/
/*
  rotateFctFFTWS_mem: Just like the above rotateFctFFTWS, but a
  little friendlier on the memory. THIS FUNCTION WILL WRITE OVER
  THE ARRAYS CONTAINING THE INPUT SIGNAL!!!

  Also, it's assumed that bwIn == bwOut


  arguments:

           bw: bandlimit of function


	   degOut: Now, instead of rotating the coefficients of
	           all the degrees 0 <= L < bwIn, for whatever
		   reason you may want to go only "so far" and
		   rotate only through L = degOut. The rotated
		   coefficients for degrees degOut+1 <= L < bwIn
		   will be considered to be 0.

           sigR: double array of the *real* parts of the signal sampled
	           on the usual 2bwIn * 2bwIn grid on the sphere;
		 WILL BE WRITTEN OVER WITH THE double PARTS OF THE SAMPLE
		 POINTS OF THE ROTATED SIGNAL

	   sigI: double array of the *imaginary* parts of the signal
	           sampled on the usual 2bwIn * 2bwIn grid on the sphere;
		 WILL BE WRITTEN OVER WITH THE IMAGINARY PARTS OF THE
		 SAMPLE POINTS OF THE ROTATED SIGNAL

           sigOutR: double array of the *real* parts of the rotated signal
	            sampled on the usual 2bwOut * 2bwOut grid on the sphere

	   sigOutI: double array of the *imaginary* parts of the rotated
	            signal sampled on the usual 2bwOut * 2bwOut grid on
		    the sphere

	   alpha, beta, gamma: the Euler angles defining the rotation

	   scratch: double array of size (gulp)
		    10*bwIn^2 + 52*bwIn - 8

	   spharmonic_pml_table: 
                  should be a (double **) pointer to
		  the result of a call to Spharmonic_Pml_Table. Because this
		  table is re-used in the inverse transform, and because for
		  timing purposes the computation of the table is not included,
		  it is passed in as an argument.
		  
		  spharmonic_pml_table will be an array of (double *) pointers
		  the array being of length TableSize(m,bwIn)

	   transpose_spharmonic_pml_table:
	          should be the (double **) result of a call
		  to Transpose_Spharmonic_Pml_Table()


  NOTE: Since this routine requires huge amounts of memory, it might
        be more convenient if, instead of using rotateFct, you instead
	use rotateCoefAll on its own, e.g.

	1) use your favourite S^2 program to get the coefficients
	   of the function, and write them to disk

	2) in a separate program, read in the coefficients and then
	   call rotateCoefAll, then write out the massaged coefficients
	   to disk

	3) use your favourite S^2 program to take the inverse transform
	   of the massaged coefficients and hence obtain the rotated
	   function.

	Most of the memory in this function is devoted to doing the forward
	and inverse S^2 transforms, e.g. arrays to contain the precomputed
	DCTs of the Legendre functions.



*/

void rotateFctFFTWS_mem( int bw, int degOut,
			 double *sigR, double *sigI,
			 double alpha, double beta, double gamma )
{
  double *rcoeffs, *icoeffs;
  double *scratch, *workspace, *weights ;
  fftw_plan dctPlan, idctPlan, fftPlan, ifftPlan ;
  int rank, howmany_rank ;
  fftw_iodim dims[1], howmany_dims[1];

  scratch = (double *) malloc(sizeof(double)*((10*bw*bw) + (52*bw) + (2*bw)));

  rcoeffs = scratch ;
  icoeffs = rcoeffs + (bw*bw) ;
  weights = icoeffs + (bw*bw) ;
  workspace = weights + (4*bw) ;

  /* forward DCT plan */
  dctPlan = fftw_plan_r2r_1d( 2*bw, weights, workspace,
			      FFTW_REDFT10, FFTW_ESTIMATE ) ;

  /* inverse DCT */
  idctPlan = fftw_plan_r2r_1d( 2*bw, weights, workspace,
			       FFTW_REDFT01, FFTW_ESTIMATE );

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
    note that this plan places the output in a transposed array
  */
  rank = 1 ;
  dims[0].n = 2*bw ;
  dims[0].is = 1 ;
  dims[0].os = 2*bw ;
  howmany_rank = 1 ;
  howmany_dims[0].n = 2*bw ;
  howmany_dims[0].is = 2*bw ;
  howmany_dims[0].os = 1 ;
  
  /* forward fft */
  fftPlan = fftw_plan_guru_split_dft( rank, dims,
				      howmany_rank, howmany_dims,
				      sigR, sigI,
				      scratch, scratch+(4*bw*bw),
				      FFTW_ESTIMATE );
  
  /*
    now plan for inverse fft - note that this plans assumes
    that I'm working with a transposed array, e.g. the inputs
    for a length 2*bwOut transform are placed every 2*bwOut apart,
    the output will be consecutive entries in the array
  */
  rank = 1 ;
  dims[0].n = 2*bw ;
  dims[0].is = 2*bw ;
  dims[0].os = 1 ;
  howmany_rank = 1 ;
  howmany_dims[0].n = 2*bw ;
  howmany_dims[0].is = 1 ;
  howmany_dims[0].os = 2*bw ;

  /* inverse fft */
  ifftPlan = fftw_plan_guru_split_dft( rank, dims,
				       howmany_rank, howmany_dims,
				       sigR, sigI,
				       scratch, scratch+(4*bw*bw),
				       FFTW_ESTIMATE );

  
  /* make the weights for forward transform */
  makeweights( bw, weights );


  /* compute spherical coefficients of input signal */
  FST_semi_fly( sigR, sigI,
		rcoeffs, icoeffs,
		bw,
		workspace, 0, bw,
		&dctPlan, &fftPlan,
		weights );

  /* now massage the coefficients */
  /* note that I'm using the workspace array again */
  rotateCoefAll_mem( bw, degOut,
		 alpha, beta, gamma,
		 rcoeffs, icoeffs,
		 workspace ) ;

  /* take inverse spherical transform */
  InvFST_semi_fly( rcoeffs, icoeffs,
		   sigR, sigI,
		   bw,
		   workspace,
		   0, bw,
		   &idctPlan, &ifftPlan );



  /* cleanup */
  fftw_destroy_plan( ifftPlan );
  fftw_destroy_plan( fftPlan );
  fftw_destroy_plan( idctPlan );
  fftw_destroy_plan( dctPlan );

  free( scratch ) ;


  /* and that should be that ... */

}

