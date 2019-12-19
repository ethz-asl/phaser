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

 a somewhat memory-friendly test routine to rotate a spherical function
 by massaging its S^2 Fourier coefficients with Wigner-D functions

 bwIn = bandwidth of input signal
 bwOut = bandwidth of output signal (can up- or down-sample)
 degOut = max degree of spherical harmonic you want to use ( < bwOut )
 alpha, beta, gamma -> the three Euler angles

             0 <= alpha, gamma < 2*pi
             0 <= beta <= pi

 inputSamples -> filename of input samples
 outputSamples -> filename of output (rotated) samples

 isReal = 1: samples are strictly real (so no imaginary parts)
 isReal = 0: samples are complex (so in interleaved format)


 Here are order of rotation events:
  1) rotate by gamma about the z-axis
  2) rotate by beta about the y-axis
  3) rotate by alpha about the z-axis.

 example: test_s2_rotate_fftw bwIn bwOut degOut alpha beta gamma inputSamples outputSamples isReal

 example: test_s2_rotate_fftw 32 16 15 0.37 2.32 4.37 fctIn.dat fctOut.dat 0



 NOTE: Sometimes there is a segmentation fault *after* all the rotating and
 writing out of the output file is complete. I haven't tracked this down yet,
 but I believe it has to do with freeing up the memory associated with doing
 the S^2 transforms ... my array of double pointers are not pointing in the
 right places when I try to free memory. However, the rotation itself is
 correct.

*/

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "soft/csecond.h"
#include "soft/s2_cospmls.h"
#include "soft/rotate_so3_fftw.h"

/***********************

 s2RotateFFTW: wrapper function version of

           test_s2_rotate_fftw.c

 will rotate a possibly complex-valued function defined on the sphere by
 the Euler angles alpha, beta, gamma, where

	           0 <= alpha, gamma < 2*pi
	           0 <= beta <= pi

 and in the following order

                   1) rotate by gamma about the z-axis
                   2) rotate by beta about the y-axis
                   3) rotate by alpha about the z-axis.
		   
 bw: bandwidth of function

 sigIn : ptr to the input signal; if complex, then it's an interleaved
        array of length 2*(2*bw)^2; if strictly real, then it's an
        array of length (2*bw)^2

 sigOut : ptr to the rotated signal; if complex, then it's an interleaved
          array of length 2*(2*bw)^2; if strictly real, then it's an
          array of length (2*bw)^2

 alpha, beta, gamma: doubles specifying the Euler angles.

 isReal: = 1 data is strictly real
         = 0 data is complex (interleaved format)

 NOTE: Sometimes there is a segmentation fault *after* all the rotating and
 writing out of the output file is complete. I haven't tracked this down yet,
 but I believe it has to do with freeing up the memory associated with doing
 the S^2 transforms ... my array of double pointers are not pointing in the
 right places when I try to free memory. However, the rotation itself is
 correct.

****************/

void s2RotateFFTW( int bw,
		   double *sigIn,
		   double *sigOut,
		   double alpha, double beta, double gamma,
		   int isReal)
{
  int i, degOut ;
  double *scratch ;
  double *tmpInR, *tmpInI, *tmpOutR, *tmpOutI ;
  double *seminaive_naive_tablespace ;
  double *trans_seminaive_naive_tablespace;
  double **seminaive_naive_table ;
  double **trans_seminaive_naive_table;

  degOut = bw - 1 ;


  tmpInR = (double *) malloc(sizeof(double)*(4*bw*bw));
  tmpInI = (double *) malloc(sizeof(double)*(4*bw*bw));
  tmpOutR = (double *) malloc(sizeof(double)*(4*bw*bw));
  tmpOutI = (double *) malloc(sizeof(double)*(4*bw*bw));


  scratch = (double *) malloc(sizeof(double)*((14*bw*bw) + (52*bw) + (2*bw)));
    
  seminaive_naive_tablespace =
    (double *) malloc(sizeof(double) *
		      (Reduced_Naive_TableSize(bw,bw) +
		       Reduced_SpharmonicTableSize(bw,bw)));

  trans_seminaive_naive_tablespace =
    (double *) malloc(sizeof(double) *
		      (Reduced_Naive_TableSize(bw,bw) +
		       Reduced_SpharmonicTableSize(bw,bw)));


  /****
       At this point, check to see if all the memory has been
       allocated. If it has not, there's no point in going further.
  ****/

  if ( (scratch == NULL) || 
       (tmpInR == NULL) || (tmpInI == NULL) ||
       (tmpOutR == NULL) || (tmpOutI == NULL) ||
       (seminaive_naive_tablespace == NULL) ||
       (trans_seminaive_naive_tablespace == NULL) )
    {
      perror("Error in allocating memory");
      exit( 1 ) ;
    }
  

  /* precompute for the S^2 transform */
  seminaive_naive_table = SemiNaive_Naive_Pml_Table(bw, bw,
						    seminaive_naive_tablespace,
						    scratch);

  trans_seminaive_naive_table =
    Transpose_SemiNaive_Naive_Pml_Table(seminaive_naive_table,
					bw, bw,
					trans_seminaive_naive_tablespace,
					scratch);

  /* load in signal */
  if ( isReal )
    {
      for( i = 0 ; i < 4*bw*bw ; i ++ )
	{
	  tmpInR[i] = sigIn[i];
	  tmpInI[i] = 0. ;
	}
    }
  else
    {
      for( i = 0 ; i < 4*bw*bw ; i ++ )
	{
	  tmpInR[i] = sigIn[2*i];
	  tmpInI[i] = sigIn[2*i+1];
	}
    }

  /* now rotate */
  rotateFctFFTW( bw, bw, degOut,
		 tmpInR, tmpInI,
		 tmpOutR, tmpOutI,
		 alpha, beta, gamma,
		 scratch,
		 seminaive_naive_table,
		 trans_seminaive_naive_table ) ;

  if ( isReal )
    {
      for( i = 0 ; i < 4*bw*bw ; i ++ )
	{
	  sigOut[i] = tmpOutR[i];
	}
    }
  else
    {
      for( i = 0 ; i < 4*bw*bw ; i ++ )
	{
	  sigOut[2*i] = tmpOutR[i];
	  sigOut[2*i+1] = tmpOutI[i];
	}
    }




  /* clean up */
  free(trans_seminaive_naive_table);
  free(seminaive_naive_table);
  free(trans_seminaive_naive_tablespace);
  free(seminaive_naive_tablespace); 
  free(scratch);
  free(tmpOutI);
  free(tmpOutR);
  free(tmpInI);
  free(tmpInR);


}


