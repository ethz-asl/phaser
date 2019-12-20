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

/***********************************************************************

  some "primitive" utility vector functions, designed with COMPLEX data
  in mind

  vec_add_cx() - add two COMPLEX vectors
  vec_mul_cx() - multiply COMPLEX vector by a scalar
  vec_pt_mul_cx() - pointwise-multiply double vector with a COMPLEX vector

  transpose_cx() - transpose a COMPLEX matrix

************************************************************************/

#include <fftw3/fftw3.h>
#include "soft/complex.h"
#include "soft/utils_vec_cx.h"


/************************************************************************/
/* vector arithmetic operations */
/************************************************************************/

/* does result = data1 + data2 */
/* result and data are COMPLEX vectors of length n */

void vec_add_cx( fftw_complex *data1,
		 fftw_complex *data2,
		 fftw_complex *result,
		 int n )
{
  int k ;

  for (k = 0; k < n % 4; ++k)
    {
      result[k][0] = CXADDRE( data1[k], data2[k] );
      result[k][1] = CXADDIM( data1[k], data2[k] );
    }
      
  for ( ; k < n ; k += 4)
    {
      result[k][0] = CXADDRE( data1[k], data2[k] );
      result[k][1] = CXADDIM( data1[k], data2[k] );
      result[k+1][0] = CXADDRE( data1[k+1], data2[k+1] );
      result[k+1][1] = CXADDIM( data1[k+1], data2[k+1] );
      result[k+2][0] = CXADDRE( data1[k+2], data2[k+2] );
      result[k+2][1] = CXADDIM( data1[k+2], data2[k+2] );
      result[k+3][0] = CXADDRE( data1[k+3], data2[k+3] );
      result[k+3][1] = CXADDIM( data1[k+3], data2[k+3] );
    }
}



/************************************************************************/
/************************************************************************/
/*
   vec_mul_cx(scalar,data1,result,n) multiplies the COMPLEX vector
   'data1' of length n by 'scalar' and returns in result 
*/
void vec_mul_cx( double scalar,
		 fftw_complex *data1,
		 fftw_complex *result,
		 int n)
{
   int k;

   for( k = 0; k < n % 4; ++k)
     {
       result[k][0] = CXSMULRE( scalar, data1[k] );
       result[k][1] = CXSMULIM( scalar, data1[k] );
     }

   for( ; k < n; k +=4)
     {
       result[k][0] = CXSMULRE( scalar, data1[k] );
       result[k][1] = CXSMULIM( scalar, data1[k] );
       result[k+1][0] = CXSMULRE( scalar, data1[k+1] );
       result[k+1][1] = CXSMULIM( scalar, data1[k+1] );
       result[k+2][0] = CXSMULRE( scalar, data1[k+2] );
       result[k+2][1] = CXSMULIM( scalar, data1[k+2] );
       result[k+3][0] = CXSMULRE( scalar, data1[k+3] );
       result[k+3][1] = CXSMULIM( scalar, data1[k+3] );
     }

}


/************************************************************************/
/************************************************************************/

/* point-by-point multiplication of length n double vector data1,
   with a length n COMPLEX vector data2,
   places result in result */

void vec_pt_mul_cx( double *data1,
		    fftw_complex *data2,
		    fftw_complex *result,
		    int n)
{
  int k;
  
  for(k = 0; k < n % 4; ++k)
    {
      result[k][0] = CXSMULRE( data1[k], data2[k] );
      result[k][1] = CXSMULIM( data1[k], data2[k] );
    }

  
  for( ; k < n; k +=4)
    {
      result[k][0] = CXSMULRE( data1[k], data2[k] );
      result[k][1] = CXSMULIM( data1[k], data2[k] );
      result[k+1][0] = CXSMULRE( data1[k+1], data2[k+1] );
      result[k+1][1] = CXSMULIM( data1[k+1], data2[k+1] );
      result[k+2][0] = CXSMULRE( data1[k+2], data2[k+2] );
      result[k+2][1] = CXSMULIM( data1[k+2], data2[k+2] );
      result[k+3][0] = CXSMULRE( data1[k+3], data2[k+3] );
      result[k+3][1] = CXSMULIM( data1[k+3], data2[k+3] );
    }
 
}


/*******************************************************/
/*******************************************************/
/*
  transpose_cx() - transpose an m x n COMPLEX matrix
                   (assumed to be stored in a 1-D array
		   in ROW-MAJOR format)

  This function won't be efficient, but it oughta work.

 arguments: arrayIn: 1-D COMPLEX input array of length m*n,
                     treated as an m x n matrix
	    
	    arrayOut: 1-D COMPLEX output array of length m*n,
                      the TRANSPOSE of arrayIn

*/

void transpose_cx( fftw_complex *arrayIn,
		   fftw_complex *arrayOut,
		   int m,
		   int n )
{

  /* is this faster? no, not really
    int j, k;
    fftw_complex *inPtr, *outPtr;

    inPtr = arrayIn ;
    outPtr = arrayOut ;

    for( j = 0 ; j < n ; j ++ )
    for( k = j ; k < m*n ; k += n )
    {
    *outPtr[0] = inPtr[k][0] ;
    *outPtr[1] = inPtr[k][1] ;
    outPtr++ ;
    }
  */

  int k, d, mud ;
  fftw_complex *inPtr, *outPtr;

  inPtr = arrayIn ;
  outPtr = arrayOut ;

  if( n >= m )
    {
      for( d = n - m + 1 ; d < n ; d ++ )
	{
	  mud = n - d ;
	  for( k = 0 ; k < mud ; k ++ )
	    {
	      outPtr[ m*(k+d) + k ][0] = inPtr[ n*k + k + d ][0] ;
	      outPtr[ m*(k+d) + k ][1] = inPtr[ n*k + k + d ][1] ;
	    }
	}
      
      for( d = 0 ; d < n - m + 1 ; d ++ )
	{
	  mud = m ;
	  for( k = 0 ; k < mud ; k ++ )
	    {
	      outPtr[ m*(k+d) + k ][0] = inPtr[ n*k + k + d][0] ;
	      outPtr[ m*(k+d) + k ][1] = inPtr[ n*k + k + d][1] ;
	    }
	}

      for( d = -m + 1 ; d < 0 ; d ++ )
	{
	  mud = m + d ;
	  for( k = 0 ; k < mud ; k ++ )
	    {
	      outPtr[ m*k + k - d ][0] = inPtr[ n*(k-d) + k ][0] ;
	      outPtr[ m*k + k - d ][1] = inPtr[ n*(k-d) + k ][1] ;
	    }
	}
    }
  else
    {
      for( d = 0 ; d < n  ; d ++ )
	{
	  mud = n - d ;
	  for( k = 0 ; k < mud ; k ++ )
	    {
	      outPtr[ m*(k+d) + k ][0] = inPtr[ n*k + k + d ][0] ;
	      outPtr[ m*(k+d) + k ][1] = inPtr[ n*k + k + d ][1] ;
	    }
	}

      for( d = n - m  ; d < 0 ; d ++ )
	{
	  mud = n ;
	  for( k = 0 ; k < mud ; k ++ )
	    {
	      outPtr[ m*(k) + k - d ][0] = inPtr[ n*(k-d) + k ][0] ;
	      outPtr[ m*(k) + k - d ][1] = inPtr[ n*(k-d) + k ][1] ;
	    }
	}

      for( d = -m + 1 ; d < n - m ; d ++ )
	{
	  mud = m + d ;
	  for( k = 0 ; k < mud ; k ++ )
	    {
	      outPtr[ m*k + k - d ][0] = inPtr[ n*(k-d) + k ][0] ;
	      outPtr[ m*k + k - d ][1] = inPtr[ n*(k-d) + k ][1] ;
	    }
	}
    }

}

