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

  some "primitive" utility functions:

  vec_add_so3() - add two vectors together
  vec_add_scalar_so3() - add a scalar to a vector
  vec_mul_so3() - multiply vector by a scalar
  vec_mul_inplace_so3() - multiply vector by scalar IN PLACE
  vec_pt_mul_so3() - pointwise-multiply two vectors
  vec_inner_so3() - inner (i.e. dot) product of two vectors
  transpose_so3() - transpose an n x m matrix (assumed to be stored
                in a 1-D array in ROW-MAJOR format)
  totalCoeffs_so3() - total number of coefficients calculated in a
                  full SO3 forward transform
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


************************************************************************/

#include "soft/utils_so3.h"

/************************************************************************/
/* vector arithmetic operations */
/************************************************************************/
/* does result = data1 + data2 */
/* result and data are vectors of length n */

void vec_add_so3( double *data1,
		  double *data2,
		  double *result,
		  int n )
{
  int k ;

  for (k = 0; k < n % 4; ++k)
    result[k] = data1[k] + data2[k];

  for ( ; k < n ; k += 4)
    {
      result[k] = data1[k] + data2[k];
      result[k + 1] = data1[k + 1] + data2[k + 1];
      result[k + 2] = data1[k + 2] + data2[k + 2];
      result[k + 3] = data1[k + 3] + data2[k + 3];
    }
}
/************************************************************************/
/************************************************************************/
/*
   vec_mul(scalar,data1,result,n) multiplies the vector 'data1' by
   'scalar' and returns in result 
*/
void vec_mul_so3( double scalar,
		  double *data1,
		  double *result,
		  int n)
{
   int k;

   for( k = 0; k < n % 4; ++k)
     result[k] = scalar * data1[k];

   for( ; k < n; k +=4)
     {
       result[k] = scalar * data1[k];
       result[k + 1] = scalar * data1[k + 1];
       result[k + 2] = scalar * data1[k + 2];
       result[k + 3] = scalar * data1[k + 3];
     }

}

/************************************************************************/
/************************************************************************/
/*
   vec_mul_inplace_so3(scalar,data,n) multiplies the vector 'data1' by
   'scalar' IN PLACE!!! 
*/
void vec_mul_inplace_so3( double scalar,
			  double *data,
			  int n)
{
   int k;

   for( k = 0; k < n ; k++ )
     data[k] *= scalar ;
}

/************************************************************************/
/************************************************************************/
/*
   vec_add_scalar_so3(scalar,data1,result,n) adds scalar to the vector 'data1'
   and returns in result 
*/
void vec_add_scalar_so3( double scalar,
			 double *data1,
			 double *result,
			 int n)
{
   int k;

   for( k = 0; k < n % 4; ++k)
     result[k] = scalar + data1[k];

   for( ; k < n; k +=4)
     {
       result[k] = scalar + data1[k];
       result[k + 1] = scalar + data1[k + 1];
       result[k + 2] = scalar + data1[k + 2];
       result[k + 3] = scalar + data1[k + 3];
     }

}



/************************************************************************/
/************************************************************************/

/* point-by-point multiplication of length n vectors data1, data2,
 places result in result */

void vec_pt_mul_so3( double *data1,
		     double *data2,
		     double *result,
		     int n)
{
  int k;
  
  for(k = 0; k < n % 4; ++k)
    result[k] = data1[k] * data2[k];
  
  for( ; k < n; k +=4)
    {
      result[k] = data1[k] * data2[k];
      result[k + 1] = data1[k + 1] * data2[k + 1];
      result[k + 2] = data1[k + 2] * data2[k + 2];
      result[k + 3] = data1[k + 3] * data2[k + 3];
    }
 
}

/**********************************************************/
/**********************************************************/
/*
  compute the inner (dot) product of two vectors data1
  and data2, each of length n. The result will be RETURNED
*/

double vec_inner_so3( double *data1,
		      double *data2,
		      int n )
{
  int k ;
  double tmp0, tmp1, tmp2, tmp3 ;

  tmp0 = 0.0 ;
  tmp1 = 0.0 ;
  tmp2 = 0.0 ;
  tmp3 = 0.0 ;

  for ( k = 0 ; k < n % 4 ; ++k )
    tmp0 += data1[ k ] * data2[ k ] ;

  for ( ; k < n ; k += 4 )
    {
      tmp0 += data1[ k ] * data2[ k ] ;
      tmp1 += data1[ k + 1] * data2[ k + 1] ;
      tmp2 += data1[ k + 2] * data2[ k + 2] ;
      tmp3 += data1[ k + 3] * data2[ k + 3] ;
    }
  
  return ( tmp0 + tmp1 + tmp2 + tmp3 ) ;
}

/*******************************************************/
/*******************************************************/
/*
  transpose_so3() - transpose an m x n matrix (assumed to be stored
                in a 1-D array in ROW-MAJOR format)

  This function won't be efficient, but it oughta work.

 arguments: arrayIn: 1-D input array of length m*n,
                     treated as an m x n matrix
	    
	    arrayOut: 1-D output array of length m*n,
                      the TRANSPOSE of arrayIn

*/

void transpose_so3( double *arrayIn,
		    double *arrayOut,
		    int m,
		    int n )
{
  /*

    int j, k;
    double *inPtr, *outPtr;
    
    inPtr = arrayIn ;
    outPtr = arrayOut ;
    
    for( j = 0 ; j < n ; j ++ )
    for( k = j ; k < m*n ; k += n )
    *outPtr++ = inPtr[ k ] ;

  */


  int k, d, mud ;
  double *inPtr, *outPtr;

  inPtr = arrayIn ;
  outPtr = arrayOut ;

  if( n >= m )
    {
      for( d = n - m + 1 ; d < n ; d ++ )
        {
          mud = n - d ;
          for( k = 0 ; k < mud ; k ++ )
            {
              outPtr[ m*(k+d) + k ] = inPtr[ n*k + k + d ] ;
            }
        }

      for( d = 0 ; d < n - m + 1 ; d ++ )
        {
          mud = m ;
          for( k = 0 ; k < mud ; k ++ )
            {
              outPtr[ m*(k+d) + k ] = inPtr[ n*k + k + d] ;
            }
        }

      for( d = -m + 1 ; d < 0 ; d ++ )
        {
          mud = m + d ;
          for( k = 0 ; k < mud ; k ++ )
            {
              outPtr[ m*k + k - d ] = inPtr[ n*(k-d) + k ] ;
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
              outPtr[ m*(k+d) + k ] = inPtr[ n*k + k + d ] ;
            }
        }

      for( d = n - m  ; d < 0 ; d ++ )
        {
          mud = n ;
          for( k = 0 ; k < mud ; k ++ )
            {
              outPtr[ m*(k) + k - d ] = inPtr[ n*(k-d) + k ] ;
            }
        }

      for( d = -m + 1 ; d < n - m ; d ++ )
        {
          mud = m + d ;
          for( k = 0 ; k < mud ; k ++ )
            {
              outPtr[ m*k + k - d ] = inPtr[ n*(k-d) + k ] ;
            }
        }
    }


}

/***********************************************************/
/*

  What follows are some "indexing/quantity" type functions
  useful when doing the full transform

*/

/*
  At bandwidth bw, what are the TOTAL NUMBER of coefficients
  that will be computed when doing a full SO3 forward transform ?

  total number = 1/3 * bw * (4 * bw^2 - 1 )

*/

int totalCoeffs_so3( int bw )
{

  /* note that 4 * bw^3 - bw is always a multiple of 3,
     so integer division won't be messy */

  return ( (4 * bw * bw * bw - bw ) / 3 ) ;
}

/***************************************************/

/*
  At bandwidth bw and order m1, what's the total number of
  coefficients that will be computed, when doing a full SO3
  forward transform, at this order?

  That is, fix m1. At each "legal" m2, I will compute so many
  coefficients. Let me add all those numbers up.

  total number at order m1, for bandwidth bw: bw^2 - m1^2

*/

int numCoeffs_so3( int bw,
		   int m1 )
{

  return ( bw*bw - m1*m1 ) ;

}

/************************************************/
/*
  At orders m1, m2, and bandwidth bw, how many coefficients
  will be computed ?

  let m = Max( |m1|, |m2| )

  The number of coefficients is = bw - m

*/

int howMany_so3( int m1,
		 int m2,
		 int bw )
{
  int m ;

  m = MAX( ABS( m1 ) , ABS( m2 ) );

  return ( bw - m ) ;

}

/************************************************/
/*
 At order m1, m2, bandwidth bw, where in the 3-D,
 fft'd array does the data necessary for the Wigner-(m1,m2)
 transform start? I.e. I need the location of the fft'd data
 in order to compute f_{m1,m2}^l for all legal l. With respect
 to the beginning of that block of memory, where does this
 occur?

 Note that the array is of size 2bw x 2bw x 2bw, so I
 need to multiply by that 2*bw (I'm sampling at twice
 the bandwidth)

*/

int sampLoc_so3( int m1, int m2, int bw )
{
  if ( m1 >= 0 )
    {
      if ( m2 >= 0 )
	return ( (2*bw)*((2*bw*m1 + m2)) );
      else
	return ( (2*bw)*((2*bw*m1 + (2*bw+m2))) );
    }
  else
    {
      if ( m2 >= 0 )
	return ( (2*bw)*((2*bw)*(2*bw+m1) + m2) );
      else
	return ( (2*bw)*((2*bw)*(2*bw+m1) + (2*bw+m2)) );
    }
}

	  
/************************************************/
/*

 So the above function tells me where in the array
 I should start taking the samples from, in order
 to do the order (m1,m2)-Wigner-transform on the
 just fft'd data.

 This function, coefLoc, tells where in the coefficient
 array I should start placing this evaluated inner-products
 of this bandwidth bw transform.

*/

int coefLoc_so3( int m1, int m2, int bw )
{

  int k, tmp ;

  if ( m1 >= 0 )
    {
      if ( m2 >= 0 )
	{
	  tmp = (bw*bw)*m1 ;
	  tmp -= ( (m1-1)*m1*(2*m1-1)/6 ) ;

	  for( k = 0 ; k < m2 ; k ++ )
	    tmp += howMany_so3( m1, k, bw ) ;

	  return ( tmp ) ;
	}
      else
	{
	  tmp = (bw*bw)*(m1+1) ;
	  tmp -= ( (m1+1)*m1*(2*m1+1)/6 ) ;

	  for( k = m2 ; k < 0 ; k ++ )
	    tmp -= howMany_so3( m1, k, bw ) ;

	  return ( tmp ) ;
	}
    }
  else
    {
      if ( m2 >= 0 )
	{

	  tmp = totalCoeffs_so3( bw ) ;
	  tmp -= (bw*bw)*(-m1) ;
	  tmp += ( (-m1+1)*(-m1)*(2*(-m1)+1)/6 ) ;

	  for ( k = 0 ; k < m2 ; k ++ )
	    tmp += howMany_so3( m1, k, bw ) ;

	  return ( tmp ) ;
	}
      else
	{
	  tmp = totalCoeffs_so3( bw ) ;
	  tmp -= (bw*bw)*(-m1-1) ;
	  tmp += ( (-m1-1)*(-m1)*(2*(-m1)-1)/6 );

	  for ( k = m2 ; k < 0 ; k ++ )
	    tmp -= howMany_so3( m1, k, bw );

	  return ( tmp ) ;
	}
    }
}

	  

/******************************************************/
/*
  so3CoefLoc: Where, in the coefficient array of the
              SO(3) signal of bandwidth bw does the
	      coefficient f_{MM'}^l reside?

  Unfortunately, the indexing is a little hairy.

                 bw: bandwidth
		 l : degree
		 m, mp : orders

  NOTE: Here's the convention I'm observing:

        D_{MM'}^l(alpha, beta, gamma) =

	exp(-I M alpha) * d_{MM'}^l(beta) * exp(-I M gamma)

	where m == M, mp == M'

*/

int so3CoefLoc( int m,
		int mp,
		int l,
		int bw )
{
  int k ;
  int tmpA, tmpB ;

  if ( m >= 0 )
    {
      if ( mp >= 0 )
	{
	  tmpA = ( 6*bw*bw + 3*m - 2*m*m - 1);
	  tmpA *= m ;
	  tmpA /= 6 ;
	  
	  tmpB = 0 ;
	  for( k = 0 ; k < mp ; k ++ )
	    tmpB += HPP(m, k, bw) ;
	  
	  tmpA += tmpB ;
	  tmpA += (l - MAX(m,mp));
	}
      else /* mp < 0 */
	{
	  tmpA = -(m*(1+m)*(1+2*m))/6 ;
	  tmpA += bw*bw*(1+m);

	  tmpB = 0;
	  for( k = mp; k < 0 ; k ++ )
	    tmpB += HPM(m, k, bw) ;

	  tmpA -= tmpB ;
	  tmpA += (l - MAX(m,-mp));
	}
    }
  else /* so m < 0 */
    {
      if ( mp >= 0 )
	{
	  tmpA = (4*bw*bw*bw-bw)/3 ;
	  tmpB = (-m)*(1-m)*(1-2*m) ;
	  tmpB /= -6 ;
	  tmpB += bw*bw*(-m) ;
	  tmpA -= tmpB ;
	  
	  tmpB = 0 ;
	  for( k = 0 ; k < mp ; k ++ )
	    tmpB += HMP(m, k, bw) ;
	  
	  tmpA += tmpB ;
	  tmpA += (l - MAX(-m,mp));
	}
      else /* mp < 0 */
	{
	  tmpA = (4*bw*bw*bw-bw)/3 ;
	  tmpB = ((-m)-1)*(6*bw*bw-m-2*(m*m));
	  tmpB /= 6 ;
	  tmpA -= tmpB ;

	  tmpB = 0;
	  for( k = mp; k < 0 ; k ++ )
	    tmpB += HMM(m, k, bw) ;

	  tmpA -= tmpB ;
	  tmpA += (l - MAX(-m,-mp));
	}
    }

  return tmpA ;
}

