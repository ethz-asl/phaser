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
 
  functions involved in doing Wigner (little d) transforms:

  wigNaiveAnalysis(): forward wigner transform (spatial -> spectral)
  wigNaiveSynthesis(): inverse wigner transform (spectral -> spatial)

*/

#include "soft/utils_so3.h"


/****************************************************************/
/**
   wigNaiveAnalysis

   Given a bandwidth bw, and orders m1, m2, this function will analyse
   the signal of length n = 2*bw by projecting it onto the L2-normed
   little d wigners of degrees l = Max(|m1|, |m2|) ... bw - 1, i.e. I
   am doing the integrals

   <signal, d_{m1,m2}^l> for l = Max(|m1|, |m2|) ... bw - 1

   where the little d is normalized such that

   \int_0^pi (d_{m1,m2}^l) (d_{m1,m2}^{lx}) \sin\theta d\theta = \delta_{l,lx}

   NOTE: to use this routine as part of the full transform on SO(3), I still
   need to multiply these coefficients by (PI/bw) ... the PI is because
   my basis functions are the big D's now, the bw because of the adjustment
   in scaling doing the Fourier transform introduces. This multiplying HAS
   TO HAPPEN outside of this routine.


   NOTE that this function was written to be part of a full
   SO(3) harmonic transform, so a certain amount of precomputation
   has been assumed.

   arguments: m1, m2 = orders of the transform
              bw = bandwidth
	      signal = array of sample values, length n = 2*bw
	      wigners = array of length (bw - max(|m1|,|m2|))*( 2 * bw );
                        this array holds the wigner little d's that I
			need - PRECOMPUTED by genWig_L2() 
	      coeffs = array of length (bw - max(|m1|,|m2|)), to store the
	               final results
	      workspace = scratch area, of length n
	      weights = ptr to length 2*bw array containing the
	                quadrature weights - PRECOMPUTED by makeweights()


**/

void wigNaiveAnalysis( int m1,
		       int m2,
		       int bw,
		       double *signal,
		       double *wigners,
		       double *weights,
		       double *coeffs,
		       double *workspace )
{
  int i, j, m, n ;
  double tmp0, tmp1, tmp2, tmp3 ;

  /* m is the degree of the "first" wigner function at
     this order */
  m = MAX( ABS( m1 ) , ABS( m2 ) ) ;
  n = 2 * bw ;

  /* weight the signal with the appropriate quadrature weights */
  vec_pt_mul_so3( weights, signal, workspace, n ) ;

  /* and now I start analysing, i.e. multiply the matrix "wigners" by
     the vector "workspace" */

  for( i = 0 ; i < bw - m ; i ++ )
    {
      tmp0 = 0. ; tmp1 = 0. ; tmp2 = 0. ; tmp3 = 0. ;
      for ( j = 0 ; j < n % 4 ; ++j )
	tmp0 += wigners[j] * workspace[ j ];
      
      for ( ; j < n ; j += 4 )
	{
	  tmp0 += wigners[j] * workspace[j] ;
	  tmp1 += wigners[j+1] * workspace[j+1] ;
	  tmp2 += wigners[j+2] * workspace[j+2] ;
	  tmp3 += wigners[j+3] * workspace[j+3] ;
      	}
      coeffs[ i ] = tmp0 + tmp1 + tmp2 + tmp3 ;
      wigners += n ;
    }

  /* and that should be all */
}

/****************************************************************/
/**
   wigNaiveSynthesis

   Given a bandwidth bw, and orders m1, m2, this function will synthesize
   the signal of length n = 2*bw by "summing up the coefficients". More
   plainly, this is the inverse transform of wigNaiveAnalysis.

   Let l = Max(|m1|, |m2|). In matrix-lingo, wigNaiveAnalysis may be
   written as:

   c = P W f

   where f is the data vector, W is the quadrature matrix (i.e. weights),
   P is the (bw-l) x n matrix of sample values of the L2-normed wigners
   d_{m1,m2}^l d_{m1,m2}^{l+1} ... d_{m1,m2}^{bw-1}, and c is the
   wigner series representation (i.e. coefficients) of f (c is
   a vector of length bw-l).

   So wigNaiveSynthesis can be written as

   f = Transpose(P) c

   No quadrature matrix is necessary.

   NOTE that this function was written to be part of a full
   SO(3) harmonic transform, so a certain amount of precomputation
   has been assumed.

   NOTE: to use this routine as part of the full transform on SO(3), I still
   need to multiply these coefficients by (PI/bw) ... the PI is because
   my basis functions are the big D's now, the bw because of the adjustment
   in scaling doing the Fourier transform introduces. This multiplying HAS
   TO HAPPEN outside of this routine.

   arguments: m1, m2 = orders of the transform
              bw = bandwidth
	      coeffs = array of coefficients, length (bw - max(|m1|,|m2|))
	      wignersTrans = array of length (bw - max(|m1|,|m2|))*( 2 * bw );
                             this array holds the wigner little d's that I
			     need at this order - PRECOMPUTED genWigTrans_L2()
	      signal = array of length n = 2*bw, to store the final results,
                       the reconstructed sample values
	      workspace = scratch area, of length 0 * n
                          (that's right, 0 * n ... I'm keeping this
			   argument here just so that this function
			   call looks identical to wigNaiveAnalysis)

**/

void wigNaiveSynthesis( int m1,
			int m2,
			int bw,
			double *coeffs,
			double *wignersTrans,
			double *signal,
			double *workspace )
{
  int i, j, m, n ;
  double tmp0, tmp1, tmp2, tmp3 ;

  /* m is the degree of the "first" wigner function at
     this order */
  m = MAX( ABS( m1 ) , ABS( m2 ) ) ;
  n = 2 * bw ;

  for ( i = 0 ; i < n ; i ++ )
    {
      tmp0 = 0. ; tmp1 = 0. ; tmp2 = 0. ; tmp3 = 0. ;
      for ( j = 0 ; j < (bw - m) % 4 ; ++j )
	tmp0 += wignersTrans[j] * coeffs[ j ] ;

      for ( ; j < (bw - m) ; j += 4 )
	{
	  tmp0 += wignersTrans[ j ] * coeffs[ j ] ;
	  tmp1 += wignersTrans[ j+1 ] * coeffs[ j+1 ] ;
	  tmp2 += wignersTrans[ j+2 ] * coeffs[ j+2 ] ;
	  tmp3 += wignersTrans[ j+3 ] * coeffs[ j+3 ] ;
	}
      signal[ i ] = tmp0 + tmp1 + tmp2 + tmp3 ;
      wignersTrans += (bw - m);
    }

  /* that's it */
}
