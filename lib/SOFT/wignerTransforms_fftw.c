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
 
  functions involved in doing Wigner (little d) transforms
  using fftw and symmetries

  wigNaiveAnalysis_fftw(): forward wigner transform (spatial -> spectral)
  wigNaiveAnalysis_fftwX(): forward wigner transform (spatial -> spectral)
  wigNaiveAnalysis_fftwY(): forward wigner transform (spatial -> spectral)
  wigNaiveSynthesis_fftw(): inverse wigner transform (spectral -> spatial)
  wigNaiveSynthesis_fftwX(): inverse wigner transform (spectral -> spatial)
  wigNaiveSynthesis_fftwY(): inverse wigner transform (spectral -> spatial)

*/

#include <string.h>

#include <fftw3/fftw3.h>
#include "soft/utils_so3.h"
#include "soft/utils_vec_cx.h"

/****************************************************************/
/**
   wigNaiveAnalysis_fftw

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
	      signal = COMPLEX array of sample values, length n = 2*bw
	      wigners = array of length (bw - max(|m1|,|m2|))*( 2 * bw );
                        this array holds the wigner little d's that I
			need - PRECOMPUTED by genWig_L2() 
	      weights -> ptr to double array of size 2*bw - this array holds
	         the PRECOMPUTED quadrature weights
	      coeffs = COMPLEX array of length (bw - max(|m1|,|m2|)), to
	               store the final results
	      workspace = COMPLEX scratch area, of length n

**/

void wigNaiveAnalysis_fftw( int m1,
			    int m2,
			    int bw,
			    fftw_complex *signal,
			    double *wigners,
			    double *weights,
			    fftw_complex *coeffs,
			    fftw_complex *workspace )
{
  int i, j, l, m, n ;
  double *wignersPtr ;
  fftw_complex *weightedSignal;
  double tmpA, tmpB ;

  /* l is the degree of the "first" wigner function at
     this order */
  l = MAX( ABS( m1 ) , ABS( m2 ) ) ;
  m = l ;
  n = 2 * bw ;

  weightedSignal = workspace ;
  wignersPtr = wigners ;

  /* weight the signal with the appropriate quadrature weights */
  vec_pt_mul_cx( weights, signal, weightedSignal, n ) ;

  /* and now I start analysing, i.e. multiply the matrix "wigners" by
     the vector "weightedSignal" */

  for( i = 0 ; i < bw - m ; i ++ )
    {
      tmpA = 0.0 ;
      tmpB = 0.0 ;
      for ( j = 0 ; j < n ; j ++ )
	{
	  tmpA += *wignersPtr * weightedSignal[ j ][0] ;
	  tmpB += *wignersPtr * weightedSignal[ j ][1] ;
	  wignersPtr++ ;
	}
      coeffs[ i ][0] = tmpA ;
      coeffs[ i ][1] = tmpB ;
    }

  /* and that should be all */

}


/****************************************************************/
/**
   wigNaiveAnalysis_fftwX

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

   NOTE: This routine assumes that the SIGNS of m1 and m2 are
         IDENTICAL. This routine is written under the assumption
	 that I precomputed the Wigner-ds for order (|m1|,|m2|),
	 i.e. both positive. Since I could conceivably use this
	 routine for (|m2|,|m1|), (-|m1|,-|m2|) or (-|m2|,-|m1|), 
	 I need to compute a sign "fudge factor" -> I need to get
	 the correct power of -1!!!


   arguments: m1, m2 = orders of the transform
              bw = bandwidth
	      signal = COMPLEX array of sample values, length n = 2*bw
	      wigners = array of length (bw - max(|m1|,|m2|))*( 2 * bw );
                        this array holds the wigner little d's that I
			need - PRECOMPUTED by genWig_L2() 
	      weights -> ptr to double array of size 2*bw - this array holds
	         the PRECOMPUTED quadrature weights
	      coeffs = COMPLEX array of length (bw - max(|m1|,|m2|)), to
	               store the final results
	      workspace = COMPLEX scratch area, of length n

**/

void wigNaiveAnalysis_fftwX( int m1,
			     int m2,
			     int bw,
			     fftw_complex *signal,
			     double *wigners,
			     double *weights,
			     fftw_complex *coeffs,
			     fftw_complex *workspace )
{
  int i, j, l, m, n ;
  int fudge ;
  double *wignersPtr ;
  fftw_complex *weightedSignal;
  double tmpA, tmpB ;

  /* l is the degree of the "first" wigner function at
     this order */
  l = MAX( ABS( m1 ) , ABS( m2 ) ) ;
  m = l ;
  n = 2 * bw ;

  if ( ABS( m1 - m2 ) % 2 )
    fudge = -1 ;
  else
    fudge = 1 ;


  weightedSignal = workspace ;
  wignersPtr = wigners ;

  /* weight the signal with the appropriate quadrature weights */
  vec_pt_mul_cx( weights, signal, weightedSignal, n ) ;

  /* and now I start analysing, i.e. multiply the matrix "wigners" by
     the vector "weightedSignal" */

  for( i = 0 ; i < bw - m ; i ++ )
    {
      tmpA = 0.0 ;
      tmpB = 0.0 ;
      
      for ( j = 0 ; j < n ; j ++ )
	{
	  tmpA += *wignersPtr * weightedSignal[ j ][0] ;
	  tmpB += *wignersPtr * weightedSignal[ j ][1] ;
	  wignersPtr++ ;
	}
      coeffs[ i ][0] = fudge*tmpA ;
      coeffs[ i ][1] = fudge*tmpB ;
    }

  /* and that should be all */

}

/****************************************************************/
/**
   wigNaiveAnalysis_fftwY

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

   NOTE: This routine assumes that the SIGNS of m1 and m2 are
         DIFFERENT. This routine is written under the assumption
	 that I precomputed the Wigner-ds for order (|m1|,|m2|),
	 i.e. both positive. Since I could conceivably use this
	 routine for (-|m1|,|m2|), (-|m2|,|m1|), (-|m1|,|m2|) or
	 (-|m2|,|m1|), 
	 I need to compute a sign "fudge factor" -> I need to get
	 the correct power of -1!!! Note that, given the identities,
	 I need to FLIP either the signal or wigner-d's.


   arguments: m1, m2 = orders of the transform
              bw = bandwidth
	      signal = COMPLEX array of sample values, length n = 2*bw
	      wigners = array of length (bw - max(|m1|,|m2|))*( 2 * bw );
                        this array holds the wigner little d's that I
			need - PRECOMPUTED by genWig_L2() 
	      weights -> ptr to double array of size 2*bw - this array holds
	         the PRECOMPUTED quadrature weights
	      coeffs = COMPLEX array of length (bw - max(|m1|,|m2|)), to
	               store the final results
	      workspace = COMPLEX scratch area, of length n

**/

void wigNaiveAnalysis_fftwY( int m1,
			     int m2,
			     int bw,
			     fftw_complex *signal,
			     double *wigners,
			     double *weights,
			     fftw_complex *coeffs,
			     fftw_complex *workspace )
{
  int i, j, l, m, n ;
  int fudge ;
  double *wignersPtr ;
  fftw_complex *weightedSignal;
  double tmpA, tmpB ;


  /* l is the degree of the "first" wigner function at
     this order */
  l = MAX( ABS( m1 ) , ABS( m2 ) ) ;
  m = l ;
  n = 2 * bw ;

  if ( m1 < 0 )
    {
      if ( (l - m2) % 2 )
	fudge = -1 ;
      else
	fudge = 1 ;
    }
  else
    {
      if ( (l + m1) % 2 )
	fudge = -1 ;
      else
	fudge = 1 ;
    }

  weightedSignal = workspace ;
  wignersPtr = wigners ;

  /* weight the signal with the appropriate quadrature weights */
  vec_pt_mul_cx( weights, signal, weightedSignal, n ) ;

  /* and now I start analysing, i.e. multiply the matrix "wigners" by
     the vector "weightedSignal" */
  for( i = 0 ; i < bw - m ; i ++ )
    {
      tmpA = 0.0 ;
      tmpB = 0.0 ;
      for( j = 0 ; j < n ; j ++ )
	{
	  tmpA += *wignersPtr * weightedSignal[ n - 1 - j ][0] ;
	  tmpB += *wignersPtr * weightedSignal[ n - 1 - j ][1] ;
	  wignersPtr++ ;
	}
      coeffs[ i ][0] = fudge * tmpA ;
      coeffs[ i ][1] = fudge * tmpB ;
      fudge *= -1 ;
    }

  /* and that should be all */

}

/****************************************************************/
/**
   wigNaiveSynthesis_fftw

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
	      coeffs = COMPLEX array of coefficients,
	               length (bw - max(|m1|,|m2|))
	      wignersTrans = array of length (bw - max(|m1|,|m2|))*( 2 * bw );
                             this array holds the wigner little d's that I
			     need at this order - PRECOMPUTED genWigTrans_L2()
	      signal = COMPLEX array of length n = 2*bw, to store the
	               final results, the reconstructed sample values
	      workspace = scratch area, of length 0 * n
                          (that's right, 0 * n ... I'm keeping this
			   argument here just so that this function
			   call looks identical to wigNaiveAnalysis)

**/

void wigNaiveSynthesis_fftw( int m1,
			     int m2,
			     int bw,
			     fftw_complex *coeffs,
			     double *wignersTrans,
			     fftw_complex *signal,
			     fftw_complex *workspace )
{
  int i, j, m, n ;
  double *wignersTransPtr ;
  double tmpA, tmpB ;

  /* l is the degree of the "first" wigner function at
     this order */
  m = MAX( ABS( m1 ) , ABS( m2 ) ) ;
  n = 2 * bw ;

  wignersTransPtr = wignersTrans ;

  /* just do the sums */
  for ( i = 0 ; i < n ; i ++ )
    {
      tmpA = 0.0 ;
      tmpB = 0.0 ;
      for ( j = 0 ; j < (bw - m) ; j ++ )
	{
	  tmpA += *wignersTransPtr * coeffs[j][0] ;
	  tmpB += *wignersTransPtr * coeffs[j][1] ;
	  wignersTransPtr++ ;
	}
      signal[ i ][0] = tmpA ;
      signal[ i ][1] = tmpB ;
    }

  /* that's it */
}


/****************************************************************/
/**
   wigNaiveSynthesis_fftwX

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


   NOTE: This routine assumes that the SIGNS of m1 and m2 are
         IDENTICAL. This routine is written under the assumption
	 that I precomputed the Wigner-ds for order (|m1|,|m2|),
	 i.e. both positive. Since I could conceivably use this
	 routine for (|m2|,|m1|), (-|m1|,-|m2|) or (-|m2|,-|m1|), 
	 I need to compute a sign "fudge factor" -> I need to get
	 the correct power of -1!!!


   arguments: m1, m2 = orders of the transform
              bw = bandwidth
	      coeffs = COMPLEX array of coefficients,
	               length (bw - max(|m1|,|m2|))
	      wignersTrans = array of length (bw - max(|m1|,|m2|))*( 2 * bw );
                             this array holds the wigner little d's that I
			     need at this order - PRECOMPUTED genWigTrans_L2()
	      signal = COMPLEX array of length n = 2*bw, to store the
	               final results, the reconstructed sample values
	      workspace = scratch area, of length 0 * n
                          (that's right, 0 * n ... I'm keeping this
			   argument here just so that this function
			   call looks identical to wigNaiveAnalysis)

**/

void wigNaiveSynthesis_fftwX( int m1,
			      int m2,
			      int bw,
			      fftw_complex *coeffs,
			      double *wignersTrans,
			      fftw_complex *signal,
			      fftw_complex *workspace )
{
  int i, j, m, n ;
  int fudge ;
  double *wignersTransPtr ;
  double tmpA, tmpB ;

  /* l is the degree of the "first" wigner function at
     this order */
  m = MAX( ABS( m1 ) , ABS( m2 ) ) ;
  n = 2 * bw ;

  if ( ABS( m1 - m2 ) % 2 )
    fudge = -1 ;
  else
    fudge = 1 ;

  wignersTransPtr = wignersTrans ;

  /* just do the sums */
  for ( i = 0 ; i < n ; i ++ )
    {
      tmpA = 0.0 ;
      tmpB = 0.0 ;
      for ( j = 0 ; j < (bw - m) ; j ++ )
	{
	  tmpA += *wignersTransPtr * coeffs[j][0] ;
	  tmpB += *wignersTransPtr * coeffs[j][1] ;
	  wignersTransPtr++ ;
	}
      signal[ i ][0] = fudge * tmpA ;
      signal[ i ][1] = fudge * tmpB ;
    }

  /* that's it */
}
/****************************************************************/
/**
   wigNaiveSynthesis_fftwY

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

   NOTE: This routine assumes that the SIGNS of m1 and m2 are
         DIFFERENT. This routine is written under the assumption
	 that I precomputed the Wigner-ds for order (|m1|,|m2|),
	 i.e. both positive. Since I could conceivably use this
	 routine for (-|m1|,|m2|), (-|m2|,|m1|), (-|m1|,|m2|) or
	 (-|m2|,|m1|), 
	 I need to compute a sign "fudge factor" -> I need to get
	 the correct power of -1!!! Note that, given the identities,
	 I need to FLIP either the signal or wigner-d's.

   arguments: m1, m2 = orders of the transform
              bw = bandwidth
	      coeffs = COMPLEX array of coefficients,
	               length (bw - max(|m1|,|m2|))
	      wignersTrans = array of length (bw - max(|m1|,|m2|))*( 2 * bw );
                             this array holds the wigner little d's that I
			     need at this order - PRECOMPUTED genWigTrans_L2()
	      signal = COMPLEX array of length n = 2*bw, to store the
	               final results, the reconstructed sample values
	      workspace = COMPLEX scratch area, of length 1 * n
**/

void wigNaiveSynthesis_fftwY( int m1,
			      int m2,
			      int bw,
			      fftw_complex *coeffs,
			      double *wignersTrans,
			      fftw_complex *signal,
			      fftw_complex *workspace )
{
  int i, j, m, n ;
  int fudge ;
  double *wignersTransPtr ;
  double tmpA, tmpB ;

  /* l is the degree of the "first" wigner function at
     this order */
  m = MAX( ABS( m1 ) , ABS( m2 ) ) ;
  n = 2 * bw ;

  if ( m1 < 0 )
    {
      if ( (m - m2) % 2 )
	fudge = -1 ;
      else
	fudge = 1 ;
    }
  else
    {
      if ( (m + m1) % 2 )
	fudge = -1 ;
      else
	fudge = 1 ;
    }

  /*
    place the ptr at the *end* of the array - (bw-m)
  */

  wignersTransPtr = wignersTrans  + (bw - m ) * n - (bw-m) ;

  memcpy( workspace, coeffs, sizeof(fftw_complex) * n );
  for ( i = 0 ; i < n ; i ++ )
    {
      workspace[i][0] *= fudge ;
      workspace[i][1] *= fudge ;
      fudge *= -1 ;
    }

  for ( i = 0 ; i < n ; i ++ )
    {
      tmpA = 0.0 ;
      tmpB = 0.0 ;
      for ( j = 0 ; j < ( bw - m ) ; j ++ )
	{
	  tmpA += *wignersTransPtr * workspace[ j ][0] ;
	  tmpB += *wignersTransPtr * workspace[ j ][1] ;
	  wignersTransPtr++ ;
	}
      signal[ i ][0] = tmpA ;
      signal[ i ][1] = tmpB ;
      wignersTransPtr = wignersTrans + (n - i - 1)*(bw-m) - (bw - m) ;
    }

  /* that's it */
}
