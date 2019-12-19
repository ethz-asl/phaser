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

  functions involved of the generation of wigner functions:


  L2_aN_so3(), L2_bN_so3(), L2_cN_so3():
          generate terms in the recurrence relation

  L2_3term_so3():
          generate terms in the recurrecne relation in one function call

  EvalPtsW(), CosEvalPts(), CosEvalPts2(), SinEvalPts(), SinEvalPts2():
          where to sample Wigners

  wigSpec_L2():
          make a wigner whose degree equals the absolute value of one of
	  its order

  genWig_L2():
          make an array of wigners for particular orders m1, m2

  genWig_L2_U():
	  make an array of wigners for particular orders m1, m2
	  at user-specified angles

  genWigTrans_L2():
          make an array of wigners (the transpose of above function)

  genAllWig():
          make ALL the Wigner little-d's necessary to do a full
	  FORWARD SOFT (i.e. SO(3)) transform

  genAllWigTrans():
          make ALL the Wigner little-d's necessary to do a full
	  INVERSE SOFT (i.e. SO(3)) transform


  */

#include <stdio.h>
#include <math.h>
#include <string.h>  /* to declare memcpy */
#include "soft/utils_so3.h"
#include "soft/weights.h"


/************************************************************************/
/* Recurrence coefficients */
/************************************************************************/
/*
  Recurrence coefficients for the L2-normed Wigner d functions.
  When using these coefficients, make sure that the initial
  d^l_{m1,m2} function is also normed.

  NOTE: These functions are normed in the sense that, as functions
  only of theta,

  \int_0^pi (d_{m1,m2}^l) (d_{m1,m2}^lx) \sin\theta d\theta = delta_{l,lx}

  SO I'm treating these functions "on their own" and not as part of
  the big D function (a function of all three Euler angles):

  bigD_{m1,m2}^{j}(alpha, theta, gamma) =
  exp(-i * m1 * alpha ) * d_{m1,m2}^l(theta) * exp(-i * m2 * gamma )

  NOTE: when I say "wigner", I mean the little d function (of one angle)
  and not the bigD function (three angles)

  If

  normWig[l, m1, m2] = Sqrt(2/(2 l + 1))
  ( so \int_0^pi (d_{m1,m2}^l / normWig[l,m1,m2])^2 \sin\theta d\theta = 1 )

  then

  normD[l, m1, m2] = 2 * PI * normWig(l, m1, m2)

  Notation: j - degree
            m1, m2, orders

  d_{m1,m2}^{j+1}(theta) =
          L2_aN_so3(j, m1, m2) * d_{m1,m2}^{j-1}(theta) +
	  L2_bN_so3(j, m1, m2) * cos(theta) * d_{m1,m2}^{j}(theta) +
	  L2_cN_so3(j, m1, m2) * d_{m1,m2}^{j}(theta)

***************************************************************/

double L2_aN_so3( int j,
		  int m1,
		  int m2 )
{
  double dj, dm1, dm2;
  
  dj = (double) j ;
  dm1 = (double) m1 ;
  dm2 = (double) m2 ;
  
  if ( j > 0 )
    return (
	    -sqrt( (2.*dj + 3.)/(2.*dj - 1.) ) *
	    (dj + 1.)/sqrt(((dj+1.)*(dj+1.) - dm1*dm1)*
			   ((dj+1.)*(dj+1.) - dm2*dm2)) *
	    sqrt( (dj*dj - dm1*dm1)*(dj*dj - dm2*dm2) ) / dj
	    );
  else
    return 0. ;
  
}

/***************************/


double L2_bN_so3( int j,
		  int m1,
		  int m2 )
{
  double dj, dm1, dm2;

  dj = (double) j ;
  dm1 = (double) m1 ;
  dm2 = (double) m2 ;

  return (
	  sqrt( (2.*dj + 3.)/(2.*dj + 1.) ) *
	  (dj + 1.)*(2.*dj + 1.)/sqrt(((dj+1.)*(dj+1.) - dm1*dm1)*
				      ((dj+1.)*(dj+1.) - dm2*dm2))
	  ) ;
}

/**************************/

double L2_cN_so3( int j,
		  int m1,
		  int m2 )
{
  double dj, dm1, dm2;

  dj = (double) j ;
  dm1 = (double) m1 ;
  dm2 = (double) m2 ;

  if ( j != 0 )
    return (
	    -L2_bN_so3(j, m1, m2) * dm1 * dm2 / ( dj * ( dj + 1. ) )
	    ) ;
  else
    return 0. ;
}      

/***************************/

void L2_3term_so3( int j, int m1, int m2,
		   double *an, double *bn, double *cn )
{
  double dj, dm1, dm2 ;
  double t1, t2, t3, t4, t5 ;

  dj = (double) j ;
  dm1 = ( double ) m1 ;
  dm2 = ( double ) m2 ;

  t1 = sqrt( (2*dj+3)/(2*dj+1) );
  t3 = (dj+1.)*(2*dj+1);
  t5 = sqrt(((dj+1)*(dj+1)-dm1*dm1)*((dj+1)*(dj+1)-dm2*dm2));
  t5 = 1/t5 ;

  if ( j == 0 )
    {
      *an = 0 ;
      *cn = 0 ;
    }
  else
    {
      t2 = sqrt( (2*dj+3)/(2*dj-1) ) * (dj+1.)/dj ;
      t4 = sqrt((dj*dj-dm1*dm1)*(dj*dj-dm2*dm2));
      *an = -t2*t4*t5 ;
      *cn = -dm1*dm2/(dj*(dj+1));
    }

  *bn = t1*t3*t5 ;

}
  
/************************************************************************/
/* returns an array of the angular arguments of n Chebyshev nodes */
/* eval_pts points to a double array of length n */

void EvalPtsW( int n,
	       double *eval_pts )
{
  int i;
  double twoN;

  twoN = (double) (2 * n);

  for (i=0; i<n; i++)
    eval_pts[i] = (( 2.0*((double)i)+1.0 ) * M_PI) / twoN;

}

/************************************************************************/
/* returns an array of n Chebyshev nodes, i.e.
   cos( EvalPts )
*/

void CosEvalPts( int n,
		 double *eval_pts )
{
  int i;
  double twoN;

  twoN = (double) (2*n);

  for (i=0; i<n; i++)
    eval_pts[i] = cos((( 2.0*((double)i)+1.0 ) * M_PI) / twoN);

}


/************************************************************************/
/* returns an array of sin evaluated at the n Chebyshev pts, i.e.
   sin( EvalPts )
*/

void SinEvalPts( int n,
		 double *eval_pts )
{
  int i;
  double twoN;

  twoN = (double) (2*n);

  for (i=0; i<n; i++)
    eval_pts[i] = sin((( 2.0*((double)i)+1.0 ) * M_PI) / twoN);

}


/************************************************************************/
/* returns an array of n slightly changed Chebyshev nodes, i.e.
   cos( EvalPts/2 )
*/

void CosEvalPts2( int n,
		  double *eval_pts )
{
  int i;
  double twoN;

  twoN = (double) (2*n);

  for (i=0; i<n; i++)
    eval_pts[i] = cos(0.5 * (( 2.0*((double)i)+1.0 ) * M_PI) / twoN);

}


/************************************************************************/
/* returns an array of sin evaluated at slightly modified n Chebyshev
   pts, i.e. sin( EvalPts/2 )
*/

void SinEvalPts2( int n,
		  double *eval_pts )
{
  int i;
  double twoN;

  twoN = (double) (2*n);

  for (i=0; i<n; i++)
    eval_pts[i] = sin( 0.5 * (( 2.0*((double)i)+1.0 ) * M_PI) / twoN);

}

/************************************************************************/
/* L2 normed wigner little d, WHERE THE DEGREE OF THE FUNCTION IS EQUAL
   TO ONE OF ITS ORDERS. This is the function to use when starting the
   three-term recurrence at orders (m1,m2)

   arguments: m1, m2 - order of the function
              sinEval - sine values of evaluation pts
	      cosEval - cosine values of the evaluation pts
	      n - how many points there are, i.e. length of sinEval, cosEval
	      result - where to place the result, length n


   Note that, by definition, since I am starting the recurrence with this
   function, that the degree j of the function is equal to max(abs(m1), abs(m2) ).


   This has been tested and stably computes Pmm functions thru bw=512

*/

void wigSpec_L2( int m1,
		 int m2,
		 double *sinEval,
		 double *cosEval,
		 int n,
		 double *result )
{

  int i, l, delta;
  int cosPower, sinPower;
  int absM1, absM2 ;
  double dl, dm1, dm2, normFactor, sinSign ;
  double dCP, dSP ;

  absM1 = ABS( m1 ) ;
  absM2 = ABS( m2 ) ;
  l = MAX( absM1, absM2 ) ;
  delta = l - MIN( absM1, absM2 ) ;

  dl = ( double ) l ;
  dm1 = ( double ) m1 ;
  dm2 = ( double ) m2 ;
  sinSign = 1. ;
  normFactor = 1. ;
  
  for ( i = 0 ; i < delta ; i ++ )
    normFactor *= sqrt( (2.*dl - ((double) i) )/( ((double) i) + 1.) );

  /* need to adjust to make the L2-norm equal to 1 */
  normFactor *= sqrt((2.*dl+1.)/2.);

  if ( l == absM1 )
    if ( m1 >= 0 )
      {
	cosPower = l + m2 ;
	sinPower = l - m2 ;
	if ( (l - m2) % 2 )
	  sinSign = -1. ;
      }
    else
      {
	cosPower = l - m2 ;
	sinPower = l + m2 ;
      }
  else if ( m2 >= 0 )
    {
      cosPower = l + m1 ;
      sinPower = l - m1 ;
    }
  else
    {
      cosPower = l - m1 ;
      sinPower = l + m1 ;
      if ( (l + m1) % 2 )
	sinSign = -1. ;
    }

  dCP = ( double ) cosPower ;
  dSP = ( double ) sinPower ;

  for ( i = 0 ; i < n ; i++ )
    result[i] = normFactor * sinSign *
      pow( sinEval[i], dSP ) * pow( cosEval[i], dCP ) ;
}

/******************************************************************/
/**
   Given orders m1, m2, and a bandwidth bw, this function will
   generate the all the Wigner little d functions whose orders
   are (m1, m2) and degrees are j = max(|m1|, |m2|) through j = bw - 1
   using the 3-term recurrence.

   all of these Wigners will have L2 norm = 1


   let j = max(|m1|, |m2|)

   The functions generated will be

   d_{m1,m2}^j, d_{m1,m2}^{j+1}, ..., d_{m1,m2}^{bw-1}

   Each of these functions will be evaluated at the n = 2*bw-many
   points

   pi*(2 * [0..n-1] + 1) / ( 2 * n )

   If theta(k) = pi*(2*k+1)/(2*n), then what's returned will be the
   array

   d_{m1,m2}^j(theta(0)) ... d_{m1,m2}^j(theta(n-1))
   d_{m1,m2}^{j+1}(theta(0)) ... d_{m1,m2}^{j+1}(theta(n-1))
   d_{m1,m2}^{j+2}(theta(0)) ... d_{m1,m2}^{j+2}(theta(n-1)) ...
   d_{m1,m2}^{bw-1}(theta(0)) ... d_{m1,m2}^{bw-1}(theta(n-1))

   arguments: m1, m2 = orders of the functions
              bw = bandwidth
              sinEval = sine values of evaluation pts, length n = 2*bw
	      cosEval = cosine values of the evaluation pts, length n = 2*bw

	      I need the two arrays for wigSpec purposes

              sinEval2 = sine values of evaluation pts, length n = 2*bw
	      cosEval2 = cosine values of the evaluation pts, length n = 2*bw

	      result = array to store result, length (bw-m)*n
                       where m = max( |m1|, |m2| );
	      workspace = scratch area, length 6 * n = 12 * bw ;
**/

void genWig_L2( int m1,
		int m2,
		int bw,
		double *sinEval,
		double *cosEval,
		double *sinEval2,
		double *cosEval2,
		double *result,
		double *workspace )
{
  int i, lCtr, m, n ;
  int tmpM1, tmpM2 ;
  double *prevprev, *prev;
  double *tmp0, *tmp1, *tmp2, *tmp3 ;
  double an, bn, cn ;

  lCtr = 0 ;
  tmpM1 = ABS( m1 ) ;
  tmpM2 = ABS( m2 ) ;
  m = MAX( tmpM1, tmpM2 ) ;
  n = 2 * bw ;

  prevprev = workspace ;
  prev = prevprev + n ;
  tmp0 = prev + n ;
  tmp1 = tmp0 + n ;
  tmp2 = tmp1 + n ;
  tmp3 = tmp2 + n ;

  /* initialize arrays for the recurrence */
  /* set prevprev to be all 0s */
  memset( prevprev, 0, sizeof( double ) * n );

  /* prev is filled by a call to wigSpec_L2 */
  wigSpec_L2( m1, m2, sinEval2, cosEval2, n, prev );

  /* since prev is the first Wigner fct, copy it to
     the results array */
  memcpy(result, prev, sizeof(double) * n );

  /* now start doing the recurrence */
  for( i = 0 ; i < bw - m - 1; i++ )
    {
      L2_3term_so3(m+i,m1,m2,&an,&bn,&cn);
      vec_mul_so3( an, prevprev, tmp0, n ) ;     
      vec_add_scalar_so3( cn, cosEval, tmp1, n ) ;
      vec_mul_inplace_so3( bn, tmp1, n );
      vec_pt_mul_so3( prev, tmp1, tmp2, n );
      
      /* tmp3 contains d_{m1,m2}^{m+i+1} */
      vec_add_so3( tmp0, tmp2, tmp3, n );

      /* store the function values */
      memcpy( result + (i+1)*n, tmp3, sizeof(double) * n );

      /* update for the next iteration of the recurrence */
      memcpy( prevprev, prev, sizeof( double ) * n ) ;
      memcpy( prev, tmp3, sizeof( double ) * n ) ;
    }

  /* and that should be that */
}

/***************************************************************

 Just like the above genWig_L2() EXCEPT it evaluates the Wigner-d's
 at user-specified angles (in radians) and not the Chebyshev points


   arguments: m1, m2 = orders of the functions
              bw = bandwidth
              n = number of angles
              sinEval = sine values of angles, length n
	      cosEval = cosine values of angles, length n

	      I need the two arrays for wigSpec purposes

              sinEval2 = sine values of angles/2, length n
	      cosEval2 = cosine values of angles/2, length n

	      result = array to store result, length (bw-m)*n
                       where m = max( |m1|, |m2| );
	      workspace = scratch area, length 6 * n = 12 * bw ;

***************************************************/

void genWig_L2_U( int m1,
		  int m2,
		  int bw,
		  int n,
		  double *sinEval,
		  double *cosEval,
		  double *sinEval2,
		  double *cosEval2,
		  double *result,
		  double *workspace )
{
  int i, lCtr, m ;
  int tmpM1, tmpM2 ;
  double *prevprev, *prev;
  double *tmp0, *tmp1, *tmp2, *tmp3 ;
  double an, bn, cn ;

  lCtr = 0 ;
  tmpM1 = ABS( m1 ) ;
  tmpM2 = ABS( m2 ) ;
  m = MAX( tmpM1, tmpM2 ) ;

  prevprev = workspace ;
  prev = prevprev + n ;
  tmp0 = prev + n ;
  tmp1 = tmp0 + n ;
  tmp2 = tmp1 + n ;
  tmp3 = tmp2 + n ;

  /* initialize arrays for the recurrence */
  /* set prevprev to be all 0s */
  memset( prevprev, 0, sizeof( double ) * n );

  /* prev is filled by a call to wigSpec_L2 */
  wigSpec_L2( m1, m2, sinEval2, cosEval2, n, prev );

  /* since prev is the first Wigner fct, copy it to
     the results array */
  memcpy(result, prev, sizeof(double) * n );

  /* now start doing the recurrence */
  for( i = 0 ; i < bw - m - 1; i++ )
    {
      L2_3term_so3(m+i,m1,m2,&an,&bn,&cn);
      vec_mul_so3( an, prevprev, tmp0, n ) ;     
      vec_add_scalar_so3( cn, cosEval, tmp1, n ) ;
      vec_mul_inplace_so3( bn, tmp1, n );
      vec_pt_mul_so3( prev, tmp1, tmp2, n );
      
      /* tmp3 contains d_{m1,m2}^{m+i+1} */
      vec_add_so3( tmp0, tmp2, tmp3, n );

      /* store the function values */
      memcpy( result + (i+1)*n, tmp3, sizeof(double) * n );

      /* update for the next iteration of the recurrence */
      memcpy( prevprev, prev, sizeof( double ) * n ) ;
      memcpy( prev, tmp3, sizeof( double ) * n ) ;
    }

  /* and that should be that */
}


/******************************************************************/
/**
   Given orders m1, m2, and a bandwidth bw, this function will
   generate the all the Wigner little d functions whose orders
   are (m1, m2) and degrees are j = max(|m1|, |m2|) through j = bw - 1
   using the 3-term recurrence. The matrix made here will be the
   TRANSPOSE of the one produced by genWig_L2. It will be used
   for synthesis (as the above functions produces a matrix used
   for analysis).

   All of these Wigners will have L2 norm = 1


   let j = max(|m1|, |m2|)

   The functions generated will be

   d_{m1,m2}^j, d_{m1,m2}^{j+1}, ..., d_{m1,m2}^{bw-1}

   Each of these functions will be evaluated at the n = 2*bw-many
   points

   pi*(2 * [0..n-1] + 1) / ( 2 * n )

   If theta(k) = pi*(2*k+1)/(2*n), then what's returned will be the
   array

   d_{m1,m2}^j(theta(0)) ... d_{m1,m2}^{bw-1}(theta(0))
   d_{m1,m2}^j(theta(1)) ... d_{m1,m2}^{bw-1}(theta(1))
   d_{m1,m2}^j(theta(2)) ... d_{m1,m2}^{bw-1}(theta(2)) ...
   d_{m1,m2}^j(theta(n-1)) ... d_{m1,m2}^{bw-1}(theta(n-1))
   
   arguments: m1, m2 = orders of the functions
              bw = bandwidth
              sinEval = sine values of evaluation pts, length n = 2*bw
	      cosEval = cosine values of the evaluation pts, length n = 2*bw

	      I need the two arrays for wigSpec purposes

              sinEval2 = sine values of evaluation pts, length n = 2*bw
	      cosEval2 = cosine values of the evaluation pts, length n = 2*bw

	      result = array to store result, length (bw-m)*n
                       where m = max( |m1|, |m2| );
	      workspace = scratch area, length 6 * n = 12 * bw ;

   The routine won't be efficient, but it'll work.

**/

	      
void genWigTrans_L2( int m1,
		     int m2,
		     int bw,
		     double *sinEval,
		     double *cosEval,
		     double *sinEval2,
		     double *cosEval2,
		     double *result,
		     double *workspace )
{
  int i, j, lCtr, m, n ;
  int tmpM1, tmpM2 ;
  double *prevprev, *prev;
  double *tmp0, *tmp1, *tmp2, *tmp3 ;
  double an, bn, cn ;

  lCtr = 0 ;
  tmpM1 = ABS( m1 ) ;
  tmpM2 = ABS( m2 ) ;
  m = MAX( tmpM1, tmpM2 ) ;
  n = 2 * bw ;

  prevprev = workspace ;
  prev = prevprev + n ;
  tmp0 = prev + n ;
  tmp1 = tmp0 + n ;
  tmp2 = tmp1 + n ;
  tmp3 = tmp2 + n ;
  
  /* initialize arrays for the recurrence */
  /* set prevprev to be all 0s */
  memset( prevprev, 0, sizeof( double ) * n );

  /* prev is filled by a call to wigSpec_L2 */
  wigSpec_L2( m1, m2, sinEval2, cosEval2, n, prev );

  /* since prev is the first Wigner fct, copy it to
     the results array; note how I'm copying it ... every
     (bw-m) element */
  for ( i = 0 ; i < n ; i ++ )
    result[ (bw - m) * i ] = prev[ i ] ;

  /* now start doing the recurrence */
  for( i = 0 ; i < bw - m - 1; i++ )
    {
      L2_3term_so3(m+i,m1,m2,&an,&bn,&cn);
      vec_mul_so3( an, prevprev, tmp0, n ) ;     
      vec_add_scalar_so3( cn, cosEval, tmp1, n ) ;
      vec_mul_inplace_so3( bn, tmp1, n );
      vec_pt_mul_so3( prev, tmp1, tmp2, n );
      
      /* tmp3 contains d_{m1,m2}^{m+i+1} */
      vec_add_so3( tmp0, tmp2, tmp3, n );

      /* store the function values */
      for ( j = 0 ; j < n ; j ++ )
	result[ i + 1 + (bw-m)*j ] = tmp3[ j ] ;

      /* update for the next iteration of the recurrence */
      memcpy( prevprev, prev, sizeof( double ) * n ) ;
      memcpy( prev, tmp3, sizeof( double ) * n ) ;
    }

  /* and that should be that */
}

/******************************************************************/
/*

genAllWig: make ALL the Wigner little-d's necessary to do a full
           FORWARD SOFT (i.e. SO(3)) transform. Designed to be used in
	   the SOFT routines which rely on the symmetries of the
	   Wigner little-d's.

 arguments:

  bw: bandwidth of transform

  wigners: array to store ALL the wigners, of size (gulp)

     1/3 * bw^2 * (2 + 3*bw + bw^2)

  workspace: scratch space, of size 12 * n, where n = 2*bw
  
*/

void genWigAll( int bw,
		double *wigners,
		double *workspace )
{
  int m1, m2, n ;
  double *sinPts, *cosPts ;
  double *sinPts2, *cosPts2 ;
  double *scratch ;

  n = 2 * bw ;

  /* assign pointers */
  sinPts = workspace ;
  cosPts = sinPts + n ;
  sinPts2 = cosPts + n ;
  cosPts2 = sinPts2 + n ;
  scratch = cosPts2 + n ;

  /* precompute appropriate sine and cosine values */
  SinEvalPts( n, sinPts );
  CosEvalPts( n, cosPts );
  SinEvalPts2( n, sinPts2 );
  CosEvalPts2( n, cosPts2 );
  
  /* precompute Wigner little-d's for m1 = m2 = 0 */
  genWig_L2( 0, 0, bw,
	     sinPts, cosPts,
	     sinPts2, cosPts2,
	     wigners, scratch ) ;

  /* advance Wigner ptr */
  wigners += bw * n ;

  /*
    precompute Wigner little-d's for abs(m1)=abs(m2)
  */
  
  for ( m1 = 1 ; m1 < bw ; m1 ++ )
    {
      genWig_L2( m1, m1, bw,
		 sinPts, cosPts,
		 sinPts2, cosPts2,
		 wigners, scratch ) ;

      /* advance ptr */
      wigners += n * ( bw - m1 );
    }


  /*
    precompute Wigner little-d's for one order being 0,
    and the other not
  */
  
  for ( m1 = 1 ; m1 < bw ; m1 ++ )
    {
      genWig_L2( m1, 0, bw,
		 sinPts, cosPts,
		 sinPts2, cosPts2,
		 wigners, scratch ) ;

      /* advance ptr */
      wigners += n * ( bw - m1 );
    }

  /*
    precompute Wigner little-d's for m1, m2
  */
  
  for ( m1 = 1 ; m1 < bw ; m1 ++ )
    for ( m2 = m1 + 1 ; m2 < bw ; m2 ++ )
      {
	genWig_L2( m1, m2, bw,
		   sinPts, cosPts,
		   sinPts2, cosPts2,
		   wigners, scratch ) ;

	/* advance ptr */
	wigners += n * ( bw - m2 );
      }

  /* that should be that */

}


/******************************************************************/
/*

genAllWigTrans: make ALL the Wigner little-d's necessary to do a full
           INVERSE SOFT (i.e. SO(3)) transform. Designed to be used in
	   the SOFT routines which rely on the symmetries of the
	   Wigner little-d's.

 arguments:

  bw: bandwidth of transform

  wigners: array to store ALL the wigners, of size (gulp)

     1/3 * bw^2 * (2 + 3*bw + bw^2)

  workspace: scratch space, of size 12 * n, where n = 2*bw
  
*/

void genWigAllTrans( int bw,
		     double *wigners,
		     double *workspace )
{
  int m1, m2, n ;
  double *sinPts, *cosPts ;
  double *sinPts2, *cosPts2 ;
  double *scratch ;

  n = 2 * bw ;

  /* assign pointers */
  sinPts = workspace ;
  cosPts = sinPts + n ;
  sinPts2 = cosPts + n ;
  cosPts2 = sinPts2 + n ;
  scratch = cosPts2 + n ;

  /* precompute appropriate sine and cosine values */
  SinEvalPts( n, sinPts );
  CosEvalPts( n, cosPts );
  SinEvalPts2( n, sinPts2 );
  CosEvalPts2( n, cosPts2 );
  
  /* precompute Wigner little-d's for m1 = m2 = 0 */
  genWigTrans_L2( 0, 0, bw,
		  sinPts, cosPts,
		  sinPts2, cosPts2,
		  wigners, scratch ) ;

  /* advance Wigner ptr */
  wigners += bw * n ;

  /*
    precompute Wigner little-d's for abs(m1)=abs(m2)
  */
  
  for ( m1 = 1 ; m1 < bw ; m1 ++ )
    {
      genWigTrans_L2( m1, m1, bw,
		      sinPts, cosPts,
		      sinPts2, cosPts2,
		      wigners, scratch ) ;
      
      /* advance ptr */
      wigners += n * ( bw - m1 );
    }
  
  
  /*
    precompute Wigner little-d's for one order being 0,
    and the other not
  */
  
  for ( m1 = 1 ; m1 < bw ; m1 ++ )
    {
      genWigTrans_L2( m1, 0, bw,
		      sinPts, cosPts,
		      sinPts2, cosPts2,
		      wigners, scratch ) ;
      
      /* advance ptr */
      wigners += n * ( bw - m1 );
    }
  
  /*
    precompute Wigner little-d's for m1, m2
  */
  
  for ( m1 = 1 ; m1 < bw ; m1 ++ )
    for ( m2 = m1 + 1 ; m2 < bw ; m2 ++ )
      {
	genWigTrans_L2( m1, m2, bw,
			sinPts, cosPts,
			sinPts2, cosPts2,
			wigners, scratch ) ;
	
	/* advance ptr */
	wigners += n * ( bw - m2 );
      }

  /* that should be that */

}
