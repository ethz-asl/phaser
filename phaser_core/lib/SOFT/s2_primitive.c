/***************************************************************************
  **************************************************************************
  
  S2kit 1.0
  A lite version of Spherical Harmonic Transform Kit

  Copyright (c) 2004 Peter Kostelec, Dan Rockmore

  This file is part of S2kit.

  S2kit is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  S2kit is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  See the accompanying LICENSE file for details.
  
  ************************************************************************
  ************************************************************************/

#include <math.h>
#include <string.h>  /* to declare memcpy */
#include <stdlib.h>

#ifndef PI
#define PI 3.14159265358979
#endif

#define compmult(a,b,c,d,e,f) (e) = ((a)*(c))-((b)*(d)); (f) = ((a)*(d))+((b)*(c))


/************************************************************************/
/* Recurrence coefficients */
/************************************************************************/
/* Recurrence coefficents for L2-normed associated Legendre
   recurrence.  When using these coeffs, make sure that
   inital Pmm function is also L2-normed */
/* l represents degree, m is the order */

double L2_an(int m,
	     int l)
{
  return (sqrt((((double) (2*l+3))/((double) (2*l+1))) *
	       (((double) (l-m+1))/((double) (l+m+1)))) *
	  (((double) (2*l+1))/((double) (l-m+1))));

}

/* note - if input l is zero, need to return 0 */
double L2_cn(int m,
	     int l) 
{
  if (l != 0) {
    return (-1.0 *
	  sqrt((((double) (2*l+3))/((double) (2*l-1))) *
	       (((double) (l-m+1))/((double) (l+m+1))) *
	       (((double) (l-m))/((double) (l+m)))) *
	  (((double) (l+m))/((double) (l-m+1))));
  }
  else
    return 0.0;

}

/************************************************************************/
/* vector arithmetic operations */
/************************************************************************/
/* does result = data1 + data2 */
/* result and data are vectors of length n */

void vec_add(double *data1,
	     double *data2,
	     double *result,
	     int n)
{
  int k;

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
void vec_mul(double scalar,
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
/* point-by-point multiplication of vectors */

void vec_pt_mul(double *data1,
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


/************************************************************************/
/* returns an array of the angular arguments of n Chebyshev nodes */
/* eval_pts points to a double array of length n */

void ArcCosEvalPts(int n,
		   double *eval_pts)
{
    int i;
    double twoN;

    twoN = (double) (2 * n);

   for (i=0; i<n; i++)
     eval_pts[i] = (( 2.0*((double)i)+1.0 ) * PI) / twoN;

}
/************************************************************************/
/* returns an array of n Chebyshev nodes */

void EvalPts( int n,
	      double *eval_pts)
{
    int i;
    double twoN;

    twoN = (double) (2*n);

   for (i=0; i<n; i++)
     eval_pts[i] = cos((( 2.0*((double)i)+1.0 ) * PI) / twoN);

}

/************************************************************************/
/* L2 normed Pmm.  Expects input to be the order m, an array of
 evaluation points arguments of length n, and a result vector of length n */
/* The norming constant can be found in Sean's PhD thesis */
/* This has been tested and stably computes Pmm functions thru bw=512 */

void Pmm_L2( int m,
	     double *eval_pts,
	     int n,
	     double *result)
{
  int i;
  double md, id, mcons;

  id = (double) 0.0;
  md = (double) m;
  mcons = sqrt(md + 0.5);

  for (i=0; i<m; i++) {
    mcons *= sqrt((md-(id/2.0))/(md-id));
    id += 1.0;
  }
  if (m != 0 )
    mcons *= pow(2.0,-md/2.0);
  if ((m % 2) != 0) mcons *= -1.0;

  for (i=0; i<n; i++) 
    result[i] = mcons * pow(sin(eval_pts[i]),md);

}

/************************************************************************/
/************************************************************************/
/*
  This piece of code synthesizes a function which is the weighted sum of 
  L2-normalized associated Legendre functions.

  The coeffs array should contain bw - m coefficients ordered from
  zeroth degree to bw-1, and eval_pts should be an array of the
  arguments (arccos) of the desired evaluation points of length 2*bw. 

  Answer placed in result (and has length 2*bw).
  
  workspace needs to be of size 16 * bw
  
*/

void P_eval(int m,
	    double *coeffs,
	    double *eval_args,
	    double *result,
	    double *workspace,
	    int bw)
{
    double *prev, *prevprev, *temp1, *temp2, *temp3, *temp4, *x_i;
    int i, j, n;
    double splat;

    prevprev = workspace;
    prev = prevprev + (2*bw);
    temp1 = prev + (2*bw);
    temp2 = temp1 + (2*bw);
    temp3 = temp2 + (2*bw);
    temp4 = temp3 + (2*bw);
    x_i = temp4 + (2*bw);

    n = 2*bw;

    /* now get the evaluation nodes */
    EvalPts(n,x_i);

    /*   for(i=0;i<n;i++)
      fprintf(stderr,"in P_eval evalpts[%d] = %lf\n", i, x_i[i]);
      */   
    for (i=0; i<n; i++) 
      prevprev[i] = 0.0;

    if (m == 0) {
	for (i=0; i<n; i++) {
	  prev[i] = 0.707106781186547; /* sqrt(1/2) */

	  /* now mult by first coeff and add to result */
	  result[i] = coeffs[0] * prev[i];
	}
    }
    else {
	Pmm_L2(m, eval_args, n, prev);
	splat = coeffs[0];
	for (i=0; i<n; i++)
	  result[i] = splat * prev[i];
    }

    for (i=0; i<bw-m-1; i++) {
	vec_mul(L2_cn(m,m+i),prevprev,temp1,n);
	vec_pt_mul(prev, x_i, temp2, n);
	vec_mul(L2_an(m,m+i), temp2, temp3, n);
	vec_add(temp3, temp1, temp4, n); /* temp4 now contains P(m,m+i+1) */
	/* now add weighted P(m,m+i+1) to the result */
	splat = coeffs[i+1];
	for (j=0; j<n; j++)
	  result[j] += splat * temp4[j];
	memcpy(prevprev, prev, sizeof(double) * n);
	memcpy(prev, temp4, sizeof(double) * n);
    }

}


/*****************************************************************

  Given bandwidth bw, seanindex(m,l,bw) will give the position of the
  coefficient f-hat(m,l) in the one-row array that Sean stores the spherical
  coefficients. This is needed to help preserve the symmetry that the
  coefficients have: (l = degree, m = order, and abs(m) <= l)

  f-hat(l,-m) = (-1)^m * conjugate( f-hat(l,m) )

  Thanks for your help Mark!

  ******************************************************************/

int seanindex(int m,
	      int l,
	      int bw)
{     
  int bigL;

  bigL = bw - 1;

  if( m >= 0 )
    return( m * ( bigL + 1 ) - ( ( m * (m - 1) ) /2 ) + ( l - m ) );
  else
    return( ( ( bigL * ( bigL + 3 ) ) /2 ) + 1 +
	    ( ( bigL + m ) * ( bigL + m + 1 ) / 2 ) + ( l - abs( m ) ) );
}


/*****************************************************************
  just like seanindex(m,l,bw) but returns the array coordinates
  for (l,m) AND (l,-m)

  ASSUMING THE M IS GREATER THAN 0 !!!

  this is used in the FST_semi routine

  loc is a 2 element integer array

  ******************************************************************/

void seanindex2(int m,
		int l,
		int bw,
		int *loc)
{     
  int bigL;
  
  bigL = bw - 1;
  
  /* first index for (l,m) */
  loc[0] = m * ( bigL + 1 ) - ( ( m * (m - 1) ) /2 ) + ( l - m );
  
  /* second index for (l,-m) */
  loc[1] = ( ( bigL * ( bigL + 3 ) ) /2 ) + 1 +
    ( ( bigL - m ) * ( bigL - m + 1 ) / 2 ) + ( l -  m ) ;

}


/****************************************************

  just a function to transpose a square array IN PLACE !!!

  array = array to transpose
  size = dimension of array (assuming the array is square, size * size)

  **************************************************/

void transpose(double *array,
	       int size)
{
  register int i, j;
  double t1, t2, t3, t4;

  for(i = 0; i < size; i += 2)
    {
      t1 = array[(i * size) + i + 1];
      array[(i * size) + i + 1] = array[((i + 1) * size) + i];
      array[((i + 1) * size) + i] = t1;
      for(j = (i + 2); j < size; j += 2)
	{
	  t1 = array[(i*size)+j]; t2 = array[(i*size)+j+1];
	  t3 = array[((i+1)*size)+j]; t4 = array[((i+1)*size)+j+1];
	  array[(i*size)+j] = array[(j*size)+i];
	  array[(i*size)+j+1] = array[((j+1)*size)+i];
	  array[((i+1)*size)+j] = array[(j*size)+i+1];
	  array[((i+1)*size)+j+1] = array[((j+1)*size)+i+1];
	  array[(j*size)+i] = t1;
	  array[((j+1)*size)+i] = t2;
	  array[(j*size)+i+1] = t3;
	  array[((j+1)*size)+i+1] = t4;
	}
    }
}

