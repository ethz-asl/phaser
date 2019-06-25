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

/*****************************************************************


  FST_semi_fly.c - routines to perform convolutions on the
  2-sphere using a combination of semi-naive and naive algorithms.

  Just like FST_semi_memo.c EXCEPT THAT THESE ROUTINES
  COMPUTE ASSOCIATED LEGENDRE FUNCTIONS ON THE FLY

  The primary functions in this package are

  1) FST_semi_fly() - computes the spherical harmonic expansion.
  2) InvFST_semi_fly() - computes the inverse spherical harmonic transform.
  3) FZT_semi_fly() - computes the zonal harmonic transform.
  4) TransMult() - Multiplies harmonic coefficients using Driscoll-Healy
                    result.  Dual of convolution in "time" domain.
  5) Conv2Sphere_semi_fly() - Convolves two functins defined on the 2-sphere,
                          using seminaive transform

  and one utility function:

  1) seanindex(): Given bandwidth bw, seanindex(m,l,bw) will give the
     position of the coefficient f-hat(m,l) in the one-row array

  For descriptions on calling these functions, see the documentation
  preceding each function.


*/


#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "fftw3.h"
#include "soft/makeweights.h"
#include "soft/s2_primitive.h"
#include "soft/s2_cospmls.h"
#include "soft/s2_legendreTransforms.h"


/************************************************************************/



/************************************************************************/
/*
  performs a spherical harmonic transform using the semi-naive
  and naive algorithms


  bw -> bandwidth of problem
  size -> size = 2*bw -> dimension of input array (recall that
          sampling is done at twice the bandwidth)

  The inputs rdata and idata are expected to be pointers to
  size x size arrays. The array rdata contains the real parts
  of the function samples, and idata contains the imaginary
  parts.
  
  rcoeffs and icoeffs are expected to be pointers to bw x bw arrays,
  and will contain the harmonic coefficients in a "linearized" form.
  The array rcoeffs contains the real parts of the coefficients,
  and icoeffs contains the imaginary parts.

  workspace needs to be a double pointer to an array of size
  (10 * bw^2) + (21 * bw)

  cutoff -> what order to switch from semi-naive to naive
            algorithm.

 
   Output Ordering of coeffs f(m,l) is
   f(0,0) f(0,1) f(0,2) ... f(0,bw-1)
          f(1,1) f(1,2) ... f(1,bw-1)
          etc.
                 f(bw-2,bw-2), f(bw-2,bw-1)
		               f(bw-1,bw-1)
			       f(-(bw-1),bw-1)
		 f(-(bw-2),bw-2) f(-(bw-2),bw-1)
	  etc.
	          f(-2,2) ... f(-2,bw-1)
	  f(-1,1) f(-1,2) ... f(-1,bw-1)
    
   This only requires an array of size (bw*bw).  If zero-padding
   is used to make the indexing nice, then you need a an
   (2bw-1) * bw array - but that is not done here.
   Because of the amount of space necessary for doing
   large transforms, it is important not to use any
   more than necessary.

   dataformat =0 -> samples are complex, =1 -> samples real
*/

void FST_semi_fly( double *rdata, double *idata, 
		   double *rcoeffs, double *icoeffs, 
		   int bw,
		   double *workspace,
		   int dataformat,
		   int cutoff,
		   fftw_plan *dctPlan,
		   fftw_plan *fftPlan,
		   double *weights )
{
  int size, m, i, j;
  int dummy0, dummy1 ;
  double *rres, *ires;
  double *rdataptr, *idataptr;
  double *fltres, *scratchpad;
  double *sin_values, *eval_pts;
  double *pmls ;
  double pow_one;
  double tmpA ;
  double tmpSize ;

  size = 2*bw ;
  tmpSize = 1./((double) size );
  tmpA = sqrt( 2. * M_PI ) ;

  /* assign space */
  rres = workspace;               /* needs (size*size) = (4*bw^2) */
  ires = rres + (size * size);    /* needs (size*size) = (4*bw^2) */ 
  fltres = ires + (size * size);  /* needs bw  */
  sin_values = fltres + bw;       /* needs (2*bw)  */
  eval_pts = sin_values + (2*bw); /* needs (2*bw)  */
  pmls = eval_pts + (2*bw);       /* needs (2*bw*bw) */
  scratchpad = pmls + (2*bw*bw);  /* needs (16 * bw)  */


  /* do the FFTs along phi */
  fftw_execute_split_dft( *fftPlan,
			  rdata, idata,
			  rres, ires ) ; 

  /* normalize */
  tmpSize *= tmpA ;
  for( j = 0 ; j < size*size ; j ++ )
    {
      rres[j] *= tmpSize ;
      ires[j] *= tmpSize ;
    }

  
  /* point to start of output data buffers */
  rdataptr = rcoeffs;
  idataptr = icoeffs;
  
  for (m=0; m<bw; m++) {
 
    /*** test to see if before cutoff or after ***/
    if (m < cutoff){

      /** have to generate cosine series of pmls **/
      CosPmlTableGen(bw,m,pmls,scratchpad);
      
      /* do the real part */
      SemiNaiveReduced(rres+(m*size), 
		       bw, 
		       m, 
		       fltres, 
		       scratchpad,
		       pmls,
		       weights,
		       dctPlan );
      
      /* now load real part of coefficients into output space */  
      memcpy(rdataptr, fltres, sizeof(double) * (bw - m));
      
      rdataptr += bw-m;
      
      /* do imaginary part */
      SemiNaiveReduced(ires+(m*size), 
		       bw, 
		       m, 
		       fltres,
		       scratchpad,
		       pmls,
		       weights,
		       dctPlan );
      
      /* now load imaginary part of coefficients into output space */  
      memcpy(idataptr, fltres, sizeof(double) * (bw - m));
      
      idataptr += bw-m;
      
    }
    else{

      /** have to generate pmls - note I'm using CosPmls array **/
      PmlTableGen(bw,m,pmls,scratchpad);

      /* do real part */      
      Naive_AnalysisX(rres+(m*size),
		      bw,
		      m,
		      weights,
		      fltres,
		      pmls,
		      scratchpad );
      memcpy(rdataptr, fltres, sizeof(double) * (bw - m));
      rdataptr += bw-m;
      
      /* do imaginary part */
      Naive_AnalysisX(ires+(m*size),
		      bw,
		      m,
		      weights,
		      fltres,
		      pmls,
		      scratchpad );
      memcpy(idataptr, fltres, sizeof(double) * (bw - m));
      idataptr += bw-m;
    }
  }
  
  /*** now do upper coefficients ****/
  
  /* now if the data is real, we don't have to compute the
     coefficients whose order is less than 0, i.e. since
     the data is real, we know that
     f-hat(l,-m) = (-1)^m * conjugate(f-hat(l,m)),
     so use that to get the rest of the coefficients
     
     dataformat =0 -> samples are complex, =1 -> samples real
     
     */
  
  if( dataformat == 0 ){
    
    /* note that m is greater than bw here, but this is for
       purposes of indexing the input data arrays.  
       The "true" value of m as a parameter for Pml is
       size - m  */
    
    for (m=bw+1; m<size; m++) {


      if ( (size-m) < cutoff )
	{
	  /** have to generate cosine series of pmls **/
	  CosPmlTableGen(bw,size-m,pmls,scratchpad);
 
	  /* do real part */
	  SemiNaiveReduced(rres+(m*size), 
			   bw, 
			   size-m, 
			   fltres, 
			   scratchpad,
			   pmls,
			   weights,
			   dctPlan ) ;
      
	  /* now load real part of coefficients into output space */  
	  if ((m % 2) != 0) {
	    for (i=0; i<m-bw; i++)
	      rdataptr[i] = -fltres[i];
	  }
	  else {
	    memcpy(rdataptr, fltres, sizeof(double) * (m - bw));
	  }
	  rdataptr += m-bw;
      
	  /* do imaginary part */
	  SemiNaiveReduced(ires+(m*size), 
			   bw, 
			   size-m, 
			   fltres,
			   scratchpad,
			   pmls,
			   weights,
			   dctPlan );

      
	  /* now load real part of coefficients into output space */  
	  if ((m % 2) != 0) {
	    for (i=0; i<m-bw; i++)
	      idataptr[i] = -fltres[i];
	  }
	  else {
	    memcpy(idataptr, fltres, sizeof(double) * (m - bw));
	  }
	  idataptr += m-bw;
	}
      else
	{

	  /** have to generate pmls - note I'm using CosPmls array **/
	  PmlTableGen(bw,size-m,pmls,scratchpad);

	  Naive_AnalysisX(rres+(m*size),
			  bw,
			  size-m,
			  weights,
			  fltres,
			  pmls,
			  scratchpad);

	  /* now load real part of coefficients into output space */  
	  if ( m % 2 ) {
	    for (i=0; i<m-bw; i++)
	      rdataptr[i] = -fltres[i];
	  }
	  else {
	    memcpy(rdataptr, fltres, sizeof(double) * (m - bw));
	  }
	  rdataptr += m-bw;

	  /* do imaginary part */
	  Naive_AnalysisX(ires+(m*size),
			  bw,
			  size-m,
			  weights,
			  fltres,
			  pmls,
			  scratchpad);

	  /* now load imag part of coefficients into output space */  
	  if ( m % 2 ) {
	    for (i=0; i<m-bw; i++)
	      idataptr[i] = -fltres[i];
	  }
	  else {
	    memcpy(idataptr, fltres, sizeof(double) * (m - bw));
	  }
	  idataptr += m-bw;

	}
    }
  }
  else {                   /**** if the data is real ****/
      pow_one = 1.0;
      for(i = 1; i < bw; i++){
	pow_one *= -1.0;
	for( j = i; j < bw; j++){
	  dummy0 = seanindex(i, j, bw);
	  dummy1 = seanindex(-i, j, bw);

	  rcoeffs[dummy1] =
	    pow_one * rcoeffs[dummy0];
	  icoeffs[dummy1] =
	    -pow_one * icoeffs[dummy0];
	}
      }
  }
  
}


/************************************************************************/
/* Inverse spherical harmonic transform.  Inputs rcoeffs and icoeffs
   are harmonic coefficients stored in (bw * bw) arrays in the order
   spec'ed above.  rdata and idata are (size x size) arrays with
   the transformed result.  size is expected to be 2 * bw.
   transpose_spharmonic_pml_table should be the (double **) result of a call
   to Transpose_Spharmonic_Pml_Table()

   workspace is (10 * bw^2) + (24 * bw)

*/

/*      dataformat =0 -> samples are complex, =1 -> samples real */

void InvFST_semi_fly(double *rcoeffs, double *icoeffs, 
		     double *rdata, double *idata, 
		     int bw, 
		     double *workspace,
		     int dataformat,
		     int cutoff,
		     fftw_plan *idctPlan,
		     fftw_plan *ifftPlan )
{
  int size, m, i, n;
  double *rdataptr, *idataptr;
  double *rfourdata, *ifourdata;
  double *rinvfltres, *iminvfltres, *scratchpad;
  double *sin_values, *eval_pts;
  double *pmls ;
  double tmpA ;

  size = 2*bw ;

  /* allocate space */
  rfourdata = workspace;                  /* needs (size * size) */
  ifourdata = rfourdata + (size * size);  /* needs (size * size) */
  rinvfltres = ifourdata + (size * size); /* needs (2 * bw) */
  iminvfltres = rinvfltres + (2 * bw);    /* needs (2 * bw) */
  sin_values = iminvfltres + (2 * bw);    /* needs (2 * bw) */
  eval_pts = sin_values + (2 * bw);       /* needs (2 * bw) */
  pmls = eval_pts + (2*bw) ;              /* needs (2*bw*bw) */
  scratchpad = pmls + (2*bw*bw);          /* needs (16 * bw) */
  
  /* load up the sin_values array */
  n = 2*bw;
  ArcCosEvalPts(n, eval_pts);
  for (i=0; i<n; i++)
    sin_values[i] = sin(eval_pts[i]);

  /* Now do all of the inverse Legendre transforms */
  rdataptr = rcoeffs;
  idataptr = icoeffs;

  for (m=0; m<bw; m++) {
    
    if(m < cutoff){

      /** have to generate cosine series of pmls **/
      CosPmlTableGen(bw,m,pmls,scratchpad);
      /** now take transpose **/
      Transpose_CosPmlTableGen(bw,m,pmls,pmls+TableSize(m,bw));
      
      /* do real part first */       
      InvSemiNaiveReduced(rdataptr,
			  bw,
			  m,
			  rinvfltres,
			  pmls+TableSize(m,bw),
			  sin_values,
			  scratchpad,
			  idctPlan ) ;
      
      /* now do imaginary part */      
      InvSemiNaiveReduced(idataptr,
			  bw,
			  m,
			  iminvfltres,
			  pmls+TableSize(m,bw),
			  sin_values,
			  scratchpad,
			  idctPlan );
      
      /* will store normal, then tranpose before doing inverse fft */
      memcpy(rfourdata+(m*size), rinvfltres, sizeof(double) * size);
      memcpy(ifourdata+(m*size), iminvfltres, sizeof(double) * size);

      /* move to next set of coeffs */      
      rdataptr += bw-m;
      idataptr += bw-m;
      
    }
    else
      {
	/** have to generate pmls - note I'm using CosPmls array,
	    and that I don't have to transpose **/
	PmlTableGen(bw,m,pmls,scratchpad);

	/* first do the real part */	  
	Naive_SynthesizeX(rdataptr,
			  bw,
			  m,
			  rinvfltres,
			  pmls );
	  
	/* now do the imaginary */	
	Naive_SynthesizeX(idataptr,
			  bw,
			  m,
			  iminvfltres,
			  pmls );

	/* will store normal, then tranpose before doing inverse fft    */
	memcpy(rfourdata+(m*size), rinvfltres, sizeof(double) * size);
	memcpy(ifourdata+(m*size), iminvfltres, sizeof(double) * size);
	  
	/* move to next set of coeffs */
	rdataptr += bw-m;
	idataptr += bw-m;
      }
    
  } /* closes m loop */
  
  
  /* now fill in zero values where m = bw (from problem definition) */
  memset(rfourdata + (bw * size), 0, sizeof(double) * size);
  memset(ifourdata + (bw * size), 0, sizeof(double) * size);

  
  /* now if the data is real, we don't have to compute the
     coefficients whose order is less than 0, i.e. since
     the data is real, we know that
     invf-hat(l,-m) = conjugate(invf-hat(l,m)),
     so use that to get the rest of the real data
     
     dataformat =0 -> samples are complex, =1 -> samples real
     
  */
  if(dataformat == 0){
    
    /* now do negative m values */
    
    for (m=bw+1; m<size; m++) 
      {
	/*
	  fprintf(stderr,"m = %d\n",-(size-m));
	*/

	if ( (size-m) < cutoff )
	  {
	    /** have to generate cosine series of pmls **/
	    CosPmlTableGen(bw,size-m,pmls,scratchpad);
	    /** now take transpose **/
	    Transpose_CosPmlTableGen(bw,size-m,pmls,pmls+TableSize(size-m,bw));

	    /* do real part first */
	    InvSemiNaiveReduced(rdataptr,
				bw,
				size - m,
				rinvfltres,
				pmls+TableSize(size-m,bw),
				sin_values,
				scratchpad,
				idctPlan ) ;
	
	    /* now do imaginary part */	
	    InvSemiNaiveReduced(idataptr,
				bw,
				size - m,
				iminvfltres,
				pmls+TableSize(size-m,bw),
				sin_values,
				scratchpad,
				idctPlan ) ;
	
	    /* will store normal, then tranpose before doing inverse fft    */
	    if ( m % 2 )
	      for(i=0; i< size; i++){
		rinvfltres[i] *= -1.0 ;
		iminvfltres[i] *= -1.0 ;
	      }
	
	    memcpy(rfourdata + (m*size), rinvfltres, sizeof(double) * size);
	    memcpy(ifourdata + (m*size), iminvfltres, sizeof(double) * size);
	
	    /* move to next set of coeffs */
	    rdataptr += bw-(size-m);
	    idataptr += bw-(size-m);
	  }
	else
	  {

	    /** have to generate pmls - note I'm using CosPmls array,
		and that I don't have to transpose **/
	    PmlTableGen(bw,size-m,pmls,scratchpad);

	    /* first do the real part */
	    Naive_SynthesizeX(rdataptr,
			      bw,
			      size-m,
			      rinvfltres,
			      pmls );
	  
	    /* now do the imaginary */	
	    Naive_SynthesizeX(idataptr,
			      bw,
			      size-m,
			      iminvfltres,
			      pmls );

	    /* will store normal, then tranpose before doing inverse fft    */
	    if ( m % 2 )
	      for(i=0; i< size; i++){
		rinvfltres[i] *= -1.0 ;
		iminvfltres[i] *= -1.0 ;
	      }
	
	    memcpy(rfourdata + (m*size), rinvfltres, sizeof(double) * size);
	    memcpy(ifourdata + (m*size), iminvfltres, sizeof(double) * size);
	
	    /* move to next set of coeffs */
	    rdataptr += bw-(size-m);
	    idataptr += bw-(size-m);

	  }

      } /* closes m loop */
  }
  else {
    for(m = bw + 1; m < size; m++){

      memcpy(rfourdata+(m*size), rfourdata+((size-m)*size),
	     sizeof(double) * size);
      memcpy(ifourdata+(m*size), ifourdata+((size-m)*size),
	     sizeof(double) * size);
      for(i = 0; i < size; i++)
	ifourdata[(m*size)+i] = -ifourdata[(m*size)+i];
    }
  }

  /* normalize */
  tmpA = 1./(sqrt(2.*M_PI) );
  for(i=0;i<4*bw*bw;i++)
    {
      rfourdata[i] *= tmpA ;
      ifourdata[i] *= tmpA ;
    }

  fftw_execute_split_dft( *ifftPlan,
			  ifourdata, rfourdata,
			  idata, rdata );
  
  /* amscray */
  
}
