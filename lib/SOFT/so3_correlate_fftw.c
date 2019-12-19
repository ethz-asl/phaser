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

  so3CombineCoef_fftw() :
  routine for combining spherical (S^2) Fourier coefficients of
  two functions to get the SO(3) Fourier coefficients of their
  correlation.


  - will use Wigner-d symmetries and FFTW in the inverse SO(3) transform

*/

#include <math.h>
#include <string.h>

#include <fftw3/fftw3.h>
#include "soft/s2_primitive.h"
#include "soft/utils_so3.h"



/******************************************************/
/*
  so3CombineCoef_fftw:

                  combine the S^2 spherical coefficients
                  of the PATTERN h and the SIGNAL f into a
		  SO(3) coefficient array in such a way
		  so that the inverse SO(3) transform will
		  result in the correlation of f and h, and
		  the location of the max value of the
		  inverse transform will tell you the Euler
		  angles required to rotate the PATTERN h to
		  align it with the SIGNAL f

  bwIn: bandlimit of signal and pattern

  bwOut: bandlimit of created SO(3) coefficients
         i.e. may not want to take inverse SO(3)
	 transform of a bandlimit = bwIn function

  degLim: you may want to consider coefficients through
          degLim. In which case, the coefficients for
	  degrees degLim + 1 through bwOut-1 will all be 
	  set to 0 in the SO(3) coefficient array
	  
  sigCoefR, sigCoefR:
	  Real and imaginary parts of the S^2 coefficients of
	  the SIGNAL, ordered as they would be when produced
	  by a SpharmonicKit forward spherical transform routine;
	  each is a double array of size bw^2
	  
  patCoefR, patCoefI:
	  Real and imaginary parts of the S^2 coefficients of
	  the PATTERN, ordered as they would be when produced
	  by a SpharmonicKit forward spherical transform routine;
	  each is a double array of size bw^2
	  
  so3Coef:
	  Real and imaginary parts of SO(3) coefficients of the
	  correlation of f and h; it's a FFTW_COMPLEX array of
	  size (4*bw^3-bw)/3

  NOTE about the double "wigNorm" in the function: The derivation
       of the recipe for combining S^2 coefficients to produce the
       SO(3) coefficients assumes that the Wigner-Ds are "plain"
       ones, i.e. they're not normalized. Since the C code routines
       do assume that, I have to scale the combined S^2 coefficients
       appropriately. Hence, I multiply by wigNorm in the inner-most
       for-loop.

*/

void so3CombineCoef_fftw( int bwIn,
			  int bwOut,
			  int degLim,
			  double *sigCoefR, double *sigCoefI,
			  double *patCoefR, double *patCoefI,
			  fftw_complex *so3Coef )
{
  int l, m1, m2 ;
  int fudge, dummy ;
  double tmpSigCoefR, tmpSigCoefI ;
  double tmpPatCoefR, tmpPatCoefI ;
  double wigNorm ;

  /* for sanity, set all so3coefs to 0 */
  memset( so3Coef, 0, sizeof(fftw_complex) * totalCoeffs_so3( bwOut ) );

  for( l = 0 ; l <= degLim ; l ++ )
    {
      wigNorm = 2.*M_PI*sqrt(2./(2.*l+1.)) ;
      for( m1 = -l ; m1 <= l ; m1 ++ )
	{
	  /* grab signal coefficient, at this degree and order */
	  dummy = seanindex(-m1, l, bwIn) ;
	  tmpSigCoefR = sigCoefR[ dummy ];
	  tmpSigCoefI = sigCoefI[ dummy ];

	  /* need to reset the -1 fudge factor */
	  if ( (m1 + l) % 2 )
	    fudge = -1 ;
	  else
	    fudge = 1 ;

	  for( m2 = -l ; m2 <= l ; m2 ++ )
	    {
	      dummy = seanindex( -m2, l, bwIn );

	      /*
		first take the CONJUGATE of the pattern coef
		and multiply it by the fudge factor
	      */
	      tmpPatCoefR = fudge *   patCoefR[ dummy ];
	      tmpPatCoefI = fudge * (-patCoefI[ dummy ]);

	      /* now multiply the signal coef by the pattern coef,
		 and save it in the so3 coefficient array */
	      
	      dummy = so3CoefLoc( m1, m2, l, bwOut ) ;

	      so3Coef[ dummy ][0] = wigNorm *
		( tmpSigCoefR*tmpPatCoefR - tmpSigCoefI*tmpPatCoefI ) ;

	      so3Coef[ dummy ][1] = wigNorm *
		( tmpSigCoefR*tmpPatCoefI + tmpSigCoefI*tmpPatCoefR ) ;

	      /* multiply fudge factor by -1 */
	      fudge *= -1 ;
	    }
	}
    }
}
