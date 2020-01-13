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

  header file for basic utility functions


  vec_add_cx() - add two COMPLEX vectors
  vec_mul_cx() - multiply COMPLEX vector by a scalar
  vec_pt_mul_cx() - pointwise-multiply double vector with a COMPLEX vector

  transpose_cx() - transpose a COMPLEX matrix

*/


#ifndef _UTILS_VEC_CX_H
#define _UTILS_VEC_CX_H 1

extern void vec_add_cx( fftw_complex * ,
			fftw_complex * ,
			fftw_complex * ,
			int ) ;

extern void vec_mul_cx( double ,
			fftw_complex * ,
			fftw_complex * ,
			int ) ;

extern void vec_pt_mul_cx( double * ,
			   fftw_complex * ,
			   fftw_complex * ,
			   int ) ;

extern void transpose_cx(  fftw_complex * ,
			   fftw_complex * ,
			   int ,
			   int ) ;

#endif /* _UTILS_VEC_CX_H */
