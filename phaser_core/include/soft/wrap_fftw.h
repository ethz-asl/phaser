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

#include <fftw3/fftw3.h>

#ifndef _WRAP_FFTW_H
#define _WRAP_FFTW_H

extern void Forward_SO3_Naive_fftw_W( int ,
				      fftw_complex * ,
				      fftw_complex * ,
				      int ) ;

extern void Inverse_SO3_Naive_fftw_W( int ,
				      fftw_complex * ,
				      fftw_complex * ,
				      int ) ;

extern void s2RotateFFTW( int ,
			  double *,
			  double *,
			  double ,
			  double ,
			  double ,
			  int ) ;

#endif /* _WRAP_FFTW_H */
