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
  routine for combining spherical (S^2) Fourier coefficients of
  two functions to get the SO(3) Fourier coefficients of their
  correlation.

  - will use Wigner-d symmetries in the inverse SO(3) transform

*/


#ifndef _SO3_CORRELATE_SYM_H
#define _SO3_CORRELATE_SYM_H 1

extern void so3CombineCoef( int ,
			    int ,
			    int ,
			    double * , double * ,
			    double * , double * ,
			    double * , double * ) ;

#endif
