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

/* external interface for FST_semi_fly.c */

#ifndef _S2_SEMI_FLY_H
#define _S2_SEMI_FLY_H


extern void FST_semi_fly( double *, double *,
			  double *, double *,
			  int ,
			  double *,
			  int ,
			  int ,
			  fftw_plan *,
			  fftw_plan *,
			  double * );

extern void InvFST_semi_fly( double *, double *, 
			     double *, double *,
			     int , 
			     double *,
			     int ,
			     int ,
			     fftw_plan *,
			     fftw_plan * );


#endif /* _S2_SEMI_FLY_H */
