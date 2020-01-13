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

#ifndef _S2_LEGENDRETRANS_H
#define _S2_LEGENDRETRANS_H 1


extern void Naive_SynthesizeX( double *,
			       int ,
			       int ,
			       double *,
			       double *);

extern void Naive_AnalysisX( double *,
			     int ,
			     int ,
			     double *,
			     double *,
			     double *,
			     double *);


extern void SemiNaiveReduced( double * ,
			      int , 
			      int , 
			      double * , 
			      double * ,
			      double * ,
			      double * ,
			      fftw_plan *) ;

extern void InvSemiNaiveReduced( double * ,
				 int , 
				 int , 
				 double * , 
				 double * ,
				 double * ,
				 double * ,
				 fftw_plan * ) ;

#endif /* _S2_LEGENDRETRANS_H */
