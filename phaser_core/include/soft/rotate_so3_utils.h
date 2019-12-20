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

  header file for rotate.c -> functions having to do with
  rotating bandlimited functions defined on the sphere

*/


#ifndef _ROTATESO3_UTILS_H
#define _ROTATESO3_UTILS_H 1

extern void genExp( int ,
		    double ,
		    double * ,
		    double * ) ;

extern void wignerdmat( int ,
			double * ,
			double * ,
			double * ,
			double * ,
			double * ) ;

extern void rotateCoefDegree( int ,
			      double * , double * ,
			      double * , double * ,
			      double * , double * ,
			      double * , double * ,
			      int ,
			      double * ) ;

extern void rotateCoefAll( int ,
			   int ,
			   int ,
			   double ,
			   double ,
			   double ,
			   double * , double * ,
			   double * , double * ,
			   double * ) ;

extern void genExp_mem( int ,
			double ,
			double * ,
			double * ) ;

extern void wignerdmat_mem( int ,
			    double * ,
			    double * ,
			    double * ,
			    double * ,
			    double * ) ;

extern void rotateCoefDegree_mem( int ,
				  double * , double * ,
				  double * , double * ,
				  double * , double * ,
				  double * , double * ,
				  int ,
				  double * ) ;

extern void rotateCoefAll_mem( int ,
			       int ,
			       double ,
			       double ,
			       double ,
			       double * , double * ,
			       double * ) ;

extern void rotateFct_mem( int , int ,
			   double * , double * ,
			   double , double , double ,
			   double * , 
			   double ** ,
			   double ** ) ;

#endif /* #ifndef _ROTATESO3_UTILS_H */
