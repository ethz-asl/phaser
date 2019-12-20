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

/**
   
   header file for functions that are all concerned with the construction of
   Wigner functions:

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
          make an array of wigners

   genWig_L2():
	  make an array of wigners at user-specified angles

   genWigTrans_L2():
          make an array of wigners (the transpose of above function)

   genAllWig():
          make ALL the Wigner little-d's necessary to do a full
	  FORWARD SOFT (i.e. SO(3)) transform

   genAllWigTrans():
          make ALL the Wigner little-d's necessary to do a full
	  INVERSE SOFT (i.e. SO(3)) transform


  ************************************************************************/


#ifndef _MAKEWIGNER_H
#define _MAKEWIGNER_H 1

extern double L2_aN_so3( int ,
		     int ,
		     int ) ;

extern double L2_bN_so3( int ,
		     int ,
		     int ) ;

extern double L2_cN_so3( int ,
		     int ,
		     int ) ;

extern void EvalPtsW( int ,
		      double * ) ;

extern void CosEvalPts( int ,
			double * ) ;

extern void SinEvalPts( int ,
			double * ) ;

extern void CosEvalPts2( int ,
			 double * ) ;

extern void SinEvalPts2( int ,
			 double * ) ;

extern void wigSpec_L2( int ,
			int ,
			double * ,
			double * ,
			int ,
			double * ) ;

extern void genWig_L2( int ,
		       int ,
		       int ,
		       double * ,
		       double * ,
		       double * ,
		       double * ,
		       double * ,
		       double * ) ;

extern void genWig_L2_U( int ,
			 int ,
			 int ,
			 int ,
			 double * ,
			 double * ,
			 double * ,
			 double * ,
			 double * ,
			 double * ) ;

extern void genWigTrans_L2( int ,
			    int ,
			    int ,
			    double * ,
			    double * ,
			    double * ,
			    double * ,
			    double * ,
			    double * ) ;

extern void genWigAll( int ,
		       double * ,
		       double * ) ;

extern void genWigAllTrans( int ,
			    double * ,
			    double * ) ;

#endif /* _MAKEWIGNER_H */
