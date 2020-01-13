/***************************************************************************
  **************************************************************************
    
  Spherical Harmonic Transform Kit 2.7
    
  Copyright 1997-2003  Sean Moore, Dennis Healy,
                       Dan Rockmore, Peter Kostelec
  Copyright 2004  Peter Kostelec, Dan Rockmore

  This file is part of SpharmonicKit.

  SpharmonicKit is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  SpharmonicKit is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  See the accompanying LICENSE file for details.

  ************************************************************************
  ************************************************************************/

#ifndef _COSPMLS_H
#define _COSPMLS_H

extern int Power2Ceiling( int ) ;

extern int TableSize( int ,
		      int ) ;

extern int Spharmonic_TableSize( int ) ;

extern int Reduced_SpharmonicTableSize( int ,
					int ) ;

extern int Reduced_Naive_TableSize( int ,
				    int ) ;

extern int NewTableOffset( int ,
			   int ) ;

extern int TableOffset( int ,
			int ) ;

extern void CosPmlTableGen( int ,
			    int ,
			    double * ,
			    double * ) ;

extern void CosPmlTableGenLim( int , 
			       int ,
			       int ,
			       double * ,
			       double * ) ;

extern int RowSize( int ,
		    int ) ;

extern int Transpose_RowSize( int ,
			      int ,
			      int ) ;

extern void Transpose_CosPmlTableGen( int ,
				      int ,
				      double * ,
				      double * ) ;

extern double **Spharmonic_Pml_Table( int ,
				      double * ,
				      double * ) ;

extern double **Reduced_Spharmonic_Pml_Table( int ,
					      int ,
					      double * ,
					      double * ) ;

extern double **Transpose_Spharmonic_Pml_Table( double ** ,
						int ,
						double * ,
						double * ) ;

extern double **Reduced_Transpose_Spharmonic_Pml_Table( double ** ,
							int ,
							int ,
							double * ,
							double * ) ;

extern double **SemiNaive_Naive_Pml_Table( int ,
					   int ,
					   double * ,
					   double * ) ;

extern double **Transpose_SemiNaive_Naive_Pml_Table( double ** , 
						     int ,
						     int ,
						     double * ,
						     double * ) ;

#endif

