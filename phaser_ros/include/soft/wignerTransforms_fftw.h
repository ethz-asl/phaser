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
  header file for wigner transform functions that use fftw
  and symmetries

  wigNaiveAnalysis_fftw(): forward wigner transform (spatial -> spectral)
  wigNaiveAnalysis_fftwX(): forward wigner transform (spatial -> spectral)
  wigNaiveAnalysis_fftwY(): forward wigner transform (spatial -> spectral)
  wigNaiveSynthesis_fftw(): inverse wigner transform (spectral -> spatial)
  wigNaiveSynthesis_fftwX(): inverse wigner transform (spectral -> spatial)
  wigNaiveSynthesis_fftwY(): inverse wigner transform (spectral -> spatial)


*/



#ifndef _WIGNERTRANSFORMS_FFTW_H
#define _WIGNERTRANSFORMS_FFTW_H 1

extern void wigNaiveAnalysis_fftw( int ,
				   int ,
				   int ,
				   fftw_complex * ,
				   double * ,
				   double * ,
				   fftw_complex * ,
				   fftw_complex *  ) ;

extern void wigNaiveAnalysis_fftwX( int ,
				    int ,
				    int ,
				    fftw_complex * ,
				    double * ,
				    double * ,
				    fftw_complex * ,
				    fftw_complex *  ) ;

extern void wigNaiveAnalysis_fftwY( int ,
				    int ,
				    int ,
				    fftw_complex * ,
				    double * ,
				    double * ,
				    fftw_complex * ,
				    fftw_complex *  ) ;

extern void wigNaiveSynthesis_fftw( int ,
				    int ,
				    int ,
				    fftw_complex * ,
				    double * ,
				    fftw_complex * ,
				    fftw_complex * ) ;

extern void wigNaiveSynthesis_fftwX( int ,
				     int ,
				     int ,
				     fftw_complex * ,
				     double * ,
				     fftw_complex * ,
				     fftw_complex *  ) ;

extern void wigNaiveSynthesis_fftwY( int ,
				     int ,
				     int ,
				     fftw_complex * ,
				     double * ,
				     fftw_complex * ,
				     fftw_complex *  ) ;


#endif /* _WIGNERTRANSFORMS_FFTW_H */
