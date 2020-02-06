#include "phaser/distribution/bingham-normalization-constant.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

void BinghamNormalizationConstant::compute(
    int dim, double* z, double* result, double* derivatives) {
  int i, j;
  double* la;

  double hK[4];
  double T, t;
  double* r;  // Multiple roots
  double minEl;
  double scaleFactor = 1.0;

  // Create copy of parameter in oder to be able to manipulate it.
  la = new double[dim];
  memcpy(la, z, dim * sizeof(double));

  // Sort and save minimum.
  // qsort(la, dim, sizeof(double), compareDouble);
  minEl = la[0];
  for (i = 1; i < dim; i++)
    if (la[i] < minEl)
      minEl = la[i];

  // DEBUG: printf("MinEl: %f\n",minEl);

  if (minEl <= 0.0) {
    for (i = 0; i < dim; i++)
      la[i] -= (minEl - 0.1);
    scaleFactor = std::exp(-minEl + 0.1);
    minEl = 0.1;
  }

  /* DEBUG
  for(i=0; i<m; i++) {
  printf("la[%i]=%f\n",i,la[i]);
  }
  printf("\n");*/

  if (derivatives == nullptr) {
    t = BinghamNormalizationConstant::findRootNewton(dim, la, minEl);

    xi2CGFDeriv(t, dim, la, hK, -1);

    // T = 1/8 rho4 - 5/24 rho3^2
    T = 1.0 / 8 * (hK[3] / (hK[1] * hK[1])) -
        5.0 / 24 * (hK[2] * hK[2] / (hK[1] * hK[1] * hK[1]));

    result[0] = std::sqrt(2 * std::pow(M_PI, dim - 1)) * std::exp(-t) /
                std::sqrt(hK[1]) * scaleFactor;

    for (i = 0; i < dim; i++) {
      result[0] /= sqrt(la[i] - t);
    }
    result[1] = result[0] * (1 + T);
    result[2] = result[0] * std::exp(T);

  } else {  // Derivatives should be computed
    r = findMultipleRootsNewton(dim, la, minEl);

    xi2CGFDeriv(r[0], dim, la, hK, -1);

    // T = 1/8 rho4 - 5/24 rho3^2
    T = 1.0 / 8 * (hK[3] / (hK[1] * hK[1])) -
        5.0 / 24 * (hK[2] * hK[2] / (hK[1] * hK[1] * hK[1]));

    result[0] =
        sqrt(2 * pow(M_PI, dim - 1)) * exp(-r[0]) / sqrt(hK[1]) * scaleFactor;

    for (i = 0; i < dim; i++) {
      result[0] /= sqrt(la[i] - r[0]);
    }

    result[1] = result[0] * (1 + T);
    result[2] = result[0] * exp(T);

    for (i = 0; i < dim; i++) {
      xi2CGFDeriv(r[i + 1], dim, la, hK, i);

      // T = 1/8 rho4 - 5/24 rho3^2
      T = 1.0 / 8 * (hK[3] / (hK[1] * hK[1])) -
          5.0 / 24 * (hK[2] * hK[2] / (hK[1] * hK[1] * hK[1]));

      derivatives[3 * i] = sqrt(2 * pow(M_PI, dim + 1)) * exp(-r[i + 1]) /
                           (sqrt(hK[1]) * 2 * M_PI) * scaleFactor;

      for (j = 0; j < dim; j++) {
        if (j != i)
          derivatives[3 * i] /= sqrt(la[j] - r[i + 1]);
        else
          derivatives[3 * i] /= pow(sqrt(la[j] - r[i + 1]), 3);
      }

      derivatives[3 * i + 1] = derivatives[3 * i] * (1 + T);
      derivatives[3 * i + 2] = derivatives[3 * i] * exp(T);
    }

    free(r);
  }
  delete[] la;
}

int BinghamNormalizationConstant::compareDouble(const void* a, const void* b) {
  const double* aa = static_cast<const double*>(a);
  const double* bb = static_cast<const double*>(b);
  if (*aa < *bb)
    return -1;
  if (*aa == *bb)
    return 0;
  return 1;
}

double* BinghamNormalizationConstant::findMultipleRootsNewton(
    int dim, double* la, double minEl) {
  const double prec = 1e-10;
  const double uBound = minEl - 0.5;
  double* retval;
  double err, v0, v1;

  int i, j, k;

  retval = reinterpret_cast<double*>(malloc(sizeof(double) * (dim + 1)));

  // Set starting value of Newton method.
  for (i = 0; i <= dim; i++)
    retval[i] = uBound;

  // Computes relevant function value and performs newton step for each function
  // respectively.
  i = 0;
  do {
    err = 0;

    // Iterate over the Norm const and each partial derivative
    for (j = 0; j <= dim; j++) {
      v0 = 0;
      v1 = 0;
      for (k = 0; k < dim; k++) {
        if (k != j - 1) {
          v0 += 0.5 / (la[k] - retval[j]);
          v1 += 0.5 / ((la[k] - retval[j]) * (la[k] - retval[j]));
        } else {
          v0 += 3 * 0.5 / (la[k] - retval[j]);
          v1 += 3 * 0.5 / ((la[k] - retval[j]) * (la[k] - retval[j]));
        }
      }

      v0 -= 1;  // This is because we want to solve K(t)=1;
      err += fabs(v0);
      retval[j] += -v0 / v1;  // Newton iteration.
    }

    i++;
  } while (err > prec && i < 1000);

  // DEBUG: printf("Newton method stopped after %i iterations with error %g\n",
  // i, err);

  return retval;
}

double BinghamNormalizationConstant::findRootNewton(
    int dim, double* la, double minEl) {
  const double prec =
      1e-10;  // Precision
              // const double lBound=minEl-0.5*dim; // Lower bound.
  const double uBound = minEl - 0.5;  // Upper bound.

  double x;
  double val[4];
  int i = 0;

  x = uBound;  // Initial evaluation point.
  i = 0;
  do {
    xi2CGFDeriv(x, dim, la, val, -1);

    val[0] -= 1;

    // DEBUG printf("x = %f val[0]=%g\n", x, val[0]);
    x += -val[0] / val[1];
    i++;
  } while ((val[0] > prec || val[0] < -prec) && i < 1000);
  // DEBUG: printf("Iteration count: %i\n", i);

  return x;
}

void BinghamNormalizationConstant::xi2CGFDeriv(
    double t, int dim, double* la, double* res, int derriv) {
  double scale = 1.0;

  /* Initial values */
  res[0] = 0;
  res[1] = 0;
  res[2] = 0;
  res[3] = 0;

  for (int i = 0; i < dim; i++) {
    if (i == derriv)
      scale = 3.0;
    else
      scale = 1.0;

    res[0] += scale * 0.5 / (la[i] - t);
    res[1] += scale * 0.5 / ((la[i] - t) * (la[i] - t));
    res[2] += scale * 1 / ((la[i] - t) * (la[i] - t) * (la[i] - t));
    res[3] +=
        scale * 3 / ((la[i] - t) * (la[i] - t) * (la[i] - t) * (la[i] - t));
  }
}
