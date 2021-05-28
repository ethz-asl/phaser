/*
 * Igor Gilitschenski, Gerhard Kurz, Simon J. Julier, Uwe D. Hanebeck,
 * Efficient Bingham Filtering based on Saddlepoint Approximations
 * Proceedings of the 2014 IEEE International Conference on Multisensor Fusion
 * and Information Integration (MFI 2014), Beijing, China, September 2014.
 *
 * This implementation is based on its Matlab counterpart which can be found
 * via: https://github.com/libDirectional/libDirectional
 */

#ifndef PHASER_DISTRIBUTION_BINGHAM_NORMALIZATION_CONSTANT_H_
#define PHASER_DISTRIBUTION_BINGHAM_NORMALIZATION_CONSTANT_H_

class BinghamNormalizationConstant {
 public:
  static void compute(int dim, double* z, double* result, double* derivatives);

 private:
  static int compareDouble(const void* a, const void* b);
  static double findRootNewton(int dim, double* la, double minEl);
  static double* findMultipleRootsNewton(int dim, double* la, double minEl);
  static void xi2CGFDeriv(
      double t, int dim, double* la, double* res, int derriv);
};

#endif  // PHASER_DISTRIBUTION_BINGHAM_NORMALIZATION_CONSTANT_H_
