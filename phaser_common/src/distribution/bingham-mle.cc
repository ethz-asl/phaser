#include "phaser/distribution/bingham-mle.h"

#include <Eigen/Dense>
#include <glog/logging.h>
#include <iostream>

#include "phaser/distribution/bingham-normalization-constant.h"

Eigen::VectorXd BinghamMLE::compute(Eigen::VectorXd* omega) {
  auto m = omega->rows();
  double* res = new double[m];
  double* in = omega->data();

  compute(m, in, res);

  Eigen::VectorXd eRes(m);

  for (int i = 0; i < m; ++i) {
    eRes(i) = res[i];
  }

  delete[] res;

  return eRes;
}

int BinghamMLE::compute(int dim, double* in, double* res) {
  int i, j;

  double* ncApprox = new double[3];           // SP approx of Norm Const.
  double* derivApprox = new double[3 * dim];  // SP approx. of derrivatives

  // Norm of objective function (shall be minimized)
  double oldNorm;
  double normObjFun = 0.0;

  // Temporary variables for derivative computation.
  double* tmpEig = new double[dim + 2];
  double* tmpNc = new double[3];
  double* tmpDeriv = new double[3 * (dim + 2)];

  Eigen::VectorXd ncDeriv(dim);        // Partial derivatives
  Eigen::MatrixXd ncDeriv2(dim, dim);  // Second order partial derivatives
  Eigen::VectorXd objFun(
      dim);  // Vector holding value of function which is searched for roots
  Eigen::MatrixXd objFunJacobian(dim, dim);

  // Make handling of predefined arrays as Eigen object possible.
  Eigen::Map<Eigen::VectorXd, 0, Eigen::InnerStride<3> > eTmpDeriv(
      tmpDeriv + 2, dim + 2);
  Eigen::Map<Eigen::VectorXd> eInput(in, dim);
  Eigen::Map<Eigen::VectorXd> eRes(res, dim);

  // Copy initialization values.
  memset(res, 0, dim * sizeof(double));

  // Heuristic choice of starting values for the optimization.
  for (i = 0; i < dim; i++)
    res[i] = 1 / (2 * in[i]);

  // Main optimization loop.
  for (i = 0; i <= 1000; i++) {
    // Set maximum entry to be zero.
    eRes.array() = eRes.array() + (-eRes.minCoeff());

    // std::cout << eRes.array()+(-eRes.minCoeff()) << "\n";

    // Compute the normalization constant.
    BinghamNormalizationConstant::compute(dim, res, ncApprox, nullptr);

    // Compute its gradient and Hessian
    for (j = 0; j < dim; j++) {
      memcpy(tmpEig, res, j * sizeof(double));
      tmpEig[j] = res[j];
      tmpEig[j + 1] = res[j];
      tmpEig[j + 2] = res[j];

      memcpy(tmpEig + j + 3, res + j + 1, (dim - j - 1) * sizeof(double));

      BinghamNormalizationConstant::compute(dim + 2, tmpEig, tmpNc, tmpDeriv);

      // Gradient of normalization constant.
      ncDeriv(j) = -tmpNc[2] / (2 * M_PI);

      // Hessian of normalization constant.
      ncDeriv2.row(j).head(j) = -eTmpDeriv.head(j) / (2 * M_PI);
      ncDeriv2(j, j) = -3 * tmpDeriv[3 * j + 2] / (2 * M_PI);
      ncDeriv2.row(j).tail(dim - j - 1) =
          -eTmpDeriv.tail(dim - j - 1) / (2 * M_PI);
    }

    // Compute objective function and its jacobian.
    objFun = ncDeriv / ncApprox[2] - eInput;
    objFunJacobian = (ncDeriv2 * ncApprox[2] - ncDeriv * ncDeriv.transpose()) /
                     (ncApprox[2] * ncApprox[2]);

    oldNorm = normObjFun;
    normObjFun = objFun.norm();

    // Gaus-Newton Step
    eRes = eRes - (objFunJacobian.transpose() * objFunJacobian).inverse() *
                      objFunJacobian.transpose() * objFun;

    if (fabs(oldNorm - normObjFun) <= 1e-10)
      break;
  }

  eRes.array() = -(eRes.array() + (-eRes.minCoeff()));

  delete[] ncApprox;
  delete[] derivApprox;
  delete[] tmpEig;
  delete[] tmpNc;
  delete[] tmpDeriv;

  return 0;
}
