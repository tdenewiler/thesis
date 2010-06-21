/*--------------------------------------------------------------------------
 *  Title:        metrics.cc
 *  Description:  Functions to compute error metrics between data sets.
 *                This is motivated by the paper "Discriminative Training of
 *                Kalman Filters" by Pieter Abbeel and Adam Coates and
 *                Michael Montemerlo and Andrew Y. Ng and Sebastian Thrun.
 *------------------------------------------------------------------------*/

#include "metrics.h"

/*--------------------------------------------------------------------------
 * double metrics_residual()
 * Calculates the residual prediction error between two data sets.
 *------------------------------------------------------------------------*/
double metrics_residual(int length, double *ykfx, double *ykfy,
        double *ygtx, double *ygty)
{
  // Declare variables.
  int i = 0;
  double e = 0.;
  double ydiff[length];

  // Get vector with ykf - ygt and store it in ydiff.
  metrics_calc_diff(length, ykfx, ykfy, ygtx, ygty, ydiff);

  // Compute the error using
  // e = \left(\frac{1}{T}\sum_{t=1}^T ||h(\mu_t)-y_t||^2\right)^{1/2}
  for (i = 0; i < length; i++)
  {
    e += pow(ydiff[i], 2.);
  }
  // Don't call sqrt() here -- it's too expensive.
  // Call sqrt() to fix the error in the calling function.
  //e = sqrt(e / length);

  return e;
} /* end metrics_residual() */

/*--------------------------------------------------------------------------
 * double metrics_prediction()
 * Calculates the prediction likelihod error between two data sets.
 *------------------------------------------------------------------------*/
double metrics_prediction(int length, double *ykfx, double *ykfy,
        double *ygtx, double *ygty, matrix<double> *Sigma,
        matrix<double> *H, matrix<double> *H_T, matrix<double> *P)
{
  // Declare variables.
  int i = 0;
  double e = 0.;
  double ydiff[length];
  matrix<double> *Omega;
  matrix<double> *OmegaInv;

  // Initialize variables.
  Omega = new matrix<double>(P->size1(), P->size2());
  Omega->clear();
  OmegaInv = new matrix<double>(P->size1(), P->size2());
  OmegaInv->clear();

  // Compute Omega and OmegaInv.
  metrics_calc_omega(Sigma, H, H_T, P, Omega);
  metrics_invertGaussJordanian(Omega, OmegaInv, Omega->size1());

  // Get vector with ykf - ygt.
  metrics_calc_diff(length, ykfx, ykfy, ygtx, ygty, ydiff);

  // Compute the error using
  // e = -\frac{1}{T}\sum_{t=1}^T \left(\log|2\pi\Omega_t|
  //     - (y_t-h(\mu_t))^T\Omega_t^{-1}(y_t-h(\mu_t))\right)
  for (i = 0; i < length; i++)
  {
    e += log(2. * M_PI * (*(Omega))(i, i)) -
            (ydiff[i] * (*(OmegaInv))(i, i) * ydiff[i]);
  }
  e /= (-1. * length);

  // Free array memory.
  delete Omega;

  return e;
} /* end metrics_prediction() */

/*--------------------------------------------------------------------------
 * void metrics_calc_omega()
 * Calculates matrix Omega used in the prediction likelihood error metric.
 *------------------------------------------------------------------------*/
void metrics_calc_omega(matrix<double> *Sigma, matrix<double> *H,
        matrix<double> *H_T, matrix<double> *P, matrix<double> *Omega)
{
  // Declare variables.
  matrix<double> *tmp1;
  matrix<double> *tmp2;

  // Initialize variables.
  tmp1 = new matrix<double>(H->size1(), Sigma->size2());
  tmp1->clear();
  tmp2 = new matrix<double>(H->size1(), H_T->size2());
  tmp2->clear();

  // Compute Omega using
  // \Omega = H_t\Sigma_tH_t^T+P
  *tmp1 = prod(*H, *Sigma);
  *tmp2 = prod(*tmp1, *H_T);
  *Omega = *tmp2 + *P;

  // Free array memory.
  delete tmp1;
  delete tmp2;

  return;
} /* end metrics_calc_omega() */

/*--------------------------------------------------------------------------
 * void metrics_calc_diff()
 * Generates vector containing differences between data sets at each step.
 *------------------------------------------------------------------------*/
void metrics_calc_diff(int length, double *ykfx, double *ykfy,
        double *ygtx, double *ygty, double *ydiff)
{
  // Declare variables.
  int i = 0;

  // Generate vector of differences.
  for (i = 0; i < length; i++)
  {
    // Avoid the sqrt() call and only apply it to the minimum found later.
    ydiff[i] = pow((ykfx[i] - ygtx[i]), 2.) + pow((ykfy[i] - ygty[i]), 2.);
  }

  return;
} /* end metrics_calc_diff() */

/*--------------------------------------------------------------------------
 * int metrics_invertGaussJordanian()
 * Inverts a matrix.
 *------------------------------------------------------------------------*/
int metrics_invertGaussJordanian(matrix<double> *A,
        matrix<double> *INV, int size)
{
  // Declare variables.
  double temp = 0.;
  int i = 0, j = 0, k = 0;

  // Initialize variables.
  matrix<double>hold_A(size, size);
  hold_A = *A;
  matrix<double>hold_INV(size, size);
  INV->clear();

  // Make INV the identity matrix.
  identity_matrix<double> id(size);
  *INV = id;
  hold_INV = id;

  // Make the matrix A to the unit matrix by some row operations that are
  // also applied to the unit matrix. INV gives the inverse of the matrix A.
  for (k = 0; k < size; k++)
  {
    temp = hold_A(k, k); //'temp' stores the A[k][k] value
    // so that A[k][k] will not change during
    // the operation A[i][j] /= A[k][k] when i = j = k.
    for (j = 0; j < size; j++)
    {
      hold_A(k, j) /= temp;   //It performs the following row
      hold_INV(k, j) /= temp; // operations to make A to a unit matrix:
      // R0 = R0/A[0][0], similarly for INV also
      // R1 = R1 - R0*A[1][0]
      // R2 = R2 - R0*A[2][0]
      // etc.
    }
    for (i = 0; i < size; i++)
    {
      temp = hold_A(i, k);
      for (j = 0; j < size; j++)
      {
        //R2 = R2-R1*A[2][1]
        if (i == k)
          break;
        //r2 = R2/A[2][2]
        hold_A(i, j) -= hold_A(k, j) * temp;  //R0 = R0-R2*A[0][2]
        hold_INV(i, j) -= hold_INV(k, j) * temp; //R1=R1-R2*A[1][2]
      }
    }
  }

  // Set the original matrix and its inverse to the values computed.
  *A = hold_A;
  *INV = hold_INV;

  return 0;
}
