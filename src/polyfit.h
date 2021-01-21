#ifndef POLYFIT_H
#define POLYFIT_H

// Credit modeled after Clifford Wolf's polyfit function

#include "Eigen-3.3/Eigen/QR"
#include <vector>

using std::vector;

void polyfit(const vector<double> &x, const vector<double> &y, std::vector<double> &coeff, int order)
{
  Eigen::MatrixXd A(x.size(), order+1);
  Eigen::VectorXd y_mapped = Eigen::VectorXd::Map(&y.front(), y.size());
  Eigen::VectorXd result;

  // create matrix
  for (size_t i = 0; i < x.size(); i++)
  {
    for (int j = 0; j < order+1; j++)
    {
      A(i, j) = pow(x.at(i), j);
    }
  }

  // solve for linear least squares fit
  result = A.householderQr().solve(y_mapped);

  coeff.resize(order+1);
  for (int i = 0; i < order+1; i++)
  {

    coeff[i] = result[i];
  }
}

#endif  // POLYFIT_H








