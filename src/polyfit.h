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


double evalPoly(const double x, const std::vector<double> &coeff, int dervative = 0)
{
  double retVal = 0.0;

  for (int i = int(coeff.size())-1; dervative <= i; --i)
  {
    double tmp = coeff[i];
    for (int j = 0; j < dervative; ++j)
    {
      tmp *= (i-j);
    }
    retVal += tmp;

    if (dervative < i)
    {
      retVal *= x;
    }
  }

  return retVal;
}

std::vector<double> evalPoly(const std::vector<double> x, const std::vector<double> &coeff, int dervative = 0)
{
   std::vector<double> retVal(x.size(), 0.0);

   for (size_t i = 0; i < x.size(); ++i)
   {
     retVal[i] += evalPoly(x[i], coeff, dervative);
   }

  return retVal;
}


#endif  // POLYFIT_H








