#ifndef POLY_FUNCTIONS_H
#define POLY_FUNCTIONS_H

#include "Eigen-3.3/Eigen/QR"
#include <vector>
#include <algorithm>

using std::vector;

vector<double> normalize(const vector<double> x, double& a, double& b)
{
  // normalize x vector for numerical stability
  // x = a*u - b
  // u=0 -> min(x)
  // u=1 -> max(x)

  vector<double> u(x.size());
  b = - *std::min_element(x.begin(), x.end());
  a = *std::max_element(x.begin(), x.end()) + b;
  for (size_t i = 0; i < x.size(); ++i)
  {
    u[i] = (x[i] + b) / a;
  }

  return u;
}

vector<double> unnormalizeCoefficients(const vector<double> c_u, const double& a, const double& b)
{
  // y = c0 + c1*u + c2*u^2 + c3*u^3 +..... + cn*u^n
  // y = c0 + c1*(x+b)/a + c2*(x+b)^2/a^2 + c3*(x+b)^3/a^3 + .... + cn*(x+b)^n/a^n
  // y = c0 + c1*(x+b)/a + c2*(x^2+2bx+b^2)/a^2 + c3*(x^3+3bx^2+3b^2x+b^3)/a^3 + .... + cn*(x+b)^n/a^n
  // 
  // Terms must be groups such that
  // y = a0 + a1*x + a2*x^2 + .... + an*x^n
  // a0 = c0 + c1*b/a + c2*b^2/a^2 + ...
  // a1 = c1/a + 2*c2*b/a^2 + 3*c3*b/a^3 + ...

  int order = c_u.size() - 1;
  vector<double> c_x(c_u.size());

  vector<vector<double> > poly;
  poly.push_back(vector<double>(1, 1.0));
  for (int i = 1; i < order+1; i++)
  {
    poly.push_back(vector<double>(i+1, .0));
    for (size_t k = 0; k < poly[i-1].size(); ++k)
    {
      poly[i][k] += poly[i-1][k];
      poly[i][k+1] += poly[i-1][k];
    }
  }


  for (int i = 0; i < order+1; i++)
  {
    for (size_t k = 0; k < poly[i].size(); ++k)
    {
      c_x[k] += c_u[i] * poly[i][k] * std::pow(b, poly[i].size()-1-k) * std::pow(1.0/a, i);
    }
  }

  return c_x;
}

void polyFit(const vector<double> &x, const vector<double> &y, std::vector<double> &coeff, int order)
{
  // Credit: modeled after Clifford Wolf's polyfit function

  Eigen::MatrixXd A(x.size(), order+1);
  Eigen::VectorXd y_mapped = Eigen::VectorXd::Map(&y.front(), y.size());
  Eigen::VectorXd result;

  double a, b;
  vector<double> u = normalize(x, a, b);

  // create matrix
  for (size_t i = 0; i < u.size(); i++)
  {
    for (int j = 0; j < order+1; j++)
    {
      A(i, j) = pow(u.at(i), j);
    }
  }

  // solve for linear least squares fit
  result = A.householderQr().solve(y_mapped);

  vector<double> coeff_u(order+1);
  for (int i = 0; i < order+1; i++) { coeff_u[i] = result[i]; }

  coeff = unnormalizeCoefficients(coeff_u, a, b);
}

void polyFitInterp(const vector<double> &x, const vector<double> &y, std::vector<double> &coeff, int order)
{
  // Note: this is similar to polyFit, expect the polynomial is constrained to perfectly]
  // match the first the last points.  This effectively constrains the first and last coeffocients
  // in order to match the first and last points respectively.

  Eigen::MatrixXd A(x.size(), order-1);
  Eigen::VectorXd y_mapped = Eigen::VectorXd::Map(&y.front(), y.size());
  Eigen::VectorXd result;

  double a, b;
  vector<double> u = normalize(x, a, b);

  for (size_t i = 0; i < x.size(); ++i)
  {
    y_mapped[i] += -y[0] + (y[0] - y.back()) * std::pow(u[i], order);
  }

  // create matrix
  for (size_t i = 0; i < u.size(); i++)
  {
    for (int j = 1; j < order; j++)
    {
      A(i, j-1) = pow(u.at(i), j) - pow(u.at(i), order);
    }
  }

  // solve for linear least squares fit
  result = A.householderQr().solve(y_mapped);

  vector<double> coeff_u(order+1);
  coeff_u[0] = y[0];
  coeff_u[order] = y.back() - y[0]; // sum of coefficients
  for (int j = 1; j < order; j++)
  {
    coeff_u[j] = result[j-1];
    coeff_u[order] -= result[j-1];
  }

  coeff = unnormalizeCoefficients(coeff_u, a, b);
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

#endif  // POLY_FUNCTIONS_H
