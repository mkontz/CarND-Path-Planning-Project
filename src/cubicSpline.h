#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include "baseSpline.h"
#include <cmath>
#include <vector>

// for convenience
using std::vector;

class CubicSpline : public BaseSpline
{
public:
  CubicSpline() :
    BaseSpline()
  { }

  bool update(vector<double> x, vector<double> y)
  { 
    bool retVal = false;

    m_x_coeff.clear();
    m_y_coeff.clear();

    if (2 <= x.size() &&
        x.size() == y.size())
    {
      calculateSpline(x, m_x_coeff);
      calculateSpline(y, m_y_coeff);
      retVal = true;
    }

    return retVal;
  }

private:
  void calculateSpline(const vector<double>& y, vector<vector<double> >& coeff)
  {
  	// From "Numerical Analysis" by Burden & Faires, 6th Edition
  	// Natural  cubic spline formulation (i.e. unclamped, 0 curvature ends)
    // Note: h[i] = 1.0 = x[i] - x[i-1] there h and x are not used

  	int n = y.size()-1; // n segments = number of points minus 1

    // Coefficient vectors
    const vector<double>& a = y;        // lenght: n + 1
    vector<double> b(n, 0.0);     // lenght: n
    vector<double> c(n+1, 0.0);   // lenght: n + 1
    vector<double> d(n, 0.0);     // lenght: n

    // Intermediate variable alpha
  	vector<double> alpha(n);
  	for (int i = 0; i < n; ++i)
  	{
      alpha[i] = 3 * (a[i+1] - 2 * a[i] + a[i-1]);
    }

    // Intermediate variables l, mu & z
    vector<double> l(n+1, 1.0);
    vector<double> mu(n+1, 0.0);
    vector<double> z(n+1, 0.0);
    for (int i = 1; i < n; ++i)
    {
      l[i] = 4 - mu[i-1];
      mu[i] = 1.0 / l[i];
      z[i] = (alpha[i] - z[i-1])/l[i];
    }

    // b, c & d coefficients
    for (int i = n-1; 0 <= i; --i)
    {
      c[i] = z[i] - mu[i] * c[i+1];
      b[i] = (a[i+1] - a[i]) - (c[i+1] + 2.0*c[i]) / 3.0;
      d[i] = (c[i+1] - c[i]) / 3.0;
    }

    // combine coefficent vectors
    coeff.clear();
    vector<double> tmp(4);
    for (int i = 0; i < n; ++i)
    {
      tmp[0] = a[i];
      tmp[1] = b[i];
      tmp[2] = c[i];
      tmp[3] = d[i];
      coeff.push_back(tmp);
    }
  }
};

#endif  // CUBIC_SPLINE_H