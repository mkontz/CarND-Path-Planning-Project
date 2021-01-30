#include "../polyFunctions.h"

#include <iostream>
#include <string>
#include <vector>

#define private public
#define protected public

using std::vector;
using std::cout;
using std::endl;


bool almostEqual(double v1, double v2)
{
  return ((v1-v2) * (v1-v2) < 1e-20);
}


bool checkDoubles(std::string str, double v1, double v2, int idx = -1)
{
  bool passed = almostEqual(v1, v2);

  if (!passed)
  { 
    cout << str << ", v1: " << v1 << ", v2: " << v2 << ", delta: " << std::abs(v1-v2);

    if (0 <= idx)
    {
      cout << ", idx: " << idx;
    }

    cout << endl;
  }

  return passed;
}

int main()
{
  bool allTestsPassed = true;

  vector<double> x;
  vector<double> y;

  for (int i = 0; i <=20; ++i)
  {
    x.push_back(-2.3425 + 1.0345690*i);
    double dx = double(i) - 7.987;
    y.push_back(10.3 - 13.634*dx + 1.7 * dx * dx - 0.03234 * dx * dx * dx);
  }

  if (x.size() != y.size())
  { 
    cout << "Length of vector do not match." << endl;
    allTestsPassed = false;
  }

  vector<double> c;
  polyFit(x, y, c, 5);

  // std::cout << c[0] << ", " << c[1] << ", " << c[2] << ", " << c[3] << std::endl;

  for (size_t i = 0; i < x.size(); ++i)
  {
    double y_fit = 0.0;

    for (size_t j = 0; j < c.size(); ++j)
    {
      y_fit += c[j] * std::pow(x[i], j);
    }

    allTestsPassed &= checkDoubles("Points do not match: x1", y[i], y_fit, i);
  }

  vector<double> y_eval = evalPoly(x, c, 0);
  vector<double> ddy_eval = evalPoly(x, c, 2);
  vector<double> dddy_eval = evalPoly(x, c, 3);

  for (size_t i = 0; i < x.size(); ++i)
  {
    allTestsPassed &= checkDoubles("Eval vector points do not match: y", y[i], y_eval[i], i);
    allTestsPassed &= checkDoubles("Eval vector points do not match: ddy", 2.0 * c[2] + 6.0 * c[3] * x[i], ddy_eval[i], i);
    allTestsPassed &= checkDoubles("Eval vector points do not match: dddy", 6.0 * c[3], dddy_eval[i], i);
  }

  // fitting to a cubic polynomial all points should match exactly
  vector<double> c3;
  polyFitInterp(x, y, c3, 3);
  vector<double> y3 = evalPoly(x, c3, 0);

  for (size_t i = 0; i < x.size(); ++i)
  {
    allTestsPassed &= checkDoubles("polyFitInterp with order = 3: y", y[i], y3[i], i);
  }

  // fitting to a lower over polynomial, only ends should match exactly
  vector<double> c2;
  polyFitInterp(x, y, c2, 2);
  vector<double> y2 = evalPoly(x, c2, 0);

  allTestsPassed &= checkDoubles("polyFitInterp with order = 2: y", y[0], y2[0], 0);
  // for (size_t i = 0; i < x.size(); ++i)
  // {
  //   checkDoubles("polyFitInterp with order = 2: y", y[i], y2[i], i);
  // }
  allTestsPassed &= checkDoubles("polyFitInterp with order = 2: y", y.back(), y2.back(), x.size()-1);


  ////////////////////////////////////////////////////////////////
  // Summary of all tests.
  ////////////////////////////////////////////////////////////////
  if (allTestsPassed)
  {
      std::cout << "All tests passed!!\n\n";
  }
  else
  {
      std::cout << "Not all tests passed!!\n\n";
  }
  ////////////////////////////////////////////////////////////////

  return 0;
}