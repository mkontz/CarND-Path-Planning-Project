#include "../polyfit.h"

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
    x.push_back(i);
    double dx = double(i) - 7.987;
    y.push_back(10.3 - 1.7 * dx * dx);
  }

  if (x.size() != y.size())
  { 
    cout << "Length of vector do not match." << endl;
    allTestsPassed = false;
  }

  vector<double> c;
  polyfit(x, y, c, 2);

  for (size_t i = 0; i < x.size(); ++i)
  {
    double y_fit = 0.0;

    for (size_t j = 0; j < c.size(); ++j)
    {
      y_fit += c[j] * std::pow(x[i], j);
    }

    allTestsPassed &= checkDoubles("Points do not match: x1", y[i], y_fit, i);
  }

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