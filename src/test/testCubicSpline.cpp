#include "../cubicSpline.h"

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

  for (int i = 20; i <=40; ++i)
  {
    x.push_back(2.3 * double(i)* double(i) + 0.2);
    y.push_back(10.3 - 1.7 * double(i)* double(i));
  }

  if (x.size() != y.size())
  { 
    cout << "Length of vector do not match." << endl;
    allTestsPassed = false;
  }

  CubicSpline spline;
  spline.update(x, y);


  int n = x.size()-1;

  for (int i = 0; i < n; ++i)
  {
    Marker mk1 = Marker(i, 0.0);
    Marker mk2 = Marker(i, 1.0);

    Pnt2D pnt1 = spline.getPoint(mk1);
    Pnt2D pnt2 = spline.getPoint(mk2);

    allTestsPassed &= checkDoubles("Points do not match: x1", x[i], pnt1.x, i);
    allTestsPassed &= checkDoubles("Points do not match: y1", y[i], pnt1.y, i);
    allTestsPassed &= checkDoubles("Points do not match: x2", x[i+1], pnt2.x, i);
    allTestsPassed &= checkDoubles("Points do not match: y2", y[i+1], pnt2.y, i);
  }

  for (int i = 1; i < n; ++i)
  {
    Marker mk1 = Marker(i-1, 1.0);
    Marker mk2 = Marker(i, 0.0);

    Pnt2D pnt1 = spline.getPoint(mk1);
    Pnt2D pnt2 = spline.getPoint(mk2);

    allTestsPassed &= checkDoubles("Points do not match: x-match", pnt1.x, pnt2.x, i);
    allTestsPassed &= checkDoubles("Points do not match: y-match", pnt1.y, pnt2.y, i);

    double h1 = spline.getHeadingRad(mk1);
    double h2 = spline.getHeadingRad(mk2);

    allTestsPassed &= checkDoubles("Heading do not match", h1, h2, i);

    double k1 = spline.getCurvature(mk1);
    double k2 = spline.getCurvature(mk2);

    allTestsPassed &= checkDoubles("Curvature do not match", k1, k2, i);

    double dx = 0.5;
    Marker mk = spline.findClosestMarker(x[i] + dx, y[i]);
    Pnt2D pntClose = spline.getPoint(mk);
    double dist = std::sqrt(std::pow(x[i] + dx - pntClose.x, 2) + std::pow(y[i] - pntClose.y, 2));

    if (dx < dist)
    {
      cout << "Finding closest point on spline failed: dist: " << dist << ", dx: " << dx << endl;
      allTestsPassed = false;
    }
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