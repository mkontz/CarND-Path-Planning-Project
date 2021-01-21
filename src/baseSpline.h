#ifndef BASSE_SPLINE_H
#define BASSE_SPLINE_H

#include <cmath>
#include <vector>

// for convenience
using std::vector;
using std::size_t;

struct Marker
{
  Marker() : 
    idx(-1),
    weight(0.0)
  {}

  Marker(int i, double w) : 
    idx(i),
    weight(w)
  {}

  Marker& operator = ( const Marker& other ) {
    idx = other.idx;
    weight = other.weight;
    return *this;
  }

  int idx;
  double weight; 
};

struct Pnt2D
{
  Pnt2D() : 
    x(0.0),
    y(0.0)
  {}

  Pnt2D(double xx, double yy) : 
    x(xx),
    y(yy)
  {}

  Pnt2D& operator = ( const Pnt2D& other ) {
      x = other.x;
      y = other.y;
      return *this;
  }

  double x;
  double y;
};

class BaseSpline
{
public:
  BaseSpline() :
  	m_x_coeff(),
  	m_y_coeff()
  { }

  Pnt2D getPoint(const Marker marker)
  {
    double x = getNthDerivative(marker,0, m_x_coeff);
    double y = getNthDerivative(marker,0, m_y_coeff);

    return Pnt2D(x, y);
  }

  double getHeadingRad(const Marker marker)
  {
    double dx = getNthDerivative(marker,1, m_x_coeff);
    double dy = getNthDerivative(marker,1, m_y_coeff);

    return std::atan2(dy, dx);
  }

  double getCurvature(const Marker marker)
  {
    double dx = getNthDerivative(marker,1, m_x_coeff);
    double dy = getNthDerivative(marker,1, m_y_coeff);
    double ddx = getNthDerivative(marker,2, m_x_coeff);
    double ddy = getNthDerivative(marker,2, m_y_coeff);

    return (dx*ddy - dy*ddx) / std::pow((dx*dx + dy*dy), 1.5);
  }

  Marker findClosestMarker(const double xt, const double yt)
  {
    int best_idx = -1;
    Marker marker(-1, 0.0), bestMarker(-1, 0.0);
    double dist_sq, best_dist_sq = 1e20;
    int cnt;
    double s, s_last;
    double df, ddf;
    double x, y, dx, dy, ddx, ddy;

    // Note: use distance squared rather than distance to avoid needing to take square root

    // Iterate through spline's middle point to find segments closest to point
    for (size_t i = 1; i < m_x_coeff.size(); ++i)
    {
      marker.idx = i;
      dx = getNthDerivative(marker,0, m_x_coeff) - xt;
      dy = getNthDerivative(marker,0, m_y_coeff) - yt;
      dist_sq = dx * dx + dy * dy;

      if (dist_sq < best_dist_sq)
      {
        best_dist_sq = dist_sq;
        best_idx = i;
        bestMarker = marker;
      }
    }

    { // Check first point
      marker.idx = 0;
      dx = getNthDerivative(marker,0, m_x_coeff) - xt;
      dy = getNthDerivative(marker,0, m_y_coeff) - yt;
      dist_sq = dx * dx + dy * dy;

      if (dist_sq < best_dist_sq)
      {
        best_dist_sq = dist_sq;
        bestMarker = marker;
      }
    }

    { // Check last point
      marker.idx = m_x_coeff.size() - 1;
      marker.weight = 1.0;
      dx = getNthDerivative(marker,0, m_x_coeff) - xt;
      dy = getNthDerivative(marker,0, m_y_coeff) - yt;
      dist_sq = dx * dx + dy * dy;

      if (dist_sq < best_dist_sq)
      {
        best_dist_sq = dist_sq;
        bestMarker = marker;
      }
    }

    // The best point will either be the end of a segment or
    // a place on the segment where the derivative w.r.t. s is 0
    // Use Newton methof to find where derivative is zero.

    // segment segment before and after
    for (int i = 0; i < 2; ++i)
    {
      cnt = 0;
      s = 0.5; // start in middle
      s_last = 0.0;
      marker.idx = best_idx - i;
      marker.weight = s;

      // Iterate with Newton's method
      while ((1e-6 < std::abs(s-s_last)) && (cnt < 25))
      {
        x = getNthDerivative(marker,0, m_x_coeff);
        y = getNthDerivative(marker,0, m_y_coeff);
        dx = getNthDerivative(marker,1, m_x_coeff);
        dy = getNthDerivative(marker,1, m_y_coeff);
        ddx = getNthDerivative(marker,2, m_x_coeff);
        ddy = getNthDerivative(marker,2, m_y_coeff);
        df = 2.0*((x - xt)*dx + (y - yt)*dy);
        ddf = 2.0*((x - xt)*ddx + dx*dx + (y - yt)*ddy+ dy*dy);

        s_last = s;
        s -= df / ddf; // Newton's method applied to 1st derivative of squared distance
        marker.weight = s;
      }

      if ((0.0 < s) && (s < 1.0))
      {
        dist_sq = (x - xt) * (x - xt) + (y - yt) * (y - yt);
        if (dist_sq < best_dist_sq)
        {
          bestMarker = marker;
          best_dist_sq = dist_sq;
        }
      }
    }

    return bestMarker;
  }

protected:

  double getNthDerivative(const Marker marker,
                          const int n,
                          const vector<vector<double> >& coeff)
  {
    double retVal = 0.0;
    const double s = marker.weight;
    const vector<double>& c = coeff[marker.idx];

    for (int i = n; i < int(c.size()); ++i )
    {
      double tmp = c[i];
      for (int j = 0; j < n; ++j)
      {
        tmp *= (i+j);
      }

      if (n < i)
      {
        tmp *= std::pow(s, i-n);
      }
      
      retVal += tmp;
    }

    return retVal;
  }

  double dist(double x1, double y1, double x2, double y2)
  {
    return std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2));
  }

  // x-Spline coefficients
  // For each segment: x/y = a + b*s + c*s^2 + ...
  vector<vector<double> > m_x_coeff;
  vector<vector<double> > m_y_coeff;
};

#endif  // BASSE_SPLINE_H