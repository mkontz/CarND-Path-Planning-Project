#ifndef BASSE_SPLINE_H
#define BASSE_SPLINE_H

#include "polyFunctions.h"

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

bool operator < (const Marker& lhs, const Marker& rhs)
{
  if (lhs.idx == rhs.idx) { return (lhs.weight < rhs.weight); }
  else { return (lhs.idx < rhs.idx); }
}

bool operator <= (const Marker& lhs, const Marker& rhs)
{
  if (lhs.idx == rhs.idx) { return (lhs.weight <= rhs.weight); }
  else if ((lhs.idx + 1 == rhs.idx) && (lhs.weight == 1.0) && (rhs.weight == 0.0)) { return true; }
  else if ((lhs.idx == rhs.idx + 1) && (lhs.weight == 0.0) && (rhs.weight == 1.0)) { return true; }
  else { return (lhs.idx < rhs.idx); }
}

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

  bool isValid()
  {
    return (0 < m_x_coeff.size()) && (m_x_coeff.size() == m_y_coeff.size());
  }

  Marker getStart()
  {
    if (isValid() && (m_startMarker.idx < 0))
    {
      m_startMarker = Marker(0, 0.0);
    }

    return m_startMarker;
  }

  bool setStart(Marker mk)
  {
    if (isValid() &&
        (Marker(0, 0.0) <= mk) && 
        (mk < m_endMarker))
    {
      m_startMarker = mk;
      return true;
    }
    return false;
  }

  Marker getEnd()
  {
    if (isValid() && (m_endMarker.idx < 0))
    {
      m_endMarker = Marker(m_x_coeff.size()-1, 1.0);
    }

    return m_endMarker;
  }

  bool setEnd(Marker mk)
  {
    if (isValid() &&
        (m_startMarker < mk) &&
        (mk <= Marker(m_x_coeff.size()-1, 1.0)))
    {
      m_endMarker = mk;
      return true;
    }
    return false;
  }

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

Marker advanceMarker(Marker marker, double advDist)
{
  double segDist = evalPoly(marker.weight, m_weightToSegDist_coeff[marker.idx]);

  while ((marker < getEnd()) && (0 < advDist))
  {
    if (advDist < (m_segLen[marker.idx] - segDist))
    {
      double d = segDist + advDist;

      marker.weight = evalPoly(d, m_segDistToWeight_coeff[marker.idx]);
      advDist = -1; // to break loop
    }
    else
    {
      advDist -= m_segLen[marker.idx] - segDist;
      marker.idx++;
      marker.weight = 0.0;
      segDist = 0.0;
    }
  }

  if (getEnd() <= marker)
  {
    marker = getEnd();
  }

  return marker;
}

Marker retreatMarker(Marker marker, double retDist)
{
  double segDist = evalPoly(marker.weight, m_weightToSegDist_coeff[marker.idx]);

  while ((getStart() < marker) && (0 < retDist))
  {
    if (retDist < segDist)
    {
      double d = segDist - retDist;

      marker.weight = evalPoly(d, m_segDistToWeight_coeff[marker.idx]);
      retDist = -1; // to break loop
    }
    else
    {
      retDist -= segDist;
      marker.idx--;
      marker.weight = 1.0;
      segDist = m_segLen[marker.idx];
    }
  }

  if (marker <= getStart())
  {
    marker = getStart();
  }

  return marker;
}

double getSplineDist(const Marker marker)
{
  double dist = 0;
  for (int i = 0; i < marker.idx; ++i)
  {
    dist += m_segLen[i];
  }
  dist += evalPoly(marker.weight, m_weightToSegDist_coeff[marker.idx]);

  return dist;
}

double getSignedSplineDist(const Marker mk1, const Marker mk2)
{
  return getSplineDist(mk2) - getSplineDist(mk1);
}

Marker getMarkerFromDistance(const double dist)
{
  Marker marker(0, 0.0);
  double toGo = dist;
  while ((marker < getEnd()) && (0 < toGo))
  {
    if (m_segLen[marker.idx] <= toGo)
    {
      toGo -= m_segLen[marker.idx];
      marker.idx++;
    }
    else
    {
      marker.weight = evalPoly(toGo, m_segDistToWeight_coeff[marker.idx]);
      toGo = -1; // to break loop
    }
  }

  if (getEnd() <= marker)
  {
    marker = getEnd();
  }

  return marker;
}

protected:

  double getNthDerivative(const Marker marker,
                          const int n,
                          const vector<vector<double> >& coeff)
  {
    if ((marker.idx < 0) || (int(coeff.size()) <= marker.idx))
    {
      return 0.0;
    }

    return evalPoly(marker.weight, coeff[marker.idx], n);
  }

  void updateSegmentLen()
  {
    size_t num = 16;
    vector<double> w4 = linSpace(0.0, 1.0, 4 * num);

    m_weightToSegDist_coeff.clear();
    m_segDistToWeight_coeff.clear();
    m_segLen.resize(m_x_coeff.size());
    m_startLen.resize(m_x_coeff.size());

    double last = 0.0;

    for (size_t k = 0; k < m_x_coeff.size(); ++k)
    {
      vector<double> dx = evalPoly(w4, m_x_coeff[k], 1);
      vector<double> dy = evalPoly(w4, m_y_coeff[k], 1);
      vector<double> ds(w4.size());
      for (size_t i = 0; i < w4.size(); ++i)
      {
        ds[i] = std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]);
      }

      // Integrate to find path length "s"
      vector<double> s(num + 1, 0.0);
      for (size_t i = 0; i < num; ++i)
      {
        // integrate ds using Boole's
        double c = 2.0 / (45.0 * 4.0 * double(num));

        s[i+1] = s[i] + c*(7*ds[4*i] + 32*ds[4*i+1] + 12*ds[4*i+2] + 32*ds[4*i+3] + 7*ds[4*i+4]);
      }

      m_segLen[k] = s.back();
      m_startLen[k] = last;
      last += s.back();

      // Create weight vector and normalized length "u"
      vector<double> w = linSpace(0.0, 1.0, num);

      // polynomial maping "marker" weight to path length
      vector<double> c_w2s;
      polyFitInterp(w, s, c_w2s, 5);
      m_weightToSegDist_coeff.push_back(c_w2s);

      // polynomial maping path length to "marker" weight
      vector<double> c_s2w;
      polyFitInterp(s, w, c_s2w, 5);
      m_segDistToWeight_coeff.push_back(c_s2w);
    }
  }

  vector<double> linSpace(double x1, double x2, unsigned int n)
  {
    vector<double> retVal(n+1, x1);
    double delta((x2 - x1) / n);
    for (unsigned int i = 1; i <= n; i++)
    {
      retVal[i] = retVal[i-1] + delta;
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

  // Path length variables
  vector<vector<double> > m_weightToSegDist_coeff; // weight to segment dist
  vector<vector<double> > m_segDistToWeight_coeff; // segment dist to weight
  vector<double> m_segLen;
  vector<double> m_startLen;

  // Start/end markers
  Marker m_startMarker;
  Marker m_endMarker;
};

#endif  // BASSE_SPLINE_H 