#ifndef QUINTIC_SPLINE_H
#define QUINTIC_SPLINE_H


#include "baseSpline.h"
#include <cmath>
#include <vector>

// for convenience
using std::vector;
using std::cos;
using std::sin;



class QuinticSpline : public BaseSpline
{
public:
  QuinticSpline() :
    BaseSpline()
  { }

  bool update(vector<double> x, 
              vector<double> y,
              vector<double> h, // heading
              vector<double> k) // curvature
  { 
    bool retVal = false;

    m_x_coeff.clear();
    m_y_coeff.clear();

    if (2 <= x.size() &&
        x.size() == y.size() &&
        x.size() == h.size() &&
        x.size() == k.size())
    {      
      int n = y.size()-1; // n segments = number of points minus 1
      
      // additional 'shape' parameters
      double eta_1;
      double eta_2;
      double eta_3;
      double eta_4;
      
      vector<double> x_c(6);
      vector<double> y_c(6);
      for (int i = 0; i < n; ++i)
      {
        eta_1 = dist(x[i], y[i], x[i+1], y[i+1]);
        eta_2 = eta_1;
        eta_3 = 0.0;
        eta_4 = 0.0;

        // Calculate all trig once
        double cA = cos(h[i]);
        double sA = sin(h[i]);
        double cB = cos(h[i+1]);
        double sB = sin(h[i+1]);
        
        x_c[0] = x[i];
        
        x_c[1] = eta_1 * cA;
        
        x_c[2] = 0.5 * (eta_3 * cA - eta_1*eta_1 * k[i] * sA);
        
        x_c[3] = 10. *(x[i+1] - x[i]);
        x_c[3] -= (6. * eta_1 + 1.5 * eta_3) * cA;
        x_c[3] -= (4. * eta_2 - 0.5 * eta_4) * cB;
        x_c[3] += 1.5 * eta_1*eta_1 * k[i] * sA;
        x_c[3] -= 0.5 * eta_2*eta_2 * k[i+1] * sB;
    
        x_c[4] = -15. * (x[i+1] - x[i]);
        x_c[4] += (8 * eta_1 + 1.5 * eta_3) * cA;
        x_c[4] += (7 * eta_2 - eta_4) * cB;
        x_c[4] -= 1.5 * eta_1*eta_1 * k[i] * sA;
        x_c[4] += eta_2*eta_2 * k[i+1] * sB;
        
        x_c[5] = 6. * (x[i+1] - x[i]);
        x_c[5] -= (3. * eta_1 + 0.5 * eta_3) * cA;
        x_c[5] -= (3. * eta_2 - 0.5 * eta_4) * cB;
        x_c[5] += 0.5 * eta_1*eta_1 * k[i] * sA;
        x_c[5] -= 0.5 * eta_2*eta_2 * k[i+1] * sB;
        
        y_c[0] = y[i];
        
        y_c[1] = eta_1 * sA;
        
        y_c[2] = 0.5 * (eta_3 * sA + eta_1*eta_1 * k[i] * cA);
        
        y_c[3] = 10. *(y[i+1] - y[i]);
        y_c[3] -= (6. * eta_1 + 1.5 * eta_3) * sA;
        y_c[3] -= (4. * eta_2 - 0.5 * eta_4) * sB;
        y_c[3] -= 1.5 * eta_1*eta_1 * k[i] * cA;
        y_c[3] += 0.5 * eta_2*eta_2 * k[i+1] * cB;
    
        y_c[4] = -15. * (y[i+1] - y[i]);
        y_c[4] += (8 * eta_1 + 1.5 * eta_3) * sA;
        y_c[4] += (7 * eta_2 - eta_4) * sB;
        y_c[4] += 1.5 * eta_1*eta_1 * k[i] * cA;
        y_c[4] -= eta_2*eta_2 * k[i+1] * cB;
        
        y_c[5] = 6. * (y[i+1] - y[i]);
        y_c[5] -= (3. * eta_1 + 0.5 * eta_3) * sA;
        y_c[5] -= (3. * eta_2 - 0.5 * eta_4) * sB;
        y_c[5] -= 0.5 * eta_1*eta_1 * k[i] * cA;
        y_c[5] += 0.5 * eta_2*eta_2 * k[i+1] * cB;

        m_x_coeff.push_back(x_c);
        m_y_coeff.push_back(y_c);
      }

      retVal = true;
    }

    // update segment length data
    updateSegmentLen();

    // update start and end markers
    getStart();
    getEnd();

    return retVal;
  }
};

#endif  // QUINTIC_SPLINE_H