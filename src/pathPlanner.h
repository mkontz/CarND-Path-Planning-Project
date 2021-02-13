#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "helpers.h"
#include "sensorFusion.h"
#include "cubicSpline.h"
#include "quinticSpline.h"
#include "polyFunctions.h"

#include <math.h>
#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <fstream>

// for convenience
using std::string;
using std::vector;
using std::map;

class PathPlanner
{
public:
  PathPlanner() : 
    m_targetVel(22.352 * 0.98),    // target velocity (m/s) - 22.352m/s = 50mph
    m_targetAccel(3.0),     // target acceleration (m/s^2)
    m_targetDecel(3.0),     // target deceleration (m/s^2)
    m_hardDecel(10.0),      // hard deceleration (m/s^2)
    m_maxS(6945.554),       // track length (m)
    m_width(4.0),           // lane width (m)
    m_dt(0.02),             // update time (s)
    m_lane(m_centerLane)
  { }

  struct Path {
    double cost;
    vector<double> next_x_vals;
    vector<double> next_y_vals;
  };

  void updateWaypoints(vector<double> map_waypoints_x,
                       vector<double> map_waypoints_y,
                       vector<double> map_waypoints_s,
                       vector<double> map_waypoints_dx,
                       vector<double> map_waypoints_dy)
  {
    m_map_waypoints_x = map_waypoints_x;
    m_map_waypoints_y = map_waypoints_y;
    m_map_waypoints_s = map_waypoints_s;
    m_map_waypoints_dx = map_waypoints_dx;
    m_map_waypoints_dy = map_waypoints_dy;
  }

  void updateData(double car_x,
                  double car_y,
                  double car_s,
                  double car_d,
                  double car_yaw,
                  double car_speed,
                  vector<double> previous_path_x,
                  vector<double> previous_path_y,
                  double end_path_s,
                  double end_path_d,
                  vector<SFData> sensor_fusion)
  {
    m_car_x = car_x;
    m_car_y = car_y;
    m_car_s = car_s;
    m_car_d = car_d;
    m_car_yaw = car_yaw;
    m_car_speed = car_speed;
    m_previous_path_x = previous_path_x;
    m_previous_path_y = previous_path_y;
    m_end_path_s = end_path_s;
    m_end_path_d = end_path_d;

    m_sensorFusion.update(sensor_fusion);
  }

  void plan(vector<double>& next_x_vals, 
            vector<double>& next_y_vals)
  {
    static int planNum = 0;
    m_planNum = ++planNum;

    // Fit lanes splines
    fitLanes(150.0, 250);

    // Generate initial path the follows previous path
    makeInitialPath();

    // Generate desired path
    makePath();

    // // // Set points on path to determine speed
    speedControl(next_x_vals, next_y_vals);

    // Write plan data to file for visualization
    // std::cout << "Writing to file" << std::endl;
    // logPlan();
  }

private:
  void fitLanes(double distBefore, double distAfter)
  {
    // Calculate closest waypoint to current x, y position
    int i_cur = ClosestWaypoint(m_car_x, m_car_y, m_map_waypoints_x, m_map_waypoints_y);

    int i_start = (i_cur - 1) % m_map_waypoints_s.size();
    while ( sDist(m_map_waypoints_s[i_start], m_car_s) < distBefore)
    {
      i_start = (i_start-1) % m_map_waypoints_s.size();
    }

    int i_end = (i_cur + 1) % m_map_waypoints_s.size();
    while (sDist(m_car_s, m_map_waypoints_s[i_end]) < distAfter)
    {
      i_end = (i_end+1) % m_map_waypoints_s.size();
    }

    vector<double> x_left;
    vector<double> y_left;
    vector<double> x_center;
    vector<double> y_center;
    vector<double> x_right;
    vector<double> y_right;
    if (i_start < i_end)
    {
      for (int i = i_start; i <= i_end; ++i)
      {
        x_left.push_back(m_map_waypoints_x[i] + 2.0 * m_map_waypoints_dx[i]);
        y_left.push_back(m_map_waypoints_y[i] + 2.0 * m_map_waypoints_dy[i]);
        x_center.push_back(m_map_waypoints_x[i] + 6.0 * m_map_waypoints_dx[i]);
        y_center.push_back(m_map_waypoints_y[i] + 6.0 * m_map_waypoints_dy[i]);
        x_right.push_back(m_map_waypoints_x[i] + 10.0 * m_map_waypoints_dx[i]);
        y_right.push_back(m_map_waypoints_y[i] + 10.0 * m_map_waypoints_dy[i]);
      }
    }
    else
    {
      for (int i = i_start; i < m_map_waypoints_x.size(); ++i)
      {
        x_left.push_back(m_map_waypoints_x[i] + 2.0 * m_map_waypoints_dx[i]);
        y_left.push_back(m_map_waypoints_y[i] + 2.0 * m_map_waypoints_dy[i]);
        x_center.push_back(m_map_waypoints_x[i] + 6.0 * m_map_waypoints_dx[i]);
        y_center.push_back(m_map_waypoints_y[i] + 6.0 * m_map_waypoints_dy[i]);
        x_right.push_back(m_map_waypoints_x[i] + 10.0 * m_map_waypoints_dx[i]);
        y_right.push_back(m_map_waypoints_y[i] + 10.0 * m_map_waypoints_dy[i]);
      }
      for (int i = 0; i <= i_end; ++i)
      {
        x_left.push_back(m_map_waypoints_x[i] + 2.0 * m_map_waypoints_dx[i]);
        y_left.push_back(m_map_waypoints_y[i] + 2.0 * m_map_waypoints_dy[i]);
        x_center.push_back(m_map_waypoints_x[i] + 6.0 * m_map_waypoints_dx[i]);
        y_center.push_back(m_map_waypoints_y[i] + 6.0 * m_map_waypoints_dy[i]);
        x_right.push_back(m_map_waypoints_x[i] + 10.0 * m_map_waypoints_dx[i]);
        y_right.push_back(m_map_waypoints_y[i] + 10.0 * m_map_waypoints_dy[i]);
      }
    }

    m_leftLane.update(x_left, y_left);
    m_centerLane.update(x_center, y_center);
    m_rightLane.update(x_right, y_right);
  }

  void makeInitialPath()
  {
    m_init_path_x.clear();
    m_init_path_y.clear();

    if ((0 < m_previous_path_x.size()) &&
        (m_previous_path_x.size() == m_previous_path_x.size()))
    {
      size_t idx = previousPathCurrentIndex(m_car_x, m_car_y);
      
      size_t n = 0.5 / m_dt;
      for (size_t i = idx; (i <= n) && (i < m_previous_path_x.size()); ++i)
      {
        m_init_path_x.push_back(m_previous_path_x[i]);
        m_init_path_y.push_back(m_previous_path_y[i]);
      }

      {
        vector<double> d;
        vector<double> x;
        vector<double> y;

        size_t i0 = (6 < m_init_path_x.size()) ? m_init_path_x.size() - 6 : 0;
        for (size_t i = i0; i < m_init_path_x.size(); ++i)
        {
          x.push_back(m_init_path_x[i]);
          y.push_back(m_init_path_y[i]);

          if (d.empty())
          {
            d.push_back(0.0);
          }
          else
          {
            d.push_back(d.back() + distance(m_init_path_x[i-1], 
                                            m_init_path_y[i-1],
                                            m_init_path_x[i],
                                            m_init_path_y[i]));
          }
        }

        vector<double> c_x;
        polyFit(d, x, c_x, 2);

        vector<double> c_y;
        polyFit(d, y, c_y, 2);

        double dx = evalPoly(d.back(), c_x, 1);
        double dy = evalPoly(d.back(), c_y, 1);
        double ddx = evalPoly(d.back(), c_x, 2);
        double ddy = evalPoly(d.back(), c_y, 2);

        m_init_heading = std::atan2(dy, dx);
        m_init_curvature = (dx*ddy - dy*ddx) / std::pow((dx*dx + dy*dy), 1.5);
      }
    }
    else
    {
      // set initial point
      m_init_path_x.push_back(m_car_x);
      m_init_path_y.push_back(m_car_y);

      // set 2nd point to initial path speed (could be 0)
      if (0.01 < m_car_speed)
      {
        m_init_path_x.push_back(m_car_x + m_car_speed * m_dt * cos(m_car_yaw));
        m_init_path_y.push_back(m_car_y + m_car_speed * m_dt * sin(m_car_yaw));
      }

      m_init_heading = m_car_yaw;
      m_init_curvature = 0;
    }
  }

  void makePath()
  {
    // determine target lane from previous path
    setTargetLane();

    static bool activeTransition = true;
    static double s_endTransition = std::fmod(m_car_s + 15.0, m_maxS);

    // static -> member (retreive history)
    m_activeTransition = activeTransition;
    m_s_endTransition = s_endTransition;

    vector<double> sd = getFrenet(m_init_path_x.back(),
                                  m_init_path_y.back(),
                                  m_init_heading,
                                  m_map_waypoints_x,
                                  m_map_waypoints_y);

    double s_minTrans = 5.0;
    if (m_activeTransition)
    {
      double ds = sSignedDist(sd[0], m_s_endTransition);
      // std::cout << "Plan: " << m_planNum << ", ds: " << ds;
      // std::cout << ", s_init: " << sd[0] << ", s_end: " << m_s_endTransition;
      // std::cout << std::endl;
      if ((ds < 0.0) &&
          (m_lane.isOnSpline(m_init_path_x.back(), m_init_path_y.back(), 0.001)))
      {
        m_activeTransition = false;
      }    
      else
      {
        
        if (ds < s_minTrans)
        {
          m_s_endTransition = sd[0] + s_minTrans;
        }
      }
    }
    else
    {
      // This can happen if the lanes shift due to adding/removing points
      if (!m_lane.isOnSpline(m_init_path_x.back(), m_init_path_y.back(), 0.001))
      {
        m_activeTransition = true;
        m_s_endTransition = sd[0] + 10.0;
      }
    }

    if (!m_activeTransition)
    {
      double sFrontLeft = 1e4;
      double sRearLeft = -1e4;
      int idxFrontLeft = -1;
      double sFrontCenter = 1e4;
      double sRearCenter = -1e4;
      int idxFrontCenter = -1;
      double sFrontRight = 1e4;
      double sRearRight = -1e4;
      int idxFrontRight = -1;

      const map<int, SFData>& otherCars = m_sensorFusion.getData();
      map<int, SFData>::const_iterator it;
      for (it = otherCars.begin(); it != otherCars.end(); ++it)
      {
        double ds = sSignedDist(m_car_s, it->second.s);

        if (it->second.d < 4.0)
        {
          if ((0.0 <= ds) && (ds < sFrontLeft)) { sFrontLeft = ds; idxFrontLeft = it->first; }
          if ((sRearLeft < ds) && (ds <= 0.0)) { sRearLeft = ds; }
        }
        else if (8.0 < it->second.d)
        {
          if ((0.0 <= ds) && (ds < sFrontRight)) { sFrontRight = ds; idxFrontRight = it->first; }
          if ((sRearRight < ds) && (ds <= 0.0)) { sRearRight = ds; }
        }
        else 
        {
          if ((0.0 <= ds) && (ds < sFrontCenter)) { sFrontCenter = ds; idxFrontCenter = it->first; }
          if ((sRearCenter < ds) && (ds <= 0.0)) { sRearCenter = ds; }
        }
      }

      if (m_d_target < 4.0) // in left lane
      {
        if (sFrontLeft < 75.0) // car ahead
        {
          if ((25.0 < sFrontCenter) && (sRearCenter < -25.0)) // room to move
          {
            double leftSpd = sfCarSpeed(idxFrontLeft);
            double centerSpd = sfCarSpeed(idxFrontCenter);
            if ((leftSpd < centerSpd) || (100.0 < sFrontCenter))
            {
              // Go to center lane
              // std::cout <<"Plan: " << m_planNum << " l->c" << std::endl;
              tranisionLane(6.0);
            }
          }
        }
      }
      else if (8.0 < m_d_target)
      {
        if (sFrontRight < 75.0)
        {
          if ((25.0 < sFrontCenter) && (sRearCenter < -25.0))
          {
            double rightSpd = sfCarSpeed(idxFrontRight);
            double centerSpd = sfCarSpeed(idxFrontCenter);
            if ((rightSpd < centerSpd) || (100.0 < sFrontCenter))
            {
              // Go to center lane
              // std::cout <<"Plan: " << m_planNum << " r->c" << std::endl;
              tranisionLane(6.0);
            }
          }
        }
      }
      else 
      {
        if (sFrontCenter < 75.0)
        {
          double leftSpd = sfCarSpeed(idxFrontLeft);
          double centerSpd = sfCarSpeed(idxFrontCenter);
          double rightSpd = sfCarSpeed(idxFrontRight);

          if ((25.0 < sFrontLeft) && (sRearLeft < -25.0) && 
              ((centerSpd < leftSpd) || (100.0 < sFrontLeft)))
          {
            // Go to left lane
            // std::cout <<"Plan: " << m_planNum << " c->l" << std::endl;
            tranisionLane(2.0);           
          }
          else if ((25.0 < sFrontRight) && (sRearRight < -25.0) && 
                   ((centerSpd < rightSpd) || (100.0 < sFrontRight)))
          {
            // Go to right lane
            // std::cout <<"Plan: " << m_planNum << " c->r" << std::endl;
            tranisionLane(10.0);
          }
        }
      }
    }

    if (m_activeTransition)
    {
      // std::cout <<"Plan: " << m_planNum << " transitioning: " << m_d_target << std::endl;
      makeTransitionSpline();
    }
    else
    {
      // std::cout <<"Plan: " << m_planNum << " tracking: " << m_d_target << std::endl;
    }

    // member -> static (store history)
    activeTransition = m_activeTransition;
    s_endTransition = m_s_endTransition;
  }

  void setTargetLane(double d = -1.0)
  {
    static double d_target = -1.0;

    // check for deliberate change
    if (0.0 < d)
    {
      d_target = d;
    }

    // check if there is not history
    if (d_target < 0)
    {
      if (m_car_d < 4.0)
      {
        d_target = 2.0;
      }
      else if (8.0 < m_car_d)
      {
        d_target = 10.0;
      }
      else
      {
        d_target = 6.0;
      }
    } 

    if (d_target < 4.0)
    {
      m_lane = m_leftLane;
    }
    else if (8.0 < d_target)
    {
      m_lane = m_rightLane;
    }
    else
    {
      m_lane = m_centerLane;
    }

    m_d_target = d_target;
  }

  void makeTransitionSpline()
  {
    vector<double> xy = getXY(m_s_endTransition,
                              m_d_target,
                              m_map_waypoints_s,
                              m_map_waypoints_x,
                              m_map_waypoints_y);

    Marker laneMk = m_lane.findClosestMarker(xy[0], xy[1]);
    Pnt2D endPnt = m_lane.getPoint(laneMk);
    vector<double> x = {m_init_path_x.back(), endPnt.x};
    vector<double> y = {m_init_path_y.back(), endPnt.y};
    vector<double> h = {m_init_heading, m_lane.getHeadingRad(laneMk)};
    vector<double> k = {m_init_curvature, m_lane.getCurvature(laneMk)};

    m_tranSpline.update(x, y, h, k);
  }

  void tranisionLane(double d_target)
  {
    m_activeTransition = true;
    setTargetLane(d_target);

    vector<double> sd = getFrenet(m_init_path_x.back(),
                                  m_init_path_y.back(),
                                  m_init_heading,
                                  m_map_waypoints_x,
                                  m_map_waypoints_y);

    double tranDistance = 100.0;
    m_s_endTransition = std::fmod(sd[0] + tranDistance, m_maxS);
  }

  void speedControl(vector<double>& next_x_vals, 
                    vector<double>& next_y_vals)
  {
    Path path;

    // time it takes to travel 25.0m + stopping distance at target speed/decel
    double t_path = 25.0 / m_targetVel + m_targetVel / m_targetDecel;
    int n_path = std::ceil(t_path / m_dt);

    // predicted state of car
    double car_s = m_car_s;
    double car_spd = m_car_speed;

    // Start with initial path vectors
    path.next_x_vals = m_init_path_x;
    path.next_y_vals = m_init_path_y;
    if (1 < path.next_x_vals.size())
    {

      car_spd = distance(m_init_path_x[m_init_path_x.size()-2],
                         m_init_path_y[m_init_path_y.size()-2],
                         m_init_path_x.back(),
                         m_init_path_y.back()) / m_dt;
    }

    // predicted state of next car in lane
    int nextIdx = findCarInFront(car_s, m_car_d);
    double next_s;
    double next_spd;
    if (0 <= nextIdx)
    {
      SFData sfData = m_sensorFusion.getData(nextIdx);
      next_s = sfData.s;
      next_spd = sfCarSpeed(nextIdx);

      // predict for equal to initial path
      next_s += m_dt * next_spd * (m_init_path_x.size() - 1);
    }

    // transition from initial path to lane
    Marker laneMk;
    if (m_activeTransition)
    {
      Pnt2D pnt = m_tranSpline.getPoint(m_tranSpline.getEnd());
      laneMk = m_lane.findClosestMarker(pnt.x, pnt.y);
    }
    else
    {
      laneMk = m_lane.findClosestMarker(m_init_path_x.back(), m_init_path_y.back());
    }

    Marker tranMk(0, 0.0);
    while (path.next_x_vals.size() < n_path)
    {
      double newSpd = std::min(car_spd + m_dt * m_targetAccel, m_targetVel);

      if (0 <= nextIdx)
      {
        double s_follow = 10 + next_spd * next_spd / (2.0 * m_hardDecel);
        double ds = sDist(car_s, next_s);

        // Calc speed for distance tracking control
        // feedforward speed + p distance control
        double controlSpd = next_spd + 0.5 * (ds - s_follow);

        newSpd = std::min(controlSpd, newSpd);

        // update next car position
        next_s += m_dt * next_spd;
      }

      // update car position & speed
      car_spd = newSpd;

      double stepDist = m_dt * car_spd;
      double remainingTranDist = -1.0;
      if (m_activeTransition)
      {
        remainingTranDist = m_tranSpline.getSignedSplineDist(tranMk, m_tranSpline.getEnd());
      }

      if (0.0 < remainingTranDist)
      {
        if (stepDist < remainingTranDist)
        {
          tranMk = m_tranSpline.advanceMarker(tranMk, stepDist);
        }
        else
        {
          tranMk = m_tranSpline.getEnd();
          laneMk = m_lane.advanceMarker(laneMk, stepDist - remainingTranDist);
        }
      }
      else
      {
        laneMk = m_lane.advanceMarker(laneMk, stepDist);
      }

      Pnt2D pnt = (tranMk < m_tranSpline.getEnd()) 
          ? m_tranSpline.getPoint(tranMk)
          : m_lane.getPoint(laneMk);

      double heading = (tranMk < m_tranSpline.getEnd()) 
          ? m_tranSpline.getHeadingRad(tranMk)
          : m_lane.getHeadingRad(laneMk);

      path.next_x_vals.push_back(pnt.x);
      path.next_y_vals.push_back(pnt.y);

      vector<double> sd = getFrenet(pnt.x,
                                    pnt.y,
                                    heading,
                                    m_map_waypoints_x,
                                    m_map_waypoints_y);

      car_s = sd[0];
    }

    // Add new path
    m_paths.push_back(path);

    next_x_vals = path.next_x_vals;
    next_y_vals = path.next_y_vals;
  }

  int findCarInFront(double s, double d)
  {
    const map<int, SFData>& sfData = m_sensorFusion.getData();

    double minDeltaS = 1e10;
    int minIdx = -1;
    double ds;
    double dd;

    for (auto it = sfData.begin(); it != sfData.end(); ++it)
    {
      ds = sDist(s, it->second.s);
      dd = it->second.d - d;

      if ((-3.0 <=dd) && (dd <= 3.0) && 
          (0.0 < ds) && (ds < minDeltaS))
      {
        minDeltaS = ds;
        minIdx = it->first;
      }
    }

    return minIdx;
  }

  double sfCarSpeed(int idx)
  {
    SFData sfData = m_sensorFusion.getData(idx);

    // unit vector in s direction
    vector<double> xy0 = getXY(sfData.s,
                               sfData.d,
                               m_map_waypoints_s,
                               m_map_waypoints_x,
                                m_map_waypoints_y);
    vector<double> xy1 = getXY(sfData.s + 1,
                               sfData.d,
                               m_map_waypoints_s,
                               m_map_waypoints_x,
                                m_map_waypoints_y);
    double dx = xy1[0] - xy0[0];
    double dy = xy1[1] - xy0[1];

    // dot velocity vector with unit vector in s direction to get forward lane speed.
    double speed = (dx * sfData.vx + dy * sfData.vy) / std::sqrt(dx*dx + dy*dy);

    return speed;
  }

  double sDist(double s1, double s2)
  {
    s1 = std::fmod(s1, m_maxS);
    s2 = std::fmod(s2, m_maxS);

    double dist = s2 - s1;
    if (s2 < s1)
    {
      dist = m_maxS - dist;
    }

    return dist;
  }

  double sSignedDist(double s1, double s2)
  {
    return std::fmod(0.5 * m_maxS + s2 - s1, m_maxS) - 0.5 * m_maxS;
  }

  size_t previousPathCurrentIndex(double x, double y)
  {
    size_t bestIdx = 0;
    double dx = m_previous_path_x[0] - x;
    double dy = m_previous_path_y[0] - y;
    double bestDistSq = dx*dx + dy*dy;
    double distSq;
    for (size_t i = 1; i < m_previous_path_x.size(); ++i)
    {
      dx = m_previous_path_x[0] - x;
      dy = m_previous_path_y[0] - y;
      distSq = dx*dx + dy*dy;
      if (distSq < bestDistSq)
      {
        bestDistSq = distSq;
        bestIdx = i;
      }
    }

    // If possible go backward 1
    bestIdx -= (0 < bestIdx) ? 1 : 0;

    return bestIdx;
  }

  void logPlan()
  {
    std::stringstream ss;

    logVector("map_x", m_map_waypoints_x, ss);
    logVector("map_y", m_map_waypoints_y, ss);

    {
      vector<double> leftPnts_x = m_map_waypoints_x;
      vector<double> leftPnts_y = m_map_waypoints_y;
      vector<double> centerPnts_x = m_map_waypoints_x;
      vector<double> centerPnts_y = m_map_waypoints_y;
      vector<double> rightPnts_x = m_map_waypoints_x;
      vector<double> rightPnts_y = m_map_waypoints_y;

      for (size_t i = 0; i < m_map_waypoints_y.size(); ++i)
      {
        leftPnts_x[i] += 2.0 * m_map_waypoints_dx[i];
        leftPnts_y[i] += 2.0 * m_map_waypoints_dy[i];
        centerPnts_x[i] += 6.0 * m_map_waypoints_dx[i];
        centerPnts_y[i] += 6.0 * m_map_waypoints_dy[i];
        rightPnts_x[i] += 10.0 * m_map_waypoints_dx[i];
        rightPnts_y[i] += 10.0 * m_map_waypoints_dy[i];
      }

      logVector("leftPnts_x", leftPnts_x, ss);
      logVector("leftPnts_y", leftPnts_y, ss);
      logVector("centerPnts_x", centerPnts_x, ss);
      logVector("centerPnts_y", centerPnts_y, ss);
      logVector("rightPnts_x", rightPnts_x, ss);
      logVector("rightPnts_y", rightPnts_y, ss);
    }

    logSpline("leftLane", m_leftLane, ss);
    logSpline("centerLane", m_centerLane, ss);
    logSpline("rightLane", m_rightLane, ss);

    logVector("init_path_x", m_init_path_x, ss);
    logVector("init_path_y", m_init_path_y, ss);

    logVector("previous_path_x", m_previous_path_x, ss);
    logVector("previous_path_y", m_previous_path_y, ss);

    for (size_t i = 0; i < m_paths.size(); ++i)
    {
      Path path = m_paths[i];   

      logVector("path_" + std::to_string(i) + "_x", path.next_x_vals, ss);
      logVector("path_" + std::to_string(i) + "_y", path.next_y_vals, ss);
    }

    {
      string fName = std::getenv("HOME");
      fName += "/tmp/plans.txt";
      std::ofstream outFile;
      if (1 == m_planNum)
      {
        outFile.open(fName.c_str(), std::ofstream::trunc);
      }
      else
      {
        outFile.open(fName.c_str(), std::ofstream::out | std::ofstream::app);
      }
      outFile << "Starting new plan: " << m_planNum << std::endl;
      outFile << "car_x = " << m_car_x << std::endl;
      outFile << "car_y = " << m_car_y << std::endl;
      outFile << "car_yaw = " << m_car_yaw << std::endl;
      outFile << ss.str();
      outFile.close();
    }
  }

  void logVector(const string name, const vector<double>& vec, std::stringstream& ss)
  {
    ss << std::setprecision(4) << std::fixed << name << " = [";

    if (!vec.empty())
    {
      ss << vec[0];
      for (size_t i = 1; i < vec.size(); ++i)
      {
        ss << "," << vec[i];
      }
    }
    ss << "]\n";
  }

  void logSpline(const string name, BaseSpline spline, std::stringstream& ss)
  {
    vector<double> x;
    vector<double> y;

    Marker start = spline.getStart();
    Marker end = spline.getEnd();

    Marker runner = start;
    while (runner < end)
    {
      Pnt2D pnt = spline.getPoint(runner);
      x.push_back(pnt.x);
      y.push_back(pnt.y);

      runner.weight += 0.1;
      if (0.95 < runner.weight)
      {
        runner.idx++;
        runner.weight = 0.0;
      }
    }
    Pnt2D pnt = spline.getPoint(end);
    x.push_back(pnt.x);
    y.push_back(pnt.y);

    logVector(name + "_x", x, ss);
    logVector(name + "_y", y, ss);
  }

  //parameters
  double m_targetVel;
  double m_targetAccel;
  double m_targetDecel;
  double m_hardDecel;
  double m_maxS;
  double m_width;
  double m_dt;

  // Possible paths
  vector<Path> m_paths;

  // Lanes
  CubicSpline m_leftLane;
  CubicSpline m_centerLane;
  CubicSpline m_rightLane;

  // Waypoints
  vector<double> m_map_waypoints_x;
  vector<double> m_map_waypoints_y;
  vector<double> m_map_waypoints_s;
  vector<double> m_map_waypoints_dx;
  vector<double> m_map_waypoints_dy;

  // input data
  double m_car_x;
  double m_car_y;
  double m_car_s;
  double m_car_d;
  double m_car_yaw;
  double m_car_speed;
  vector<double> m_previous_path_x;
  vector<double> m_previous_path_y;
  double m_end_path_s;
  double m_end_path_d;

  SensorFusion m_sensorFusion;

  vector<double> m_init_path_x;
  vector<double> m_init_path_y;
  double m_init_heading;
  double m_init_curvature;

  // Path variable
  double m_d_target;
  CubicSpline& m_lane;
  bool m_activeTransition;
  double m_s_endTransition;
  QuinticSpline m_tranSpline;

  int m_planNum;
};

#endif  // PATH_PLANNER_H