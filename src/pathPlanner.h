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
    m_targetVel(22.352 * 0.995),    // target velocity (m/s) - 22.352m/s = 50mph
    m_targetAccel(3.0),     // target acceleration (m/s^2)
    m_targetDecel(3.0),     // target deceleration (m/s^2)
    m_hardDecel(5.0),      // hard deceleration (m/s^2)
    m_maxS(6945.554),       // track length (m)
    m_width(4.0),           // lane width (m)
    m_dt(0.02)              // update time (s)
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
    // Fit lanes splines
    fitLanes(100.0, 200);

    // Generate initial path the follows previous path
    makeInitialPath();

    // Generate and score paths
    makePaths();

    // find best path 
    findBestPath(next_x_vals, next_y_vals);

    // Write plan data to file for visualization
    std::cout << "Writing to file" << std::endl;
    logPlan();
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
      idx -= (0 < idx) ? 1 : 0;

      size_t n = 1.0 / m_dt;
      for (size_t i = idx; (i <= n) && (i < m_previous_path_x.size()); ++i)
      {
        m_init_path_x.push_back(m_previous_path_x[i]);
        m_init_path_y.push_back(m_previous_path_y[i]);
      }

      {
        vector<double> d;
        vector<double> x;
        vector<double> y;

        size_t i0 = (4 < m_init_path_x.size()) ? m_init_path_x.size() - 4 : 0;
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


  void makePaths()
  {
    // Path that follows current lane and slow downs for car in front
    addKeepInLane();
  }

  void addKeepInLane()
  {
    Path path;

    double d_target = -1.0;
    CubicSpline& lane = m_centerLane;

    if ((1.0 <= m_car_d) && (m_car_d < 3.0))
    {
      d_target = 2.0;
      lane = m_leftLane;
    }
    else if ((5.0 <= m_car_d) && (m_car_d < 7.0))
    {
      d_target = 6.0;
      lane = m_centerLane;
    }
    else if ((9.0 <= m_car_d) && (m_car_d < 11.0))
    {
      d_target = 10.0;
      lane = m_rightLane;
    }

    if (0 < d_target)
    {
      // time it takes to travel 50.0m + stopping distance at target speed/decel
      double t_path = 50.0 / m_targetVel + m_targetVel / m_targetDecel;
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

      // double ds = sDist(car_s, next_s);
      // std::cout << "Dist to next car: " << ds << ", id: " << nextIdx;
      // std::cout << ", spd: " << car_spd << ", next spd: " << next_spd << std::endl;

      // transition from initial path to lane
      QuinticSpline tranSpline;
      Marker laneMk;
      {
        double tranDistance = 15;
        vector<double> sd = getFrenet(m_init_path_x.back(),
                                      m_init_path_y.back(),
                                      m_init_heading,
                                      m_map_waypoints_x,
                                      m_map_waypoints_y);

        vector<double> xy = getXY(sd[0] + tranDistance,
                                  d_target,
                                  m_map_waypoints_s,
                                  m_map_waypoints_x,
                                  m_map_waypoints_y);

        laneMk = lane.findClosestMarker(xy[0], xy[1]);
        Pnt2D endPnt = lane.getPoint(laneMk);
        vector<double> x = {m_init_path_x.back(), endPnt.x};
        vector<double> y = {m_init_path_y.back(), endPnt.y};
        vector<double> h = {m_init_heading, lane.getHeadingRad(laneMk)};
        vector<double> k = {m_init_curvature, lane.getCurvature(laneMk)};

        tranSpline.update(x, y, h, k);
      }

      Marker tranMk(0, 0.0);
      while (path.next_x_vals.size() < n_path)
      {
        double newSpd = std::min(car_spd + m_dt * m_targetAccel, m_targetVel);

        if (0 <= nextIdx)
        {
          double s_follow = 10 + next_spd * next_spd / (2.0 * m_hardDecel);
          double ds = sDist(car_s, next_s);

          // calc deceleration
          double delta_spd = car_spd - next_spd;
          double decel = delta_spd * delta_spd / (2 * (ds - s_follow));

          // Calc speed based on desired decel
          double calcDecelSpd = car_spd - m_dt * decel;

          // Calc speed for distance tracking control
          // feedforward speed + p distance control
          double controlSpd = next_spd + 0.01 * (ds - s_follow);

          // If above target decel or close to follow distance
          if ((m_targetDecel < decel) || ((ds - s_follow) < 10.0))
          {
            if ((0.0 < (ds - s_follow)) && (calcDecelSpd < controlSpd))
            {
              newSpd = calcDecelSpd;
            }
            else
            {
              newSpd = controlSpd;
            }
          }

          // update next car position
          next_s += m_dt * next_spd;
        }

        // update car position & speed
        car_spd = newSpd;

        double stepDist = m_dt * newSpd;
        double remainingTranDist = tranSpline.getSignedSplineDist(tranMk, tranSpline.getEnd());
        if (0 < remainingTranDist)
        {
          if (stepDist < remainingTranDist)
          {
            tranMk = tranSpline.advanceMarker(tranMk, stepDist);
          }
          else
          {
            tranMk = tranSpline.getEnd();
            laneMk = lane.advanceMarker(laneMk, stepDist - remainingTranDist);
          }
        }
        else
        {
          laneMk = lane.advanceMarker(laneMk, stepDist);
        }

        Pnt2D pnt = (tranMk < tranSpline.getEnd()) 
            ? tranSpline.getPoint(tranMk)
            : lane.getPoint(laneMk);

        double heading = (tranMk < tranSpline.getEnd()) 
            ? tranSpline.getHeadingRad(tranMk)
            : lane.getHeadingRad(laneMk);

        path.next_x_vals.push_back(pnt.x);
        path.next_y_vals.push_back(pnt.y);

        vector<double> sd = getFrenet(pnt.x,
                                      pnt.y,
                                      heading,
                                      m_map_waypoints_x,
                                      m_map_waypoints_y);

        car_s = sd[0];
      }

      // Calculate cost
      calcCost(path);

      // Add new path
      m_paths.push_back(path);
    }
  }

  void calcCost(Path& path)
  {
    double cost = 1.0;

    path.cost = cost;
  }

  void findBestPath(vector<double>& next_x_vals, 
                    vector<double>& next_y_vals)
  {
    if (m_paths.empty())
    {
      // return (pass back) empty path
      next_x_vals.clear();
      next_y_vals.clear();
    }
    else
    {
      size_t best_idx;
      double best_cost = 1e20;

      for (size_t k = 0; k < m_paths.size(); ++k)
      {
        if (m_paths[k].cost < best_cost)
        {
          best_idx = k;
          best_cost = m_paths[k].cost;
        }
      }

      // return (pass back) best path
      next_x_vals = m_paths[best_idx].next_x_vals;
      next_y_vals = m_paths[best_idx].next_y_vals;
    }
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
      static int planNum = 0;

      string fName = std::getenv("HOME");
      fName += "/tmp/plans.txt";
      std::ofstream outFile;
      if (0 == planNum)
      {
        outFile.open(fName.c_str(), std::ofstream::trunc);
      }
      else
      {
        outFile.open(fName.c_str(), std::ofstream::out | std::ofstream::app);
      }
      outFile << "Starting new plan: " << ++planNum << std::endl;
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
};

#endif  // PATH_PLANNER_H