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

// for convenience
using std::string;
using std::vector;
using std::map;

class PathPlanner
{
public:
  PathPlanner() : 
    m_targetVel(22.352 * 0.995),    // target velocity (m/s) - 22.352m/s = 50mph
    m_targetAccel(5.0),     // target acceleration (m/s^2)
    m_targetDecel(5.0),     // target deceleration (m/s^2)
    m_hardDecel(10.0),      // hard deceleration (m/s^2)
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
  }

private:
  void fitLanes(double distBefore, double distAfter)
  {
    // Calculate closest waypoint to current x, y position
    int i_cur = ClosestWaypoint(m_car_x, m_car_y, m_map_waypoints_x, m_map_waypoints_y);

    int i_start = i_cur;
    while (distBefore < sDist(m_car_s, m_map_waypoints_s[i_start]))
    {
      i_start = (i_start-1) % m_map_waypoints_s.size();
    }

    int i_end = i_cur;
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

      vector<double> dist;
      for (size_t i = idx; (i <= 10) && (i < m_previous_path_x.size()); ++i)
      {
        m_init_path_x.push_back(m_previous_path_x[i]);
        m_init_path_y.push_back(m_previous_path_y[i]);
        if (dist.empty())
        {
          dist.push_back(0.0);
        }
        else
        {
          dist.push_back(dist.back() + distance(m_init_path_x[i-1], 
                                                m_init_path_y[i-1],
                                                m_init_path_x[i],
                                                m_init_path_y[i]));
        }
      }

      vector<double> c_x;
      polyFit(dist, m_init_path_x, c_x, 5);

      vector<double> c_y;
      polyFit(dist, m_init_path_y, c_y, 3);

      double dx = evalPoly(dist.back(), c_x, 1);
      double dy = evalPoly(dist.back(), c_y, 1);
      double ddx = evalPoly(dist.back(), c_x, 2);
      double ddy = evalPoly(dist.back(), c_y, 2);

      m_init_heading = std::atan2(dy, dx);
      m_init_curvature = (dx*ddy - dy*ddx) / std::pow((dx*dx + dy*dy), 1.5);
    }
    else
    {
      // set initial point
      m_init_path_x.push_back(m_car_x);
      m_init_path_y.push_back(m_car_y);

      // set 2nd point to initial path speed (could be 0)
      m_init_path_x.push_back(m_car_x + m_car_speed * m_dt * cos(m_car_yaw));
      m_init_path_y.push_back(m_car_y + m_car_speed * m_dt * sin(m_car_yaw));

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

    // time it takes to travel 50.0m + stopping distance at target speed/decel
    double t_path = 50.0 / m_targetVel + m_targetVel / m_targetDecel;
    int n_path = std::ceil(t_path / m_dt);

    // predicted state of car
    double car_s = m_car_s;
    double car_spd = m_car_speed;

    // predicted state of next car in lane
    int nextIdx = findCarInFront(car_s, m_car_d);
    SFData sfData = m_sensorFusion.getData(nextIdx);
    double next_s = sfData.s;
    double next_spd = sfCarSpeed(nextIdx);

    // Start path vectors
    path.next_x_vals.push_back(m_car_x);
    path.next_y_vals.push_back(m_car_y);

    double ds = sDist(car_s, next_s);
    std::cout << "Dist to next car: " << ds << ", id: " << nextIdx;
    std::cout << ", spd: " << car_spd << ", next spd: " << next_spd << std::endl;

    for (int i = 0; i < n_path; ++i)
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
        double controlSpd = next_spd + 0.2 * (ds - s_follow);

        // If above target decel or close to follow distance
        if ((m_targetDecel < decel) || ((ds - s_follow) < 10.0))
        {
          calcDecelSpd = car_spd - m_dt * decel;
          controlSpd = next_spd + 0.2 * (ds - s_follow);

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
      car_s =  std::fmod(car_s + m_dt * car_spd, m_maxS);

      // add position to path 
      vector<double> xy = getXY(car_s,
                                m_car_d,
                                m_map_waypoints_s,
                                m_map_waypoints_x,
                                m_map_waypoints_y);

      path.next_x_vals.push_back(xy[0]);
      path.next_y_vals.push_back(xy[1]);
    }

    // Calculate cost
    calcCost(path);

    // Add new path
    m_paths.push_back(path);
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
    return std::fmod((s2 - s1 + 0.5 * m_maxS), m_maxS) - 0.5 * m_maxS;
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