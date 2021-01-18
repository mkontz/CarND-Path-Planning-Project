#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <vector>
#include <map>

// for convenience
using std::vector;
using std::map;

struct SFData {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
};

class SensorFusion
{
public:
  SensorFusion() 
  { }

  void update(vector<SFData> data)
  {
    m_dataMap.clear();

    for (auto it = data.begin(); it != data.end(); ++it)
    {
        m_dataMap[it->id] = *it;  
    }
  }

  const map<int, SFData>& getData() const { return m_dataMap; }

  SFData getData(int idx)
  {
    SFData data;

    if (m_dataMap.find(idx) != m_dataMap.end())
    {
      data = m_dataMap[idx];
    }

    return data;
  }

private:
  map<int, SFData> m_dataMap;
};

#endif  // HELPERS_H