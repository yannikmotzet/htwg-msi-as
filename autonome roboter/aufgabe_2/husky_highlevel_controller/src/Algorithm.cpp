#include "husky_highlevel_controller/Algorithm.hpp"
#include <limits>

namespace husky_highlevel_controller
{
    Algorithm::Algorithm(){}
  Algorithm::~Algorithm() = default;

  int Algorithm::getMinimalDistance(const sensor_msgs::LaserScan& msg){
    int size = msg.ranges.size();
    int min_index;
    double min_dist = std::numeric_limits<int>::max();

    for (int i = 0; i < size; i++)
    {
      if (msg.ranges[i] < min_dist)
      {
        min_dist = msg.ranges[i];
        min_index = i;
      }
    }
    return min_dist;
  }
}
