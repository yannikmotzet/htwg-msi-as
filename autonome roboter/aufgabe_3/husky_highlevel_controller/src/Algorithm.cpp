#include "husky_highlevel_controller/Algorithm.hpp"
#include <limits>

namespace husky_highlevel_controller
{
    Algorithm::Algorithm(){}
  Algorithm::~Algorithm() = default;

  std::tuple<float, double> Algorithm::getMinimalDistance(const sensor_msgs::LaserScan &msg){
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
    return std::make_tuple(min_index*msg.angle_increment, min_dist);
  }

  geometry_msgs::Twist Algorithm::calculateAngularVelocity(double angle, float kp){
    geometry_msgs::Twist new_cmd_vel;
    new_cmd_vel.linear.x = kp * cosf(angle);
    new_cmd_vel.angular.z = angle * kp;
    return new_cmd_vel;
  }

}
