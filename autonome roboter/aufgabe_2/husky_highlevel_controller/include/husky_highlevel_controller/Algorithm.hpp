#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tuple>

namespace husky_highlevel_controller
{

class Algorithm
{
public:

  Algorithm();
  virtual ~Algorithm();

  int getMinimalDistance(const sensor_msgs::LaserScan& scan);
};
}
