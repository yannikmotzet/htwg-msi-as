#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tuple>

namespace husky_highlevel_controller
{

class Algorithm
{
public:

  Algorithm();
  virtual ~Algorithm();

  std::tuple<float, double> getMinimalDistance(const sensor_msgs::LaserScan& scan);
  geometry_msgs::Twist calculateAngularVelocity(double angle, float kp);
};
}
