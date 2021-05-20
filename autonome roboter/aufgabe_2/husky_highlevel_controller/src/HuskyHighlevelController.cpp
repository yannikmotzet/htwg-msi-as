#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  HuskyHighlevelController::loadParameters();
  subscriber_ = nodeHandle_.subscribe(scan_topic_, queue_size_, &HuskyHighlevelController::scanCallback, this);
  ROS_INFO("HuskyHighlevelController started");
}

HuskyHighlevelController::~HuskyHighlevelController() = default;


void HuskyHighlevelController::scanCallback(const sensor_msgs::LaserScan& msg){
  int minimalDistance;
  minimalDistance = algorithm_.getMinimalDistance(msg);
  ROS_INFO("minimum laser distance: %i m\n", minimalDistance);
}

void HuskyHighlevelController::loadParameters(){
  if (!nodeHandle_.getParam("topic_name", scan_topic_)) {ROS_ERROR("Could not find topic_name parameter!");}
  if (!nodeHandle_.getParam("queue_size", queue_size_)) {ROS_ERROR("Could not find queue_size parameter!");}
}

} /* namespace */
