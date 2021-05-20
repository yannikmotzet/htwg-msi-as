#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle), tfListener_(tfBuffer_)
{
  HuskyHighlevelController::loadParameters();
  subscriber_ = nodeHandle_.subscribe(scan_topic_, queue_size_, &HuskyHighlevelController::scanCallback, this);

  cmd_vel_publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  vis_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  ROS_INFO("HuskyHighlevelController started");
}

HuskyHighlevelController::~HuskyHighlevelController() = default;


void HuskyHighlevelController::scanCallback(const sensor_msgs::LaserScan& msg){
  // get angle and distance to pillar
  double minimalDistance;
  float pillarAngle;
  std::tie(pillarAngle, minimalDistance) = algorithm_.getMinimalDistance(msg);
  ROS_INFO("pillar: %f m, %f degrees \n", minimalDistance, pillarAngle);

  // drive towards pillar
  double angle_offset = ((msg.angle_max - msg.angle_min) / 2) - pillarAngle;
  geometry_msgs::Twist new_cmd_vel;
  new_cmd_vel = algorithm_.calculateAngularVelocity(angle_offset, kp_);
  cmd_vel_publisher_.publish(new_cmd_vel);

  // publish marker
  double x = minimalDistance * cosf(angle_offset);
  double y = minimalDistance * sinf(angle_offset);
  publishMarker(x, y, "base_laser", "odom");

}

void HuskyHighlevelController::loadParameters(){
  if (!nodeHandle_.getParam("topic_name", scan_topic_)) {ROS_ERROR("Could not find topic_name parameter!");}
  if (!nodeHandle_.getParam("queue_size", queue_size_)) {ROS_ERROR("Could not find queue_size parameter!");}
  if (!nodeHandle_.getParam("kp", kp_)) {ROS_ERROR("Could not find kp parameter!");}
}

void HuskyHighlevelController::publishMarker(double x, double y, std::string originFrame, std::string targetFrame){
  geometry_msgs::Pose pose;
  geometry_msgs::TransformStamped transform_stamped;
  visualization_msgs::Marker marker;

  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = -1.5;
  pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI_2, 0);

  marker.header.stamp = ros::Time();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;

  // // aufgabe 7 a
  // marker.header.frame_id = originFrame;
  // marker.pose = pose;
  // marker.color.r = 0.0;
  // marker.color.g = 1.0;
  // marker.color.b = 0.0;
  // vis_pub_.publish(marker);

  // aufgabe 7 b
  marker.header.frame_id = targetFrame;
  marker.id = 1;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  try{
    transform_stamped = tfBuffer_.lookupTransform(targetFrame, originFrame, ros::Time(0));
    tf2::doTransform(pose, pose, transform_stamped);
    marker.pose = pose;
    vis_pub_.publish(marker);
  }
  catch(tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      return;
  }

}

} /* namespace */
