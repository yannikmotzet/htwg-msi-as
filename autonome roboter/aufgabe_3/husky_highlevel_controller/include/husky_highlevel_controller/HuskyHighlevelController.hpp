#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include "husky_highlevel_controller/Algorithm.hpp"
#include <string>

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber subscriber_;
	ros::Publisher cmd_vel_publisher_;
	void scanCallback(const sensor_msgs::LaserScan& msg);
	void loadParameters();
	void publishMarker(double x, double y, std::string originFrame, std::string targetFrame);

	Algorithm algorithm_;
	std::string scan_topic_;
	int queue_size_;
	float kp_;
	tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener tfListener_;
	ros::Publisher vis_pub_;

};

} /* namespace */
