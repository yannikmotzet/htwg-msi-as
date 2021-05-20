#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
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
	void scanCallback(const sensor_msgs::LaserScan& msg);
	void loadParameters();

	Algorithm algorithm_;
	std::string scan_topic_;
	int queue_size_;

};

} /* namespace */
